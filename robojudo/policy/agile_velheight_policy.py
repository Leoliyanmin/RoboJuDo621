# [ih] AGILE velocity-HEIGHT frozen-hands recurrent (LSTM) policy for RoboJuDo sim2sim.
#
# Deploys the AGILE `Velocity-Height-G1-Dev-FrozenHands-Wrist20-Distillation-Recurrent-v0`
# student. Unlike the velocity-history policy (MLP, config-only), this one is RECURRENT and
# has a different obs layout, so it needs its own policy class.
#
# Obs in this exact order (from the exported IO descriptor):
#   generated_commands : 4   [vx, vy, wz, height]            (scale 1)
#   base_ang_vel       : 3                                   (scale 1)
#   projected_gravity  : 3                                   (scale 1)
#   joint_pos_rel      : 29 body (q-q0) + optional hand zeros (scale 1)
#   joint_vel_rel      : 29 body (qdot) + optional hand zeros (scale 0.1)
#   last_action        : 12   (raw policy output, pre-scale)
# The wrist20/frozen-hands policy uses 24 DFQ hand zero slots (128 obs total). The stock
# 29-DoF unitree recurrent student uses no hand slots (80 obs total).
#
# Action: 12 leg joints, PER-JOINT scale, + leg default offset. JIT carries LSTM hidden
# state internally (buffers hidden_state/cell_state) — call with a 1D obs tensor; reset
# by zeroing those buffers.

import numpy as np
import torch

from robojudo.policy import Policy, policy_registry
from robojudo.policy.policy_cfgs import PolicyCfg
from robojudo.utils.util_func import command_remap, get_gravity_orientation

DEFAULT_FROZEN_HAND_OBS = 24  # frozen DFQ hand joints (obs slots 29..52), always 0


@policy_registry.register
class AgileVelHeightRecurrentPolicy(Policy):
    cfg_policy: "PolicyCfg"

    def __init__(self, cfg_policy, device):
        super().__init__(cfg_policy=cfg_policy, device=device)
        self.obs_scales = self.cfg_policy.obs_scales
        self.max_cmd = np.asarray(self.cfg_policy.max_cmd, dtype=np.float32)  # [vx, vy, wz]
        self.commands_map = self.cfg_policy.commands_map
        self.per_joint_scale = np.asarray(self.cfg_policy.action_scales, dtype=np.float32)  # (12,)
        self.num_frozen_hand_obs = int(getattr(self.cfg_policy, "num_frozen_hand_obs", DEFAULT_FROZEN_HAND_OBS))
        # height command state (persistent; r/f keys adjust it)
        self.height_default = float(self.cfg_policy.height_default)
        self.height_min = float(self.cfg_policy.height_min)
        self.height_max = float(self.cfg_policy.height_max)
        self.height_step = float(self.cfg_policy.height_step)
        self.reset()

    def reset(self):
        self.timestep = 0
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)  # 12, raw (pre-scale)
        self._height = self.height_default
        # [ih] velocity command EMA state (cfg.cmd_smooth_alpha, 1.0 = off). Ramps the
        # key-release vx->0 snap so the move->stop transient doesn't kick the legs into a
        # latency-driven resonance (same lever as the 23dof real deploy — see 29dof部署.md).
        self._cmd_smoothed = np.zeros(3, dtype=np.float32)
        # Clear the LSTM hidden state carried inside the exported JIT.
        for name, buf in self.model.named_buffers():
            if "hidden" in name or "cell" in name:
                buf.zero_()

    def post_step_callback(self, commands=None):
        self.timestep += 1

    def _get_commands(self, ctrl_data) -> np.ndarray:
        """Return [vx, vy, wz, height]. vx/vy/wz from held velocity keys / joystick axes;
        height persistent (r/f). Reads the timeout-managed ``keys_pressed`` set so it works
        over an SSH terminal (no key-release events) exactly like UnitreePolicy — the pynput
        backend populates ``keys_pressed`` too, so sim behaviour is unchanged."""
        vel = np.zeros(3, dtype=np.float32)
        for key in ctrl_data.keys():
            if key in ["JoystickCtrl", "UnitreeCtrl"]:
                axes = ctrl_data[key]["axes"]
                lx, ly, rx = axes["LeftX"], axes["LeftY"], axes["RightX"]
                vel[0] = command_remap(ly, self.commands_map[0])
                vel[1] = command_remap(lx, self.commands_map[1])
                vel[2] = command_remap(rx, self.commands_map[2])
                break
            if key in ["KeyboardCtrl"]:
                # height: step on r/f PRESS edges. In terminal mode there are no release
                # events, but OS key-repeat re-emits presses while held, so holding r/f keeps
                # stepping and a single tap steps once — controllable in both backends.
                for event in ctrl_data[key]["keyboard_event"]:
                    if event["type"] != "keyboard" or not event["pressed"]:
                        continue
                    if event["name"] == "r":  # stand taller
                        self._height = min(self.height_max, self._height + self.height_step)
                    elif event["name"] == "f":  # squat lower
                        self._height = max(self.height_min, self._height - self.height_step)
                # velocity: read the timeout-managed keys_pressed set (SSH-terminal safe).
                keys_pressed = {name.lower() for name in ctrl_data[key].get("keys_pressed", [])}
                axis_sign = [
                    float(("w" in keys_pressed) - ("s" in keys_pressed)),  # vx
                    float(("d" in keys_pressed) - ("a" in keys_pressed)),  # vy
                    float(("e" in keys_pressed) - ("q" in keys_pressed)),  # wz
                ]
                for i, sign in enumerate(axis_sign):
                    if sign != 0.0:
                        # full scale (1.0, not 1.5): map exactly to the command_map endpoint,
                        # no linear over-extrapolation past the trained command range.
                        vel[i] = command_remap(sign, self.commands_map[i])
                break
        vel = vel * self.max_cmd  # scale normalized vel-cmd to m/s, rad/s
        # [ih] EMA-smooth the velocity command (height is already a slow r/f ramp, left raw).
        alpha = getattr(self.cfg_policy, "cmd_smooth_alpha", 1.0)
        if alpha < 1.0:
            self._cmd_smoothed = alpha * vel + (1.0 - alpha) * self._cmd_smoothed
            vel = self._cmd_smoothed.copy()
        return np.array([vel[0], vel[1], vel[2], self._height], dtype=np.float32)

    def get_observation(self, env_data, ctrl_data):
        commands = self._get_commands(ctrl_data)  # (4,)
        gravity = get_gravity_orientation(env_data.base_quat)  # (3,)
        # env_data.dof_pos / dof_vel are already sliced to the 29 body joints (obs_dof order)
        body_pos_rel = (env_data.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos  # (29,)
        body_vel_rel = env_data.dof_vel * self.obs_scales.dof_vel  # (29,)
        hand_zeros = np.zeros(self.num_frozen_hand_obs, dtype=np.float32)

        obs = np.concatenate([
            commands,                                    # 4
            env_data.base_ang_vel * self.obs_scales.ang_vel,  # 3
            gravity,                                     # 3
            body_pos_rel, hand_zeros,                    # 29 + optional frozen hand obs
            body_vel_rel, hand_zeros,                    # 29 + optional frozen hand obs
            self.last_action,                            # 12
        ]).astype(np.float32)
        return obs, {"commands": commands}

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        # Recurrent JIT: 1D obs in -> (12,) out; hidden state carried internally.
        x = torch.from_numpy(obs).float().to(self.device)
        with torch.no_grad():
            raw = self.model(x).cpu().numpy().reshape(-1)  # (12,) raw network output
        self.last_action = raw.copy()  # obs uses the RAW (pre-scale) action
        return raw * self.per_joint_scale  # PolicyWrapper adds default_pos -> pd target


@policy_registry.register
class AgileVelHeightTeacherPolicy(AgileVelHeightRecurrentPolicy):
    """[ih] AGILE velheight TEACHER (privileged, non-recurrent MLP) for RoboJuDo sim2sim.

    Diagnostic-only: the teacher's actor obs group (PrivilegedVelocityPolicyCfg) is the
    student's obs PLUS base_lin_vel(3) inserted right after the commands -> 131 dims. That
    extra term is the privileged bit; it's UNAVAILABLE on hardware but IS available in a
    MuJoCo sim (RoboJuDo computes it in body frame, quat_rotate_inverse of qvel[0:3]). So
    the teacher can run in RoboJuDo (NOT on the real robot). Used to check whether the
    teacher stays stable in MuJoCo too (vs the recurrent student's feet-converge), closing
    the 'student-specific drift vs MuJoCo-contact' question. The teacher has NO LSTM, so
    there is no hidden state to carry/reset.

    Obs (131) order (from PrivilegedVelocityPolicyCfg, concatenate_terms=False -> flattened):
      commands 4 | base_lin_vel 3 | base_ang_vel 3 | projected_gravity 3 |
      joint_pos_rel 53 (29 body + 24 frozen hand zeros) | joint_vel_rel 53 | last_action 12
    Action space / gains / per-joint scale are identical to the student (distillation matched
    actions), so this reuses the velheight PolicyCfg unchanged except policy_name/policy_type.
    """

    def get_observation(self, env_data, ctrl_data):
        commands = self._get_commands(ctrl_data)  # (4,)
        gravity = get_gravity_orientation(env_data.base_quat)  # (3,)
        lin_scale = getattr(self.obs_scales, "lin_vel", 1.0)
        base_lin_vel = env_data.base_lin_vel  # (3,) body frame; MujocoEnv already rotates it
        if base_lin_vel is None:
            base_lin_vel = np.zeros(3, dtype=np.float32)
        body_pos_rel = (env_data.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos  # (29,)
        body_vel_rel = env_data.dof_vel * self.obs_scales.dof_vel  # (29,)
        hand_zeros = np.zeros(self.num_frozen_hand_obs, dtype=np.float32)

        obs = np.concatenate([
            commands,                                          # 4
            np.asarray(base_lin_vel, dtype=np.float32) * lin_scale,  # 3  [privileged, teacher-only]
            env_data.base_ang_vel * self.obs_scales.ang_vel,   # 3
            gravity,                                           # 3
            body_pos_rel, hand_zeros,                          # 29 + optional frozen hand obs
            body_vel_rel, hand_zeros,                          # 29 + optional frozen hand obs
            self.last_action,                                  # 12
        ]).astype(np.float32)
        return obs, {"commands": commands}
