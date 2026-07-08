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
        # [ih] arm-obs mask: physically the arms may swing (walk_elbow/box) for a natural look,
        # but driving them injects OUT-OF-DISTRIBUTION arm joint_pos/vel obs the policy never saw
        # in training (arms pinned ~0). That obs perturbation makes the policy squat ~9 cm lower
        # (cmd 0.72 -> 0.65) via the OBS channel, NOT physics (verified: zeroing arm obs restores
        # height even with arms still swinging; the swing's TURN aid is physical and survives).
        # See docs/清空手臂obs.md. When enabled, feed the policy training-default arm obs (0),
        # decoupling the cosmetic arm motion from the locomotion policy. Off by default so stock
        # checkpoints stay bit-for-bit compatible.
        self._arm_obs_idx = [
            i for i, n in enumerate(self.cfg_obs_dof.joint_names)
            if ("shoulder" in n) or ("elbow" in n) or ("wrist" in n)
        ]
        self._zero_arm_obs_default = bool(getattr(self.cfg_policy, "zero_arm_obs_default", False))
        self._zero_arm_obs_kb = bool(getattr(self.cfg_policy, "zero_arm_obs_keyboard", False))
        # height command state (persistent; r/f keys adjust it)
        self.height_default = float(self.cfg_policy.height_default)
        self.height_min = float(self.cfg_policy.height_min)
        self.height_max = float(self.cfg_policy.height_max)
        self.height_step = float(self.cfg_policy.height_step)
        self.turn_scale_keyboard = bool(getattr(self.cfg_policy, "turn_scale_keyboard", False))
        self.turn_scale_default = float(getattr(self.cfg_policy, "turn_scale_default", 1.0))
        self.turn_scale_step = float(getattr(self.cfg_policy, "turn_scale_step", 0.1))
        self.turn_scale_min = float(getattr(self.cfg_policy, "turn_scale_min", 0.6))
        self.turn_scale_max = float(getattr(self.cfg_policy, "turn_scale_max", 1.5))
        self.turn_prime_enabled = bool(getattr(self.cfg_policy, "turn_prime_enabled", False))
        turn_prime_idle_s = float(getattr(self.cfg_policy, "turn_prime_idle_seconds", 0.20))
        turn_prime_duration_s = float(getattr(self.cfg_policy, "turn_prime_duration_seconds", 0.80))
        self.turn_prime_idle_steps = max(0, int(round(turn_prime_idle_s * self.freq)))
        self.turn_prime_steps = max(1, int(round(turn_prime_duration_s * self.freq)))
        self.turn_prime_vx = float(getattr(self.cfg_policy, "turn_prime_vx", 0.18))
        self.turn_prime_yaw_scale = float(getattr(self.cfg_policy, "turn_prime_yaw_scale", 0.0))
        turn_prime_height = float(getattr(self.cfg_policy, "turn_prime_height", self.height_default))
        self.turn_prime_height = min(self.height_max, max(self.height_min, turn_prime_height))
        # [ih] basin-break lever: "forward" (step), "height" (stand taller, no drift), or "both".
        self.turn_prime_mode = str(getattr(self.cfg_policy, "turn_prime_mode", "forward"))
        turn_prime_boost = float(getattr(self.cfg_policy, "turn_prime_boost_height", 0.90))
        # not clamped to height_max — standing above the r/f cap is the whole point of the boost.
        self.turn_prime_boost_height = min(1.0, max(self.height_min, turn_prime_boost))
        self.reset()

    def reset(self):
        self.timestep = 0
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)  # 12, raw (pre-scale)
        self._height = self.height_default
        self._turn_scale = self.turn_scale_default
        self._zero_arm_obs = self._zero_arm_obs_default  # [ih] toggled by 'b' (see _get_commands)
        # [ih] velocity command EMA state (cfg.cmd_smooth_alpha, 1.0 = off). Ramps the
        # key-release vx->0 snap so the move->stop transient doesn't kick the legs into a
        # latency-driven resonance (same lever as the 23dof real deploy — see 29dof部署.md).
        self._cmd_smoothed = np.zeros(3, dtype=np.float32)
        self._idle_cmd_steps = self.turn_prime_idle_steps
        self._turn_prime_steps_left = 0
        self._turn_prime_active = False
        # Clear the LSTM hidden state carried inside the exported JIT.
        for name, buf in self.model.named_buffers():
            if "hidden" in name or "cell" in name:
                buf.zero_()

    def post_step_callback(self, commands=None):
        self.timestep += 1

    def _apply_turn_prime(self, vel: np.ndarray, height: float) -> tuple[np.ndarray, float]:
        """Break the dead-stop pure-yaw "planted crouch" basin (feet stick, torso twists =
        stick-slip 原地拧). Two levers, selected by turn_prime_mode:
          - "forward": inject a tiny forward step for turn_prime_duration on the idle->yaw edge
            (robot takes a step -> escapes the basin). Cost: lurches forward a little.
          - "height": raise the height command to turn_prime_boost_height (~0.9) for as long as
            pure yaw is held (robot stands taller -> leaves the planted-crouch basin -> turns).
            Cost: stands up while turning, returns to the set height on release. No forward drift.
          - "both": apply both.
        Intentionally outside the learned policy: the exported obs/action ABI is unchanged, this
        only reshapes user commands. See docs/清空手臂obs.md (basin discussion)."""
        self._turn_prime_active = False
        if not self.turn_prime_enabled:
            return vel, height

        eps = 1e-4
        is_idle_cmd = np.max(np.abs(vel)) <= eps
        is_pure_yaw = (
            abs(float(vel[2])) > eps
            and abs(float(vel[0])) <= eps
            and abs(float(vel[1])) <= eps
        )

        if not is_pure_yaw:
            self._turn_prime_steps_left = 0
            if is_idle_cmd:
                self._idle_cmd_steps = min(self.turn_prime_idle_steps, self._idle_cmd_steps + 1)
            else:
                self._idle_cmd_steps = 0
            return vel, height

        do_forward = self.turn_prime_mode in ("forward", "both")
        do_height = self.turn_prime_mode in ("height", "both")

        # height lever: boost the height command for the whole pure-yaw command (no forward drift)
        if do_height:
            height = max(float(height), self.turn_prime_boost_height)
            self._turn_prime_active = True

        # forward lever: transient forward step engaged on the idle -> pure-yaw edge
        if do_forward and self._turn_prime_steps_left <= 0 and self._idle_cmd_steps >= self.turn_prime_idle_steps:
            self._turn_prime_steps_left = self.turn_prime_steps
        self._idle_cmd_steps = 0
        if do_forward:
            height = max(float(height), self.turn_prime_height)
            if self._turn_prime_steps_left > 0:
                primed_vel = vel.copy()
                primed_vel[0] = self.turn_prime_vx
                primed_vel[1] = 0.0
                primed_vel[2] = vel[2] * self.turn_prime_yaw_scale
                self._turn_prime_steps_left -= 1
                self._turn_prime_active = True
                return primed_vel, height

        return vel, height

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
                    elif self.turn_scale_keyboard and event["name"] == "m":
                        self._turn_scale = min(self.turn_scale_max, self._turn_scale + self.turn_scale_step)
                    elif self.turn_scale_keyboard and event["name"] == "n":
                        self._turn_scale = max(self.turn_scale_min, self._turn_scale - self.turn_scale_step)
                    elif self._zero_arm_obs_kb and event["name"] == "b":  # toggle arm-obs mask
                        self._zero_arm_obs = not self._zero_arm_obs
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
        vel[2] *= self._turn_scale
        height = self._height
        vel, height = self._apply_turn_prime(vel, height)
        # [ih] EMA-smooth the velocity command (height is already a slow r/f ramp, left raw).
        alpha = getattr(self.cfg_policy, "cmd_smooth_alpha", 1.0)
        if alpha < 1.0:
            self._cmd_smoothed = alpha * vel + (1.0 - alpha) * self._cmd_smoothed
            vel = self._cmd_smoothed.copy()
        return np.array([vel[0], vel[1], vel[2], height], dtype=np.float32)

    def _mask_arm_obs(self, body_pos_rel, body_vel_rel):
        """[ih] zero the arm joint_pos/vel obs slots when the arm-obs mask is on, so cosmetic
        arm choreography does not perturb the locomotion policy. See docs/清空手臂obs.md."""
        if self._zero_arm_obs and self._arm_obs_idx:
            body_pos_rel = body_pos_rel.copy()
            body_vel_rel = body_vel_rel.copy()
            for i in self._arm_obs_idx:
                body_pos_rel[i] = 0.0
                body_vel_rel[i] = 0.0
        return body_pos_rel, body_vel_rel

    def get_observation(self, env_data, ctrl_data):
        commands = self._get_commands(ctrl_data)  # (4,)
        gravity = get_gravity_orientation(env_data.base_quat)  # (3,)
        # env_data.dof_pos / dof_vel are already sliced to the 29 body joints (obs_dof order)
        body_pos_rel = (env_data.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos  # (29,)
        body_vel_rel = env_data.dof_vel * self.obs_scales.dof_vel  # (29,)
        body_pos_rel, body_vel_rel = self._mask_arm_obs(body_pos_rel, body_vel_rel)
        hand_zeros = np.zeros(self.num_frozen_hand_obs, dtype=np.float32)

        obs = np.concatenate([
            commands,                                    # 4
            env_data.base_ang_vel * self.obs_scales.ang_vel,  # 3
            gravity,                                     # 3
            body_pos_rel, hand_zeros,                    # 29 + optional frozen hand obs
            body_vel_rel, hand_zeros,                    # 29 + optional frozen hand obs
            self.last_action,                            # 12
        ]).astype(np.float32)
        return obs, {"commands": commands, "turn_scale": self._turn_scale,
                     "turn_prime": self._turn_prime_active, "zero_arm_obs": self._zero_arm_obs}

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
        body_pos_rel, body_vel_rel = self._mask_arm_obs(body_pos_rel, body_vel_rel)
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
        return obs, {"commands": commands, "turn_scale": self._turn_scale,
                     "turn_prime": self._turn_prime_active, "zero_arm_obs": self._zero_arm_obs}
