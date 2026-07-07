import logging
import time

import mujoco
import mujoco_viewer
import numpy as np

from robojudo.environment import Environment, env_registry
from robojudo.environment.env_cfgs import MujocoEnvCfg
from robojudo.environment.utils.mujoco_viz import MujocoVisualizer
from robojudo.utils.util_func import quat_rotate_inverse_np, quatToEuler

logger = logging.getLogger(__name__)

G1_ARM_HANG_POSE: dict[str, float] = {
    "left_shoulder_pitch_joint": 0.15,
    "left_shoulder_roll_joint": 0.02,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.05,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "right_shoulder_pitch_joint": 0.15,
    "right_shoulder_roll_joint": -0.02,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.05,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

G1_ARM_BOX_POSE: dict[str, float] = {
    "left_shoulder_pitch_joint": -0.75,
    "left_shoulder_roll_joint": 0.12,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.10,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "right_shoulder_pitch_joint": -0.75,
    "right_shoulder_roll_joint": -0.12,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.10,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

G1_ARM_SPREAD_JOINTS = {
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
}


@env_registry.register
class MujocoEnv(Environment):
    cfg_env: MujocoEnvCfg

    def __init__(self, cfg_env: MujocoEnvCfg, device="cpu"):
        super().__init__(cfg_env=cfg_env, device=device)

        self.sim_duration = cfg_env.sim_duration
        self.sim_dt = cfg_env.sim_dt
        self.sim_decimation = cfg_env.sim_decimation
        self.control_dt = self.sim_dt * self.sim_decimation

        self.model = mujoco.MjModel.from_xml_path(cfg_env.xml)  # pyright: ignore[reportAttributeAccessIssue]
        self.model.opt.timestep = self.sim_dt
        self.data = mujoco.MjData(self.model)  # pyright: ignore[reportAttributeAccessIssue]
        # mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_step(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]

        # [ih] wrist-load (sustained world-down force, e.g. box carry) + runtime control.
        self._wrist_load_n = float(getattr(cfg_env, "wrist_load_n", 0.0) or 0.0)
        self._wrist_load_kb = bool(getattr(cfg_env, "wrist_load_keyboard", False))
        self._wrist_load_step = float(getattr(cfg_env, "wrist_load_step", 2.0))
        self._wrist_load_max = float(getattr(cfg_env, "wrist_load_max", 30.0))
        self._wrist_load_show = bool(getattr(cfg_env, "wrist_load_show", True))
        self._wrist_load_body_ids = []
        self._cmd_readout = None  # [ih] (vel_xyz_real, height_cmd_or_None) from the policy
        self._cmd_extras = {}
        # [ih] deterministic waist-pitch forward lean vs height command (Phase 2)
        self._waist_lean_max = float(np.radians(getattr(cfg_env, "waist_squat_lean_deg", 0.0) or 0.0))
        self._waist_lean_hi = float(getattr(cfg_env, "waist_lean_hi", 0.65))
        self._waist_lean_lo = float(getattr(cfg_env, "waist_lean_lo", 0.50))
        self._waist_lean_rate = float(np.radians(getattr(cfg_env, "waist_lean_rate_dps", 60.0)))
        self._waist_cur = 0.0
        self._waist_dof = None
        self._waist_lo, self._waist_hi = -0.52, 0.52
        # [ih] manual keyboard waist override
        self._waist_manual = bool(getattr(cfg_env, "waist_manual_keyboard", False))
        self._waist_manual_step = float(np.radians(getattr(cfg_env, "waist_manual_step_deg", 2.0)))
        self._waist_manual_val = 0.0
        if self._waist_lean_max != 0.0 or self._waist_manual:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "waist_pitch_joint")
            if jid >= 0:
                self._waist_dof = int(self.model.jnt_qposadr[jid] - 7)  # index into pd_target (free joint=0..6)
                self._waist_lo = float(self.model.jnt_range[jid][0])
                self._waist_hi = float(self.model.jnt_range[jid][1])

        self._arm_motion_mode = getattr(cfg_env, "arm_motion_mode", "off")
        self._arm_squat_height = float(getattr(cfg_env, "arm_squat_height", 0.52))
        self._arm_stand_height = float(getattr(cfg_env, "arm_stand_height", 0.66))
        self._arm_swing_amp = float(getattr(cfg_env, "arm_swing_amp", 0.10))
        self._arm_swing_hz = float(getattr(cfg_env, "arm_swing_hz", 1.15))
        self._arm_swing_speed_ref = float(getattr(cfg_env, "arm_swing_speed_ref", 0.45))
        self._arm_stride_ref = float(getattr(cfg_env, "arm_stride_ref", 0.28))
        self._arm_stride_filter_alpha = float(getattr(cfg_env, "arm_stride_filter_alpha", 0.35))
        self._arm_elbow_swing_amp = float(getattr(cfg_env, "arm_elbow_swing_amp", 0.12))
        self._arm_wrist_swing_amp = float(getattr(cfg_env, "arm_wrist_swing_amp", 0.08))
        self._arm_walk_spread_amp = float(getattr(cfg_env, "arm_walk_spread_amp", 0.10))
        self._arm_motion_rate = float(np.radians(getattr(cfg_env, "arm_motion_rate_dps", 180.0)))
        self._arm_swing_kb = bool(getattr(cfg_env, "arm_swing_keyboard", False))
        self._arm_swing_scale = float(getattr(cfg_env, "arm_swing_scale", 1.0))
        self._arm_swing_step = float(getattr(cfg_env, "arm_swing_step", 0.1))
        self._arm_swing_min = float(getattr(cfg_env, "arm_swing_min", 0.0))
        self._arm_swing_max = float(getattr(cfg_env, "arm_swing_max", 2.0))
        self._arm_reach_kb = bool(getattr(cfg_env, "arm_reach_keyboard", False))
        self._arm_reach_scale = float(getattr(cfg_env, "arm_reach_scale", 1.0))
        self._arm_reach_step = float(getattr(cfg_env, "arm_reach_step", 0.1))
        self._arm_reach_min = float(getattr(cfg_env, "arm_reach_min", 0.5))
        self._arm_reach_max = float(getattr(cfg_env, "arm_reach_max", 1.6))
        self._arm_mode_kb = bool(getattr(cfg_env, "arm_mode_keyboard", False))
        self._arm_mode_cycle = ("off", "walk_elbow", "walk_squat_box")
        self._arm_spread_kb = bool(getattr(cfg_env, "arm_spread_keyboard", False))
        self._arm_spread_scale = float(getattr(cfg_env, "arm_spread_scale", 1.0))
        self._arm_spread_step = float(getattr(cfg_env, "arm_spread_step", 0.1))
        self._arm_spread_min = float(getattr(cfg_env, "arm_spread_min", 0.5))
        self._arm_spread_max = float(getattr(cfg_env, "arm_spread_max", 2.2))
        self._arm_phase = 0.0
        self._arm_stride_signal = 0.0
        self._arm_stride_rest = np.zeros(2, dtype=np.float64)
        self._arm_stride_body_ids: tuple[int, int, int] | None = None
        self._arm_dofs: dict[str, int] = {}
        self._arm_cur: dict[str, float] = {}
        self._arm_motion_label = "off"
        if self._arm_motion_mode != "off":
            stride_body_ids = []
            for body_name in ("torso_link", "left_ankle_roll_link", "right_ankle_roll_link"):
                stride_body_ids.append(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name))
            if all(body_id >= 0 for body_id in stride_body_ids):
                self._arm_stride_body_ids = tuple(stride_body_ids)
                self._arm_stride_rest = np.asarray(self._get_arm_stride_components(), dtype=np.float64)
            else:
                logger.warning("[ih] arm swing using timer fallback; missing stride body ids")
            for joint_name in G1_ARM_HANG_POSE:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if jid < 0:
                    continue
                dof_idx = int(self.model.jnt_qposadr[jid] - 7)
                if 0 <= dof_idx < self.num_dofs:
                    self._arm_dofs[joint_name] = dof_idx
                    self._arm_cur[joint_name] = float(self.data.qpos[self.model.jnt_qposadr[jid]])
            missing = sorted(set(G1_ARM_HANG_POSE) - set(self._arm_dofs))
            if missing:
                logger.warning(f"[ih] arm motion disabled; missing joints: {missing}")
                self._arm_motion_mode = "off"
        # resolve bodies if there is (or could be, via keyboard) a load to apply
        if self._wrist_load_n > 0.0 or self._wrist_load_kb:
            for bn in getattr(cfg_env, "wrist_load_bodies", []):
                bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, bn)
                if bid >= 0:
                    self._wrist_load_body_ids.append(bid)
        # optional keyboard listener for '[' / ']' load, ',' / '.' waist,
        # j/k arm swing, z/x arm reach, c/v arm spread
        # (own pynput thread; coexists with KeyboardCtrl — pynput allows multiple).
        self._kb_queue = None
        if (
            self._wrist_load_kb
            or self._waist_manual
            or self._arm_mode_kb
            or self._arm_swing_kb
            or self._arm_reach_kb
            or self._arm_spread_kb
        ):
            try:
                from queue import Queue

                from robojudo.controller.utils.keyboard import KeyboardThread

                self._kb_queue = Queue(maxsize=100)
                KeyboardThread(self._kb_queue).start()
            except Exception as e:  # no display / pynput unavailable — silently skip
                logger.warning(f"[ih] env keyboard disabled: {e}")
                self._kb_queue = None

        self.viewer = mujoco_viewer.MujocoViewer(
            self.model,
            self.data,
            width=1200,
            height=900,
            hide_menus=True,
            diable_key_callbacks=True,
        )
        self.viewer.cam.distance = 3.0
        self.viewer.cam.elevation = -10.0
        self.viewer.cam.azimuth = 180.0
        # self.viewer._paused = True

        if cfg_env.visualize_extras:
            self.visualizer = MujocoVisualizer(self.viewer)
        else:
            self.visualizer = None

        self.last_time = time.time()
        self.random_heading = cfg_env.random_heading

        self._apply_random_heading()

        self.update()  # get initial state

    def _apply_random_heading(self):
        """Rotate the root body by a random yaw if random_heading is enabled."""
        if not self.random_heading:
            return
        yaw = np.random.uniform(0, 2 * np.pi)
        c, s = np.cos(yaw / 2), np.sin(yaw / 2)
        q = self.data.qpos[3:7].copy()  # MuJoCo [w, x, y, z]
        # Pre-multiply by yaw rotation q_yaw=[c,0,0,s]: q_new = q_yaw ⊗ q
        self.data.qpos[3] = c * q[0] - s * q[3]
        self.data.qpos[4] = c * q[1] - s * q[2]
        self.data.qpos[5] = c * q[2] + s * q[1]
        self.data.qpos[6] = c * q[3] + s * q[0]

    def reborn(self, init_qpos=None):
        if init_qpos is not None:
            self.data.qpos[0:7] = init_qpos
            self.data.qvel[:] = 0.0
            self.data.ctrl[:] = 0.0
        else:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # pyright: ignore[reportAttributeAccessIssue]
            self._apply_random_heading()
        mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]

    def reset(self):
        if self.born_place_align:  # TODO: merge
            self.born_place_align = False  # disable during reset
            self.update()
            self.born_place_align = True  # enable after reset
            self.set_born_place()
            self.update()

    def set_gains(self, stiffness, damping):
        assert len(stiffness) == self.num_dofs and len(damping) == self.num_dofs
        self.stiffness = np.asarray(stiffness)
        self.damping = np.asarray(damping)

    def self_check(self):
        pass

    def set_born_place(self, quat: np.ndarray | None = None, pos: np.ndarray | None = None):
        quat_ = self.base_quat if quat is None else quat
        pos_ = self.base_pos if pos is None else pos
        super().set_born_place(quat_, pos_)

    def update(self, simple=False):  # TODO: clean sensors in xml
        """simple: only update dof pos & vel"""
        dof_pos = self.data.qpos.astype(np.float32)[-self.num_dofs :]
        dof_vel = self.data.qvel.astype(np.float32)[-self.num_dofs :]

        self._dof_pos = dof_pos.copy()
        self._dof_vel = dof_vel.copy()

        if simple:
            return

        quat = self.data.qpos.astype(np.float32)[3:7][[1, 2, 3, 0]]
        ang_vel = self.data.qvel.astype(np.float32)[3:6]
        base_pos = self.data.qpos.astype(np.float32)[:3]
        lin_vel = self.data.qvel.astype(np.float32)[0:3]

        if self.born_place_align:
            quat, base_pos = self.base_align.align_transform(quat, base_pos)

        lin_vel = quat_rotate_inverse_np(quat, lin_vel)
        rpy = quatToEuler(quat)

        self._base_rpy = rpy.copy()
        self._base_quat = quat.copy()
        self._base_ang_vel = ang_vel.copy()

        self._base_pos = base_pos.copy()
        self._base_lin_vel = lin_vel.copy()

        if self.update_with_fk:
            fk_info = self.fk()
            self._fk_info = fk_info.copy()
            self._torso_ang_vel = fk_info[self._torso_name]["ang_vel"]
            self._torso_quat = fk_info[self._torso_name]["quat"]
            self._torso_pos = fk_info[self._torso_name]["pos"]

    def set_cmd_readout(self, commands, max_cmd=None, extras=None):
        """[ih] store the policy's command (set target) for the viewer readout.

        commands len 4 -> [vx, vy, wz, height] already in real units (velheight policy);
        len 3 -> normalized [vx, vy, wz] (UnitreeWoGaitPolicy) -> scale by max_cmd to m/s.
        """
        self._cmd_extras = extras or {}
        if commands is None:
            self._cmd_readout = None
            return
        c = np.asarray(commands, dtype=np.float32).reshape(-1)
        if c.shape[0] >= 4:
            self._cmd_readout = (c[:3].copy(), float(c[3]))
        else:
            vel = c[:3].copy()
            if max_cmd is not None:
                vel = vel * np.asarray(max_cmd, dtype=np.float32).reshape(-1)[:3]
            self._cmd_readout = (vel, None)

    def _poll_env_keys(self):
        """[ih] drain this step's keyboard events: '['/']' wrist load, ','/'.' manual waist."""
        if self._kb_queue is None:
            return
        from queue import Empty

        while not self._kb_queue.empty():
            try:
                ev = self._kb_queue.get_nowait()
            except Empty:
                break
            if ev.get("type") != "keyboard" or not ev.get("pressed"):
                continue
            name = ev.get("name")
            if name == "]":
                self._wrist_load_n = min(self._wrist_load_max, self._wrist_load_n + self._wrist_load_step)
                logger.info(f"[ih] wrist load -> {self._wrist_load_n:.1f} N/wrist")
            elif name == "[":
                self._wrist_load_n = max(0.0, self._wrist_load_n - self._wrist_load_step)
                logger.info(f"[ih] wrist load -> {self._wrist_load_n:.1f} N/wrist")
            elif self._waist_manual and name == ".":  # lean forward (+)
                self._waist_manual_val = min(self._waist_hi, self._waist_manual_val + self._waist_manual_step)
                logger.info(f"[ih] waist_pitch -> {np.degrees(self._waist_manual_val):.0f} deg")
            elif self._waist_manual and name == ",":  # lean back (-)
                self._waist_manual_val = max(self._waist_lo, self._waist_manual_val - self._waist_manual_step)
                logger.info(f"[ih] waist_pitch -> {np.degrees(self._waist_manual_val):.0f} deg")
            elif self._arm_swing_kb and name == "k":
                self._arm_swing_scale = min(self._arm_swing_max, self._arm_swing_scale + self._arm_swing_step)
                logger.info(f"[ih] arm swing -> {self._arm_swing_scale:.2f}")
            elif self._arm_swing_kb and name == "j":
                self._arm_swing_scale = max(self._arm_swing_min, self._arm_swing_scale - self._arm_swing_step)
                logger.info(f"[ih] arm swing -> {self._arm_swing_scale:.2f}")
            elif self._arm_reach_kb and name == "x":
                self._arm_reach_scale = min(self._arm_reach_max, self._arm_reach_scale + self._arm_reach_step)
                logger.info(f"[ih] arm reach -> {self._arm_reach_scale:.2f}")
            elif self._arm_reach_kb and name == "z":
                self._arm_reach_scale = max(self._arm_reach_min, self._arm_reach_scale - self._arm_reach_step)
                logger.info(f"[ih] arm reach -> {self._arm_reach_scale:.2f}")
            elif self._arm_spread_kb and name == "v":
                self._arm_spread_scale = min(self._arm_spread_max, self._arm_spread_scale + self._arm_spread_step)
                logger.info(f"[ih] arm spread -> {self._arm_spread_scale:.2f}")
            elif self._arm_spread_kb and name == "c":
                self._arm_spread_scale = max(self._arm_spread_min, self._arm_spread_scale - self._arm_spread_step)
                logger.info(f"[ih] arm spread -> {self._arm_spread_scale:.2f}")
            elif self._arm_mode_kb and name == "m":
                idx = self._arm_mode_cycle.index(self._arm_motion_mode) if self._arm_motion_mode in self._arm_mode_cycle else 0
                self._arm_motion_mode = self._arm_mode_cycle[(idx + 1) % len(self._arm_mode_cycle)]
                logger.info(f"[ih] arm mode -> {self._arm_motion_mode}")

    @staticmethod
    def _smoothstep(alpha: float) -> float:
        alpha = float(np.clip(alpha, 0.0, 1.0))
        return alpha * alpha * (3.0 - 2.0 * alpha)

    def _get_arm_stride_components(self) -> tuple[float, float]:
        """Return left-minus-right foot delta in torso forward/lateral coordinates."""
        torso_id, left_foot_id, right_foot_id = self._arm_stride_body_ids
        torso_mat = self.data.xmat[torso_id].reshape(3, 3)
        foot_delta = self.data.xpos[left_foot_id] - self.data.xpos[right_foot_id]
        return float(np.dot(foot_delta, torso_mat[:, 0])), float(np.dot(foot_delta, torso_mat[:, 1]))

    def _get_arm_stride_signal(self, speed_alpha: float, vel: np.ndarray) -> float:
        """Return signed gait phase from feet: + means left leg leads, right arm forward."""
        self._arm_phase += 2.0 * np.pi * self._arm_swing_hz * self.control_dt
        timer_raw = float(np.sin(self._arm_phase))
        if self._arm_stride_body_ids is None:
            return speed_alpha * timer_raw

        if speed_alpha < 0.05:
            raw_stride = 0.0
        else:
            stride_now = np.asarray(self._get_arm_stride_components(), dtype=np.float64)
            stride_delta = stride_now - self._arm_stride_rest
            move_norm = float(np.linalg.norm(vel[:2]))
            if move_norm > 1e-6:
                move_dir = np.asarray(vel[:2], dtype=np.float64) / move_norm
                stride_delta_cmd = float(np.dot(stride_delta, move_dir))
            else:
                stride_delta_cmd = float(stride_delta[0])
            raw_stride = float(stride_delta_cmd / max(self._arm_stride_ref, 1e-6))
            raw_stride = float(np.clip(raw_stride, -1.0, 1.0))
            if abs(raw_stride) < 0.08 and abs(float(vel[1])) > abs(float(vel[0])) * 0.75:
                raw_stride = timer_raw

        alpha = float(np.clip(self._arm_stride_filter_alpha, 0.0, 1.0))
        self._arm_stride_signal += alpha * (raw_stride - self._arm_stride_signal)
        return self._arm_stride_signal * speed_alpha

    def _apply_arm_motion(self, pd_target):
        if self._arm_motion_mode == "off":
            return pd_target

        vel = np.zeros(3, dtype=np.float32)
        height_cmd = None
        if self._cmd_readout is not None:
            vel, height_cmd = self._cmd_readout

        squat_alpha = 0.0
        if height_cmd is not None:
            denom = max(self._arm_stand_height - self._arm_squat_height, 1e-6)
            squat_alpha = self._smoothstep((self._arm_stand_height - height_cmd) / denom)

        speed = float(np.linalg.norm(vel[:2]) + 0.25 * abs(float(vel[2])))
        speed_alpha = float(np.clip(speed / max(self._arm_swing_speed_ref, 1e-6), 0.0, 1.0))
        walk_alpha = 1.0 - squat_alpha
        stride_signal = self._get_arm_stride_signal(speed_alpha, vel)
        swing_signal = self._arm_swing_scale * walk_alpha * stride_signal
        swing = self._arm_swing_amp * swing_signal

        target_pose = dict(G1_ARM_HANG_POSE)
        # shoulder roll spread — common to all non-off modes
        walk_spread = self._arm_walk_spread_amp * (self._arm_spread_scale - 1.0) * walk_alpha
        target_pose["left_shoulder_roll_joint"] += walk_spread
        target_pose["right_shoulder_roll_joint"] -= walk_spread

        if self._arm_motion_mode == "walk_squat_box":
            # Primary swing: shoulder pitch forward/backward; elbow/wrist as secondary.
            target_pose["left_shoulder_pitch_joint"] += swing
            target_pose["right_shoulder_pitch_joint"] -= swing
            left_forward = max(-swing_signal, 0.0)
            right_forward = max(swing_signal, 0.0)
            target_pose["left_elbow_joint"] += self._arm_elbow_swing_amp * left_forward
            target_pose["right_elbow_joint"] += self._arm_elbow_swing_amp * right_forward
            target_pose["left_wrist_pitch_joint"] -= self._arm_wrist_swing_amp * left_forward
            target_pose["right_wrist_pitch_joint"] -= self._arm_wrist_swing_amp * right_forward
            for joint_name, box_value in G1_ARM_BOX_POSE.items():
                box_delta = box_value - G1_ARM_HANG_POSE[joint_name]
                if joint_name in G1_ARM_SPREAD_JOINTS:
                    box_delta *= self._arm_spread_scale
                box_value = G1_ARM_HANG_POSE[joint_name] + self._arm_reach_scale * box_delta
                target_pose[joint_name] = (1.0 - squat_alpha) * target_pose[joint_name] + squat_alpha * box_value
        elif self._arm_motion_mode == "walk_elbow":
            # Primary swing: elbow flex/extend sinusoidally; shoulder pitch stays at hang.
            # Sign: elbow bends MORE when that arm is going backward (matches natural gait).
            #   left_elbow  -= swing  →  bends when stride_signal < 0 (left arm back)
            #   right_elbow += swing  →  bends when stride_signal > 0 (right arm back)
            target_pose["left_elbow_joint"] -= swing
            target_pose["right_elbow_joint"] += swing

        pd_target = np.array(pd_target, dtype=np.float64)
        max_delta = self._arm_motion_rate * self.control_dt
        for joint_name, target in target_pose.items():
            dof_idx = self._arm_dofs[joint_name]
            lo, hi = self.position_limits[dof_idx]
            target = float(np.clip(target, lo, hi))
            cur = self._arm_cur[joint_name]
            cur += float(np.clip(target - cur, -max_delta, max_delta))
            self._arm_cur[joint_name] = cur
            pd_target[dof_idx] = cur

        if self._arm_motion_mode == "walk_squat_box" and squat_alpha > 0.65:
            self._arm_motion_label = "box reach"
        elif speed_alpha > 0.05:
            self._arm_motion_label = "walk stride"
        else:
            self._arm_motion_label = "hang"
        return pd_target

    def step(self, pd_target, hand_pose=None):
        assert len(pd_target) == self.num_dofs, "pd_target len should be num_dofs of env"

        if hand_pose is not None:
            logger.info("Hand pose-->", hand_pose)

        # [ih] poll '[' / ']' to adjust the wrist load before rendering this frame
        self._poll_env_keys()
        pd_target = self._apply_arm_motion(pd_target)
        # [ih] floating readout above the robot. Velocity/height show the COMMAND (set
        # target, like the wrist load) when the policy provides it via set_cmd_readout;
        # pelvis height also shows the MEASURED value so the command/actual gap is visible
        # (e.g. a deep-squat height cmd of 0.40 that the policy only tracks to ~0.48).
        if self._wrist_load_show:
            root = self.data.qpos.astype(np.float32)[:3]
            x, y, z = float(root[0]), float(root[1]), float(root[2])
            cmd = self._cmd_readout

            def _readout(dz, color, text, mid):
                self.viewer.add_marker(
                    pos=[x, y, z + dz],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.03, 0.03, 0.03],
                    rgba=color,
                    label=text,
                    id=mid,
                )

            if cmd is not None:
                vel, h_cmd = cmd
                turn_scale = self._cmd_extras.get("turn_scale")
                turn_text = "" if turn_scale is None else f"  turn={float(turn_scale):.1f} (n/m)"
                turn_text += " prime" if self._cmd_extras.get("turn_prime") else ""
                _readout(
                    1.28,
                    [0.2, 0.8, 1.0, 0.9],
                    f"vel cmd  vx={float(vel[0]):+.2f} vy={float(vel[1]):+.2f} wz={float(vel[2]):+.2f}{turn_text}",
                    97,
                )
                if h_cmd is not None:
                    _readout(1.14, [0.4, 1.0, 0.4, 0.9],
                             f"height cmd {h_cmd:.2f} / now {z:.2f} m", 98)
                else:
                    _readout(1.14, [0.4, 1.0, 0.4, 0.9], f"pelvis height = {z:.2f} m", 98)
            else:  # fallback: measured velocity (no command piped in)
                vb = getattr(self, "_base_lin_vel", np.zeros(3))
                wb = getattr(self, "_base_ang_vel", np.zeros(3))
                _readout(1.28, [0.2, 0.8, 1.0, 0.9],
                         f"vel  vx={float(vb[0]):+.2f} vy={float(vb[1]):+.2f} wz={float(wb[2]):+.2f}", 97)
                _readout(1.14, [0.4, 1.0, 0.4, 0.9], f"pelvis height = {z:.2f} m", 98)
            if self._wrist_load_body_ids:
                _readout(1.00, [1.0, 0.55, 0.0, 0.9],
                         f"wrist load = {self._wrist_load_n:.0f} N/wrist  ([ - ] +)", 99)
            if self._waist_manual:
                _readout(0.86, [1.0, 0.9, 0.2, 0.95],
                         f"waist_pitch = {np.degrees(self._waist_manual_val):+.0f} deg  (, back / . fwd)", 96)
            if self._arm_motion_mode != "off" or self._arm_mode_kb:
                mode_hint = f"  [m]={self._arm_motion_mode}" if self._arm_mode_kb else ""
                _readout(
                    0.72,
                    [0.8, 0.45, 1.0, 0.9] if self._arm_motion_mode != "off" else [0.5, 0.5, 0.5, 0.8],
                    f"arms:{mode_hint}  {self._arm_motion_label}  swing={self._arm_swing_scale:.1f} j/k"
                    f"  reach={self._arm_reach_scale:.1f} z/x"
                    f"  spread={self._arm_spread_scale:.1f} c/v",
                    95,
                )

        self.viewer.cam.lookat = self.data.qpos.astype(np.float32)[:3]
        if self.viewer.is_alive:
            self.viewer.render()

        # [ih] apply sustained world-frame-down wrist load (box carry) each step
        if self._wrist_load_body_ids:
            for bid in self._wrist_load_body_ids:
                self.data.xfrc_applied[bid, :3] = [0.0, 0.0, -self._wrist_load_n]

        # [ih] waist_pitch override of the (non-policy) waist_pitch pd_target. MANUAL keyboard
        # (',' / '.', clamped to the joint limit) takes precedence; else the deterministic
        # height-conditioned forward lean (Phase 2, rate-limited smoothstep).
        if self._waist_dof is not None:
            if self._waist_manual:
                self._waist_cur = float(np.clip(self._waist_manual_val, self._waist_lo, self._waist_hi))
                pd_target = np.array(pd_target, dtype=np.float64)
                pd_target[self._waist_dof] = self._waist_cur
            elif self._waist_lean_max != 0.0 and self._cmd_readout is not None and self._cmd_readout[1] is not None:
                h = self._cmd_readout[1]
                sl = np.clip((self._waist_lean_hi - h) / max(self._waist_lean_hi - self._waist_lean_lo, 1e-6), 0.0, 1.0)
                tgt = (sl * sl * (3 - 2 * sl)) * self._waist_lean_max  # smoothstep * max (+ = forward)
                dmax = self._waist_lean_rate * (self.sim_dt * self.sim_decimation)
                self._waist_cur += float(np.clip(tgt - self._waist_cur, -dmax, dmax))
                pd_target = np.array(pd_target, dtype=np.float64)
                pd_target[self._waist_dof] = self._waist_cur

        for _ in range(self.sim_decimation):
            torque = (pd_target - self.dof_pos) * self.stiffness - self.dof_vel * self.damping
            torque = np.clip(torque, -self.torque_limits, self.torque_limits)

            self.data.ctrl = torque

            mujoco.mj_step(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
            self.update(simple=True)
        self.update(simple=False)

    def shutdown(self):
        self.viewer.close()


if __name__ == "__main__":
    from robojudo.config.g1.env.g1_mujuco_env_cfg import G1MujocoEnvCfg

    mujoco_env = MujocoEnv(cfg_env=G1MujocoEnvCfg())
    mujoco_env.viewer._paused = False

    while True:
        # mujoco_env.update()
        mujoco_env.step(np.zeros(mujoco_env.num_dofs))
        time.sleep(0.02)
