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
                self._waist_lo, self._waist_hi = float(self.model.jnt_range[jid][0]), float(self.model.jnt_range[jid][1])
        # resolve bodies if there is (or could be, via keyboard) a load to apply
        if self._wrist_load_n > 0.0 or self._wrist_load_kb:
            for bn in getattr(cfg_env, "wrist_load_bodies", []):
                bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, bn)
                if bid >= 0:
                    self._wrist_load_body_ids.append(bid)
        # optional keyboard listener for '[' / ']' load + ',' / '.' waist (own pynput thread;
        # coexists with the controller's KeyboardCtrl listener — pynput allows multiple).
        self._kb_queue = None
        if self._wrist_load_kb or self._waist_manual:
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

    def set_cmd_readout(self, commands, max_cmd=None):
        """[ih] store the policy's command (set target) for the viewer readout.

        commands len 4 -> [vx, vy, wz, height] already in real units (velheight policy);
        len 3 -> normalized [vx, vy, wz] (UnitreeWoGaitPolicy) -> scale by max_cmd to m/s.
        """
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

    def step(self, pd_target, hand_pose=None):
        assert len(pd_target) == self.num_dofs, "pd_target len should be num_dofs of env"

        if hand_pose is not None:
            logger.info("Hand pose-->", hand_pose)

        # [ih] poll '[' / ']' to adjust the wrist load before rendering this frame
        self._poll_env_keys()
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
                _readout(1.28, [0.2, 0.8, 1.0, 0.9],
                         f"vel cmd  vx={float(vel[0]):+.2f} vy={float(vel[1]):+.2f} wz={float(vel[2]):+.2f}", 97)
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
