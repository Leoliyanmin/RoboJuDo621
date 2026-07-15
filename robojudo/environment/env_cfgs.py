from typing import Literal

from pydantic import model_validator

from robojudo.config import Config
from robojudo.tools.tool_cfgs import DoFConfig, ForwardKinematicCfg, ZedOdometryCfg


class EnvCfg(Config):
    env_type: str  # name of the environment class
    is_sim: bool = False

    urdf: str | None = None
    xml: str
    body_names: list[str] | None = None

    dof: DoFConfig

    forward_kinematic: ForwardKinematicCfg | None = None
    update_with_fk: bool = False
    """Whether to update info from fk"""
    torso_name: str = "torso_link"
    """Name of the torso link, used in fk info extraction"""

    born_place_align: bool = True
    """Whether to align the born place to zero position and heading"""


class MujocoEnvCfg(EnvCfg):
    env_type: str = "MujocoEnv"
    is_sim: bool = True
    # ====== ENV CONFIGURATION ======
    sim_duration: float = 60.0
    sim_dt: float = 0.001
    sim_decimation: int = 20

    visualize_extras: bool = True  # TODO: remove

    random_heading: bool = False

    # [ih] sustained world-frame-down force (N) applied at each wrist body — emulates a
    # box-carry / wrist load (e.g. for the AGILE wrist20-trained policies). 0 = off.
    wrist_load_n: float = 0.0
    wrist_load_bodies: list[str] = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
    # [ih] runtime wrist-load control + on-screen readout (MujocoEnv only):
    #   keyboard: '[' decrease / ']' increase by wrist_load_step, clamped [0, wrist_load_max].
    #   display:  a floating "Wrist load: N N/wrist" label above the robot in the viewer.
    wrist_load_keyboard: bool = False
    wrist_load_step: float = 2.0
    wrist_load_max: float = 30.0
    wrist_load_show: bool = True
    """Show the wrist-load readout label in the viewer when the load feature is active."""

    # [ih] deterministic waist-pitch forward lean as a function of the height command (Phase 2
    # of the deep-squat plan). Overrides the waist_pitch pd_target (a non-policy joint held at
    # default) with smoothstep(hi->lo of height) * max lean (+ = forward). 0 deg = off.
    # Needs the height command piped in via set_cmd_readout (velheight pipeline).
    waist_squat_lean_deg: float = 0.0
    waist_lean_hi: float = 0.65  # height >= hi -> no lean
    waist_lean_lo: float = 0.50  # height <= lo -> full lean
    waist_lean_rate_dps: float = 60.0  # rate limit on the applied lean (deg/s)

    # [ih] MANUAL keyboard waist_pitch override (overrides the auto lean above):
    #   ',' lean back / '.' lean forward by waist_manual_step_deg, clamped to the joint limit.
    #   Current angle shown in the viewer readout. For live hand-testing of waist posture.
    waist_manual_keyboard: bool = False
    waist_manual_step_deg: float = 2.0

    # [ih] Optional sim-only arm choreography for velheight demos. It overrides only the
    # non-policy arm PD targets after the policy output has been expanded to the full G1 DoF.
    arm_motion_mode: Literal["off", "walk_squat_box", "walk_elbow"] = "off"
    arm_squat_height: float = 0.52
    arm_stand_height: float = 0.66
    # [ih] drive the squat box<->hang arm blend from a smoothed MEASURED pelvis height instead
    # of the height command, so box->hang completes only after the body has actually risen
    # (avoids the fast arm-CoM snap on a quick stand-up that tips the robot backward when the
    # arm-obs mask is on). See mujoco_env._apply_arm_motion / docs/清空手臂obs.md.
    arm_squat_use_measured: bool = False
    arm_squat_measured_alpha: float = 0.08
    arm_squat_hold_s: float = 0.0
    arm_squat_return_rate_dps: float = 0.0
    arm_swing_amp: float = 0.10
    arm_swing_hz: float = 1.15
    arm_swing_speed_ref: float = 0.45
    arm_stride_ref: float = 0.28
    arm_stride_filter_alpha: float = 0.35
    arm_elbow_swing_amp: float = 0.12
    arm_wrist_swing_amp: float = 0.08
    arm_walk_spread_amp: float = 0.10
    arm_motion_rate_dps: float = 180.0
    arm_swing_keyboard: bool = False
    arm_swing_scale: float = 1.0
    arm_swing_step: float = 0.1
    arm_swing_min: float = 0.0
    arm_swing_max: float = 2.0
    arm_reach_keyboard: bool = False
    arm_reach_scale: float = 1.0
    arm_reach_step: float = 0.1
    arm_reach_min: float = 0.5
    arm_reach_max: float = 1.6
    arm_mode_keyboard: bool = False
    arm_spread_keyboard: bool = False
    arm_spread_scale: float = 1.0
    arm_spread_step: float = 0.1
    arm_spread_min: float = 0.5
    arm_spread_max: float = 2.2


class RobotEnvCfg(EnvCfg):
    env_type: str = "DummyEnv"
    is_sim: bool = False
    # ====== ENV CONFIGURATION ======
    act: bool = True

    odometry_type: Literal["NONE", "DUMMY", "ZED"] = "NONE"
    zed_cfg: ZedOdometryCfg | None = None
    """ZED odometry config, if odometry_type is "ZED", this must be set"""

    @model_validator(mode="after")
    def check_zed_config(self):
        if self.odometry_type == "ZED" and self.zed_cfg is None:
            raise ValueError("zed_cfg must be set if odometry_type is 'ZED'")
        return self


class UnitreeEnvCfg(RobotEnvCfg):
    """
    Configuration for Unitree Robot environment.
    """

    class UnitreeCfg(Config):
        """Unitree SDK configuration"""

        net_if: str = "eth0"
        """network interface to communicate with the robot"""

        robot: Literal["h1", "g1"]
        msg_type: Literal["hg", "go"]
        control_mode: str = "position"
        hand_type: Literal["Dex-3", "Inspire", "NONE"] = "NONE"

        lowcmd_topic: str = "rt/lowcmd"
        lowstate_topic: str = "rt/lowstate"

        enable_odometry: bool = False
        sport_state_topic: str = "rt/odommodestate"

        control_dt: float = 0.02
        """control command dt"""

        motor_cmd_num_dofs: int | None = None
        """Number of low-level motor command slots, if different from env DoFs."""

        arm_sdk_motor_idx: list[int] | None = None
        """Motor slots controlled through the G1 rt/arm_sdk topic instead of lowcmd."""

        arm_sdk_enable_idx: int = 29
        """HG LowCmd motor_cmd slot used by Unitree to enable arm_sdk control."""

        arm_sdk_topic: str = "rt/arm_sdk"
        """DDS topic for Unitree G1 arm SDK commands."""

    env_type: str = "UnitreeEnv"  # For unitree_sdk2py
    # env_type: str = "UnitreeCppEnv" # For unitree_cpp
    """UnitreeEnv for unitree_sdk2py, UnitreeCppEnv for unitree_cpp, check README for more details"""

    unitree: UnitreeCfg

    odometry_type: Literal["NONE", "DUMMY", "UNITREE", "ZED"] = "DUMMY"  # pyright: ignore[reportIncompatibleVariableOverride]

    joint2motor_idx: list[int] | None = None
    """Mapping from env dof to motor index, None for direct mapping"""
    weak_motor: list[int] = []

    hand_retarget: None = None  # TODO

    @model_validator(mode="after")
    def check_joint2motor_idx(self):
        if self.joint2motor_idx is not None and len(self.joint2motor_idx) != self.dof.num_dofs:
            raise ValueError("joint2motor_idx length must match dof.num_dofs")
        return self
