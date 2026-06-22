# [ih] Config for the AGILE velocity-HEIGHT frozen-hands recurrent (LSTM) policy.
# Pairs with robojudo/policy/agile_velheight_policy.py. See AGILE_VELHEIGHT_SETUP.md.
#
# obs_dof = 29 body joints (AGILE order) — used to slice/reorder env dof_pos/vel for the
#   joint_pos_rel/joint_vel_rel obs (the 24 frozen DFQ hand joints are zero-padded in the
#   policy, so the robot mjcf needs only the 29 body joints).
# action_dof = 12 leg joints with the AGILE per-joint gains/offsets.

from robojudo.config import ASSETS_DIR, Config
from robojudo.policy.policy_cfgs import PolicyCfg
from robojudo.tools.tool_cfgs import DoFConfig


class G1AgileVHBodyDoF(DoFConfig):
    """29 body joints in the AGILE joint_pos_rel order (obs slice/reorder)."""

    joint_names: list[str] = [
        "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
        "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
        "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
        "left_knee_joint", "right_knee_joint",
        "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
        "left_ankle_pitch_joint", "right_ankle_pitch_joint",
        "left_shoulder_roll_joint", "right_shoulder_roll_joint",
        "left_ankle_roll_joint", "right_ankle_roll_joint",
        "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
        "left_elbow_joint", "right_elbow_joint",
        "left_wrist_roll_joint", "right_wrist_roll_joint",
        "left_wrist_pitch_joint", "right_wrist_pitch_joint",
        "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    ]
    default_pos: list[float] | None = [
        -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.0, 0.0, -0.2, -0.2,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]


class G1AgileVHLegsDoF(DoFConfig):
    """12 controlled leg joints (AGILE action order) with AGILE gains/offset."""

    joint_names: list[str] = [
        "left_hip_pitch_joint", "right_hip_pitch_joint",
        "left_hip_roll_joint", "right_hip_roll_joint",
        "left_hip_yaw_joint", "right_hip_yaw_joint",
        "left_knee_joint", "right_knee_joint",
        "left_ankle_pitch_joint", "right_ankle_pitch_joint",
        "left_ankle_roll_joint", "right_ankle_roll_joint",
    ]
    default_pos: list[float] | None = [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, -0.2, -0.2, 0.0, 0.0]
    stiffness: list[float] | None = [
        40.179, 40.179, 99.098, 99.098, 40.179, 40.179, 99.098, 99.098, 28.501, 28.501, 28.501, 28.501,
    ]
    damping: list[float] | None = [
        2.558, 2.558, 6.309, 6.309, 2.558, 2.558, 6.309, 6.309, 1.814, 1.814, 1.814, 1.814,
    ]
    torque_limits: list[float] | None = [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 300.0, 300.0, 40.0, 40.0, 40.0, 40.0]


class G1AgileVHObsScales(Config):
    ang_vel: float = 1.0
    gravity: float = 1.0
    dof_pos: float = 1.0
    dof_vel: float = 0.1


class G1AgileVelHeightPolicyCfg(PolicyCfg):
    policy_type: str = "AgileVelHeightRecurrentPolicy"
    robot: str = "g1"
    policy_name: str = "velheight_frozenhands_wrist20_recurrent"

    @property
    def policy_file(self) -> str:
        return (ASSETS_DIR / f"models/{self.robot}/agile/{self.policy_name}.pt").as_posix()

    obs_dof: DoFConfig = G1AgileVHBodyDoF()
    action_dof: DoFConfig = G1AgileVHLegsDoF()

    freq: int = 50
    action_scale: float = 1.0  # unused — per-joint scale applied in the policy
    action_clip: float | None = None
    action_beta: float = 1.0
    history_length: int = 0  # recurrent (LSTM carries temporal state), no frame stacking

    # AGILE per-joint action scale (12 legs, from the IO descriptor).
    action_scales: list[float] = [
        0.5475, 0.5475, 0.3507, 0.3507, 0.5475, 0.5475, 0.3507, 0.3507, 0.4386, 0.4386, 0.4386, 0.4386,
    ]
    obs_scales: G1AgileVHObsScales = G1AgileVHObsScales()

    # velocity command remap (vx, vy, wz). height is a separate persistent command.
    max_cmd: list[float] = [0.8, 0.5, 1.0]
    commands_map: list[list[float]] = [[-1.0, 0.0, 1.0], [1.0, 0.0, -1.0], [1.0, 0.0, -1.0]]

    # height command (target base height): r = taller, f = squat lower.
    # [ih] match the AGILE training range base_height=(0.4, DEFAULT_PELVIS_HEIGHT=0.72).
    # (was 0.50/0.74 — squat clamped 10 cm short of trained, stand 2 cm past it.)
    height_default: float = 0.72
    height_min: float = 0.40
    height_max: float = 0.72
    height_step: float = 0.01


# [ih] TEACHER variant (privileged, non-recurrent) for the RoboJuDo teacher-in-MuJoCo check.
# Same action space / gains as the recurrent student (distillation matched actions); only the
# policy class (adds base_lin_vel to obs) and the checkpoint differ. Diagnostic, sim-only.
class G1AgileVelHeightTeacherPolicyCfg(G1AgileVelHeightPolicyCfg):
    policy_type: str = "AgileVelHeightTeacherPolicy"
    policy_name: str = "velheight_frozenhands_wrist20_teacher"
