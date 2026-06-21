# [ih] AGILE 29-DOF velocity-history policy deployed in RoboJuDo (sim2sim).
#
# Maps the AGILE ``Velocity-G1-History-v0`` (29-DOF G1) policy onto RoboJuDo's
# existing ``UnitreeWoGaitPolicy``. That policy's observation assembly is a
# byte-for-byte match to the AGILE velocity-history layout:
#
#   obs_current = [ ang_vel*0.2, gravity*1.0, cmd*1.0, (dof_pos-default)*1.0,
#                   dof_vel*0.05, last_action ]
#   history_buf (len 5);  obs = concat over per-term history (zip(*buf))
#
# which equals AGILE's flattened per-term 5-frame history. So NO new policy
# class is needed -- only this config.
#
# IMPORTANT: AGILE's 29-DOF velocity task only CONTROLS / OBSERVES 14 joints
# (the 12 legs + waist_roll + waist_pitch -- NOT waist_yaw). The other 15 joints
# (arms / waist_yaw / wrists) are held at their default pose by the env. RoboJuDo's
# ``DoFAdapter`` slices the env's 29 DOF down to these 14 by joint name for the obs,
# and expands the 14 actions back to 29 (uncontrolled <- env default) for PD. So
# the obs/action DoF below lists exactly those 14 joints, in the AGILE obs order.
#
# Ground truth: agile/data/policy/velocity_g1/unitree_g1_velocity_history.yaml
# (IO descriptor). JIT verified: obs (1,255) -> action (1,14).

from robojudo.policy.policy_cfgs import UnitreeWoGaitPolicyCfg
from robojudo.tools.tool_cfgs import DoFConfig


class G1AgileVelocity29DoF(DoFConfig):
    """The 14 controlled/observed joints of AGILE Velocity-G1-History-v0 (29-DOF).

    Order MUST match the IO descriptor's action/joint_pos_rel joint_names. Gains and
    default_pos come from the AGILE IO descriptor (the 14-joint subset); torque_limits
    are the physical G1 motor limits (from RoboJuDo's G1_29DoF).
    """

    joint_names: list[str] = [
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "left_hip_roll_joint",
        "right_hip_roll_joint",
        "waist_roll_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
        "waist_pitch_joint",
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
    ]

    # AGILE default_joint_pos (== action offset) for these 14 joints.
    default_pos: list[float] | None = [
        -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, -0.2, -0.2, 0.0, 0.0,
    ]

    # AGILE default_joint_stiffness (subset, in the order above).
    stiffness: list[float] | None = [
        100.0, 100.0, 100.0, 100.0, 300.0, 100.0, 100.0, 300.0, 200.0, 200.0, 20.0, 20.0, 20.0, 20.0,
    ]

    # AGILE default_joint_damping (subset, in the order above).
    damping: list[float] | None = [
        2.5, 2.5, 2.5, 2.5, 5.0, 2.5, 2.5, 5.0, 5.0, 5.0, 0.2, 0.2, 0.1, 0.1,
    ]

    # Physical G1 motor torque limits (from RoboJuDo G1_29DoF), subset to these 14.
    torque_limits: list[float] | None = [
        200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 300.0, 300.0, 40.0, 40.0, 40.0, 40.0,
    ]


class G1AgileVelocityPolicyCfg(UnitreeWoGaitPolicyCfg):
    """AGILE 29-DOF velocity-history policy. Reuses UnitreeWoGaitPolicy verbatim.

    Inherited obs_scales already match AGILE (ang_vel 0.2, gravity 1.0, dof_pos 1.0,
    dof_vel 0.05, command 1.0). Only the action post-processing and the DoF set differ.
    """

    robot: str = "g1"
    # -> assets/models/g1/unitree/velocity_history_29dof.pt
    policy_name: str = "velocity_history_29dof"

    obs_dof: DoFConfig = G1AgileVelocity29DoF()
    action_dof: DoFConfig = obs_dof

    # AGILE: action = clip(raw, +-10) * 0.5 + default_pos; no action smoothing.
    action_scale: float = 0.5
    action_clip: float | None = 10.0
    action_beta: float = 1.0

    history_length: int = 5
    # Order + dims MUST mirror UnitreeWoGaitPolicy.get_observation's obs_current list.
    history_obs_dims: dict[str, int] = {
        "ang_vel": 3,
        "gravity": 3,
        "commands": 3,
        "dof_pos": obs_dof.num_dofs,
        "dof_vel": obs_dof.num_dofs,
        "actions": action_dof.num_dofs,
    }
