# [ih] AGILE 23-DOF velocity-history policy deployed in RoboJuDo (sim2sim).
#
# Sibling of g1_agile_velocity_cfg.py (29-DOF). Same velocity-history family, so it
# reuses RoboJuDo's UnitreeWoGaitPolicy verbatim (config-only) — that policy's obs
# assembly is byte-for-byte the AGILE velocity-history layout:
#
#   obs_current = [ ang_vel*0.2, gravity*1.0, cmd*1.0, (dof_pos-default)*1.0,
#                   dof_vel*0.05, last_action ];  history_buf len 5, per-term concat.
#
# DIFFERENCE vs 29-DOF: the AGILE 23-DOF task CONTROLS/OBSERVES 13 joints =
# the 12 legs + waist_yaw (the 29-DOF task used 14 = legs + waist_roll + waist_pitch).
# The other 10 joints (arms / wrists) are held at the env default; RoboJuDo's DoFAdapter
# slices the env's 23 DOF down to these 13 by name for obs and expands the 13 actions
# back to 23 for PD.
#
# Ground truth (exact values below): the run's exported IO descriptor
#   logs/rsl_rl/velocity_g1_lower_23dof_wrist20/2026-06-21_07-33-47_.../exported/
#   velocity_g1_history_23dof_wrist20_v0_IO_descriptors.yaml
# (re-export: eval.py --task Velocity-G1-History-23DOF-Wrist20-v0 --checkpoint model_3298.pt --export_io)
# JIT obs (1,240) -> action (1,13).

from robojudo.policy.policy_cfgs import UnitreeWoGaitPolicyCfg
from robojudo.tools.tool_cfgs import DoFConfig


class G1AgileVelocity23DoF(DoFConfig):
    """The 13 controlled/observed joints of AGILE Velocity-G1-History-23DOF (legs + waist_yaw).

    Order MUST match the IO descriptor's action / joint_pos_rel joint_names (IsaacLab
    articulation order — waist_yaw sits at index 2). default_pos, stiffness, damping are
    the IO-descriptor subset; torque_limits are the physical G1 motor limits.
    """

    joint_names: list[str] = [
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "waist_yaw_joint",
        "left_hip_roll_joint",
        "right_hip_roll_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
    ]

    # AGILE default_joint_pos (== action offset) for these 13 joints.
    default_pos: list[float] | None = [
        -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, -0.2, -0.2, 0.0, 0.0,
    ]

    # AGILE default_joint_stiffness (IO-descriptor subset, in the order above).
    stiffness: list[float] | None = [
        100.0, 100.0, 300.0, 100.0, 100.0, 100.0, 100.0, 200.0, 200.0, 20.0, 20.0, 20.0, 20.0,
    ]

    # AGILE default_joint_damping (IO-descriptor subset, in the order above).
    damping: list[float] | None = [
        2.5, 2.5, 5.0, 2.5, 2.5, 2.5, 2.5, 5.0, 5.0, 0.2, 0.2, 0.1, 0.1,
    ]

    # Physical G1 motor torque limits (RoboJuDo G1_29DoF): hip/waist 200, knee 300, ankle 40.
    torque_limits: list[float] | None = [
        200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 300.0, 300.0, 40.0, 40.0, 40.0, 40.0,
    ]


class G1AgileVelocity23DOFPolicyCfg(UnitreeWoGaitPolicyCfg):
    """AGILE 23-DOF velocity-history policy. Reuses UnitreeWoGaitPolicy (config-only).

    Inherited obs_scales already match AGILE (ang_vel 0.2, gravity 1.0, dof_pos 1.0,
    dof_vel 0.05, command 1.0).
    """

    robot: str = "g1"
    # -> assets/models/g1/unitree/velocity_history_23dof_wrist20.pt
    policy_name: str = "velocity_history_23dof"

    obs_dof: DoFConfig = G1AgileVelocity23DoF()
    action_dof: DoFConfig = obs_dof

    # AGILE: action = clip(raw, +-10) * 0.5 + default_pos; no smoothing.
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
