"""Print the EFFECTIVE Kp/Kd, joint order and joint->motor mapping that the real
robot actually receives for a given pipeline config. Pure config inspection — does
NOT connect to the robot.

Usage:
    python scripts/inspect_real_gains.py -c g1_ih_velocity_23dof_real_keyboard
"""

import argparse

from robojudo.config.config_manager import ConfigManager
from robojudo.tools.dof import merge_dof_cfgs


def main():
    parser = argparse.ArgumentParser()
    # [ih] default to the real deployment config so bare `python scripts/inspect_real_gains.py`
    # just works; pass -c <name> to inspect any other pipeline config.
    parser.add_argument("-c", "--config", default="g1_ih_velocity_23dof_real_keyboard")
    args = parser.parse_args()

    cfg = ConfigManager(config_name=args.config).get_cfg()

    env_dof = cfg.env.dof
    action_dof = cfg.policy.action_dof  # policy override (training / IO-descriptor gains)

    # This mirrors what the pipeline does: env.update_dof_cfg(override_cfg=policy.action_dof)
    merged = merge_dof_cfgs(env_dof, action_dof)

    j2m = getattr(cfg.env, "joint2motor_idx", None)
    arm_sdk = getattr(getattr(cfg.env, "unitree", None), "arm_sdk_motor_idx", None) or []

    print(f"\nconfig: {args.config}")
    print(f"env DoF cfg: {type(env_dof).__name__}   policy action DoF: {type(action_dof).__name__}")
    print(f"joint2motor_idx: {j2m}")
    print(f"arm_sdk_motor_idx (kp/kd forced 0 in lowcmd, driven by arm_sdk): {arm_sdk}\n")

    names = merged.joint_names
    kp = merged.stiffness
    kd = merged.damping
    dpos = merged.default_pos

    hdr = f"{'env_idx':>7} {'motor':>5} {'joint':<28} {'Kp':>7} {'Kd':>6} {'default_pos':>11} {'src':>9}"
    print(hdr)
    print("-" * len(hdr))
    for i, name in enumerate(names):
        motor = j2m[i] if j2m is not None and i < len(j2m) else i
        # was this joint's gain overridden by the policy (training) cfg?
        src = "TRAIN" if name in action_dof.joint_names else "env"
        if motor in arm_sdk:
            src = "arm_sdk(0)"
        print(
            f"{i:>7} {motor:>5} {name:<28} {kp[i]:>7.1f} {kd[i]:>6.2f} {dpos[i]:>11.3f} {src:>9}"
        )
    print()


if __name__ == "__main__":
    main()