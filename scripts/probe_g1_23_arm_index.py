"""Probe native G1 23-DoF arm motor indices with tiny position offsets."""

from __future__ import annotations

import argparse
import time

import numpy as np

from robojudo.config.g1.env.g1_real_env_cfg import G1UnitreeCfg, G1_23RealEnvCfg
from robojudo.environment.unitree_cpp_env import UnitreeCppEnv


ARM_INDEX_NAMES = {
    13: "left_shoulder_pitch_joint",
    14: "left_shoulder_roll_joint",
    15: "left_shoulder_yaw_joint",
    16: "left_elbow_joint",
    17: "left_wrist_roll_joint",
    18: "right_shoulder_pitch_joint",
    19: "right_shoulder_roll_joint",
    20: "right_shoulder_yaw_joint",
    21: "right_elbow_joint",
    22: "right_wrist_roll_joint",
}


def smoothstep(alpha: float) -> float:
    alpha = min(max(alpha, 0.0), 1.0)
    return alpha * alpha * (3.0 - 2.0 * alpha)


def send_for(env: UnitreeCppEnv, target: np.ndarray, duration: float, dt: float) -> None:
    steps = max(1, int(round(duration / dt)))
    for _ in range(steps):
        env.step(target)
        time.sleep(dt)


def ramp(env: UnitreeCppEnv, start: np.ndarray, target: np.ndarray, duration: float, dt: float) -> None:
    steps = max(1, int(round(duration / dt)))
    for step in range(steps):
        alpha = smoothstep((step + 1) / steps)
        env.step((1.0 - alpha) * start + alpha * target)
        time.sleep(dt)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move one native G1 23-DoF arm command index by a very small delta, "
            "then return to the measured starting pose."
        )
    )
    parser.add_argument("--index", type=int, choices=sorted(ARM_INDEX_NAMES), required=True)
    parser.add_argument("--delta", type=float, default=0.04, help="Probe offset in radians.")
    parser.add_argument("--ramp", type=float, default=1.0, help="Seconds to ramp out/back.")
    parser.add_argument("--hold", type=float, default=1.0, help="Seconds to hold the offset.")
    parser.add_argument("--dt", type=float, default=0.02, help="Command period in seconds.")
    parser.add_argument("--net-if", default="enP8p1s0", help="Unitree network interface.")
    parser.add_argument("--yes", action="store_true", help="Required acknowledgement for real robot motion.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.yes:
        raise SystemExit(
            "This script sends real robot commands. Re-run with --yes after supporting the robot "
            "and confirming the E-stop is reachable."
        )

    cfg = G1_23RealEnvCfg(
        env_type="UnitreeCppEnv",
        unitree=G1UnitreeCfg(net_if=args.net_if),
        forward_kinematic=None,
        update_with_fk=False,
    )
    env = UnitreeCppEnv(cfg_env=cfg)

    try:
        env.update()
        base = env.dof_pos
        target = base.copy()
        target[args.index] += args.delta

        name = ARM_INDEX_NAMES[args.index]
        motor_index = cfg.joint2motor_idx[args.index] if cfg.joint2motor_idx is not None else args.index
        print(f"Probing index {args.index}: {name}")
        print(f"lowcmd motor slot: {motor_index}")
        print(f"start={base[args.index]:+.4f} rad, target={target[args.index]:+.4f} rad")
        print("Watch the robot and note which physical joint moves.")

        ramp(env, base, target, args.ramp, args.dt)
        send_for(env, target, args.hold, args.dt)
        env.update()
        held = env.dof_pos[args.index]
        print(f"measured_after_hold={held:+.4f} rad, measured_delta={held - base[args.index]:+.4f} rad")
        ramp(env, target, base, args.ramp, args.dt)
        send_for(env, base, 0.5, args.dt)
        env.update()
        returned = env.dof_pos[args.index]
        print(f"measured_after_return={returned:+.4f} rad, return_error={returned - base[args.index]:+.4f} rad")

        zero_stiffness = [0.0] * env.num_dofs
        soft_damping = [2.0] * env.num_dofs
        env.set_gains(zero_stiffness, soft_damping)
        env.step(base)
        print("Returned to start pose for this probe.")
    finally:
        env.enabled = False


if __name__ == "__main__":
    main()
