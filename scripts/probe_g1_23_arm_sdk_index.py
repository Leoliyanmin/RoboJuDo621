"""Probe G1 23-DoF arm indices through the official rt/arm_sdk topic."""

from __future__ import annotations

import argparse
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


ARM_INDEX_TO_MOTOR = {
    13: (15, "left_shoulder_pitch_joint"),
    14: (16, "left_shoulder_roll_joint"),
    15: (17, "left_shoulder_yaw_joint"),
    16: (18, "left_elbow_joint"),
    17: (19, "left_wrist_roll_joint"),
    18: (22, "right_shoulder_pitch_joint"),
    19: (23, "right_shoulder_roll_joint"),
    20: (24, "right_shoulder_yaw_joint"),
    21: (25, "right_elbow_joint"),
    22: (26, "right_wrist_roll_joint"),
}
ARM_MOTOR_SLOTS = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]
ARM_SDK_ENABLE_SLOT = 29


class LowStateCache:
    def __init__(self) -> None:
        self.low_state: LowState_ | None = None

    def handler(self, msg: LowState_) -> None:
        self.low_state = msg

    def wait(self, timeout: float) -> LowState_:
        deadline = time.time() + timeout
        while self.low_state is None and time.time() < deadline:
            time.sleep(0.02)
        if self.low_state is None:
            raise TimeoutError("Timed out waiting for rt/lowstate.")
        return self.low_state


def smoothstep(alpha: float) -> float:
    alpha = min(max(alpha, 0.0), 1.0)
    return alpha * alpha * (3.0 - 2.0 * alpha)


def read_q(low_state: LowState_, motor_slot: int) -> float:
    return float(low_state.motor_state[motor_slot].q)


def fill_arm_command(cmd: LowCmd_, q_by_slot: dict[int, float], kp: float, kd: float, weight: float) -> None:
    cmd.motor_cmd[ARM_SDK_ENABLE_SLOT].q = weight
    for slot in ARM_MOTOR_SLOTS:
        motor_cmd = cmd.motor_cmd[slot]
        motor_cmd.tau = 0.0
        motor_cmd.q = q_by_slot[slot]
        motor_cmd.dq = 0.0
        motor_cmd.kp = kp
        motor_cmd.kd = kd


def write_cmd(
    pub: ChannelPublisher,
    crc: CRC,
    q_by_slot: dict[int, float],
    kp: float,
    kd: float,
    weight: float,
    timeout: float | None = None,
) -> bool:
    cmd = unitree_hg_msg_dds__LowCmd_()
    fill_arm_command(cmd, q_by_slot=q_by_slot, kp=kp, kd=kd, weight=weight)
    cmd.crc = crc.Crc(cmd)
    return bool(pub.Write(cmd, timeout=timeout))


def run_for(
    pub: ChannelPublisher,
    crc: CRC,
    q_by_slot: dict[int, float],
    kp: float,
    kd: float,
    weight: float,
    duration: float,
    dt: float,
) -> None:
    steps = max(1, int(round(duration / dt)))
    for _ in range(steps):
        write_cmd(pub, crc, q_by_slot, kp, kd, weight)
        time.sleep(dt)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe one G1 arm motor through rt/arm_sdk.")
    parser.add_argument("--index", type=int, choices=sorted(ARM_INDEX_TO_MOTOR), required=True)
    parser.add_argument("--delta", type=float, default=0.06, help="Probe offset in radians.")
    parser.add_argument("--ramp", type=float, default=1.0, help="Seconds to ramp out/back.")
    parser.add_argument("--hold", type=float, default=1.0, help="Seconds to hold the offset.")
    parser.add_argument("--release", type=float, default=1.0, help="Seconds to ramp arm_sdk weight to zero.")
    parser.add_argument("--dt", type=float, default=0.02, help="Command period in seconds.")
    parser.add_argument("--kp", type=float, default=60.0, help="Arm position gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Arm damping gain.")
    parser.add_argument("--match-timeout", type=float, default=1.0, help="Seconds to wait for an arm_sdk subscriber.")
    parser.add_argument("--net-if", default="enP8p1s0", help="Unitree network interface.")
    parser.add_argument("--yes", action="store_true", help="Required acknowledgement for real robot motion.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.yes:
        raise SystemExit(
            "This script sends real arm_sdk commands. Re-run with --yes after supporting the robot "
            "and confirming the E-stop is reachable."
        )

    motor_slot, name = ARM_INDEX_TO_MOTOR[args.index]

    ChannelFactoryInitialize(0, args.net_if)
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()

    state_cache = LowStateCache()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_cache.handler, 10)
    low_state = state_cache.wait(timeout=3.0)

    base_by_slot = {slot: read_q(low_state, slot) for slot in ARM_MOTOR_SLOTS}
    target_by_slot = dict(base_by_slot)
    target_by_slot[motor_slot] += args.delta

    print(f"Probing env index {args.index}: {name}")
    print(f"arm_sdk motor slot: {motor_slot}")
    print(f"start={base_by_slot[motor_slot]:+.4f} rad, target={target_by_slot[motor_slot]:+.4f} rad")

    crc = CRC()
    matched = write_cmd(pub, crc, base_by_slot, args.kp, args.kd, weight=1.0, timeout=args.match_timeout)
    print(f"arm_sdk_write_matched={matched}")
    if not matched:
        print("No matched rt/arm_sdk subscriber was detected before the first command.")

    steps = max(1, int(round(args.ramp / args.dt)))
    for step in range(steps):
        alpha = smoothstep((step + 1) / steps)
        q_by_slot = dict(base_by_slot)
        q_by_slot[motor_slot] = (1.0 - alpha) * base_by_slot[motor_slot] + alpha * target_by_slot[motor_slot]
        write_cmd(pub, crc, q_by_slot, args.kp, args.kd, weight=1.0)
        time.sleep(args.dt)

    run_for(pub, crc, target_by_slot, args.kp, args.kd, weight=1.0, duration=args.hold, dt=args.dt)
    low_state = state_cache.wait(timeout=0.1)
    held = read_q(low_state, motor_slot)
    print(f"measured_after_hold={held:+.4f} rad, measured_delta={held - base_by_slot[motor_slot]:+.4f} rad")

    for step in range(steps):
        alpha = smoothstep((step + 1) / steps)
        q_by_slot = dict(base_by_slot)
        q_by_slot[motor_slot] = (1.0 - alpha) * target_by_slot[motor_slot] + alpha * base_by_slot[motor_slot]
        write_cmd(pub, crc, q_by_slot, args.kp, args.kd, weight=1.0)
        time.sleep(args.dt)

    run_for(pub, crc, base_by_slot, args.kp, args.kd, weight=1.0, duration=0.5, dt=args.dt)
    low_state = state_cache.wait(timeout=0.1)
    returned = read_q(low_state, motor_slot)
    print(f"measured_after_return={returned:+.4f} rad, return_error={returned - base_by_slot[motor_slot]:+.4f} rad")

    release_steps = max(1, int(round(args.release / args.dt)))
    for step in range(release_steps):
        weight = 1.0 - (step + 1) / release_steps
        write_cmd(pub, crc, base_by_slot, args.kp, args.kd, weight=weight)
        time.sleep(args.dt)
    print("Released arm_sdk weight.")


if __name__ == "__main__":
    main()
