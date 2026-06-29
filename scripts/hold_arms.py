"""[废除 2026-06-29] 本脚本未被最终方案采用，仅作记录保留。

原因：实测确认本部署中 `rt/arm_sdk` 不可用——`run_pipeline` 启动时 MotionSwitcher 释放了
订阅 arm_sdk 的高层服务（matched=False）。最终手臂改为与腿同走 `rt/lowcmd`（在
g1_agile_velocity_23dof_real 里 arm_sdk_motor_idx=None，姿态取自 G1_23RaisedArmsDoF.default_pos）。
详见 docs/23dof部署.md。以下为当时的独立 arm_sdk 保持脚本，保留以备其它(高层)场景参考。

[ih] Hold the G1 23-DoF arms at the trained default pose via the official rt/arm_sdk.

WHY a separate process: the main deployment (UnitreeCppEnv) drives the legs through the
C++ unitree_cpp cyclonedds. Initialising the Python cyclonedds arm_sdk publisher IN THE
SAME process now throws cyclonedds BAD_PARAMETER (C++ and Python share /usr/local/lib/
libddsc.so -> one process-global DDS type registry -> LowCmd type registered twice with
different descriptors). Running arm_sdk in a SEPARATE process sidesteps that: this process
has only the Python cyclonedds, no C++ DDS, so no in-process type clash.

USAGE (two terminals):
  1) terminal A:  python scripts/run_pipeline.py -c g1_ih_velocity_23dof_real_keyboard
     wait until it prints "prepare done — holding default pose" (MotionSwitcher has released
     the high-level service, so the robot's rt/arm_sdk subscriber is live).
  2) terminal B:  python scripts/hold_arms.py --yes
     ramps the arm_sdk weight 0->1 while moving the arms to the default pose, then holds.
     Ctrl-C ramps the weight back to 0 (arms go limp) and exits.

The arm_sdk LowCmd only sets the 10 arm slots + the enable weight at motor_cmd[29]; the leg
slots are left at kp=0 so this NEVER fights the leg lowcmd from the main pipeline.
"""

from __future__ import annotations

import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


ARM_SDK_ENABLE_SLOT = 29

# motor slot -> (trained default pose [rad], kp, kd). Matches G1_23RaisedArmsDoF default_pos
# and the env arm gains (shoulders/elbow kp=40, wrist_roll kp=20, kd=2).
ARM_TARGETS: dict[int, tuple[float, float, float]] = {
    15: (0.2, 40.0, 2.0),  # left_shoulder_pitch
    16: (0.0, 40.0, 2.0),  # left_shoulder_roll
    17: (0.0, 40.0, 2.0),  # left_shoulder_yaw
    18: (0.6, 40.0, 2.0),  # left_elbow
    19: (0.0, 20.0, 2.0),  # left_wrist_roll
    22: (0.2, 40.0, 2.0),  # right_shoulder_pitch
    23: (0.0, 40.0, 2.0),  # right_shoulder_roll
    24: (0.0, 40.0, 2.0),  # right_shoulder_yaw
    25: (0.6, 40.0, 2.0),  # right_elbow
    26: (0.0, 20.0, 2.0),  # right_wrist_roll
}
ARM_MOTOR_SLOTS = list(ARM_TARGETS)


class LowStateCache:
    def __init__(self) -> None:
        self.low_state: LowState_ | None = None

    def handler(self, msg: LowState_) -> None:
        self.low_state = msg

    def wait(self, timeout: float) -> LowState_ | None:
        deadline = time.time() + timeout
        while self.low_state is None and time.time() < deadline:
            time.sleep(0.02)
        return self.low_state


def smoothstep(alpha: float) -> float:
    alpha = min(max(alpha, 0.0), 1.0)
    return alpha * alpha * (3.0 - 2.0 * alpha)


def write_cmd(pub, crc, q_by_slot: dict[int, float], weight: float, timeout: float | None = None) -> bool:
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.motor_cmd[ARM_SDK_ENABLE_SLOT].q = weight
    for slot, (_, kp, kd) in ARM_TARGETS.items():
        mc = cmd.motor_cmd[slot]
        mc.tau = 0.0
        mc.q = q_by_slot[slot]
        mc.dq = 0.0
        mc.kp = kp
        mc.kd = kd
    cmd.crc = crc.Crc(cmd)
    return bool(pub.Write(cmd, timeout=timeout))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Hold the G1 arms at the default pose via rt/arm_sdk.")
    p.add_argument("--net-if", default="enP8p1s0", help="Unitree network interface.")
    p.add_argument("--ramp", type=float, default=2.0, help="Seconds to ramp weight 0->1 and move to pose.")
    p.add_argument("--dt", type=float, default=0.02, help="Command period [s] (50 Hz).")
    p.add_argument("--release", type=float, default=1.0, help="Seconds to ramp weight back to 0 on exit.")
    p.add_argument("--match-timeout", type=float, default=1.0, help="Seconds to wait for an arm_sdk subscriber.")
    p.add_argument("--yes", action="store_true", help="Required acknowledgement for real robot arm motion.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if not args.yes:
        raise SystemExit(
            "This moves the real G1 arms via rt/arm_sdk. Re-run with --yes after run_pipeline is up "
            "(arms ready) and the E-stop is reachable."
        )

    ChannelFactoryInitialize(0, args.net_if)
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()

    state_cache = LowStateCache()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_cache.handler, 10)
    low_state = state_cache.wait(timeout=3.0)
    if low_state is None:
        raise SystemExit("No rt/lowstate received — is run_pipeline running and the net-if correct?")

    start_by_slot = {slot: float(low_state.motor_state[slot].q) for slot in ARM_MOTOR_SLOTS}
    target_by_slot = {slot: ARM_TARGETS[slot][0] for slot in ARM_MOTOR_SLOTS}

    crc = CRC()
    matched = write_cmd(pub, crc, start_by_slot, weight=0.0, timeout=args.match_timeout)
    print(f"arm_sdk matched subscriber = {matched}")
    if not matched:
        print("WARNING: no rt/arm_sdk subscriber yet. Make sure run_pipeline has finished 'prepare' "
              "(MotionSwitcher released) before starting this. Continuing anyway...")

    # Ramp weight 0->1 while moving arms start->target (smoothstep), so nothing snaps.
    steps = max(1, int(round(args.ramp / args.dt)))
    print(f"Ramping arms to default pose over {args.ramp:.1f}s ...")
    for step in range(steps):
        a = smoothstep((step + 1) / steps)
        q_by_slot = {s: (1.0 - a) * start_by_slot[s] + a * target_by_slot[s] for s in ARM_MOTOR_SLOTS}
        write_cmd(pub, crc, q_by_slot, weight=a)
        time.sleep(args.dt)

    print("Holding arms at default pose. Ctrl-C to release.")
    try:
        while True:
            write_cmd(pub, crc, target_by_slot, weight=1.0)
            time.sleep(args.dt)
    except KeyboardInterrupt:
        print("\nReleasing arm_sdk weight ...")
        release_steps = max(1, int(round(args.release / args.dt)))
        for step in range(release_steps):
            weight = 1.0 - (step + 1) / release_steps
            write_cmd(pub, crc, target_by_slot, weight=weight)
            time.sleep(args.dt)
        print("Released. Arms are limp again.")


if __name__ == "__main__":
    main()
