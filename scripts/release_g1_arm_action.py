"""Release the Unitree G1 high-level arm action service."""

from __future__ import annotations

import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send the G1 arm action 'release arm' command.")
    parser.add_argument("--net-if", default="enP8p1s0", help="Unitree network interface.")
    parser.add_argument("--timeout", type=float, default=10.0, help="RPC timeout in seconds.")
    parser.add_argument("--wait", type=float, default=1.0, help="Seconds to wait after the release command.")
    parser.add_argument("--yes", action="store_true", help="Required acknowledgement for real robot arm release.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.yes:
        raise SystemExit(
            "This script sends a real G1 arm-release command. Re-run with --yes after confirming "
            "the robot is supported and the E-stop is reachable."
        )

    ChannelFactoryInitialize(0, args.net_if)

    client = G1ArmActionClient()
    client.SetTimeout(args.timeout)
    client.Init()

    action_id = action_map["release arm"]
    code = client.ExecuteAction(action_id)
    print(f"release arm action_id={action_id}, return_code={code}")
    time.sleep(args.wait)


if __name__ == "__main__":
    main()
