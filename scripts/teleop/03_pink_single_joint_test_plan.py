"""Print recommended single-joint teleoperation test commands.

This script does not open serial ports and does not move the robot. It only
prints the next commands to run manually, one joint at a time.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.teleop_gain_config import load_joint_gains


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print pink single-joint follow test commands.")
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--include-tested", action="store_true", help="also print joint1 and joint6")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    gains = load_joint_gains("pink_slave")
    joints = [f"joint{i}" for i in range(1, 8)]
    if not args.include_tested:
        joints = [joint for joint in joints if joint not in {"joint1", "joint6"}]

    print("Pink single-joint teleoperation test plan")
    print("Run only one command at a time. Keep physical E-stop ready.")
    print("Stop each test with Ctrl+C before moving to the next joint.")
    print()
    for joint in joints:
        gain = gains[joint]
        print(
            f"# {joint}: mode={gain.control_mode}, kp={gain.kp}, kd={gain.kd}, tau={gain.tau}, "
            f"posvel_velocity={gain.posvel_velocity}, switch_mode={gain.switch_mode}, "
            f"enable_old_mode={gain.enable_old_mode}, slave_read_every={gain.slave_read_every}, "
            f"limit_relax_rad={gain.limit_relax_rad}  ({gain.note})"
        )
        command = (
            "python scripts\\teleop\\02_pink_single_joint_follow_with_limit.py "
            f"--joint {joint} "
            f"--master-port {args.master_port} "
            f"--slave-port {args.slave_port} "
            "--print-every 1 "
            "--enable-motion YES"
        )
        print(command)
        print()


if __name__ == "__main__":
    main()
