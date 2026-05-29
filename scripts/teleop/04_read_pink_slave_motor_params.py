"""Read selected Damiao motor parameters for the pink slave arm.

This diagnostic script is read-only. It does not enable motors, does not switch
control mode, and does not send motion commands.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import open_arm_connection


CONTROL_MODE_NAMES = {
    1: "MIT",
    2: "POS_VEL",
    3: "VEL",
    4: "Torque_Pos",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read pink slave Damiao parameters.")
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--joints", nargs="*", default=["joint1", "joint2"])
    return parser


def main() -> None:
    args = build_parser().parse_args()
    connection = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)
    try:
        dm_variable = _dm_variable()
        rid_items = [
            ("CTRL_MODE", dm_variable.CTRL_MODE),
            ("MST_ID", dm_variable.MST_ID),
            ("ESC_ID", dm_variable.ESC_ID),
            ("MAX_SPD", dm_variable.MAX_SPD),
            ("PMAX", dm_variable.PMAX),
            ("VMAX", dm_variable.VMAX),
            ("TMAX", dm_variable.TMAX),
            ("hw_ver", dm_variable.hw_ver),
            ("sw_ver", dm_variable.sw_ver),
        ]
        selected = set(args.joints)
        print("Pink slave Damiao parameter read / 粉色从臂达妙参数只读诊断")
        print(f"Serial: {args.slave_port}, baudrate: {args.slave_baudrate}")
        print("Safety: read-only; no enable; no mode switch; no motion")
        print()
        for item in connection.items:
            if item.config.name not in selected:
                continue
            print(
                f"{item.config.name}: CAN ID=0x{item.config.can_id:02X}, "
                f"Master ID=0x{item.config.master_id:02X}"
            )
            for label, rid in rid_items:
                value = connection.controller.read_motor_param(item.motor, rid)
                if label == "CTRL_MODE" and value is not None:
                    mode_int = int(round(float(value)))
                    print(f"  {label:<9}: {value} ({CONTROL_MODE_NAMES.get(mode_int, 'unknown')})")
                else:
                    print(f"  {label:<9}: {value}")
            print()
    finally:
        connection.close()


def _dm_variable() -> Any:
    dm_can = sys.modules.get("DM_CAN")
    dm_variable = getattr(dm_can, "DM_variable", None) if dm_can is not None else None
    if dm_variable is None:
        raise RuntimeError("DM_CAN.DM_variable is not available")
    return dm_variable


if __name__ == "__main__":
    main()
