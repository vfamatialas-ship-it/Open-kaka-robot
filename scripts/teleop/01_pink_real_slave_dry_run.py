"""Dry-run preview: pink master maps to real pink slave targets.

Safety:
- Reads pink master and pink slave.
- Computes slave target positions from configs/teleop_mapping.yaml.
- Does not enable motors.
- Does not send any motion command.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import ArmStatusSnapshot, open_arm_connection
from robot_core.services.joint_limit_service import load_limits_text, parse_limits_text
from robot_core.services.teleop_mapping_config import load_teleop_mapping
from robot_core.services.teleop_service import compute_mapping_preview
from robot_core.services.teleop_service import target_status_is_commandable
from robot_core.services.teleop_service import update_continuous_position_map


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink master -> pink slave dry-run preview.")
    parser.add_argument("--master-port", default="COM8", help="pink master Feetech serial port")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7", help="pink slave Damiao USB2CAN serial port")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument("--no-clear", action="store_true", help="do not clear console between cycles")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    mapping = load_teleop_mapping("pink_master", "pink_slave")
    joint_settings = mapping.to_joint_settings()
    limits = parse_limits_text(load_limits_text("pink_slave"))

    print("Pink master -> real pink slave Dry Run")
    print("Safety: read-only preview; no enable; no motion command.")
    print(f"Master: pink_master @ {args.master_port} {args.master_baudrate}")
    print(f"Slave:  pink_slave  @ {args.slave_port} {args.slave_baudrate}")
    print(f"Mapping: configs/teleop_mapping.yaml, alpha={mapping.runtime.alpha}, max_step={mapping.runtime.max_step_rad}")
    print()

    master = open_arm_connection("pink_master", port=args.master_port, baudrate=args.master_baudrate)
    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)

    master_last_raw: dict[str, float] = {}
    master_continuous: dict[str, float] = {}
    previous_targets: dict[str, float] = {}

    try:
        master_anchor_snapshot = master.read_snapshot()
        slave_anchor_snapshot = slave.read_snapshot(damiao_response_wait=mapping.runtime.damiao_response_wait)
        update_continuous_position_map(
            _position_map(master_anchor_snapshot),
            master_last_raw,
            master_continuous,
        )
        master_anchor = dict(master_continuous)
        slave_anchor = _required_anchor(slave_anchor_snapshot)

        cycle = 0
        period = 1.0 / max(1.0, args.hz)
        while args.cycles <= 0 or cycle < args.cycles:
            cycle += 1
            started = time.perf_counter()
            master_snapshot = master.read_snapshot()
            slave_snapshot = slave.read_snapshot(damiao_response_wait=mapping.runtime.damiao_response_wait)
            update_continuous_position_map(
                _position_map(master_snapshot),
                master_last_raw,
                master_continuous,
            )

            previews, previous_targets = compute_mapping_preview(
                master_snapshot=master_snapshot,
                slave_snapshot=slave_snapshot,
                joint_settings=joint_settings,
                runtime_settings=mapping.runtime,
                slave_limits=limits,
                master_anchor=master_anchor,
                slave_anchor=slave_anchor,
                previous_targets=previous_targets,
                master_continuous_positions=master_continuous,
            )

            if not args.no_clear:
                os.system("cls" if os.name == "nt" else "clear")
            _print_preview(cycle, previews)

            elapsed = time.perf_counter() - started
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        master.close()
        slave.close()


def _position_map(snapshot: ArmStatusSnapshot) -> dict[str, float | None]:
    return {joint.name: joint.position_rad for joint in snapshot.joints}


def _required_anchor(snapshot: ArmStatusSnapshot) -> dict[str, float]:
    anchor = {}
    for joint in snapshot.joints:
        if joint.position_rad is None:
            raise RuntimeError(f"{joint.name}: missing slave position, cannot build dry-run anchor")
        anchor[joint.name] = joint.position_rad
    return anchor


def _print_preview(cycle: int, previews) -> None:
    now = time.strftime("%H:%M:%S")
    print(f"Pink master -> real pink slave Dry Run  cycle={cycle}  time={now}")
    print("No command is sent to the slave. This is only a target preview.")
    print()
    print("joint   en  master(rad)  slave_now(rad)  target(rad)  status      would_send")
    print("------  --  -----------  --------------  -----------  ----------  ----------")
    for item in previews:
        enabled = "Y" if item.enabled else "N"
        master = _fmt(item.master_rad)
        slave = _fmt(item.slave_current_rad)
        target = _fmt(item.target_rad)
        would_send = "YES" if item.enabled and item.target_rad is not None and target_status_is_commandable(item.limit_status) else "NO"
        print(f"{item.name:<6}  {enabled:<2}  {master:>11}  {slave:>14}  {target:>11}  {item.limit_status:<10}  {would_send:<10}")


def _fmt(value: float | None) -> str:
    if value is None:
        return "--"
    return f"{value:.4f}"


if __name__ == "__main__":
    main()
