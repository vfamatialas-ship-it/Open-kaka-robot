"""WebSocket server for real pink slave Dry Run preview.

Reads:
- pink_master from Feetech serial
- pink_slave from Damiao USB2CAN serial

Sends to the browser:
- master_state
- slave_state
- mapping/limit/anchor config

Safety: this script never enables motors and never sends motion commands.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import ArmStatusSnapshot, open_arm_connection
from robot_core.services.joint_limit_service import load_limits_text, parse_limits_text
from robot_core.services.teleop_mapping_config import load_teleop_mapping
from robot_core.services.teleop_mapping_config import mapping_to_virtual_config_payload
from robot_core.services.zero_service import load_zero_text
from robot_core.visualization.socket_protocol import snapshot_to_message, urdf_info_message
from robot_core.visualization.urdf_light import default_pink_slave_urdf, load_arm_joints_from_urdf
from scripts.visualization.pink_master_virtual_slave_ws import WebSocketHub, make_handler
from scripts.visualization.pink_master_virtual_slave_ws import parse_zero_positions


class DryRunViewerState:
    """Shared state used by the generic WebSocket handler."""

    def __init__(self, hub: WebSocketHub, urdf_path: str) -> None:
        self.hub = hub
        self.urdf_joints = load_arm_joints_from_urdf(urdf_path, count=7)
        self.config = build_dry_run_config({})

    def handle_client_message(self, text: str) -> dict[str, Any] | None:
        return None

    def update_slave_anchor(self, slave_anchor: dict[str, float]) -> None:
        self.config = build_dry_run_config(slave_anchor)
        self.hub.broadcast_json(self.config)


def build_dry_run_config(slave_anchor: dict[str, float]) -> dict[str, Any]:
    mapping = load_teleop_mapping("pink_master", "pink_slave")
    slave_limits = {
        limit.name: {"min": limit.min_rad, "max": limit.max_rad}
        for limit in parse_limits_text(load_limits_text("pink_slave"))
    }
    return {
        "protocol": "dual_arm_robot.real_slave_dry_run.v1",
        "type": "virtual_teleop_config",
        "timestamp": time.time(),
        "master_arm": "pink_master",
        "real_slave_arm": "pink_slave",
        "virtual_slave_arm": "pink_slave",
        "mode_label": "pink_master -> real pink_slave Dry Run",
        "locked_config": True,
        "master_zero": parse_zero_positions(load_zero_text("pink_master")),
        "slave_zero": parse_zero_positions(load_zero_text("pink_slave")),
        "slave_anchor": slave_anchor,
        "slave_limits": slave_limits,
        "mappings": mapping_to_virtual_config_payload(mapping),
        "runtime": {
            "alpha": mapping.runtime.alpha,
            "max_step_rad": mapping.runtime.max_step_rad,
        },
    }


def dry_run_broadcast_loop(
    state: DryRunViewerState,
    *,
    master_port: str,
    master_baudrate: int,
    slave_port: str,
    slave_baudrate: int,
    hz: float,
) -> None:
    mapping = load_teleop_mapping("pink_master", "pink_slave")
    master = open_arm_connection("pink_master", port=master_port, baudrate=master_baudrate)
    slave = open_arm_connection("pink_slave", port=slave_port, baudrate=slave_baudrate)
    period = 1.0 / max(1.0, hz)

    try:
        slave_anchor_snapshot = slave.read_snapshot(damiao_response_wait=mapping.runtime.damiao_response_wait)
        state.update_slave_anchor(_required_anchor(slave_anchor_snapshot))

        while True:
            started = time.perf_counter()
            master_snapshot = master.read_snapshot()
            slave_snapshot = slave.read_snapshot(damiao_response_wait=mapping.runtime.damiao_response_wait)

            master_message = snapshot_to_message(master_snapshot, arm="pink_master")
            master_message["type"] = "master_state"
            slave_message = snapshot_to_message(slave_snapshot, arm="pink_slave")
            slave_message["type"] = "slave_state"
            state.hub.broadcast_json(master_message)
            state.hub.broadcast_json(slave_message)

            elapsed = time.perf_counter() - started
            time.sleep(max(0.0, period - elapsed))
    finally:
        master.close()
        slave.close()


def _required_anchor(snapshot: ArmStatusSnapshot) -> dict[str, float]:
    anchor = {}
    for joint in snapshot.joints:
        if joint.position_rad is None:
            raise RuntimeError(f"{joint.name}: missing slave position, cannot build Dry Run anchor")
        anchor[joint.name] = joint.position_rad
    return anchor


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink master -> real pink slave Dry Run WebSocket server.")
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8769)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--urdf", default=str(default_pink_slave_urdf()))
    return parser


def main() -> None:
    args = build_parser().parse_args()
    hub = WebSocketHub()
    state = DryRunViewerState(hub, args.urdf)
    handler = make_handler(state)

    from http.server import ThreadingHTTPServer

    server = ThreadingHTTPServer((args.host, args.http_port), handler)
    print("Pink master -> real pink slave Dry Run WebSocket server")
    print(f"  master: pink_master @ {args.master_port} {args.master_baudrate}")
    print(f"  slave:  pink_slave  @ {args.slave_port} {args.slave_baudrate}")
    print(f"  ws:     ws://{args.host}:{args.http_port}/ws")
    print("Safety: read-only Dry Run; real slave is opened for reading only; no motion command.")
    print()

    worker = threading.Thread(
        target=dry_run_broadcast_loop,
        kwargs={
            "state": state,
            "master_port": args.master_port,
            "master_baudrate": args.master_baudrate,
            "slave_port": args.slave_port,
            "slave_baudrate": args.slave_baudrate,
            "hz": args.hz,
        },
        daemon=True,
    )
    worker.start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
