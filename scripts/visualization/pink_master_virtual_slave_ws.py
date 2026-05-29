"""WebSocket data server: pink master drives virtual pink slave.

只读取粉色主臂 Feetech 舵机，不控制真实从臂。网页端根据主臂关节角计算
虚拟粉色从臂 target，用于调试主从映射关系。
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import re
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import open_arm_connection
from robot_core.services.joint_limit_service import load_limits_text, parse_limits_text
from robot_core.services.teleop_mapping_config import load_teleop_mapping
from robot_core.services.teleop_mapping_config import mapping_to_virtual_config_payload
from robot_core.services.zero_service import load_zero_text
from robot_core.visualization.socket_protocol import snapshot_to_message, urdf_info_message
from robot_core.visualization.urdf_light import default_pink_slave_urdf, load_arm_joints_from_urdf


WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
TELEOP_MAPPING_PATH = PROJECT_ROOT / "configs" / "teleop_mapping.yaml"
PINK_MASTER_TO_SLAVE_DEFAULT_SIGNS = {
    "joint1": -1,
    "joint2": 1,
    "joint3": 1,
    "joint4": -1,
    "joint5": -1,
    "joint6": -1,
    "joint7": -1,
}


class WebSocketHub:
    """Tiny broadcast-only WebSocket hub."""

    def __init__(self) -> None:
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()

    def add(self, sock: socket.socket) -> None:
        with self.lock:
            self.clients.append(sock)

    def broadcast_json(self, message: dict[str, Any]) -> None:
        payload = json.dumps(message, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        frame = _encode_ws_text_frame(payload)
        with self.lock:
            alive: list[socket.socket] = []
            for client in self.clients:
                try:
                    client.sendall(frame)
                except OSError:
                    try:
                        client.close()
                    except OSError:
                        pass
                    continue
                alive.append(client)
            self.clients = alive


class ViewerState:
    """Shared state for HTTP handlers and broadcaster."""

    def __init__(self, hub: WebSocketHub, urdf_path: str) -> None:
        self.hub = hub
        self.urdf_joints = load_arm_joints_from_urdf(urdf_path, count=7)
        self.config = build_virtual_teleop_config()

    def handle_client_message(self, text: str) -> dict[str, Any] | None:
        """Handle a JSON message sent by the browser."""

        try:
            message = json.loads(text)
        except json.JSONDecodeError as exc:
            return {"type": "error", "message": f"bad JSON from browser: {exc}"}

        if message.get("type") != "save_virtual_mapping":
            return None

        try:
            saved_path = save_virtual_mapping_config(message)
        except Exception as exc:  # noqa: BLE001
            return {"type": "error", "message": f"mapping save failed: {exc}"}
        return {
            "type": "mapping_saved",
            "path": str(saved_path),
            "timestamp": time.time(),
        }


def make_handler(state: ViewerState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/ws":
                self._handle_websocket()
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write("pink master virtual slave websocket server\n".encode("utf-8"))

        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            print(f"[web] {self.address_string()} - {fmt % args}")

        def _handle_websocket(self) -> None:
            key = self.headers.get("Sec-WebSocket-Key")
            if not key:
                self.send_error(HTTPStatus.BAD_REQUEST, "missing websocket key")
                return
            accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")
            self.send_response(HTTPStatus.SWITCHING_PROTOCOLS)
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", accept)
            self.end_headers()

            sock = self.connection
            state.hub.add(sock)
            state.hub.broadcast_json(urdf_info_message("pink_slave", state.urdf_joints))
            state.hub.broadcast_json(state.config)
            try:
                while True:
                    text = _read_ws_text_message(sock)
                    if text is None:
                        break
                    response = state.handle_client_message(text)
                    if response is not None:
                        sock.sendall(_encode_ws_text_frame(json.dumps(response, ensure_ascii=False).encode("utf-8")))
                    time.sleep(0.05)
            except OSError:
                pass

    return Handler


def build_virtual_teleop_config() -> dict[str, Any]:
    """Build initial mapping/limit/zero config for the browser."""

    mapping = load_teleop_mapping("pink_master", "pink_slave")
    master_zero = parse_zero_positions(load_zero_text("pink_master"))
    slave_zero = parse_zero_positions(load_zero_text("pink_slave"))
    slave_limits = {
        limit.name: {"min": limit.min_rad, "max": limit.max_rad}
        for limit in parse_limits_text(load_limits_text("pink_slave"))
    }
    return {
        "protocol": "dual_arm_robot.virtual_teleop.v1",
        "type": "virtual_teleop_config",
        "timestamp": time.time(),
        "master_arm": "pink_master",
        "virtual_slave_arm": "pink_slave",
        "master_zero": master_zero,
        "slave_zero": slave_zero,
        "slave_limits": slave_limits,
        "mappings": mapping_to_virtual_config_payload(mapping),
        "runtime": {
            "alpha": mapping.runtime.alpha,
            "max_step_rad": mapping.runtime.max_step_rad,
        },
    }


def parse_zero_positions(text: str) -> dict[str, float]:
    """Parse the simple zero YAML written by zero_service without extra deps."""

    zeros: dict[str, float] = {}
    current_name: str | None = None
    for raw_line in text.splitlines():
        line = raw_line.strip()
        name_match = re.match(r"-\s+name:\s+(\w+)", line)
        if name_match:
            current_name = name_match.group(1)
            continue
        value_match = re.match(r"zero_position_rad:\s+([-+0-9.eE]+)", line)
        if current_name is not None and value_match:
            zeros[current_name] = float(value_match.group(1))
            current_name = None
    return zeros


def save_virtual_mapping_config(message: dict[str, Any]) -> Path:
    """Save browser-tuned pink master -> pink slave mapping to teleop_mapping.yaml."""

    mappings = message.get("mappings")
    runtime = message.get("runtime", {})
    if not isinstance(mappings, dict):
        raise ValueError("missing mappings object")

    document = _load_mapping_document()
    document.setdefault(
        "safety",
        {
            "default_read_only": True,
            "enable_slave_motion": False,
            "require_limit_check_before_motion": True,
            "emergency_stop_enabled": True,
        },
    )
    entries = [
        entry
        for entry in document.get("mappings", [])
        if not (entry.get("master_arm") == "pink_master" and entry.get("slave_arm") == "pink_slave")
    ]

    joints = []
    for index in range(1, 8):
        joint_name = f"joint{index}"
        item = mappings.get(joint_name, {})
        joint = {
            "master_joint": joint_name,
            "slave_joint": joint_name,
            "enabled": bool(item.get("enabled", True)),
            "scale": float(item.get("scale", 1.0)),
            "sign": int(item.get("sign", PINK_MASTER_TO_SLAVE_DEFAULT_SIGNS[joint_name])),
            "offset_rad": float(item.get("offset", item.get("offset_rad", 0.0))),
        }
        mapping_mode = str(item.get("mapping_mode", "anchor_delta"))
        if mapping_mode == "zero_delta":
            joint["mapping_mode"] = "zero_delta"
        if mapping_mode == "range":
            joint["mapping_mode"] = "range"
            for key in ("master_min_rad", "master_max_rad", "slave_min_rad", "slave_max_rad"):
                value = item.get(key)
                if value is not None:
                    joint[key] = float(value)
        joints.append(joint)

    entries.append(
        {
            "master_arm": "pink_master",
            "slave_arm": "pink_slave",
            "mode": "master_to_slave_verified_virtual",
            "description": "Verified in web virtual teleop before real slave motion.",
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "runtime": {
                "alpha": float(runtime.get("alpha", 0.35)),
                "max_step_rad": float(runtime.get("max_step_rad", 0.06)),
            },
            "joints": joints,
        }
    )
    document["mappings"] = entries

    TELEOP_MAPPING_PATH.parent.mkdir(parents=True, exist_ok=True)
    text = _dump_mapping_document(document)
    TELEOP_MAPPING_PATH.write_text(text, encoding="utf-8")
    return TELEOP_MAPPING_PATH


def _load_mapping_document() -> dict[str, Any]:
    if not TELEOP_MAPPING_PATH.exists():
        return _default_mapping_document()
    try:
        import yaml

        data = yaml.safe_load(TELEOP_MAPPING_PATH.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return _default_mapping_document()


def _dump_mapping_document(document: dict[str, Any]) -> str:
    try:
        import yaml

        return (
            "# Teleoperation mapping configuration.\n"
            "# 主从遥操作映射配置。真实从臂运动前必须先通过 Dry Run/虚拟调试验证。\n"
            + yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
        )
    except Exception:
        return _dump_mapping_document_manual(document)


def _default_mapping_document() -> dict[str, Any]:
    return {
        "safety": {
            "default_read_only": True,
            "enable_slave_motion": False,
            "require_limit_check_before_motion": True,
            "emergency_stop_enabled": True,
        },
        "mappings": [
            _disabled_mapping_entry("master_left", "slave_left"),
            _disabled_mapping_entry("master_right", "slave_right"),
        ],
    }


def _disabled_mapping_entry(master_arm: str, slave_arm: str) -> dict[str, Any]:
    return {
        "master_arm": master_arm,
        "slave_arm": slave_arm,
        "joints": [
            {
                "master_joint": f"joint{index}",
                "slave_joint": f"joint{index}",
                "enabled": False,
                "scale": 1.0,
                "sign": 1,
                "offset_rad": 0.0,
            }
            for index in range(1, 8)
        ],
    }


def _dump_mapping_document_manual(document: dict[str, Any]) -> str:
    lines = [
        "# Teleoperation mapping configuration.",
        "# 主从遥操作映射配置。真实从臂运动前必须先通过 Dry Run/虚拟调试验证。",
        "safety:",
    ]
    for key, value in document.get("safety", {}).items():
        lines.append(f"  {key}: {_yaml_scalar(value)}")
    lines.append("")
    lines.append("mappings:")
    for entry in document.get("mappings", []):
        lines.append(f"  - master_arm: {entry.get('master_arm', '')}")
        lines.append(f"    slave_arm: {entry.get('slave_arm', '')}")
        if "mode" in entry:
            lines.append(f"    mode: {entry['mode']}")
        if "description" in entry:
            lines.append(f"    description: {entry['description']}")
        if "updated_at" in entry:
            lines.append(f"    updated_at: {entry['updated_at']}")
        if "runtime" in entry:
            lines.append("    runtime:")
            for key, value in entry["runtime"].items():
                lines.append(f"      {key}: {_yaml_scalar(value)}")
        lines.append("    joints:")
        for joint in entry.get("joints", []):
            lines.append(f"      - master_joint: {joint.get('master_joint', '')}")
            lines.append(f"        slave_joint: {joint.get('slave_joint', '')}")
            lines.append(f"        enabled: {_yaml_scalar(joint.get('enabled', False))}")
            lines.append(f"        scale: {_yaml_scalar(joint.get('scale', 1.0))}")
            lines.append(f"        sign: {_yaml_scalar(joint.get('sign', 1))}")
            lines.append(f"        offset_rad: {_yaml_scalar(joint.get('offset_rad', 0.0))}")
    lines.append("")
    return "\n".join(lines)


def _yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _encode_ws_text_frame(payload: bytes) -> bytes:
    header = bytearray([0x81])
    length = len(payload)
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.append(126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(127)
        header.extend(struct.pack("!Q", length))
    return bytes(header) + payload


def _read_ws_text_message(sock: socket.socket) -> str | None:
    """Read one client-to-server WebSocket text frame."""

    header = _recv_exact(sock, 2)
    if not header:
        return None
    first, second = header
    opcode = first & 0x0F
    if opcode == 0x8:
        return None
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _recv_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _recv_exact(sock, 8))[0]
    mask = _recv_exact(sock, 4) if masked else b""
    payload = _recv_exact(sock, length)
    if masked:
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    if opcode != 0x1:
        return ""
    return payload.decode("utf-8")


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return b""
        data.extend(chunk)
    return bytes(data)


def _master_broadcast_loop(
    hub: WebSocketHub,
    *,
    serial_port: str,
    baudrate: int,
    hz: float,
) -> None:
    connection = open_arm_connection("pink_master", port=serial_port, baudrate=baudrate)
    period = 1.0 / max(1.0, hz)
    try:
        while True:
            snapshot = connection.read_snapshot()
            message = snapshot_to_message(snapshot, arm="pink_master")
            message["type"] = "master_state"
            hub.broadcast_json(message)
            time.sleep(period)
    finally:
        connection.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink master to virtual pink slave WebSocket server.")
    parser.add_argument("--port", default="COM8", help="pink master Feetech serial port")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8768)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--urdf", default=str(default_pink_slave_urdf()), help="virtual slave URDF path")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    hub = WebSocketHub()
    state = ViewerState(hub, args.urdf)
    handler = make_handler(state)
    server = ThreadingHTTPServer((args.host, args.http_port), handler)

    print("Pink master -> virtual pink slave WebSocket server")
    print(f"  hardware: pink_master @ {args.port} {args.baudrate}")
    print(f"  ws:       ws://{args.host}:{args.http_port}/ws")
    print(f"  URDF:     {args.urdf}")
    print("Safety: virtual only; real slave is not opened; no slave motion command.")
    print()

    broadcaster = threading.Thread(
        target=_master_broadcast_loop,
        kwargs={
            "hub": hub,
            "serial_port": args.port,
            "baudrate": args.baudrate,
            "hz": args.hz,
        },
        daemon=True,
    )
    broadcaster.start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
