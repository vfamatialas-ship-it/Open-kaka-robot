"""Start pink slave socket server and virtual viewer together.

一键启动：
  1. 真实粉色从臂 socket server
  2. 虚拟粉色从臂 viewer

如果你想分别观察日志，也可以单独启动 server 和 viewer 两个脚本。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Start pink slave real-to-virtual socket demo.")
    parser.add_argument("--port", default="COM7", help="pink slave USB2CAN serial port")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--socket-port", type=int, default=8765)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--damiao-wait", type=float, default=0.005)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    server_script = PROJECT_ROOT / "scripts" / "visualization" / "pink_slave_socket_server.py"
    viewer_script = PROJECT_ROOT / "scripts" / "visualization" / "pink_slave_virtual_viewer.py"

    server_cmd = [
        sys.executable,
        str(server_script),
        "--port",
        args.port,
        "--baudrate",
        str(args.baudrate),
        "--host",
        args.host,
        "--socket-port",
        str(args.socket_port),
        "--hz",
        str(args.hz),
        "--damiao-wait",
        str(args.damiao_wait),
    ]
    viewer_cmd = [
        sys.executable,
        str(viewer_script),
        "--host",
        args.host,
        "--socket-port",
        str(args.socket_port),
    ]

    server = subprocess.Popen(server_cmd, cwd=PROJECT_ROOT)
    time.sleep(1.0)
    viewer = subprocess.Popen(viewer_cmd, cwd=PROJECT_ROOT)
    try:
        viewer.wait()
    finally:
        server.terminate()
        try:
            server.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            server.kill()


if __name__ == "__main__":
    main()
