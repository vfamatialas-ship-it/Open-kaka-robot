"""Start web mapping tool: pink master drives virtual pink slave.

网页模式，用粉色主臂控制 URDF/STL 虚拟粉色从臂，用来调 scale/sign/offset。

首次使用前：
  cd web_viewer/pink_slave_3d
  npm.cmd install

启动：
  python scripts/visualization/start_pink_master_virtual_slave_3d.py --master-port COM8
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
import urllib.parse
import webbrowser
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = PROJECT_ROOT / "web_viewer" / "pink_slave_3d"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Start pink master to virtual pink slave 3D mapping tool.")
    parser.add_argument("--master-port", default="COM8", help="pink master Feetech serial port")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--ws-http-port", type=int, default=8768)
    parser.add_argument("--vite-port", type=int, default=5173)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--no-browser", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    _ensure_frontend_installed()

    data_server = PROJECT_ROOT / "scripts" / "visualization" / "pink_master_virtual_slave_ws.py"
    data_cmd = [
        sys.executable,
        str(data_server),
        "--port",
        args.master_port,
        "--baudrate",
        str(args.master_baudrate),
        "--host",
        args.host,
        "--http-port",
        str(args.ws_http_port),
        "--hz",
        str(args.hz),
    ]
    vite_cmd = [
        "npm.cmd",
        "run",
        "dev",
        "--",
        "--host",
        args.host,
        "--port",
        str(args.vite_port),
    ]

    data_process = subprocess.Popen(data_cmd, cwd=PROJECT_ROOT)
    time.sleep(1.0)
    vite_process = subprocess.Popen(vite_cmd, cwd=WEB_ROOT)

    url = _viewer_url(args.host, args.vite_port, args.host, args.ws_http_port)
    print()
    print("Pink master -> virtual pink slave 3D mapping tool starting...")
    print(f"  Web viewer: {url}")
    print(f"  Data WS:    ws://{args.host}:{args.ws_http_port}/ws")
    print("  Real slave: not opened, no motion command")
    print("Close this terminal or press Ctrl+C to stop both processes.")
    print()

    if not args.no_browser:
        time.sleep(1.5)
        webbrowser.open(url)

    try:
        while True:
            if data_process.poll() is not None:
                raise RuntimeError("master data WebSocket server exited")
            if vite_process.poll() is not None:
                raise RuntimeError("Vite dev server exited")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        for process in (vite_process, data_process):
            process.terminate()
        for process in (vite_process, data_process):
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()


def _ensure_frontend_installed() -> None:
    node_modules = WEB_ROOT / "node_modules"
    if node_modules.exists():
        return
    raise SystemExit(
        "缺少前端依赖 node_modules。\n"
        "请先运行：\n"
        f"  cd {WEB_ROOT}\n"
        "  npm.cmd install\n"
        "然后再启动网页映射调参工具。"
    )


def _viewer_url(vite_host: str, vite_port: int, ws_host: str, ws_port: int) -> str:
    urdf_path = (PROJECT_ROOT / "assets" / "pink_slave_urdf" / "urdf" / "kaka_arm_v7.urdf").as_posix()
    package_root = (PROJECT_ROOT / "assets" / "pink_slave_urdf").as_posix()
    query = urllib.parse.urlencode(
        {
            "mode": "master_teleop",
            "ws": f"ws://{ws_host}:{ws_port}/ws",
            "urdf": f"/@fs/{urdf_path}",
            "packageRoot": f"/@fs/{package_root}",
        }
    )
    return f"http://{vite_host}:{vite_port}/?{query}"


if __name__ == "__main__":
    main()
