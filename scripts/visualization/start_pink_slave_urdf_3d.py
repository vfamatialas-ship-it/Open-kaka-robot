"""Start the full URDF/STL 3D web viewer for the real pink slave arm.

高级 3D 版本：
  - Python 读取真实粉色从臂达妙电机
  - Python 提供 WebSocket joint-state 数据流
  - Vite + Three.js + URDFLoader 在浏览器中加载 URDF/STL

首次使用前需要安装前端依赖：
  cd web_viewer/pink_slave_3d
  npm.cmd install

启动：
  python scripts/visualization/start_pink_slave_urdf_3d.py --port COM7
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
    parser = argparse.ArgumentParser(description="Start pink slave full URDF/STL 3D viewer.")
    parser.add_argument("--port", default="COM7", help="pink slave USB2CAN serial port")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--ws-http-port", type=int, default=8766, help="Python WebSocket server port")
    parser.add_argument("--vite-port", type=int, default=5173, help="Vite web viewer port")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--damiao-wait", type=float, default=0.005)
    parser.add_argument("--no-browser", action="store_true", help="do not open browser automatically")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    _ensure_frontend_installed()

    data_server = PROJECT_ROOT / "scripts" / "visualization" / "pink_slave_3d_web_viewer.py"
    data_cmd = [
        sys.executable,
        str(data_server),
        "--port",
        args.port,
        "--baudrate",
        str(args.baudrate),
        "--host",
        args.host,
        "--http-port",
        str(args.ws_http_port),
        "--hz",
        str(args.hz),
        "--damiao-wait",
        str(args.damiao_wait),
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
    print("Full URDF/STL 3D viewer starting...")
    print(f"  Web viewer: {url}")
    print(f"  Data WS:    ws://{args.host}:{args.ws_http_port}/ws")
    print("Close this terminal or press Ctrl+C to stop both processes.")
    print()

    if not args.no_browser:
        time.sleep(1.5)
        webbrowser.open(url)

    try:
        while True:
            if data_process.poll() is not None:
                raise RuntimeError("data WebSocket server exited")
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
        "然后再启动 3D viewer。"
    )


def _viewer_url(vite_host: str, vite_port: int, ws_host: str, ws_port: int) -> str:
    urdf_path = (PROJECT_ROOT / "assets" / "pink_slave_urdf" / "urdf" / "kaka_arm_v7.urdf").as_posix()
    package_root = (PROJECT_ROOT / "assets" / "pink_slave_urdf").as_posix()
    query = urllib.parse.urlencode(
        {
            "ws": f"ws://{ws_host}:{ws_port}/ws",
            "urdf": f"/@fs/{urdf_path}",
            "packageRoot": f"/@fs/{package_root}",
        }
    )
    return f"http://{vite_host}:{vite_port}/?{query}"


if __name__ == "__main__":
    main()
