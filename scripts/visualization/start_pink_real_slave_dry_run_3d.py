"""Start 3D web Dry Run: pink master maps to real pink slave target preview.

The real slave is opened for status reading only. No enable or motion command is
sent. The browser displays:
- real slave current joint state
- mapped target joint state
- limit saturation status
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
    parser = argparse.ArgumentParser(description="Start pink real slave Dry Run 3D viewer.")
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--ws-http-port", type=int, default=8769)
    parser.add_argument("--vite-port", type=int, default=5174)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--no-browser", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    _ensure_frontend_installed()

    data_server = PROJECT_ROOT / "scripts" / "visualization" / "pink_real_slave_dry_run_ws.py"
    data_cmd = [
        sys.executable,
        str(data_server),
        "--master-port",
        args.master_port,
        "--master-baudrate",
        str(args.master_baudrate),
        "--slave-port",
        args.slave_port,
        "--slave-baudrate",
        str(args.slave_baudrate),
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
    print("Pink master -> real pink slave Dry Run 3D viewer starting...")
    print(f"  Web viewer: {url}")
    print(f"  Data WS:    ws://{args.host}:{args.ws_http_port}/ws")
    print("  Real slave: read-only, no motion command")
    print("Close this terminal or press Ctrl+C to stop both processes.")
    print()

    if not args.no_browser:
        time.sleep(1.5)
        webbrowser.open(url)

    try:
        while True:
            if data_process.poll() is not None:
                raise RuntimeError("Dry Run WebSocket server exited")
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
    if (WEB_ROOT / "node_modules").exists():
        return
    raise SystemExit(
        "Missing frontend dependencies. Run:\n"
        f"  cd {WEB_ROOT}\n"
        "  npm.cmd install"
    )


def _viewer_url(vite_host: str, vite_port: int, ws_host: str, ws_port: int) -> str:
    urdf_path = (PROJECT_ROOT / "assets" / "pink_slave_urdf" / "urdf" / "kaka_arm_v7.urdf").as_posix()
    package_root = (PROJECT_ROOT / "assets" / "pink_slave_urdf").as_posix()
    query = urllib.parse.urlencode(
        {
            "mode": "real_slave_dry_run",
            "ws": f"ws://{ws_host}:{ws_port}/ws",
            "urdf": f"/@fs/{urdf_path}",
            "packageRoot": f"/@fs/{package_root}",
        }
    )
    return f"http://{vite_host}:{vite_port}/?{query}"


if __name__ == "__main__":
    main()
