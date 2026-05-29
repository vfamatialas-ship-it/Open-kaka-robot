"""Stream real pink-slave joint states over a TCP socket.

读取真实粉色从臂的达妙电机状态，并通过 TCP socket 广播给虚拟臂界面。

示例：
  python scripts/visualization/pink_slave_socket_server.py --port COM7 --hz 20
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import open_arm_connection
from robot_core.visualization.socket_protocol import encode_message
from robot_core.visualization.socket_protocol import snapshot_to_message, urdf_info_message
from robot_core.visualization.urdf_light import default_pink_slave_urdf, load_arm_joints_from_urdf


class JointStateSocketServer:
    """Small TCP broadcast server for joint states."""

    def __init__(self, host: str, socket_port: int) -> None:
        self.host = host
        self.socket_port = socket_port
        self.clients: list[socket.socket] = []
        self.clients_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.server_socket: socket.socket | None = None

    def start(self) -> None:
        """Start accepting clients in the background."""

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.socket_port))
        self.server_socket.listen()
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def close(self) -> None:
        """Close server and connected clients."""

        self.stop_event.set()
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except OSError:
                    pass
            self.clients.clear()

    def broadcast(self, data: bytes) -> None:
        """Send bytes to all connected clients."""

        with self.clients_lock:
            alive: list[socket.socket] = []
            for client in self.clients:
                try:
                    client.sendall(data)
                except OSError:
                    try:
                        client.close()
                    except OSError:
                        pass
                    continue
                alive.append(client)
            self.clients = alive

    def _accept_loop(self) -> None:
        assert self.server_socket is not None
        while not self.stop_event.is_set():
            try:
                client, address = self.server_socket.accept()
            except OSError:
                break
            print(f"viewer connected: {address[0]}:{address[1]}")
            with self.clients_lock:
                self.clients.append(client)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink slave real-arm joint-state socket server.")
    parser.add_argument("--port", default="COM7", help="USB2CAN serial port for pink slave, default COM7")
    parser.add_argument("--baudrate", type=int, default=921600, help="USB2CAN serial baudrate")
    parser.add_argument("--host", default="127.0.0.1", help="socket bind host")
    parser.add_argument("--socket-port", type=int, default=8765, help="socket port for virtual viewer")
    parser.add_argument("--hz", type=float, default=20.0, help="broadcast frequency")
    parser.add_argument("--damiao-wait", type=float, default=0.005, help="Damiao response wait seconds")
    parser.add_argument("--urdf", default=str(default_pink_slave_urdf()), help="URDF path")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    server = JointStateSocketServer(args.host, args.socket_port)
    server.start()

    joints = load_arm_joints_from_urdf(args.urdf, count=7)
    server.broadcast(encode_message(urdf_info_message("pink_slave", joints)))

    print("Pink slave socket server")
    print(f"  hardware: pink_slave @ {args.port} {args.baudrate}")
    print(f"  socket:   {args.host}:{args.socket_port}")
    print(f"  hz:       {args.hz}")
    print(f"  URDF:     {args.urdf}")
    print("Safety: read-only hardware state streaming; no enable; no motion.")
    print()

    connection = open_arm_connection("pink_slave", port=args.port, baudrate=args.baudrate)
    period = 1.0 / max(1.0, args.hz)
    try:
        while True:
            snapshot = connection.read_snapshot(damiao_response_wait=args.damiao_wait)
            message = snapshot_to_message(snapshot, arm="pink_slave")
            server.broadcast(encode_message(message))
            time.sleep(period)
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        connection.close()
        server.close()


if __name__ == "__main__":
    main()
