"""Tkinter virtual viewer for pink slave joint-state socket stream.

虚拟粉色从臂可视化界面。

说明：
  - 连接 pink_slave_socket_server.py 的 TCP socket。
  - 根据实时 joint1~joint7 角度画一个简化 2D 虚拟臂。
  - 当前不是完整 URDF mesh/FK 渲染；URDF 信息会用于关节名称/限位显示。

示例：
  python scripts/visualization/pink_slave_virtual_viewer.py --host 127.0.0.1 --socket-port 8765
"""

from __future__ import annotations

import argparse
import math
import queue
import socket
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.visualization.socket_protocol import decode_message
from robot_core.visualization.urdf_light import default_pink_slave_urdf, load_arm_joints_from_urdf


class PinkSlaveVirtualViewer(tk.Tk):
    """Simple socket-driven virtual arm viewer."""

    def __init__(self, host: str, socket_port: int, urdf_path: str) -> None:
        super().__init__()
        self.title("Pink Slave Virtual Viewer")
        self.geometry("980x680")
        self.minsize(860, 560)

        self.host = host
        self.socket_port = socket_port
        self.urdf_path = urdf_path
        self.message_queue: queue.Queue[dict] = queue.Queue()
        self.stop_event = threading.Event()
        self.joint_positions = {f"joint{i}": 0.0 for i in range(1, 8)}
        self.last_timestamp = 0.0
        self.status_text = tk.StringVar(value="未连接")
        self.urdf_joints = load_arm_joints_from_urdf(urdf_path, count=7)
        self.link_lengths = self._estimate_link_lengths()

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        threading.Thread(target=self._socket_worker, daemon=True).start()
        self.after(30, self._poll_messages)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=0)
        root.rowconfigure(1, weight=1)

        header = ttk.Frame(root)
        header.grid(row=0, column=0, columnspan=2, sticky=tk.EW, pady=(0, 8))
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="粉色从臂虚拟可视化 / Pink Slave Virtual Arm").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(header, textvariable=self.status_text, foreground="#005A9E").grid(row=0, column=1, sticky=tk.E)

        self.canvas = tk.Canvas(root, background="#111318", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky=tk.NSEW, padx=(0, 10))

        panel = ttk.LabelFrame(root, text="关节状态 / Joint State", padding=8)
        panel.grid(row=1, column=1, sticky=tk.NS)
        columns = ("joint", "rad", "limit")
        self.table = ttk.Treeview(panel, columns=columns, show="headings", height=9)
        self.table.heading("joint", text="关节")
        self.table.heading("rad", text="位置(rad)")
        self.table.heading("limit", text="URDF限位")
        self.table.column("joint", width=70, anchor=tk.CENTER)
        self.table.column("rad", width=100, anchor=tk.CENTER)
        self.table.column("limit", width=160, anchor=tk.CENTER)
        self.table.grid(row=0, column=0, sticky=tk.NSEW)

        for joint in self.urdf_joints:
            limit_text = self._format_limit(joint.lower, joint.upper)
            self.table.insert("", tk.END, iid=joint.display_name, values=(joint.display_name, "0.0000", limit_text))

        info = ttk.LabelFrame(root, text="模型信息 / Model Info", padding=8)
        info.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Label(
            info,
            text=(
                f"Socket: {self.host}:{self.socket_port}    "
                f"URDF: {self.urdf_path}    "
                "当前为轻量 2D 显示，后续可接完整 FK/mesh 渲染。"
            ),
            foreground="#555555",
        ).pack(anchor=tk.W)

    def _socket_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                with socket.create_connection((self.host, self.socket_port), timeout=3.0) as sock:
                    self.status_text.set(f"已连接 socket {self.host}:{self.socket_port}")
                    file = sock.makefile("rb")
                    while not self.stop_event.is_set():
                        line = file.readline()
                        if not line:
                            break
                        self.message_queue.put(decode_message(line))
            except OSError as exc:
                self.status_text.set(f"等待 socket server: {exc}")
                time.sleep(1.0)

    def _poll_messages(self) -> None:
        latest = None
        while True:
            try:
                latest = self.message_queue.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self._handle_message(latest)
        self._draw_arm()
        if not self.stop_event.is_set():
            self.after(30, self._poll_messages)

    def _handle_message(self, message: dict) -> None:
        if message.get("type") == "urdf_info":
            return
        joints = message.get("joints", {})
        for name, value in joints.items():
            if name in self.joint_positions:
                self.joint_positions[name] = float(value)
        self.last_timestamp = float(message.get("timestamp", 0.0))
        for name, value in self.joint_positions.items():
            if self.table.exists(name):
                current = list(self.table.item(name, "values"))
                current[1] = f"{value:.4f}"
                self.table.item(name, values=current)

    def _draw_arm(self) -> None:
        self.canvas.delete("all")
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        origin_x = width * 0.48
        origin_y = height * 0.72
        scale = min(width, height) * 0.72

        points = [(origin_x, origin_y)]
        theta = -math.pi / 2.0
        for index, length in enumerate(self.link_lengths, start=1):
            theta += self.joint_positions.get(f"joint{index}", 0.0) * 0.35
            last_x, last_y = points[-1]
            next_x = last_x + math.cos(theta) * length * scale
            next_y = last_y + math.sin(theta) * length * scale
            points.append((next_x, next_y))

        self.canvas.create_line(0, origin_y, width, origin_y, fill="#333842", width=1)
        for index in range(len(points) - 1):
            x1, y1 = points[index]
            x2, y2 = points[index + 1]
            color = "#E9B4B4" if index >= 5 else "#8FA7E8"
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=12, capstyle=tk.ROUND)
            self.canvas.create_oval(x1 - 9, y1 - 9, x1 + 9, y1 + 9, fill="#F4D35E", outline="#111318", width=2)
            self.canvas.create_text(x1 + 16, y1 - 16, text=f"J{index + 1}", fill="#DDE4FF", anchor=tk.W)

        end_x, end_y = points[-1]
        self.canvas.create_oval(end_x - 10, end_y - 10, end_x + 10, end_y + 10, fill="#FF6B6B", outline="")
        age = time.time() - self.last_timestamp if self.last_timestamp else 0.0
        self.canvas.create_text(
            16,
            16,
            text=f"latest packet age: {age:.2f}s",
            fill="#DDE4FF",
            anchor=tk.NW,
        )

    def _estimate_link_lengths(self) -> list[float]:
        lengths: list[float] = []
        for joint in self.urdf_joints:
            x, y, z = joint.origin_xyz
            length = math.sqrt(x * x + y * y + z * z)
            lengths.append(max(0.05, min(0.22, length)))
        if len(lengths) < 7:
            lengths.extend([0.08] * (7 - len(lengths)))
        return lengths[:7]

    @staticmethod
    def _format_limit(lower: float | None, upper: float | None) -> str:
        if lower is None or upper is None:
            return "-"
        return f"[{lower:.2f}, {upper:.2f}]"

    def on_close(self) -> None:
        self.stop_event.set()
        self.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink slave virtual arm socket viewer.")
    parser.add_argument("--host", default="127.0.0.1", help="socket server host")
    parser.add_argument("--socket-port", type=int, default=8765, help="socket server port")
    parser.add_argument("--urdf", default=str(default_pink_slave_urdf()), help="URDF path")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    app = PinkSlaveVirtualViewer(args.host, args.socket_port, args.urdf)
    app.mainloop()


if __name__ == "__main__":
    main()
