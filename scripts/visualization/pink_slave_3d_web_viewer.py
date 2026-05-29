"""WebSocket 3D viewer for the real pink slave arm.

启动一个本地网页和 WebSocket 服务：
  - Python 读取真实粉色从臂 joint1~joint7
  - 浏览器通过 WebSocket 接收关节角
  - 网页根据 URDF joint origin/axis 做轻量 3D 骨架显示

示例：
  python scripts/visualization/pink_slave_3d_web_viewer.py --port COM7 --hz 20

然后打开：
  http://127.0.0.1:8766
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import socket
import struct
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import open_arm_connection
from robot_core.visualization.socket_protocol import snapshot_to_message, urdf_info_message
from robot_core.visualization.urdf_light import default_pink_slave_urdf, load_arm_joints_from_urdf


WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


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
        self.urdf_path = urdf_path
        self.urdf_joints = load_arm_joints_from_urdf(urdf_path, count=7)


def make_handler(state: ViewerState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/ws":
                self._handle_websocket()
                return
            self._serve_index()

        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            print(f"[web] {self.address_string()} - {fmt % args}")

        def _serve_index(self) -> None:
            html = _html_page()
            body = html.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

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
            try:
                while True:
                    # Keep the handler alive. We do not need client messages.
                    data = sock.recv(2)
                    if not data:
                        break
                    time.sleep(0.05)
            except OSError:
                pass

    return Handler


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


def _hardware_broadcast_loop(
    hub: WebSocketHub,
    *,
    serial_port: str,
    baudrate: int,
    hz: float,
    damiao_wait: float,
) -> None:
    connection = open_arm_connection("pink_slave", port=serial_port, baudrate=baudrate)
    period = 1.0 / max(1.0, hz)
    try:
        while True:
            snapshot = connection.read_snapshot(damiao_response_wait=damiao_wait)
            hub.broadcast_json(snapshot_to_message(snapshot, arm="pink_slave"))
            time.sleep(period)
    finally:
        connection.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink slave 3D WebSocket viewer.")
    parser.add_argument("--port", default="COM7", help="pink slave USB2CAN serial port")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--damiao-wait", type=float, default=0.005)
    parser.add_argument("--urdf", default=str(default_pink_slave_urdf()), help="URDF path")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    hub = WebSocketHub()
    state = ViewerState(hub, args.urdf)
    handler = make_handler(state)
    server = ThreadingHTTPServer((args.host, args.http_port), handler)

    print("Pink slave 3D WebSocket viewer")
    print(f"  hardware: pink_slave @ {args.port} {args.baudrate}")
    print(f"  web:      http://{args.host}:{args.http_port}")
    print(f"  ws:       ws://{args.host}:{args.http_port}/ws")
    print(f"  URDF:     {args.urdf}")
    print("Safety: read-only hardware state streaming; no enable; no motion.")
    print()

    broadcaster = threading.Thread(
        target=_hardware_broadcast_loop,
        kwargs={
            "hub": hub,
            "serial_port": args.port,
            "baudrate": args.baudrate,
            "hz": args.hz,
            "damiao_wait": args.damiao_wait,
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


def _html_page() -> str:
    return r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Pink Slave 3D Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <style>
    html, body { margin: 0; height: 100%; background: #101217; color: #eef3ff; font-family: "Segoe UI", Arial, sans-serif; }
    #app { display: grid; grid-template-columns: minmax(0, 1fr) 330px; grid-template-rows: 48px minmax(0, 1fr); height: 100%; }
    header { grid-column: 1 / 3; display: flex; align-items: center; justify-content: space-between; padding: 0 18px; border-bottom: 1px solid #252a34; background: #171b23; }
    header h1 { margin: 0; font-size: 16px; font-weight: 600; letter-spacing: 0; }
    #status { font-size: 13px; color: #9cc9ff; }
    #stage { position: relative; min-width: 0; min-height: 0; }
    canvas { display: block; width: 100%; height: 100%; background: radial-gradient(circle at 45% 35%, #1d2430 0%, #101217 65%); }
    aside { border-left: 1px solid #252a34; background: #f4f5f7; color: #111827; padding: 14px; overflow: auto; }
    .panel-title { font-weight: 600; margin-bottom: 10px; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; }
    th, td { padding: 6px 4px; text-align: right; border-bottom: 1px solid #d8dce3; }
    th:first-child, td:first-child { text-align: left; }
    .hint { margin-top: 14px; font-size: 12px; line-height: 1.5; color: #4b5563; }
    .badge { color: #0f766e; font-weight: 600; }
    #overlay { position: absolute; left: 14px; top: 14px; font-size: 13px; color: #dbeafe; pointer-events: none; line-height: 1.6; }
  </style>
</head>
<body>
  <div id="app">
    <header>
      <h1>粉色从臂 3D 可视化 / Pink Slave 3D Viewer</h1>
      <div id="status">connecting...</div>
    </header>
    <main id="stage">
      <canvas id="canvas"></canvas>
      <div id="overlay"></div>
    </main>
    <aside>
      <div class="panel-title">关节状态 / Joint State</div>
      <table>
        <thead><tr><th>关节</th><th>rad</th><th>URDF限位</th></tr></thead>
        <tbody id="jointRows"></tbody>
      </table>
      <div class="hint">
        <div><span class="badge">鼠标左键拖动</span>：旋转视角</div>
        <div><span class="badge">滚轮</span>：缩放</div>
        <div>当前是 URDF 关节轴 + origin 的轻量 3D 骨架显示；mesh/STL 渲染可作为下一步升级。</div>
      </div>
    </aside>
  </div>
  <script>
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");
    const statusEl = document.getElementById("status");
    const overlay = document.getElementById("overlay");
    const rows = document.getElementById("jointRows");

    let joints = Array.from({length: 7}, (_, i) => ({
      name: `joint${i + 1}`,
      axis: [0, 0, 1],
      origin_xyz: [0, 0, 0.08],
      origin_rpy: [0, 0, 0],
      lower: null,
      upper: null,
    }));
    let q = {};
    for (const j of joints) q[j.name] = 0;
    let lastPacket = 0;
    let yaw = -0.75;
    let pitch = 0.35;
    let zoom = 1.0;
    let dragging = false;
    let lastMouse = [0, 0];

    function connect() {
      const ws = new WebSocket(`ws://${location.host}/ws`);
      ws.onopen = () => { statusEl.textContent = "websocket connected"; };
      ws.onclose = () => {
        statusEl.textContent = "websocket disconnected, retrying...";
        setTimeout(connect, 1000);
      };
      ws.onerror = () => { statusEl.textContent = "websocket error"; };
      ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === "urdf_info") {
          joints = msg.joints.map(j => ({...j, origin_rpy: j.origin_rpy || [0,0,0]})).slice(0, 7);
          for (const j of joints) if (!(j.name in q)) q[j.name] = 0;
          buildTable();
          return;
        }
        if (msg.joints) {
          for (const [name, value] of Object.entries(msg.joints)) q[name] = value;
          lastPacket = msg.timestamp || Date.now() / 1000;
          updateTable();
        }
      };
    }

    function buildTable() {
      rows.innerHTML = "";
      for (const joint of joints) {
        const tr = document.createElement("tr");
        tr.id = `row-${joint.name}`;
        const lim = joint.lower == null || joint.upper == null ? "-" : `[${joint.lower.toFixed(2)}, ${joint.upper.toFixed(2)}]`;
        tr.innerHTML = `<td>${joint.name}<br><small>${joint.urdf_name || ""}</small></td><td class="rad">0.0000</td><td>${lim}</td>`;
        rows.appendChild(tr);
      }
    }

    function updateTable() {
      for (const joint of joints) {
        const tr = document.getElementById(`row-${joint.name}`);
        if (tr) tr.querySelector(".rad").textContent = (q[joint.name] || 0).toFixed(4);
      }
    }

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = Math.max(1, window.devicePixelRatio || 1);
      canvas.width = Math.floor(rect.width * dpr);
      canvas.height = Math.floor(rect.height * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    function matMul(a, b) {
      const r = Array(16).fill(0);
      for (let row = 0; row < 4; row++) {
        for (let col = 0; col < 4; col++) {
          for (let k = 0; k < 4; k++) r[row * 4 + col] += a[row * 4 + k] * b[k * 4 + col];
        }
      }
      return r;
    }
    function identity() { return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]; }
    function translate(x, y, z) { return [1,0,0,x, 0,1,0,y, 0,0,1,z, 0,0,0,1]; }
    function rotX(a) { const c=Math.cos(a),s=Math.sin(a); return [1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1]; }
    function rotY(a) { const c=Math.cos(a),s=Math.sin(a); return [c,0,s,0, 0,1,0,0, -s,0,c,0, 0,0,0,1]; }
    function rotZ(a) { const c=Math.cos(a),s=Math.sin(a); return [c,-s,0,0, s,c,0,0, 0,0,1,0, 0,0,0,1]; }
    function rotAxis(axis, a) {
      let [x,y,z]=axis; const n=Math.hypot(x,y,z)||1; x/=n; y/=n; z/=n;
      const c=Math.cos(a), s=Math.sin(a), t=1-c;
      return [
        t*x*x+c, t*x*y-s*z, t*x*z+s*y, 0,
        t*x*y+s*z, t*y*y+c, t*y*z-s*x, 0,
        t*x*z-s*y, t*y*z+s*x, t*z*z+c, 0,
        0,0,0,1
      ];
    }
    function transformPoint(m, p) {
      const [x,y,z]=p;
      return [m[0]*x+m[1]*y+m[2]*z+m[3], m[4]*x+m[5]*y+m[6]*z+m[7], m[8]*x+m[9]*y+m[10]*z+m[11]];
    }

    function fkPoints() {
      let T = identity();
      const pts = [[0,0,0]];
      for (const joint of joints) {
        const [x,y,z] = joint.origin_xyz || [0,0,0];
        const [r,p,yy] = joint.origin_rpy || [0,0,0];
        T = matMul(T, translate(x,y,z));
        T = matMul(T, rotZ(yy));
        T = matMul(T, rotY(p));
        T = matMul(T, rotX(r));
        T = matMul(T, rotAxis(joint.axis || [0,0,1], q[joint.name] || 0));
        pts.push(transformPoint(T, [0,0,0]));
      }
      return pts;
    }

    function worldToView(p) {
      let [x,y,z] = p;
      let cy=Math.cos(yaw), sy=Math.sin(yaw);
      let x1=cy*x+sy*z, z1=-sy*x+cy*z;
      let cp=Math.cos(pitch), sp=Math.sin(pitch);
      let y1=cp*y-sp*z1, z2=sp*y+cp*z1;
      return [x1, y1, z2];
    }
    function project(p) {
      const rect = canvas.getBoundingClientRect();
      const [x,y,z] = worldToView(p);
      const f = 620 * zoom / (2.2 + z);
      return [rect.width/2 + x*f, rect.height*0.68 - y*f, z];
    }

    function drawGrid() {
      ctx.strokeStyle = "#2f3746";
      ctx.lineWidth = 1;
      for (let i=-5; i<=5; i++) {
        drawLine3([-0.5,i*0.1,0], [0.5,i*0.1,0], "#2b3340", 1);
        drawLine3([i*0.1,-0.5,0], [i*0.1,0.5,0], "#2b3340", 1);
      }
      drawLine3([0,0,0], [0.22,0,0], "#ff6b6b", 3);
      drawLine3([0,0,0], [0,0.22,0], "#51cf66", 3);
      drawLine3([0,0,0], [0,0,0.22], "#74c0fc", 3);
    }
    function drawLine3(a,b,color,width) {
      const pa=project(a), pb=project(b);
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(pa[0],pa[1]); ctx.lineTo(pb[0],pb[1]); ctx.stroke();
    }
    function drawSphere3(p, radius, color, label) {
      const pp=project(p);
      const r = Math.max(4, radius * zoom * 900 / (2.2 + pp[2]));
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(pp[0], pp[1], r, 0, Math.PI*2); ctx.fill();
      ctx.fillStyle = "#edf2ff"; ctx.font = "13px Segoe UI";
      ctx.fillText(label, pp[0] + r + 5, pp[1] - r - 2);
    }
    function draw() {
      resize();
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0,0,rect.width,rect.height);
      drawGrid();
      const pts = fkPoints();
      for (let i=0; i<pts.length-1; i++) {
        drawLine3(pts[i], pts[i+1], i >= 5 ? "#f2aaaa" : "#8fb0ff", 10);
      }
      for (let i=0; i<pts.length; i++) {
        drawSphere3(pts[i], 0.012, i === pts.length - 1 ? "#ff6b6b" : "#ffd43b", i === 0 ? "base" : `J${i}`);
      }
      const age = lastPacket ? (Date.now()/1000-lastPacket) : 0;
      overlay.textContent = `packet age: ${age.toFixed(2)}s\nview yaw=${yaw.toFixed(2)} pitch=${pitch.toFixed(2)} zoom=${zoom.toFixed(2)}`;
      requestAnimationFrame(draw);
    }

    canvas.addEventListener("mousedown", e => { dragging = true; lastMouse=[e.clientX,e.clientY]; });
    window.addEventListener("mouseup", () => dragging = false);
    window.addEventListener("mousemove", e => {
      if (!dragging) return;
      const dx=e.clientX-lastMouse[0], dy=e.clientY-lastMouse[1];
      yaw += dx*0.006; pitch = Math.max(-1.2, Math.min(1.2, pitch + dy*0.006));
      lastMouse=[e.clientX,e.clientY];
    });
    canvas.addEventListener("wheel", e => {
      e.preventDefault();
      zoom = Math.max(0.35, Math.min(3.0, zoom * (e.deltaY > 0 ? 0.92 : 1.08)));
    }, {passive:false});
    window.addEventListener("resize", resize);

    buildTable();
    connect();
    draw();
  </script>
</body>
</html>"""


if __name__ == "__main__":
    main()
