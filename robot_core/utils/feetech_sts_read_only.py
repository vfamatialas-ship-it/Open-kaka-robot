"""Read-only helpers for Feetech STS3215 servos.

飞特 STS3215 舵机只读状态读取工具。

本模块只实现读取当前位置，不包含写寄存器、使能、回零或运动逻辑。
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Iterable


# 飞特 STS/SMS 系列常用协议：
#   Header:      0xFF 0xFF
#   Packet:      ID LENGTH INSTRUCTION PARAMS... CHECKSUM
#   Read command INSTRUCTION = 0x02
#
# STS3215 常用当前位置寄存器：
#   Present Position Low = 56
#   Length = 2 bytes
#
# 如果你的飞特 SDK 文档中地址不同，只需要改下面这两个常量。
FEETECH_HEADER = bytes([0xFF, 0xFF])
INST_READ = 0x02
PRESENT_POSITION_ADDR = 56
PRESENT_POSITION_LEN = 2
POSITION_TICKS_PER_TURN = 4096


@dataclass(frozen=True)
class ServoConfig:
    """单个主臂舵机配置。"""

    name: str
    servo_id: int
    direction: int = 1
    zero_offset_rad: float = 0.0


@dataclass
class ServoRuntime:
    """单个舵机运行时状态。"""

    config: ServoConfig
    last_position_rad: float | None = None


def parse_ids(value: str) -> list[int]:
    """解析命令行里的 ID 列表，例如：1,2,3 或 0x01,0x02。"""

    ids: list[int] = []
    for item in value.split(","):
        item = item.strip()
        if item:
            ids.append(int(item, 0))
    if not ids:
        raise argparse.ArgumentTypeError("ID list cannot be empty")
    return ids


def build_parser(default_port: str, default_ids: list[int]) -> argparse.ArgumentParser:
    """创建命令行参数。"""

    parser = argparse.ArgumentParser(description="读取飞特 STS3215 主臂舵机位置。")
    ids_text = ",".join(str(servo_id) for servo_id in default_ids)
    parser.add_argument("--port", default=default_port, help=f"串口号，默认 {default_port}")
    parser.add_argument("--baudrate", type=int, default=1000000, help="串口波特率")
    parser.add_argument("--timeout", type=float, default=0.1, help="串口超时，单位秒")
    parser.add_argument(
        "--ids",
        type=parse_ids,
        default=default_ids,
        help=f"舵机 ID 列表，例如 {ids_text}",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="连续读取时每轮间隔，单位秒",
    )
    parser.add_argument("--once", action="store_true", help="只读取一轮然后退出")
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="读取固定轮数；0 表示一直循环，按 Ctrl+C 停止",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="同时打印原始 tick 值",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="打印发送包和原始回复，便于排查串口/波特率/接线问题",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="列出当前电脑上的串口后退出",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="扫描舵机 ID，只发送只读位置读取请求",
    )
    parser.add_argument(
        "--scan-start",
        type=lambda value: int(value, 0),
        default=1,
        help="扫描起始 ID，默认 1",
    )
    parser.add_argument(
        "--scan-end",
        type=lambda value: int(value, 0),
        default=20,
        help="扫描结束 ID，默认 20",
    )
    return parser


def checksum(packet_without_header_and_checksum: Iterable[int]) -> int:
    """计算飞特协议校验和。"""

    return (~sum(packet_without_header_and_checksum)) & 0xFF


def build_read_packet(servo_id: int, address: int, length: int) -> bytes:
    """构造读取寄存器数据包。"""

    packet_body = [
        servo_id,
        4,  # LENGTH = instruction + address + read length + checksum
        INST_READ,
        address,
        length,
    ]
    return FEETECH_HEADER + bytes(packet_body + [checksum(packet_body)])


def load_serial_module():
    """延迟导入 pyserial，让 --help 在未安装依赖时也能显示。"""

    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "缺少依赖 pyserial。\n"
            "请先在当前 Python 环境中安装：\n"
            '  python -m pip install "pyserial>=3.5"'
        ) from exc
    return serial


def list_serial_ports(serial_module) -> None:
    """列出串口，帮助确认主臂实际连接在哪个 COM。"""

    try:
        ports = list(serial_module.tools.list_ports.comports())
    except AttributeError:
        from serial.tools import list_ports

        ports = list(list_ports.comports())

    if not ports:
        print("No serial ports found / 没有发现串口")
        return

    print("Serial ports / 串口列表:")
    for port in ports:
        print(f"  {port.device}: {port.description}")


def bytes_to_hex(data: bytes) -> str:
    """把字节转成十六进制字符串。"""

    return " ".join(f"{byte:02X}" for byte in data)


def read_exact_or_timeout(serial_device, size: int) -> bytes:
    """读取指定长度；超时则返回已读到的数据。"""

    data = bytearray()
    deadline = time.monotonic() + float(serial_device.timeout or 0.1)
    while len(data) < size and time.monotonic() < deadline:
        chunk = serial_device.read(size - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def read_status_packet(serial_device) -> bytes:
    """读取一个飞特状态包。

    返回完整包：FF FF ID LENGTH ERROR PARAMS... CHECKSUM
    """

    # 先同步到 0xFF 0xFF 帧头，避免串口残留字节影响解析。
    header = bytearray()
    deadline = time.monotonic() + float(serial_device.timeout or 0.1)
    while time.monotonic() < deadline:
        byte = serial_device.read(1)
        if not byte:
            continue
        header.append(byte[0])
        header = header[-2:]
        if bytes(header) == FEETECH_HEADER:
            break
    else:
        return b""

    fixed = read_exact_or_timeout(serial_device, 3)
    if len(fixed) != 3:
        return FEETECH_HEADER + fixed

    servo_id = fixed[0]
    packet_length = fixed[1]
    # fixed[2] 是 ERROR 字节。
    remaining = read_exact_or_timeout(serial_device, packet_length - 1)
    return FEETECH_HEADER + bytes([servo_id, packet_length, fixed[2]]) + remaining


def parse_position_response(packet: bytes, expected_id: int) -> tuple[int | None, str | None]:
    """解析当前位置回复。

    返回：
      position_ticks: 原始位置 tick；失败时为 None
      error_message: 错误说明；成功时为 None
    """

    if len(packet) < 8:
        return None, f"response too short: {packet.hex(' ').upper()}"
    if packet[0:2] != FEETECH_HEADER:
        return None, f"bad header: {packet.hex(' ').upper()}"

    servo_id = packet[2]
    packet_length = packet[3]
    error_byte = packet[4]

    expected_total_len = packet_length + 4
    if len(packet) != expected_total_len:
        return None, f"bad length: got {len(packet)}, expected {expected_total_len}"
    if servo_id != expected_id:
        return None, f"unexpected id: got {servo_id}, expected {expected_id}"

    body = packet[2:-1]
    received_checksum = packet[-1]
    if checksum(body) != received_checksum:
        return None, "checksum mismatch"
    if error_byte != 0:
        return None, f"servo returned error byte 0x{error_byte:02X}"

    params = packet[5:-1]
    if len(params) < 2:
        return None, "missing position bytes"

    position_ticks = params[0] | (params[1] << 8)
    return position_ticks, None


def ticks_to_rad(ticks: int, direction: int, zero_offset_rad: float) -> float:
    """将 0~4095 原始位置转换成弧度。"""

    raw_rad = (ticks / POSITION_TICKS_PER_TURN) * 2.0 * math.pi
    return direction * raw_rad + zero_offset_rad


def read_servo_position(
    serial_device,
    servo: ServoRuntime,
    *,
    debug: bool = False,
) -> tuple[int | None, float | None, str | None]:
    """读取单个舵机当前位置。"""

    serial_device.reset_input_buffer()
    packet = build_read_packet(
        servo.config.servo_id,
        PRESENT_POSITION_ADDR,
        PRESENT_POSITION_LEN,
    )
    if debug:
        print(f"    TX id={servo.config.servo_id}: {bytes_to_hex(packet)}")
    serial_device.write(packet)

    response = read_status_packet(serial_device)
    if debug:
        print(f"    RX id={servo.config.servo_id}: {bytes_to_hex(response)}")
    ticks, error = parse_position_response(response, servo.config.servo_id)
    if error is not None or ticks is None:
        return None, None, error

    position_rad = ticks_to_rad(
        ticks,
        servo.config.direction,
        servo.config.zero_offset_rad,
    )
    return ticks, position_rad, None


def create_servos(ids: list[int]) -> list[ServoRuntime]:
    """根据 ID 列表创建舵机运行对象。"""

    return [
        ServoRuntime(config=ServoConfig(name=f"joint{index}", servo_id=servo_id))
        for index, servo_id in enumerate(ids, start=1)
    ]


def print_startup(
    title: str,
    port: str,
    baudrate: int,
    servos: list[ServoRuntime],
) -> None:
    """打印启动信息，方便人工确认。"""

    print(title)
    print(f"Serial: {port}, baudrate: {baudrate}")
    print("Safety: read-only; no enable; no motion; no zero save")
    print()
    print("Servo table / 舵机表:")
    for servo in servos:
        print(f"  {servo.config.name}: servo_id={servo.config.servo_id}")
    print()


def read_all_once(
    serial_device,
    servos: list[ServoRuntime],
    *,
    show_raw: bool,
    debug: bool,
) -> None:
    """读取并打印所有舵机位置一次。"""

    print(f"[{time.strftime('%H:%M:%S')}]")
    for servo in servos:
        ticks, position_rad, error = read_servo_position(
            serial_device,
            servo,
            debug=debug,
        )
        if error is not None or position_rad is None:
            print(f"! {servo.config.name:<6} id={servo.config.servo_id:<3} ERROR: {error}")
            continue

        changed = (
            servo.last_position_rad is None
            or position_rad != servo.last_position_rad
        )
        servo.last_position_rad = position_rad
        marker = "*" if changed else " "
        raw_text = f" ticks={ticks:>4}" if show_raw else ""
        print(
            f"{marker} {servo.config.name:<6} "
            f"id={servo.config.servo_id:<3} "
            f"pos={position_rad:>9.4f} rad"
            f"{raw_text}"
        )
    print()


def scan_servo_ids(
    serial_device,
    *,
    start_id: int,
    end_id: int,
    debug: bool,
) -> None:
    """扫描舵机 ID。

    只发送读取当前位置的只读请求，不写寄存器，不运动。
    """

    if start_id < 0 or end_id > 253 or start_id > end_id:
        raise SystemExit("扫描范围不合法。示例：--scan-start 1 --scan-end 20")

    print(f"Scanning Feetech IDs {start_id}..{end_id} / 正在扫描飞特 ID")
    print("Only read-position requests are sent / 只发送读取位置请求")
    print()

    found = 0
    for servo_id in range(start_id, end_id + 1):
        servo = ServoRuntime(config=ServoConfig(name=f"id{servo_id}", servo_id=servo_id))
        ticks, position_rad, error = read_servo_position(
            serial_device,
            servo,
            debug=debug,
        )
        if error is not None or ticks is None or position_rad is None:
            continue

        found += 1
        print(f"  id={servo_id:<3} pos={position_rad:>9.4f} rad ticks={ticks}")

    if found == 0:
        print("No servos responded / 没有任何舵机回复")


def run_master_read(
    *,
    title: str = "Feetech STS3215 master-arm read-only monitor / 飞特主臂只读监视器",
    default_ids: list[int] | None = None,
    default_port: str = "COM8",
) -> None:
    """运行飞特主臂只读读取脚本。"""

    if default_ids is None:
        default_ids = [8, 9, 10, 11, 12, 13, 14]

    args = build_parser(default_port, default_ids).parse_args()
    serial = load_serial_module()

    if args.list_ports:
        list_serial_ports(serial)
        return

    servos = create_servos(args.ids)
    print_startup(title, args.port, args.baudrate, servos)

    serial_device = serial.Serial(args.port, args.baudrate, timeout=args.timeout)
    try:
        if args.scan:
            scan_servo_ids(
                serial_device,
                start_id=args.scan_start,
                end_id=args.scan_end,
                debug=args.debug,
            )
            return

        cycle = 0
        while True:
            cycle += 1
            read_all_once(serial_device, servos, show_raw=args.raw, debug=args.debug)

            if args.once or (args.cycles > 0 and cycle >= args.cycles):
                break

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("Stopped by user / 用户停止。")
    finally:
        if serial_device.is_open:
            serial_device.close()
