"""Joint limit configuration service.

关节限位配置服务。

本服务分两层：
  1. 软件限位：保存每个关节的 min/max rad，用于后续所有运动命令检查。
  2. 硬件限位：尽可能写入电机/舵机内部参数。

硬件写入说明：
  - Feetech STS3215: 写 Min/Max Angle Limit，单位 tick。
  - Damiao: 目前达妙库可写 PMAX，这是围绕 0 的对称位置范围；它不能表达
    min/max 这种非对称软件关节限位，所以仍必须保留软件限位。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_core.services.arm_read_service import ArmReadConnection, ArmStatusSnapshot
from robot_core.utils.feetech_sts_read_only import FEETECH_HEADER, POSITION_TICKS_PER_TURN
from robot_core.utils.feetech_sts_read_only import checksum, read_status_packet


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LIMIT_DIR = PROJECT_ROOT / "configs" / "joint_limits"

INST_WRITE = 0x03
FEETECH_MIN_ANGLE_LIMIT_ADDR = 9
FEETECH_MAX_ANGLE_LIMIT_ADDR = 11
FEETECH_LOCK_ADDR = 55
FEETECH_UNLOCK_VALUE = 0
FEETECH_LOCK_VALUE = 1


@dataclass(frozen=True)
class JointLimit:
    """One joint software limit."""

    name: str
    min_rad: float
    max_rad: float


def limit_file_path(profile_key: str) -> Path:
    """Return limit config file path for one arm."""

    return LIMIT_DIR / f"{profile_key}_limits.yaml"


def default_limits_from_snapshot(
    snapshot: ArmStatusSnapshot,
    margin_rad: float = 0.5,
) -> list[JointLimit]:
    """Create conservative default limits around current positions."""

    limits: list[JointLimit] = []
    for joint in snapshot.joints:
        center = joint.position_rad if joint.position_rad is not None else 0.0
        limits.append(
            JointLimit(
                name=joint.name,
                min_rad=center - margin_rad,
                max_rad=center + margin_rad,
            )
        )
    return limits


def parse_limits_text(text: str) -> list[JointLimit]:
    """Parse simple limit text.

    Accepted line format:
      joint1 -1.0 1.0

    Lines starting with # are ignored.
    """

    limits: list[JointLimit] = []
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 3:
            raise ValueError(f"line {line_number}: expected '<joint> <min_rad> <max_rad>'")
        name, min_text, max_text = parts
        min_rad = float(min_text)
        max_rad = float(max_text)
        if min_rad >= max_rad:
            raise ValueError(f"line {line_number}: min_rad must be smaller than max_rad")
        limits.append(JointLimit(name=name, min_rad=min_rad, max_rad=max_rad))
    if not limits:
        raise ValueError("no joint limits found")
    return limits


def format_limits_text(limits: list[JointLimit]) -> str:
    """Format limits as editable text."""

    lines = [
        "# Joint limits, one line per joint:",
        "# <joint_name> <min_rad> <max_rad>",
    ]
    for limit in limits:
        lines.append(f"{limit.name} {limit.min_rad:.6f} {limit.max_rad:.6f}")
    lines.append("")
    return "\n".join(lines)


def load_limits_text(profile_key: str) -> str:
    """Load saved joint limit file as text."""

    path = limit_file_path(profile_key)
    if not path.exists():
        return f"# 尚未保存限位文件：{path}\n"
    return path.read_text(encoding="utf-8")


def save_limits_text(profile_key: str, text: str) -> Path:
    """Validate and save software joint limits."""

    limits = parse_limits_text(text)
    LIMIT_DIR.mkdir(parents=True, exist_ok=True)
    path = limit_file_path(profile_key)
    path.write_text(format_limits_text(limits), encoding="utf-8")
    return path


def check_snapshot_against_limits(
    snapshot: ArmStatusSnapshot,
    limits: list[JointLimit],
) -> list[str]:
    """Check current arm state against configured software limits."""

    limit_map = {limit.name: limit for limit in limits}
    lines: list[str] = []
    for joint in snapshot.joints:
        limit = limit_map.get(joint.name)
        if limit is None:
            lines.append(f"! {joint.name}: missing limit")
            continue
        if joint.position_rad is None:
            lines.append(f"! {joint.name}: no position, error={joint.error}")
            continue
        if limit.min_rad <= joint.position_rad <= limit.max_rad:
            lines.append(
                f"OK {joint.name}: {joint.position_rad:.4f} in "
                f"[{limit.min_rad:.4f}, {limit.max_rad:.4f}]"
            )
        else:
            lines.append(
                f"OUT {joint.name}: {joint.position_rad:.4f} outside "
                f"[{limit.min_rad:.4f}, {limit.max_rad:.4f}]"
            )
    return lines


def write_hardware_limits(
    connection: ArmReadConnection,
    limits: list[JointLimit],
) -> list[str]:
    """Write hardware limits when supported."""

    if connection.profile.kind == "feetech":
        return _write_feetech_angle_limits(connection, limits)
    return _write_damiao_pmax_limits(connection, limits)


def rad_to_feetech_tick(rad: float) -> int:
    """Convert rad to STS3215 absolute position tick."""

    wrapped = rad % (2.0 * math.pi)
    tick = round((wrapped / (2.0 * math.pi)) * POSITION_TICKS_PER_TURN)
    return max(0, min(POSITION_TICKS_PER_TURN - 1, tick))


def _write_feetech_angle_limits(
    connection: ArmReadConnection,
    limits: list[JointLimit],
) -> list[str]:
    """Write Feetech Min/Max Angle Limit registers."""

    if connection.serial_device is None:
        raise RuntimeError("Feetech serial port is not open")

    limit_map = {limit.name: limit for limit in limits}
    lines = [f"Writing Feetech angle limits: {connection.profile.label}"]
    for servo in connection.items:
        limit = limit_map.get(servo.config.name)
        if limit is None:
            raise ValueError(f"missing limit for {servo.config.name}")

        min_tick = rad_to_feetech_tick(limit.min_rad)
        max_tick = rad_to_feetech_tick(limit.max_rad)
        if min_tick >= max_tick:
            raise ValueError(
                f"{servo.config.name}: Feetech hardware min_tick must be < max_tick. "
                "For ranges crossing 0 rad, keep software limits only or choose an unwrapped range."
            )

        _write_feetech_u8(connection.serial_device, servo.config.servo_id, FEETECH_LOCK_ADDR, FEETECH_UNLOCK_VALUE)
        _write_feetech_u16(connection.serial_device, servo.config.servo_id, FEETECH_MIN_ANGLE_LIMIT_ADDR, min_tick)
        _write_feetech_u16(connection.serial_device, servo.config.servo_id, FEETECH_MAX_ANGLE_LIMIT_ADDR, max_tick)
        _write_feetech_u8(connection.serial_device, servo.config.servo_id, FEETECH_LOCK_ADDR, FEETECH_LOCK_VALUE)

        lines.append(
            f"  {servo.config.name}: id={servo.config.servo_id} "
            f"min={limit.min_rad:.4f}rad({min_tick}) max={limit.max_rad:.4f}rad({max_tick})"
        )
    lines.append("Feetech hardware angle limit write finished.")
    return lines


def _write_damiao_pmax_limits(
    connection: ArmReadConnection,
    limits: list[JointLimit],
) -> list[str]:
    """Write Damiao PMAX based on software limits.

    PMAX is symmetric and cannot encode separate min/max limits.
    """

    if connection.controller is None:
        raise RuntimeError("Damiao controller is not open")

    from DM_CAN import DM_variable

    limit_map = {limit.name: limit for limit in limits}
    lines = [
        f"Writing Damiao PMAX limits: {connection.profile.label}",
        "Note: PMAX is symmetric; exact min/max remain software limits.",
    ]
    for joint in connection.items:
        limit = limit_map.get(joint.config.name)
        if limit is None:
            raise ValueError(f"missing limit for {joint.config.name}")
        pmax = max(abs(limit.min_rad), abs(limit.max_rad))
        ok = connection.controller.change_motor_param(joint.motor, DM_variable.PMAX, float(pmax))
        lines.append(
            f"  {joint.config.name}: CAN ID=0x{joint.config.can_id:02X} "
            f"PMAX={pmax:.4f} write={'OK' if ok else 'FAILED'}"
        )
    lines.append("Damiao PMAX write finished. Exact min/max are enforced in software.")
    return lines


def _write_feetech_u8(serial_device: Any, servo_id: int, address: int, value: int) -> None:
    _write_feetech_bytes(serial_device, servo_id, address, [value & 0xFF])


def _write_feetech_u16(serial_device: Any, servo_id: int, address: int, value: int) -> None:
    _write_feetech_bytes(serial_device, servo_id, address, [value & 0xFF, (value >> 8) & 0xFF])


def _write_feetech_bytes(serial_device: Any, servo_id: int, address: int, data: list[int]) -> None:
    body = [servo_id, len(data) + 3, INST_WRITE, address, *data]
    packet = FEETECH_HEADER + bytes(body + [checksum(body)])
    serial_device.reset_input_buffer()
    serial_device.write(packet)
    response = read_status_packet(serial_device)
    _parse_write_ack(response, servo_id)


def _parse_write_ack(packet: bytes, expected_id: int) -> None:
    if len(packet) < 6:
        raise RuntimeError(f"write ack too short: {packet.hex(' ').upper()}")
    if packet[0:2] != FEETECH_HEADER:
        raise RuntimeError(f"bad ack header: {packet.hex(' ').upper()}")
    servo_id = packet[2]
    packet_length = packet[3]
    error_byte = packet[4]
    if servo_id != expected_id:
        raise RuntimeError(f"unexpected ack id: got {servo_id}, expected {expected_id}")
    if len(packet) != packet_length + 4:
        raise RuntimeError(f"bad ack length: got {len(packet)}, expected {packet_length + 4}")
    if checksum(packet[2:-1]) != packet[-1]:
        raise RuntimeError("ack checksum mismatch")
    if error_byte != 0:
        raise RuntimeError(f"servo write error byte 0x{error_byte:02X}")
