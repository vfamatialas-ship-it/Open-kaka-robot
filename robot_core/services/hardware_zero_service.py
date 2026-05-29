"""Hardware zero writing service.

硬件零点写入服务。

危险说明：
  - Damiao: 调用电机库 set_zero_position()，会写入电机零点。
  - Feetech STS3215: 写 Homing_Offset EEPROM，让当前位置变为目标中心 tick。

本服务不发送运动命令，但会写电机/舵机内部参数，必须由 GUI 或 CLI 做二次确认。
"""

from __future__ import annotations

from typing import Any

from robot_core.services.arm_read_service import ArmReadConnection
from robot_core.utils.feetech_sts_read_only import FEETECH_HEADER
from robot_core.utils.feetech_sts_read_only import PRESENT_POSITION_LEN
from robot_core.utils.feetech_sts_read_only import build_read_packet
from robot_core.utils.feetech_sts_read_only import checksum
from robot_core.utils.feetech_sts_read_only import parse_position_response
from robot_core.utils.feetech_sts_read_only import read_status_packet


INST_WRITE = 0x03
STS_HOMING_OFFSET_ADDR = 31
STS_HOMING_OFFSET_LEN = 2
STS_LOCK_ADDR = 55
STS_UNLOCK_VALUE = 0
STS_LOCK_VALUE = 1
STS_DEFAULT_ZERO_TARGET_TICKS = 2048
STS_HOMING_OFFSET_SIGN_BIT = 0x0800
STS_HOMING_OFFSET_MAG_MASK = 0x07FF


def write_hardware_zero(
    connection: ArmReadConnection,
    *,
    feetech_target_ticks: int = STS_DEFAULT_ZERO_TARGET_TICKS,
) -> list[str]:
    """Write hardware zero for the connected arm.

    对当前已连接机械臂写入硬件零点。
    """

    if connection.profile.kind == "damiao":
        return _write_damiao_zero(connection)
    return _write_feetech_homing_offsets(connection, feetech_target_ticks)


def _write_damiao_zero(connection: ArmReadConnection) -> list[str]:
    """Write zero position to Damiao motors."""

    if connection.controller is None:
        raise RuntimeError("Damiao controller is not open")

    lines = [f"Writing Damiao hardware zero: {connection.profile.label}"]
    for joint in connection.items:
        connection.controller.set_zero_position(joint.motor)
        lines.append(
            f"  {joint.config.name}: CAN ID=0x{joint.config.can_id:02X} zero command sent"
        )
    lines.append("Damiao zero write finished.")
    return lines


def _write_feetech_homing_offsets(
    connection: ArmReadConnection,
    target_ticks: int,
) -> list[str]:
    """Write STS3215 Homing_Offset values.

    The formula assumes Present_Position is already affected by the current
    Homing_Offset:

        new_offset = old_offset + (present_position - target_ticks)

    This makes the current pose read back close to target_ticks.
    """

    if connection.serial_device is None:
        raise RuntimeError("Feetech serial port is not open")
    if not 0 <= target_ticks <= 4095:
        raise ValueError("target_ticks must be in 0..4095")

    lines = [
        f"Writing Feetech Homing_Offset: {connection.profile.label}",
        f"Target current position after calibration: {target_ticks} ticks",
    ]
    for servo in connection.items:
        servo_id = servo.config.servo_id
        present_ticks = _read_feetech_position_ticks(connection.serial_device, servo_id)
        old_offset = _read_feetech_homing_offset(connection.serial_device, servo_id)
        new_offset = old_offset + (present_ticks - target_ticks)
        _validate_homing_offset(new_offset)

        _write_feetech_u8(connection.serial_device, servo_id, STS_LOCK_ADDR, STS_UNLOCK_VALUE)
        _write_feetech_homing_offset(connection.serial_device, servo_id, new_offset)
        _write_feetech_u8(connection.serial_device, servo_id, STS_LOCK_ADDR, STS_LOCK_VALUE)

        verify_offset = _read_feetech_homing_offset(connection.serial_device, servo_id)
        lines.append(
            f"  {servo.config.name}: id={servo_id} present={present_ticks} "
            f"old_offset={old_offset} new_offset={new_offset} verify={verify_offset}"
        )
    lines.append("Feetech Homing_Offset write finished. Power-cycle is recommended before final verification.")
    return lines


def _read_feetech_position_ticks(serial_device: Any, servo_id: int) -> int:
    packet = build_read_packet(servo_id, 56, PRESENT_POSITION_LEN)
    serial_device.reset_input_buffer()
    serial_device.write(packet)
    response = read_status_packet(serial_device)
    ticks, error = parse_position_response(response, servo_id)
    if error is not None or ticks is None:
        raise RuntimeError(f"id={servo_id} present position read failed: {error}")
    return ticks


def _read_feetech_homing_offset(serial_device: Any, servo_id: int) -> int:
    packet = build_read_packet(servo_id, STS_HOMING_OFFSET_ADDR, STS_HOMING_OFFSET_LEN)
    serial_device.reset_input_buffer()
    serial_device.write(packet)
    response = read_status_packet(serial_device)
    params = _parse_read_params(response, servo_id, expected_len=2)
    raw = params[0] | (params[1] << 8)
    return _decode_homing_offset(raw)


def _write_feetech_homing_offset(serial_device: Any, servo_id: int, offset: int) -> None:
    raw = _encode_homing_offset(offset)
    _write_feetech_bytes(
        serial_device,
        servo_id,
        STS_HOMING_OFFSET_ADDR,
        [raw & 0xFF, (raw >> 8) & 0xFF],
    )


def _write_feetech_u8(serial_device: Any, servo_id: int, address: int, value: int) -> None:
    _write_feetech_bytes(serial_device, servo_id, address, [value & 0xFF])


def _write_feetech_bytes(serial_device: Any, servo_id: int, address: int, data: list[int]) -> None:
    body = [servo_id, len(data) + 3, INST_WRITE, address, *data]
    packet = FEETECH_HEADER + bytes(body + [checksum(body)])
    serial_device.reset_input_buffer()
    serial_device.write(packet)
    response = read_status_packet(serial_device)
    _parse_write_ack(response, servo_id)


def _parse_read_params(packet: bytes, expected_id: int, expected_len: int) -> bytes:
    if len(packet) < 6 + expected_len:
        raise RuntimeError(f"response too short: {packet.hex(' ').upper()}")
    if packet[0:2] != FEETECH_HEADER:
        raise RuntimeError(f"bad header: {packet.hex(' ').upper()}")
    servo_id = packet[2]
    packet_length = packet[3]
    error_byte = packet[4]
    if servo_id != expected_id:
        raise RuntimeError(f"unexpected id: got {servo_id}, expected {expected_id}")
    if len(packet) != packet_length + 4:
        raise RuntimeError(f"bad length: got {len(packet)}, expected {packet_length + 4}")
    if checksum(packet[2:-1]) != packet[-1]:
        raise RuntimeError("checksum mismatch")
    if error_byte != 0:
        raise RuntimeError(f"servo returned error byte 0x{error_byte:02X}")
    params = packet[5:-1]
    if len(params) != expected_len:
        raise RuntimeError(f"bad param length: got {len(params)}, expected {expected_len}")
    return params


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


def _decode_homing_offset(raw: int) -> int:
    magnitude = raw & STS_HOMING_OFFSET_MAG_MASK
    if raw & STS_HOMING_OFFSET_SIGN_BIT:
        return -magnitude
    return magnitude


def _encode_homing_offset(offset: int) -> int:
    _validate_homing_offset(offset)
    if offset < 0:
        return STS_HOMING_OFFSET_SIGN_BIT | abs(offset)
    return offset


def _validate_homing_offset(offset: int) -> None:
    if not -STS_HOMING_OFFSET_MAG_MASK <= offset <= STS_HOMING_OFFSET_MAG_MASK:
        raise ValueError(
            f"Feetech Homing_Offset {offset} is outside supported "
            f"range {-STS_HOMING_OFFSET_MAG_MASK}..{STS_HOMING_OFFSET_MAG_MASK}"
        )
