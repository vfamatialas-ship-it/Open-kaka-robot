"""Single-joint low-speed motion service.

单关节低速运动测试服务。

安全原则：
  - 每次只允许控制一个关节。
  - 运动前先读取当前位置 q_now。
  - 目标位置 q_des 必须通过软件关节限位检查。
  - “当前位置保持”不是回零，只是 q_des = q_now。

说明：
  - Feetech STS3215: 写目标位置和较低目标速度。
  - Damiao: 对选中电机发送 MIT 位置保持/小步进命令。该命令是显式运动命令，
    因此会对选中电机发送 enable；启动和读取状态仍然不会自动使能。
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Any

from robot_core.services.arm_read_service import ArmReadConnection, ArmStatusSnapshot, JointStatus
from robot_core.services.joint_limit_service import JointLimit
from robot_core.utils.feetech_sts_read_only import FEETECH_HEADER
from robot_core.utils.feetech_sts_read_only import POSITION_TICKS_PER_TURN
from robot_core.utils.feetech_sts_read_only import checksum, read_status_packet


INST_WRITE = 0x03

FEETECH_GOAL_POSITION_ADDR = 42
FEETECH_GOAL_SPEED_ADDR = 46
FEETECH_TORQUE_ENABLE_ADDR = 40
FEETECH_DEFAULT_SPEED_TICKS = 80

DAMIAO_DEFAULT_KP = 8.0
DAMIAO_DEFAULT_KD = 0.8


@dataclass(frozen=True)
class SingleJointMotionResult:
    """Result returned after one single-joint command."""

    snapshot: ArmStatusSnapshot
    joint_name: str
    current_rad: float
    target_rad: float
    lines: tuple[str, ...]


def hold_current_position(
    connection: ArmReadConnection,
    joint_name: str,
    limits: list[JointLimit],
) -> SingleJointMotionResult:
    """Read q_now and command q_des = q_now for one joint.

    读取当前位置，并把同一个位置作为目标位置下发，形成当前位置保持。
    """

    snapshot, joint = _read_joint_or_raise(connection, joint_name)
    target_rad = joint.position_rad
    assert target_rad is not None
    _check_target_against_limits(joint_name, target_rad, limits)
    lines = _send_single_joint_target(connection, joint_name, target_rad)
    return SingleJointMotionResult(
        snapshot=snapshot,
        joint_name=joint_name,
        current_rad=target_rad,
        target_rad=target_rad,
        lines=(
            f"{joint_name}: hold current position / 当前位置保持",
            f"  q_now={target_rad:.6f} rad, q_des={target_rad:.6f} rad",
            *lines,
        ),
    )


def step_single_joint(
    connection: ArmReadConnection,
    joint_name: str,
    delta_rad: float,
    limits: list[JointLimit],
) -> SingleJointMotionResult:
    """Read q_now, add a small delta, check limits, then command one joint."""

    snapshot, joint = _read_joint_or_raise(connection, joint_name)
    current_rad = joint.position_rad
    assert current_rad is not None
    target_rad = current_rad + delta_rad
    _check_target_against_limits(joint_name, target_rad, limits)
    lines = _send_single_joint_target(connection, joint_name, target_rad)
    return SingleJointMotionResult(
        snapshot=snapshot,
        joint_name=joint_name,
        current_rad=current_rad,
        target_rad=target_rad,
        lines=(
            f"{joint_name}: single-joint step / 单关节小步进",
            f"  q_now={current_rad:.6f} rad, delta={delta_rad:+.6f} rad, q_des={target_rad:.6f} rad",
            *lines,
        ),
    )


def command_single_joint_target(
    connection: ArmReadConnection,
    joint_name: str,
    target_rad: float,
    limits: list[JointLimit],
    *,
    enable_motor: bool = True,
    kp: float | None = None,
    kd: float | None = None,
    tau: float = 0.0,
    control_mode: str = "mit",
    posvel_velocity: float = 1.0,
    enable_old_mode: bool = False,
    switch_mode: bool = False,
) -> tuple[str, ...]:
    """Command one joint to an explicit target after limit checking.

    对单个关节下发明确目标位置。调用者必须已经完成状态机确认。
    """

    _check_target_against_limits(joint_name, target_rad, limits)
    return _send_single_joint_target(
        connection,
        joint_name,
        target_rad,
        enable_motor=enable_motor,
        kp=kp,
        kd=kd,
        tau=tau,
        control_mode=control_mode,
        posvel_velocity=posvel_velocity,
        enable_old_mode=enable_old_mode,
        switch_mode=switch_mode,
    )


def disable_arm_motion(connection: ArmReadConnection) -> tuple[str, ...]:
    """Best-effort hardware disable used by emergency stop.

    急停时尽力让当前连接的硬件失能。它不替代物理急停开关，但可以让软件侧
    已经使能的达妙电机或飞特舵机停止保持/运动。
    """

    if connection.profile.kind == "damiao":
        if connection.controller is None:
            raise RuntimeError("Damiao controller is not open")
        lines = [f"Emergency disable Damiao arm: {connection.profile.label}"]
        for joint in connection.items:
            connection.controller.disable(joint.motor)
            lines.append(f"  {joint.config.name}: CAN ID=0x{joint.config.can_id:02X} disable sent")
        return tuple(lines)

    if connection.serial_device is None:
        raise RuntimeError("Feetech serial port is not open")
    lines = [f"Emergency torque-off Feetech arm: {connection.profile.label}"]
    for servo in connection.items:
        _write_feetech_u8(connection.serial_device, servo.config.servo_id, FEETECH_TORQUE_ENABLE_ADDR, 0)
        lines.append(f"  {servo.config.name}: id={servo.config.servo_id} torque enable=0")
    return tuple(lines)


def _read_joint_or_raise(
    connection: ArmReadConnection,
    joint_name: str,
) -> tuple[ArmStatusSnapshot, JointStatus]:
    snapshot = connection.read_snapshot()
    joint = next((item for item in snapshot.joints if item.name == joint_name), None)
    if joint is None:
        raise RuntimeError(f"{joint_name}: joint not found in current arm profile")
    if joint.position_rad is None:
        raise RuntimeError(f"{joint_name}: no valid current position, error={joint.error}")
    return snapshot, joint


def _check_target_against_limits(
    joint_name: str,
    target_rad: float,
    limits: list[JointLimit],
) -> None:
    limit = next((item for item in limits if item.name == joint_name), None)
    if limit is None:
        raise RuntimeError(f"{joint_name}: missing software joint limit; please teach/save limits first")
    if not limit.min_rad <= target_rad <= limit.max_rad:
        raise RuntimeError(
            f"{joint_name}: target {target_rad:.6f} rad outside software limit "
            f"[{limit.min_rad:.6f}, {limit.max_rad:.6f}]"
        )


def _send_single_joint_target(
    connection: ArmReadConnection,
    joint_name: str,
    target_rad: float,
    *,
    enable_motor: bool = True,
    kp: float | None = None,
    kd: float | None = None,
    tau: float = 0.0,
    control_mode: str = "mit",
    posvel_velocity: float = 1.0,
    enable_old_mode: bool = False,
    switch_mode: bool = False,
) -> tuple[str, ...]:
    if connection.profile.kind == "feetech":
        return _send_feetech_target(connection, joint_name, target_rad)
    return _send_damiao_target(
        connection,
        joint_name,
        target_rad,
        enable_motor=enable_motor,
        kp=kp,
        kd=kd,
        tau=tau,
        control_mode=control_mode,
        posvel_velocity=posvel_velocity,
        enable_old_mode=enable_old_mode,
        switch_mode=switch_mode,
    )


def _send_feetech_target(
    connection: ArmReadConnection,
    joint_name: str,
    target_rad: float,
) -> tuple[str, ...]:
    if connection.serial_device is None:
        raise RuntimeError("Feetech serial port is not open")

    servo = _find_runtime_item(connection, joint_name)
    servo_id = servo.config.servo_id
    target_ticks = _rad_to_feetech_ticks(
        target_rad,
        direction=servo.config.direction,
        zero_offset_rad=servo.config.zero_offset_rad,
    )

    # 先写较低目标速度，再写目标位置。
    # STS 系列常用寄存器：Goal Position=42, Goal Speed=46。
    _write_feetech_u16(connection.serial_device, servo_id, FEETECH_GOAL_SPEED_ADDR, FEETECH_DEFAULT_SPEED_TICKS)
    _write_feetech_u16(connection.serial_device, servo_id, FEETECH_GOAL_POSITION_ADDR, target_ticks)
    return (
        f"  Feetech id={servo_id}: goal={target_rad:.6f} rad ({target_ticks} ticks)",
        f"  Feetech speed register set to {FEETECH_DEFAULT_SPEED_TICKS} ticks",
    )


def _send_damiao_target(
    connection: ArmReadConnection,
    joint_name: str,
    target_rad: float,
    *,
    enable_motor: bool,
    kp: float | None,
    kd: float | None,
    tau: float,
    control_mode: str,
    posvel_velocity: float,
    enable_old_mode: bool,
    switch_mode: bool,
) -> tuple[str, ...]:
    if connection.controller is None:
        raise RuntimeError("Damiao controller is not open")

    joint = _find_runtime_item(connection, joint_name)

    # 这是显式运动测试按钮触发的使能，不是启动自动使能。
    # 遥操作进入 ACTIVE 之前会先保持当前位置，因此 ACTIVE 循环可以跳过重复 enable。
    if enable_motor:
        if switch_mode:
            ok = connection.controller.switchControlMode(joint.motor, _damiao_control_type(control_mode))
            if not ok:
                raise RuntimeError(f"{joint_name}: switchControlMode({control_mode}) failed")
        if enable_old_mode:
            connection.controller.enable_old(joint.motor, _damiao_control_type(control_mode))
        else:
            connection.controller.enable(joint.motor)
        time.sleep(0.02)
    if control_mode == "posvel":
        connection.controller.control_Pos_Vel(joint.motor, target_rad, posvel_velocity)
        return (
            f"  Damiao CAN ID=0x{joint.config.can_id:02X}: posvel goal={target_rad:.6f} rad",
            f"  PosVel velocity={posvel_velocity:.3f} rad/s",
        )

    kp_value = DAMIAO_DEFAULT_KP if kp is None else kp
    kd_value = DAMIAO_DEFAULT_KD if kd is None else kd
    connection.controller.controlMIT(
        joint.motor,
        kp_value,
        kd_value,
        target_rad,
        0.0,
        tau,
    )
    try:
        connection.controller.recv()
    except Exception:
        # 有些 USB2CAN 转接器不会每次立刻返回状态帧，运动命令本身已经发出。
        pass
    return (
        f"  Damiao CAN ID=0x{joint.config.can_id:02X}: goal={target_rad:.6f} rad",
        f"  MIT gains: kp={kp_value:.2f}, kd={kd_value:.2f}, tau={tau:.3f}",
    )


def _damiao_control_type(control_mode: str) -> Any:
    dm_can = sys.modules.get("DM_CAN")
    control_type = getattr(dm_can, "Control_Type", None) if dm_can is not None else None
    if control_type is None:
        raise RuntimeError("DM_CAN.Control_Type is not available")
    if control_mode == "posvel":
        return control_type.POS_VEL
    return control_type.MIT


def _find_runtime_item(connection: ArmReadConnection, joint_name: str) -> Any:
    for item in connection.items:
        if item.config.name == joint_name:
            return item
    raise RuntimeError(f"{joint_name}: runtime item not found")


def _rad_to_feetech_ticks(
    position_rad: float,
    *,
    direction: int,
    zero_offset_rad: float,
) -> int:
    raw_rad = (position_rad - zero_offset_rad) / direction
    ticks = round((raw_rad % (2.0 * math.pi)) / (2.0 * math.pi) * POSITION_TICKS_PER_TURN)
    return max(0, min(POSITION_TICKS_PER_TURN - 1, ticks))


def _write_feetech_u16(serial_device: Any, servo_id: int, address: int, value: int) -> None:
    value = max(0, min(0xFFFF, int(value)))
    _write_feetech_bytes(serial_device, servo_id, address, [value & 0xFF, (value >> 8) & 0xFF])


def _write_feetech_u8(serial_device: Any, servo_id: int, address: int, value: int) -> None:
    _write_feetech_bytes(serial_device, servo_id, address, [value & 0xFF])


def _write_feetech_bytes(serial_device: Any, servo_id: int, address: int, data: list[int]) -> None:
    body = [servo_id, len(data) + 3, INST_WRITE, address, *data]
    packet = FEETECH_HEADER + bytes(body + [checksum(body)])
    serial_device.reset_input_buffer()
    serial_device.write(packet)
    response = read_status_packet(serial_device)
    _parse_feetech_write_ack(response, servo_id)


def _parse_feetech_write_ack(packet: bytes, expected_id: int) -> None:
    if len(packet) < 6:
        raise RuntimeError(f"Feetech write ack too short: {packet.hex(' ').upper()}")
    if packet[0:2] != FEETECH_HEADER:
        raise RuntimeError(f"bad Feetech ack header: {packet.hex(' ').upper()}")
    servo_id = packet[2]
    packet_length = packet[3]
    error_byte = packet[4]
    if servo_id != expected_id:
        raise RuntimeError(f"unexpected Feetech ack id: got {servo_id}, expected {expected_id}")
    if len(packet) != packet_length + 4:
        raise RuntimeError(f"bad Feetech ack length: got {len(packet)}, expected {packet_length + 4}")
    if checksum(packet[2:-1]) != packet[-1]:
        raise RuntimeError("Feetech ack checksum mismatch")
    if error_byte != 0:
        raise RuntimeError(f"Feetech write error byte 0x{error_byte:02X}")
