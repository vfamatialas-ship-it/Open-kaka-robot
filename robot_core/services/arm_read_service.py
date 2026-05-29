"""Read-only arm status service.

This module is the shared service layer for arm status reading. GUI buttons and
command-line scripts should call this service instead of duplicating hardware
read logic.

本模块是机械臂状态读取的共享服务层。上位机按钮和命令行脚本都应调用这里，
不要在界面代码里重复写硬件读取逻辑。
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.utils.damiao_read_only import JointConfig as DamiaoJointConfig
from robot_core.utils.damiao_read_only import create_joints as create_damiao_joints
from robot_core.utils.damiao_read_only import load_damiao_modules
from robot_core.utils.damiao_read_only import read_joint as read_damiao_joint
from robot_core.utils.feetech_sts_read_only import ServoConfig, ServoRuntime
from robot_core.utils.feetech_sts_read_only import load_serial_module
from robot_core.utils.feetech_sts_read_only import read_servo_position


ArmKind = Literal["feetech", "damiao"]


@dataclass(frozen=True)
class ArmProfile:
    """Read-only connection profile for one arm.

    一个机械臂的只读连接配置。
    """

    key: str
    label: str
    kind: ArmKind
    default_port: str
    default_baudrate: int
    ids: tuple[int, ...] = ()
    damiao_joints: tuple[DamiaoJointConfig, ...] = ()


@dataclass(frozen=True)
class JointStatus:
    """One joint status sample returned by the read service."""

    name: str
    device_id: int
    position_rad: float | None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    ticks: int | None = None
    error: str | None = None


@dataclass(frozen=True)
class ArmStatusSnapshot:
    """One arm status sample."""

    timestamp: str
    profile_label: str
    joints: tuple[JointStatus, ...]

    def to_lines(self) -> list[str]:
        """Format the snapshot as printable text lines."""

        lines = [f"[{self.timestamp}] {self.profile_label}"]
        for joint in self.joints:
            if joint.error is not None:
                lines.append(f"! {joint.name:<6} id={joint.device_id:<3} ERROR: {joint.error}")
                continue

            if joint.velocity_rad_s is None:
                lines.append(
                    f"  {joint.name:<6} id={joint.device_id:<3} "
                    f"pos={joint.position_rad:>8.4f} rad  ticks={joint.ticks:>4}"
                )
                continue

            lines.append(
                f"  {joint.name:<6} id=0x{joint.device_id:02X} "
                f"pos={joint.position_rad:>8.4f} rad  "
                f"vel={joint.velocity_rad_s:>8.4f} rad/s  "
                f"tau={joint.torque_nm:>8.4f} Nm"
            )
        return lines


ARM_PROFILES: dict[str, ArmProfile] = {
    "pink_master": ArmProfile(
        key="pink_master",
        label="粉色主臂 / Pink Master",
        kind="feetech",
        default_port="COM8",
        default_baudrate=1000000,
        ids=(8, 9, 10, 11, 12, 13, 14),
    ),
    "gray_master": ArmProfile(
        key="gray_master",
        label="灰色主臂 / Gray Master",
        kind="feetech",
        default_port="COM8",
        default_baudrate=1000000,
        ids=(1, 2, 3, 4, 5, 6, 7),
    ),
    "pink_slave": ArmProfile(
        key="pink_slave",
        label="粉色从臂 / Pink Slave",
        kind="damiao",
        default_port="COM7",
        default_baudrate=921600,
        damiao_joints=(
            DamiaoJointConfig("joint1", 0x21, 0x31, "DM4340"),
            DamiaoJointConfig("joint2", 0x22, 0x32, "DM4340"),
            DamiaoJointConfig("joint3", 0x23, 0x33, "DM4340"),
            DamiaoJointConfig("joint4", 0x24, 0x34, "DM4310"),
            DamiaoJointConfig("joint5", 0x25, 0x35, "DM4310"),
            DamiaoJointConfig("joint6", 0x26, 0x36, "DM4310"),
            DamiaoJointConfig("joint7", 0x27, 0x37, "DM4310"),
        ),
    ),
    "gray_slave": ArmProfile(
        key="gray_slave",
        label="灰色从臂 / Gray Slave",
        kind="damiao",
        default_port="COM7",
        default_baudrate=921600,
        damiao_joints=(
            DamiaoJointConfig("joint1", 0x01, 0x11, "DM4340"),
            DamiaoJointConfig("joint2", 0x02, 0x12, "DM4340"),
            DamiaoJointConfig("joint3", 0x03, 0x13, "DM4340"),
            DamiaoJointConfig("joint4", 0x04, 0x14, "DM4310"),
            DamiaoJointConfig("joint5", 0x05, 0x15, "DM4310"),
            DamiaoJointConfig("joint6", 0x06, 0x16, "DM4310"),
            DamiaoJointConfig("joint7", 0x07, 0x17, "DM4310"),
        ),
    ),
}


def get_arm_profile(profile_key: str) -> ArmProfile:
    """Return a configured arm profile by key."""

    try:
        return ARM_PROFILES[profile_key]
    except KeyError as exc:
        valid = ", ".join(ARM_PROFILES)
        raise ValueError(f"unknown arm profile {profile_key!r}; valid: {valid}") from exc


def list_serial_ports() -> list[str]:
    """Return available serial port names."""

    load_serial_module()
    from serial.tools import list_ports

    return [port.device for port in list_ports.comports()]


class ArmReadConnection:
    """Opened read-only connection to one arm."""

    def __init__(self, profile: ArmProfile, port: str, baudrate: int) -> None:
        self.profile = profile
        self.port = port
        self.baudrate = baudrate
        self.serial_device: Any | None = None
        self.controller: Any | None = None
        self.items: list[Any] = []

    def open(self) -> None:
        """Open the serial port and create read-only runtime objects."""

        if self.profile.kind == "feetech":
            serial = load_serial_module()
            self.serial_device = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.items = [
                ServoRuntime(ServoConfig(name=f"joint{index}", servo_id=servo_id))
                for index, servo_id in enumerate(self.profile.ids, start=1)
            ]
            return

        serial, dm_motor_type, motor_class, motor_control_class = load_damiao_modules()
        self.serial_device = serial.Serial(self.port, self.baudrate, timeout=0.5)
        self.controller = motor_control_class(self.serial_device)
        self.items = create_damiao_joints(
            self.profile.damiao_joints,
            dm_motor_type,
            motor_class,
        )
        for joint in self.items:
            self.controller.addMotor(joint.motor)

    def close(self) -> None:
        """Close the serial port."""

        if self.serial_device is not None and self.serial_device.is_open:
            self.serial_device.close()
        self.serial_device = None
        self.controller = None
        self.items = []

    def read_snapshot(
        self,
        *,
        joint_names: set[str] | None = None,
        damiao_response_wait: float = 0.05,
    ) -> ArmStatusSnapshot:
        """Read arm state once and return structured status."""

        if self.serial_device is None:
            raise RuntimeError("serial port is not open")

        if self.profile.kind == "feetech":
            return self._read_feetech_snapshot(joint_names=joint_names)
        return self._read_damiao_snapshot(
            joint_names=joint_names,
            response_wait=damiao_response_wait,
        )

    def read_once(self) -> list[str]:
        """Read arm state once and return printable status lines."""

        return self.read_snapshot().to_lines()

    def _read_feetech_snapshot(self, *, joint_names: set[str] | None = None) -> ArmStatusSnapshot:
        assert self.serial_device is not None

        statuses: list[JointStatus] = []
        for servo in self.items:
            if joint_names is not None and servo.config.name not in joint_names:
                continue
            ticks, position_rad, error = read_servo_position(self.serial_device, servo)
            if error is not None or ticks is None or position_rad is None:
                statuses.append(
                    JointStatus(
                        name=servo.config.name,
                        device_id=servo.config.servo_id,
                        position_rad=None,
                        ticks=None,
                        error=error,
                    )
                )
                continue
            statuses.append(
                JointStatus(
                    name=servo.config.name,
                    device_id=servo.config.servo_id,
                    position_rad=position_rad,
                    ticks=ticks,
                )
            )
        return ArmStatusSnapshot(
            timestamp=time.strftime("%H:%M:%S"),
            profile_label=self.profile.label,
            joints=tuple(statuses),
        )

    def _read_damiao_snapshot(
        self,
        *,
        joint_names: set[str] | None = None,
        response_wait: float = 0.05,
    ) -> ArmStatusSnapshot:
        assert self.controller is not None

        statuses: list[JointStatus] = []
        for joint in self.items:
            if joint_names is not None and joint.config.name not in joint_names:
                continue
            position, velocity, torque, _changed = read_damiao_joint(
                self.controller,
                joint,
                response_wait=response_wait,
            )
            statuses.append(
                JointStatus(
                    name=joint.config.name,
                    device_id=joint.config.can_id,
                    position_rad=position,
                    velocity_rad_s=velocity,
                    torque_nm=torque,
                )
            )
        return ArmStatusSnapshot(
            timestamp=time.strftime("%H:%M:%S"),
            profile_label=self.profile.label,
            joints=tuple(statuses),
        )


def open_arm_connection(profile_key: str, port: str | None = None, baudrate: int | None = None) -> ArmReadConnection:
    """Open a read-only connection for one configured arm profile."""

    profile = get_arm_profile(profile_key)
    connection = ArmReadConnection(
        profile=profile,
        port=port or profile.default_port,
        baudrate=baudrate or profile.default_baudrate,
    )
    connection.open()
    return connection
