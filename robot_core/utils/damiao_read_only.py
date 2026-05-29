"""Read-only helpers for Damiao motor status scripts.

达妙电机只读状态读取工具。

这个模块只负责读取状态，不包含任何使能、运动、保存零点或参数写入逻辑。
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DAMIAO_LIB_DIR = PROJECT_ROOT / "third_party" / "damiao" / "DM_Control_Python"


@dataclass(frozen=True)
class JointConfig:
    """单个关节的静态配置。"""

    name: str
    can_id: int
    master_id: int
    motor_type_name: str


@dataclass
class JointRuntime:
    """单个关节的运行时状态。"""

    config: JointConfig
    motor: Any
    last_position: float | None = None
    last_velocity: float | None = None
    last_torque: float | None = None


def build_parser(description: str, default_port: str) -> argparse.ArgumentParser:
    """创建通用命令行参数。"""

    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--port", default=default_port, help=f"串口号，默认 {default_port}")
    parser.add_argument("--baudrate", type=int, default=921600, help="串口波特率")
    parser.add_argument("--timeout", type=float, default=0.5, help="串口超时时间，单位秒")
    parser.add_argument(
        "--response-wait",
        type=float,
        default=0.05,
        help="每次状态请求后额外等待回复的时间，单位秒",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="连续读取时，每轮之间的间隔，单位秒",
    )
    parser.add_argument("--once", action="store_true", help="只读取一轮然后退出")
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="读取固定轮数；0 表示一直循环，按 Ctrl+C 停止",
    )
    return parser


def load_damiao_modules() -> tuple[Any, Any, Any, Any]:
    """导入 pyserial 和达妙库。

    这里故意延迟导入，让 `--help` 在未安装硬件依赖时也能正常显示。
    """

    sys.path.insert(0, str(DAMIAO_LIB_DIR))

    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "缺少依赖 pyserial。\n"
            "请先在当前 Python 环境中安装：\n"
            '  python -m pip install "pyserial>=3.5"'
        ) from exc

    try:
        from DM_CAN import DM_Motor_Type, Motor, MotorControl
    except ImportError as exc:
        raise SystemExit(
            f"无法从下面目录导入 DM_CAN.py：\n  {DAMIAO_LIB_DIR}\n"
            "请确认达妙库已经下载到 third_party/damiao/DM_Control_Python。"
        ) from exc

    return serial, DM_Motor_Type, Motor, MotorControl


def create_joints(
    joint_configs: tuple[JointConfig, ...],
    dm_motor_type: Any,
    motor_class: Any,
) -> list[JointRuntime]:
    """根据关节配置创建达妙 Motor 对象。"""

    joints: list[JointRuntime] = []
    for config in joint_configs:
        motor_type = getattr(dm_motor_type, config.motor_type_name)
        motor = motor_class(motor_type, config.can_id, config.master_id)
        joints.append(JointRuntime(config=config, motor=motor))
    return joints


def print_startup(
    title: str,
    port: str,
    baudrate: int,
    joint_configs: tuple[JointConfig, ...],
) -> None:
    """启动时打印配置，方便运行前人工确认。"""

    print(title)
    print(f"Serial: {port}, baudrate: {baudrate}")
    print("Safety: read-only; no enable; no mode switch; no motion; no zero save")
    print()
    print("Joint table / 关节表:")
    for config in joint_configs:
        print(
            f"  {config.name}: "
            f"CAN ID=0x{config.can_id:02X}, "
            f"Master ID=0x{config.master_id:02X}, "
            f"type={config.motor_type_name}"
        )
    print()


def read_joint(
    controller: Any,
    joint: JointRuntime,
    response_wait: float,
) -> tuple[float, float, float, bool]:
    """读取一个关节状态。

    返回：
      position: 位置，单位 rad
      velocity: 速度，单位 rad/s
      torque: 力矩，按达妙库解码结果显示
      changed: 本轮读数和上一轮相比是否变化
    """

    # refresh_motor_status() 只发送状态请求，不会使能，也不会运动。
    controller.refresh_motor_status(joint.motor)

    # 达妙库会在 refresh_motor_status() 内部立刻 recv()。
    # 某些 USB-CAN 回复可能稍微晚一点到，所以这里额外等待并再解析一次缓存。
    if response_wait > 0:
        time.sleep(response_wait)
        controller.recv()

    position = float(joint.motor.getPosition())
    velocity = float(joint.motor.getVelocity())
    torque = float(joint.motor.getTorque())

    changed = (
        joint.last_position is None
        or position != joint.last_position
        or velocity != joint.last_velocity
        or torque != joint.last_torque
    )

    joint.last_position = position
    joint.last_velocity = velocity
    joint.last_torque = torque

    return position, velocity, torque, changed


def read_all_once(
    controller: Any,
    joints: list[JointRuntime],
    response_wait: float,
) -> None:
    """读取并打印全部关节一次。"""

    print(f"[{time.strftime('%H:%M:%S')}]")
    for joint in joints:
        position, velocity, torque, changed = read_joint(
            controller,
            joint,
            response_wait,
        )
        marker = "*" if changed else " "
        print(
            f"{marker} {joint.config.name:<6} "
            f"pos={position:>9.4f} rad  "
            f"vel={velocity:>9.4f} rad/s  "
            f"tau={torque:>9.4f} Nm"
        )
    print()


def run_read_only_monitor(
    *,
    description: str,
    title: str,
    joint_configs: tuple[JointConfig, ...],
    default_port: str = "COM7",
) -> None:
    """运行达妙只读状态监视器。"""

    args = build_parser(description, default_port).parse_args()
    serial, dm_motor_type, motor_class, motor_control_class = load_damiao_modules()

    print_startup(title, args.port, args.baudrate, joint_configs)
    joints = create_joints(joint_configs, dm_motor_type, motor_class)

    # 到这里才打开串口。如果打开失败，此前没有向电机发送任何命令。
    serial_device = serial.Serial(args.port, args.baudrate, timeout=args.timeout)
    controller = motor_control_class(serial_device)

    for joint in joints:
        controller.addMotor(joint.motor)

    try:
        cycle = 0
        while True:
            cycle += 1
            read_all_once(controller, joints, args.response_wait)

            if args.once or (args.cycles > 0 and cycle >= args.cycles):
                break

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("Stopped by user / 用户停止。")
    finally:
        # 本工具从未使能或运动电机，所以退出时只需要关闭串口。
        if serial_device.is_open:
            serial_device.close()
