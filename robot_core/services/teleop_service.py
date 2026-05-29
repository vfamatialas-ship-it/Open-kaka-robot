"""Master-slave teleoperation mapping service.

主从遥操作映射服务。

本模块只负责状态、映射计算和限位检查，不创建 GUI 控件。Dry Run / 只读预览
模式只读取主臂和从臂并计算 target，不会向从臂发送任何运动命令。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from robot_core.services.arm_read_service import ArmStatusSnapshot
from robot_core.services.joint_limit_service import JointLimit


class TeleopState(str, Enum):
    """Teleoperation state machine."""

    IDLE = "IDLE"
    READ_MASTER = "READ_MASTER"
    PREVIEW_MAPPING = "PREVIEW_MAPPING"
    ARM_SLAVE_HOLD_CURRENT = "ARM_SLAVE_HOLD_CURRENT"
    TELEOP_ACTIVE = "TELEOP_ACTIVE"
    PAUSED = "PAUSED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass
class TeleopJointSetting:
    """Mapping setting for one joint."""

    name: str
    enabled: bool = False
    scale: float = 1.0
    sign: int = 1
    offset_rad: float = 0.0
    mapping_mode: str = "anchor_delta"
    master_min_rad: float | None = None
    master_max_rad: float | None = None
    slave_min_rad: float | None = None
    slave_max_rad: float | None = None


@dataclass
class TeleopRuntimeSettings:
    """Runtime filter and speed-limit settings."""

    alpha: float = 0.2
    max_step_rad: float = 0.02
    damiao_response_wait: float = 0.005


@dataclass(frozen=True)
class TeleopJointPreview:
    """Computed mapping preview for one joint."""

    name: str
    master_rad: float | None
    slave_current_rad: float | None
    target_rad: float | None
    ratio: float
    sign: int
    enabled: bool
    limit_status: str


def make_default_joint_settings() -> dict[str, TeleopJointSetting]:
    """Create default disabled mapping settings for joint1~joint7."""

    return {f"joint{index}": TeleopJointSetting(name=f"joint{index}") for index in range(1, 8)}


def build_anchor(snapshot: ArmStatusSnapshot) -> dict[str, float]:
    """Build joint-name to position-rad anchor map from one snapshot."""

    anchor: dict[str, float] = {}
    for joint in snapshot.joints:
        if joint.position_rad is not None:
            anchor[joint.name] = joint.position_rad
    return anchor


def update_continuous_position_map(
    raw_positions: dict[str, float | None],
    last_raw_positions: dict[str, float],
    continuous_positions: dict[str, float],
) -> None:
    """Update a continuous joint-position map from wrapped encoder readings.

    Feetech positions are usually reported inside one turn. If a joint crosses
    the 0/2pi boundary, the raw value jumps, but the physical motion is still
    continuous. This helper unwraps the reading incrementally so teleop mapping
    will not command the slave to move backward across its limit.
    """

    for name, raw in raw_positions.items():
        if raw is None:
            continue
        if name not in last_raw_positions or name not in continuous_positions:
            last_raw_positions[name] = raw
            continuous_positions[name] = raw
            continue

        delta = raw - last_raw_positions[name]
        while delta > math.pi:
            delta -= math.tau
        while delta < -math.pi:
            delta += math.tau
        continuous_positions[name] += delta
        last_raw_positions[name] = raw


def compute_mapping_preview(
    master_snapshot: ArmStatusSnapshot,
    slave_snapshot: ArmStatusSnapshot,
    joint_settings: dict[str, TeleopJointSetting],
    runtime_settings: TeleopRuntimeSettings,
    slave_limits: list[JointLimit],
    master_anchor: dict[str, float],
    slave_anchor: dict[str, float],
    previous_targets: dict[str, float] | None = None,
    master_continuous_positions: dict[str, float] | None = None,
) -> tuple[list[TeleopJointPreview], dict[str, float]]:
    """Compute slave target positions from master motion.

    映射公式：
      q_slave_des_raw = q_slave_anchor
                        + sign * scale * (q_master_now - q_master_anchor)

    然后执行：
      1. 低通滤波 alpha
      2. 单周期最大步长 max_step_rad
      3. 软件关节限位检查
    """

    previous_targets = previous_targets or {}
    master_map = {joint.name: joint.position_rad for joint in master_snapshot.joints}
    if master_continuous_positions is not None:
        master_map = {
            name: master_continuous_positions.get(name, position)
            for name, position in master_map.items()
        }
    slave_map = {joint.name: joint.position_rad for joint in slave_snapshot.joints}
    limit_map = {limit.name: limit for limit in slave_limits}

    previews: list[TeleopJointPreview] = []
    next_targets: dict[str, float] = {}
    alpha = _clamp(runtime_settings.alpha, 0.0, 1.0)
    max_step = max(0.0, runtime_settings.max_step_rad)

    for index in range(1, 8):
        name = f"joint{index}"
        setting = joint_settings.get(name, TeleopJointSetting(name=name))
        master_rad = master_map.get(name)
        slave_current = slave_map.get(name)
        limit = limit_map.get(name)

        target: float | None = None
        limit_status = "disabled"
        if not setting.enabled:
            previews.append(
                TeleopJointPreview(
                    name=name,
                    master_rad=master_rad,
                    slave_current_rad=slave_current,
                    target_rad=None,
                    ratio=_display_ratio(setting.scale),
                    sign=setting.sign,
                    enabled=False,
                    limit_status=limit_status,
                )
            )
            continue

        if master_rad is None or slave_current is None:
            limit_status = "missing position"
        elif name not in master_anchor or name not in slave_anchor:
            limit_status = "missing anchor"
        elif limit is None:
            limit_status = "missing limit"
        else:
            raw_target = _map_joint_target(name, setting, master_rad, master_anchor, slave_anchor)
            last_target = previous_targets.get(name, slave_current)
            filtered_target = alpha * raw_target + (1.0 - alpha) * last_target
            stepped_target = _limit_step(slave_current, filtered_target, max_step)
            target, limit_status = _clamp_to_limit(stepped_target, limit)
            next_targets[name] = target

        previews.append(
            TeleopJointPreview(
                name=name,
                master_rad=master_rad,
                slave_current_rad=slave_current,
                target_rad=target,
                ratio=_display_ratio(setting.scale),
                sign=setting.sign,
                enabled=setting.enabled,
                limit_status=limit_status,
            )
        )

    return previews, next_targets


def targets_are_safe(previews: list[TeleopJointPreview]) -> bool:
    """Return true only if every enabled joint target is safe to command.

    LIMIT_MIN / LIMIT_MAX mean the requested mapping has saturated at the
    slave's configured software limit. The final target is still inside the
    safe range, so real teleoperation may keep commanding that boundary value
    while the operator moves the master back toward the valid range.
    """

    for preview in previews:
        if preview.enabled and preview.limit_status not in {"OK", "LIMIT_MIN", "LIMIT_MAX"}:
            return False
    return True


def target_status_is_commandable(limit_status: str) -> bool:
    """Return true when target_rad is already clamped into a safe range."""

    return limit_status in {"OK", "LIMIT_MIN", "LIMIT_MAX"}


def _limit_step(current: float, target: float, max_step: float) -> float:
    if max_step <= 0:
        return current
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def _map_joint_target(
    name: str,
    setting: TeleopJointSetting,
    master_rad: float,
    master_anchor: dict[str, float],
    slave_anchor: dict[str, float],
) -> float:
    """Map one master joint angle to one slave target angle."""

    if setting.mapping_mode == "range":
        if (
            setting.master_min_rad is None
            or setting.master_max_rad is None
            or setting.slave_min_rad is None
            or setting.slave_max_rad is None
        ):
            raise ValueError(f"{name}: range mapping requires master/slave min/max")
        span = setting.master_max_rad - setting.master_min_rad
        if abs(span) < 1e-6:
            raise ValueError(f"{name}: master range is too small")
        ratio = (master_rad - setting.master_min_rad) / span
        ratio = _clamp(ratio, 0.0, 1.0)
        if setting.sign < 0:
            ratio = 1.0 - ratio
        return setting.slave_min_rad + ratio * (setting.slave_max_rad - setting.slave_min_rad) + setting.offset_rad

    if setting.mapping_mode == "zero_delta":
        return (
            slave_anchor[name]
            + setting.sign * setting.scale * (master_rad - master_anchor[name])
            + setting.offset_rad
        )

    return (
        slave_anchor[name]
        + setting.sign * setting.scale * (master_rad - master_anchor[name])
        + setting.offset_rad
    )


def _clamp_to_limit(target: float, limit: JointLimit) -> tuple[float, str]:
    if target < limit.min_rad:
        return limit.min_rad, "LIMIT_MIN"
    if target > limit.max_rad:
        return limit.max_rad, "LIMIT_MAX"
    return target, "OK"


def _display_ratio(scale: float) -> float:
    return _clamp(abs(scale), 0.0, 1.0)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
