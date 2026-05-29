"""Zero-position storage service.

零点查看与保存服务。

本服务只把当前读取到的位置保存为软件零点文件，不会向电机写入零点，也不会修改电机
内部参数。
"""

from __future__ import annotations

from pathlib import Path

from robot_core.services.arm_read_service import ArmStatusSnapshot


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ZERO_DIR = PROJECT_ROOT / "configs" / "zeros"


def zero_file_path(profile_key: str) -> Path:
    """Return zero file path for one arm profile."""

    return ZERO_DIR / f"{profile_key}_zero.yaml"


def load_zero_text(profile_key: str) -> str:
    """Load saved zero file as text."""

    path = zero_file_path(profile_key)
    if not path.exists():
        return f"尚未保存零点文件：{path}"
    return path.read_text(encoding="utf-8")


def format_zero_snapshot(profile_key: str, snapshot: ArmStatusSnapshot) -> str:
    """Format one status snapshot as a simple YAML document."""

    lines = [
        "# Software zero positions. Do not edit while the robot is moving.",
        "# 软件零点文件。机械臂运动时不要编辑。",
        f"profile: {profile_key}",
        f"profile_label: {snapshot.profile_label}",
        f"saved_at: {snapshot.timestamp}",
        "joints:",
    ]
    for joint in snapshot.joints:
        lines.append(f"  - name: {joint.name}")
        lines.append(f"    device_id: {joint.device_id}")
        if joint.position_rad is None:
            lines.append("    zero_position_rad: null")
        else:
            lines.append(f"    zero_position_rad: {joint.position_rad:.9f}")
        if joint.ticks is not None:
            lines.append(f"    zero_ticks: {joint.ticks}")
        if joint.error is not None:
            lines.append(f"    error: {joint.error!r}")
    lines.append("")
    return "\n".join(lines)


def save_zero_snapshot(profile_key: str, snapshot: ArmStatusSnapshot) -> Path:
    """Save current snapshot as software zero file."""

    ZERO_DIR.mkdir(parents=True, exist_ok=True)
    path = zero_file_path(profile_key)
    path.write_text(format_zero_snapshot(profile_key, snapshot), encoding="utf-8")
    return path
