"""Small URDF reader used by the socket visualization tools.

轻量 URDF 读取工具。

当前只解析关节名称、父子 link、axis、limit 和 origin。完整 FK/IK、mesh 渲染
以后再接入更专业的运动学/三维可视化模块。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class UrdfJointInfo:
    """One joint entry parsed from URDF."""

    urdf_name: str
    display_name: str
    parent_link: str
    child_link: str
    axis: tuple[float, float, float]
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    lower: float | None
    upper: float | None


def default_pink_slave_urdf() -> Path:
    """Return the project-local pink slave URDF path."""

    return Path(__file__).resolve().parents[2] / "assets" / "pink_slave_urdf" / "urdf" / "kaka_arm_v7.urdf"


def load_arm_joints_from_urdf(urdf_path: str | Path, *, count: int = 7) -> list[UrdfJointInfo]:
    """Load the first N revolute joints from a URDF.

    SolidWorks 导出的 URDF 里关节名是 J1~J9；真实机械臂控制侧使用 joint1~joint7。
    这里默认把前 7 个 revolute joints 映射为 joint1~joint7。
    """

    root = ET.parse(urdf_path).getroot()
    joints: list[UrdfJointInfo] = []
    for joint_elem in root.findall("joint"):
        if joint_elem.attrib.get("type") not in {"revolute", "continuous"}:
            continue
        index = len(joints) + 1
        parent = joint_elem.find("parent")
        child = joint_elem.find("child")
        axis = joint_elem.find("axis")
        origin = joint_elem.find("origin")
        limit = joint_elem.find("limit")
        joints.append(
            UrdfJointInfo(
                urdf_name=joint_elem.attrib.get("name", f"J{index}"),
                display_name=f"joint{index}",
                parent_link=parent.attrib.get("link", "") if parent is not None else "",
                child_link=child.attrib.get("link", "") if child is not None else "",
                axis=_parse_vec3(axis.attrib.get("xyz", "0 0 1") if axis is not None else "0 0 1"),
                origin_xyz=_parse_vec3(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"),
                origin_rpy=_parse_vec3(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"),
                lower=_parse_optional_float(limit.attrib.get("lower")) if limit is not None else None,
                upper=_parse_optional_float(limit.attrib.get("upper")) if limit is not None else None,
            )
        )
        if len(joints) >= count:
            break
    return joints


def _parse_vec3(text: str) -> tuple[float, float, float]:
    parts = [float(item) for item in text.split()]
    while len(parts) < 3:
        parts.append(0.0)
    return (parts[0], parts[1], parts[2])


def _parse_optional_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    return float(value)
