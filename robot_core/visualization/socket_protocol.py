"""JSON-lines socket protocol for live arm visualization.

协议格式是一行一个 JSON，末尾换行。这样普通 TCP socket 就够用，不依赖
websocket 或浏览器运行环境。
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any

from robot_core.services.arm_read_service import ArmStatusSnapshot
from robot_core.visualization.urdf_light import UrdfJointInfo


PROTOCOL_NAME = "dual_arm_robot.joint_state.v1"


@dataclass(frozen=True)
class JointStateMessage:
    """One transmitted joint-state message."""

    protocol: str
    timestamp: float
    arm: str
    joints: dict[str, float]


def snapshot_to_message(snapshot: ArmStatusSnapshot, arm: str) -> dict[str, Any]:
    """Convert an arm snapshot to a JSON-serializable message."""

    joints = {
        joint.name: joint.position_rad
        for joint in snapshot.joints
        if joint.position_rad is not None
    }
    return {
        "protocol": PROTOCOL_NAME,
        "timestamp": time.time(),
        "arm": arm,
        "joints": joints,
    }


def urdf_info_message(arm: str, joints: list[UrdfJointInfo]) -> dict[str, Any]:
    """Create an optional metadata message with URDF joint information."""

    return {
        "protocol": PROTOCOL_NAME,
        "type": "urdf_info",
        "timestamp": time.time(),
        "arm": arm,
        "joints": [
            {
                "name": joint.display_name,
                "urdf_name": joint.urdf_name,
                "parent": joint.parent_link,
                "child": joint.child_link,
                "axis": joint.axis,
                "origin_xyz": joint.origin_xyz,
                "origin_rpy": joint.origin_rpy,
                "lower": joint.lower,
                "upper": joint.upper,
            }
            for joint in joints
        ],
    }


def encode_message(message: dict[str, Any]) -> bytes:
    """Encode one protocol message as UTF-8 JSON line."""

    return (json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


def decode_message(line: bytes) -> dict[str, Any]:
    """Decode one UTF-8 JSON line."""

    return json.loads(line.decode("utf-8"))
