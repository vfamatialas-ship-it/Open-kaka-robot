"""Load master-slave teleoperation mapping configuration.

This module is intentionally small and dependency-light. It first tries PyYAML;
if that is unavailable, it falls back to a parser for this project's
configs/teleop_mapping.yaml structure.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_core.services.teleop_service import TeleopJointSetting, TeleopRuntimeSettings


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAPPING_PATH = PROJECT_ROOT / "configs" / "teleop_mapping.yaml"


@dataclass(frozen=True)
class TeleopJointMapping:
    """One joint mapping loaded from configs/teleop_mapping.yaml."""

    master_joint: str
    slave_joint: str
    enabled: bool
    scale: float
    sign: int
    offset_rad: float
    mapping_mode: str = "anchor_delta"
    master_min_rad: float | None = None
    master_max_rad: float | None = None
    slave_min_rad: float | None = None
    slave_max_rad: float | None = None


@dataclass(frozen=True)
class TeleopMappingConfig:
    """A complete master-arm to slave-arm mapping."""

    master_arm: str
    slave_arm: str
    runtime: TeleopRuntimeSettings
    joints: tuple[TeleopJointMapping, ...]
    mode: str = ""

    def to_joint_settings(self) -> dict[str, TeleopJointSetting]:
        """Convert config entries to the compute_mapping_preview input type."""

        settings = {}
        for joint in self.joints:
            settings[joint.slave_joint] = TeleopJointSetting(
                name=joint.slave_joint,
                enabled=joint.enabled,
                scale=joint.scale,
                sign=joint.sign,
                offset_rad=joint.offset_rad,
                mapping_mode=joint.mapping_mode,
                master_min_rad=joint.master_min_rad,
                master_max_rad=joint.master_max_rad,
                slave_min_rad=joint.slave_min_rad,
                slave_max_rad=joint.slave_max_rad,
            )
        return settings


def load_teleop_mapping(
    master_arm: str = "pink_master",
    slave_arm: str = "pink_slave",
    path: Path = DEFAULT_MAPPING_PATH,
) -> TeleopMappingConfig:
    """Load one master/slave mapping pair from teleop_mapping.yaml."""

    document = _load_document(path)
    for entry in document.get("mappings", []):
        if entry.get("master_arm") == master_arm and entry.get("slave_arm") == slave_arm:
            return _entry_to_config(entry)
    raise ValueError(f"mapping not found: {master_arm} -> {slave_arm} in {path}")


def mapping_to_virtual_config_payload(config: TeleopMappingConfig) -> dict[str, Any]:
    """Convert a mapping config to the browser's mapping dictionary shape."""

    mappings: dict[str, dict[str, Any]] = {}
    for joint in config.joints:
        mappings[joint.slave_joint] = {
            "enabled": joint.enabled,
            "scale": joint.scale,
            "sign": joint.sign,
            "offset": joint.offset_rad,
            "master_joint": joint.master_joint,
            "slave_joint": joint.slave_joint,
            "mapping_mode": joint.mapping_mode,
            "master_min_rad": joint.master_min_rad,
            "master_max_rad": joint.master_max_rad,
            "slave_min_rad": joint.slave_min_rad,
            "slave_max_rad": joint.slave_max_rad,
        }
    return mappings


def _entry_to_config(entry: dict[str, Any]) -> TeleopMappingConfig:
    runtime = entry.get("runtime", {}) or {}
    joints = []
    for index in range(1, 8):
        default_name = f"joint{index}"
        item = _joint_entry(entry.get("joints", []), default_name)
        joints.append(
            TeleopJointMapping(
                master_joint=str(item.get("master_joint", default_name)),
                slave_joint=str(item.get("slave_joint", default_name)),
                enabled=_as_bool(item.get("enabled", False)),
                scale=float(item.get("scale", 1.0)),
                sign=1 if int(item.get("sign", 1)) >= 0 else -1,
                offset_rad=float(item.get("offset_rad", item.get("offset", 0.0))),
                mapping_mode=str(item.get("mapping_mode", "anchor_delta")),
                master_min_rad=_optional_float(item.get("master_min_rad")),
                master_max_rad=_optional_float(item.get("master_max_rad")),
                slave_min_rad=_optional_float(item.get("slave_min_rad")),
                slave_max_rad=_optional_float(item.get("slave_max_rad")),
            )
        )
    return TeleopMappingConfig(
        master_arm=str(entry.get("master_arm", "")),
        slave_arm=str(entry.get("slave_arm", "")),
        mode=str(entry.get("mode", "")),
        runtime=TeleopRuntimeSettings(
            alpha=float(runtime.get("alpha", 0.35)),
            max_step_rad=float(runtime.get("max_step_rad", 0.06)),
            damiao_response_wait=float(runtime.get("damiao_response_wait", 0.005)),
        ),
        joints=tuple(joints),
    )


def _joint_entry(items: list[dict[str, Any]], default_name: str) -> dict[str, Any]:
    for item in items:
        if item.get("slave_joint") == default_name or item.get("master_joint") == default_name:
            return item
    return {
        "master_joint": default_name,
        "slave_joint": default_name,
        "enabled": False,
        "scale": 1.0,
        "sign": 1,
        "offset_rad": 0.0,
    }


def _load_document(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml

        loaded = yaml.safe_load(text)
        return loaded if isinstance(loaded, dict) else {}
    except Exception:
        return _parse_mapping_yaml_subset(text)


def _parse_mapping_yaml_subset(text: str) -> dict[str, Any]:
    """Parse the subset emitted by the project's mapping writer."""

    mappings: list[dict[str, Any]] = []
    current_entry: dict[str, Any] | None = None
    current_joint: dict[str, Any] | None = None
    in_runtime = False
    in_joints = False
    in_mappings = False

    for raw_line in text.splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        stripped = raw_line.strip()

        if stripped == "mappings:":
            in_mappings = True
            current_entry = None
            current_joint = None
            in_runtime = False
            in_joints = False
            continue

        if not in_mappings:
            continue

        if stripped.startswith("- "):
            key, value = _split_key_value(stripped[2:])
            if key == "master_arm":
                current_entry = {key: _parse_scalar(value)}
                mappings.append(current_entry)
                current_joint = None
                in_runtime = False
                in_joints = False
                continue
            if key == "master_joint" and current_entry is not None:
                current_joint = {key: _parse_scalar(value)}
                current_entry.setdefault("joints", []).append(current_joint)
                in_runtime = False
                in_joints = True
                continue

        if current_entry is None:
            continue

        if stripped == "runtime:":
            current_entry["runtime"] = {}
            current_joint = None
            in_runtime = True
            in_joints = False
            continue
        if stripped == "joints:":
            current_entry["joints"] = []
            current_joint = None
            in_runtime = False
            in_joints = True
            continue

        key, value = _split_key_value(stripped)
        if in_runtime:
            current_entry.setdefault("runtime", {})[key] = _parse_scalar(value)
            continue

        if in_joints and current_joint is not None:
            current_joint[key] = _parse_scalar(value)
            continue

        current_entry[key] = _parse_scalar(value)

    return {"mappings": mappings}


def _split_key_value(text: str) -> tuple[str, str]:
    if ":" not in text:
        return text, ""
    key, value = text.split(":", 1)
    return key.strip(), value.strip().strip("'\"")


def _parse_scalar(value: str) -> Any:
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        if any(char in value for char in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"true", "yes", "1", "on"}
    return bool(value)


def _optional_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    return float(value)
