"""Load per-joint Damiao command settings for teleoperation tests."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GAIN_PATH = PROJECT_ROOT / "configs" / "teleop_joint_gains.yaml"


@dataclass(frozen=True)
class JointGain:
    """Command settings for one Damiao joint.

    control_mode:
        "mit" uses Damiao MIT mode with kp/kd/tau.
        "posvel" uses Damiao position-velocity mode.
    """

    kp: float
    kd: float
    tau: float = 0.0
    control_mode: str = "mit"
    posvel_velocity: float = 1.0
    enable_old_mode: bool = False
    switch_mode: bool = False
    slave_read_every: int = 2
    limit_relax_rad: float = 0.0
    note: str = ""


def load_joint_gains(arm: str = "pink_slave", path: Path = DEFAULT_GAIN_PATH) -> dict[str, JointGain]:
    """Load joint gains from configs/teleop_joint_gains.yaml."""

    if not path.exists():
        return _default_gains()
    text = path.read_text(encoding="utf-8")
    try:
        import yaml

        data = yaml.safe_load(text)
        arm_data = data.get(arm, {}) if isinstance(data, dict) else {}
        if isinstance(arm_data, dict):
            return _parse_arm_gains(arm_data)
    except Exception:
        pass
    return _parse_gain_yaml_subset(text, arm)


def get_joint_gain(joint_name: str, arm: str = "pink_slave") -> JointGain:
    """Return configured gain for one joint, falling back to safe defaults."""

    return load_joint_gains(arm).get(joint_name, _default_gains().get(joint_name, JointGain(8.0, 0.8)))


def _parse_arm_gains(arm_data: dict[str, Any]) -> dict[str, JointGain]:
    gains: dict[str, JointGain] = {}
    for joint_name, item in arm_data.items():
        if not isinstance(item, dict):
            continue
        gains[str(joint_name)] = JointGain(
            kp=float(item.get("kp", 8.0)),
            kd=float(item.get("kd", 0.8)),
            tau=float(item.get("tau", 0.0)),
            control_mode=_clean_control_mode(item.get("control_mode", "mit")),
            posvel_velocity=float(item.get("posvel_velocity", 1.0)),
            enable_old_mode=bool(item.get("enable_old_mode", False)),
            switch_mode=bool(item.get("switch_mode", False)),
            slave_read_every=max(1, int(item.get("slave_read_every", 2))),
            limit_relax_rad=max(0.0, float(item.get("limit_relax_rad", 0.0))),
            note=str(item.get("note", "")),
        )
    return {**_default_gains(), **gains}


def _parse_gain_yaml_subset(text: str, arm: str) -> dict[str, JointGain]:
    gains = _default_gains()
    in_arm = False
    current_joint: str | None = None
    current: dict[str, str] = {}

    def commit() -> None:
        if current_joint is None:
            return
        gains[current_joint] = JointGain(
            kp=float(current.get("kp", gains.get(current_joint, JointGain(8.0, 0.8)).kp)),
            kd=float(current.get("kd", gains.get(current_joint, JointGain(8.0, 0.8)).kd)),
            tau=float(current.get("tau", gains.get(current_joint, JointGain(8.0, 0.8)).tau)),
            control_mode=_clean_control_mode(
                current.get("control_mode", gains.get(current_joint, JointGain(8.0, 0.8)).control_mode)
            ),
            posvel_velocity=float(
                current.get("posvel_velocity", gains.get(current_joint, JointGain(8.0, 0.8)).posvel_velocity)
            ),
            enable_old_mode=_parse_bool(
                current.get("enable_old_mode", str(gains.get(current_joint, JointGain(8.0, 0.8)).enable_old_mode))
            ),
            switch_mode=_parse_bool(
                current.get("switch_mode", str(gains.get(current_joint, JointGain(8.0, 0.8)).switch_mode))
            ),
            slave_read_every=max(
                1,
                int(current.get("slave_read_every", gains.get(current_joint, JointGain(8.0, 0.8)).slave_read_every)),
            ),
            limit_relax_rad=max(
                0.0,
                float(current.get("limit_relax_rad", gains.get(current_joint, JointGain(8.0, 0.8)).limit_relax_rad)),
            ),
            note=current.get("note", ""),
        )

    for raw_line in text.splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        stripped = raw_line.strip()
        if not raw_line.startswith(" ") and stripped.endswith(":"):
            commit()
            in_arm = stripped[:-1] == arm
            current_joint = None
            current = {}
            continue
        if not in_arm:
            continue
        if raw_line.startswith("  ") and not raw_line.startswith("    ") and stripped.endswith(":"):
            commit()
            current_joint = stripped[:-1]
            current = {}
            continue
        if current_joint is not None and ":" in stripped:
            key, value = stripped.split(":", 1)
            current[key.strip()] = value.strip()
    commit()
    return gains


def _default_gains() -> dict[str, JointGain]:
    return {
        "joint1": JointGain(16.0, 1.2, 0.0, "mit", 1.0, False, False, 2, 0.0, "tested, good initial effect"),
        "joint2": JointGain(
            18.0,
            1.5,
            0.2,
            "posvel",
            2.5,
            True,
            True,
            3,
            0.05,
            "tested: POS_VEL + switch_mode + enable_old_mode; velocity/read cadence tuned",
        ),
        "joint3": JointGain(
            30.0,
            2.5,
            -0.2,
            "mit",
            1.0,
            False,
            False,
            1,
            0.05,
            "strongest currently working MIT set; keep mapping sign=+1",
        ),
        "joint4": JointGain(8.0, 0.8, 0.0, "mit", 1.0, False, False, 2, 0.0, "conservative small-joint starting point"),
        "joint5": JointGain(8.0, 0.8, 0.0, "mit", 1.0, False, False, 2, 0.0, "conservative small-joint starting point"),
        "joint6": JointGain(8.0, 0.8, 0.0, "mit", 1.0, False, False, 2, 0.0, "tested, effective"),
        "joint7": JointGain(8.0, 0.8, 0.0, "mit", 1.0, False, False, 2, 0.0, "conservative wrist-joint starting point"),
    }


def _clean_control_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    if mode not in {"mit", "posvel"}:
        return "mit"
    return mode


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "y", "on"}
