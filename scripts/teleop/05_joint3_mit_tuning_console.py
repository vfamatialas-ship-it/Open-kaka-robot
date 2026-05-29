"""Interactive MIT tuning console for pink slave joint3.

Safety notes:
- This script sends real Damiao MIT motion commands. It refuses to run unless
  --enable-motion YES is passed.
- Only pink slave joint3 is commanded.
- Startup reads current master/slave positions and uses them as anchors.
- Every target is clamped to software joint limits.
- Keyboard:
    Up/Down arrows, c/n: move cursor
    - / =: decrease/increase selected value
    [ / ]: change selected value step size
    h: recapture master/slave anchors at current positions
    space: pause/resume command output
    q: disable and quit
"""

from __future__ import annotations

import argparse
import math
import msvcrt
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import ArmStatusSnapshot, open_arm_connection
from robot_core.services.joint_limit_service import JointLimit, load_limits_text, parse_limits_text
from robot_core.services.single_joint_motion_service import disable_arm_motion
from robot_core.services.teleop_mapping_config import load_teleop_mapping
from robot_core.services.teleop_service import update_continuous_position_map


JOINT_NAME = "joint3"


@dataclass
class TunableParam:
    name: str
    value: float
    step: float
    low: float
    high: float
    description: str

    def add(self, direction: float) -> None:
        self.value = max(self.low, min(self.high, self.value + direction * self.step))

    def adjust_step(self, factor: float) -> None:
        self.step = max(1e-4, min(10.0, self.step * factor))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive MIT tuning console for pink slave joint3.")
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--damiao-wait", type=float, default=0.005)
    parser.add_argument("--limit-relax-rad", type=float, default=0.03)
    parser.add_argument("--kp", type=float, default=18.0)
    parser.add_argument("--kp-limit", type=float, default=24.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--kd-limit", type=float, default=3.0)
    parser.add_argument("--tau-ff", type=float, default=0.0)
    parser.add_argument("--tau-limit", type=float, default=1.0)
    parser.add_argument("--max-speed-rad-s", type=float, default=0.6)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--sign", type=int, choices=[-1, 1], default=-1)
    parser.add_argument("--enable-motion", default="NO", help="must be exactly YES to send real commands")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.enable_motion != "YES":
        raise SystemExit(
            "Refusing to move. This script sends real joint3 MIT commands.\n"
            "Run again with --enable-motion YES only when the arm is supported and E-stop is ready."
        )

    params = _make_params(args)
    cursor = 0
    paused = False
    message = "Starting..."

    limits = _relax_limit(parse_limits_text(load_limits_text("pink_slave")), JOINT_NAME, args.limit_relax_rad)
    limit = _find_limit(limits, JOINT_NAME)
    mapping_sign, mapping_scale = _mapping_defaults(args.sign, args.scale)

    master = open_arm_connection("pink_master", port=args.master_port, baudrate=args.master_baudrate)
    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)
    master_last_raw: dict[str, float] = {}
    master_continuous: dict[str, float] = {}
    q_des_last: float | None = None

    try:
        master_anchor, slave_anchor = _capture_anchors(master, slave, args.damiao_wait, master_last_raw, master_continuous)
        q_des_last = slave_anchor
        _enable_joint3_mit(slave)
        period = 1.0 / max(1.0, args.hz)
        last_draw = 0.0
        last_status: dict[str, Any] = {}

        while True:
            started = time.perf_counter()
            key = _read_key()
            if key == "q":
                message = "Quitting and disabling..."
                break
            if key in {"up", "c"}:
                cursor = (cursor - 1) % len(params)
            elif key in {"down", "n"}:
                cursor = (cursor + 1) % len(params)
            elif key == "-":
                params[cursor].add(-1.0)
            elif key == "=":
                params[cursor].add(1.0)
            elif key == "[":
                params[cursor].adjust_step(0.5)
            elif key == "]":
                params[cursor].adjust_step(2.0)
            elif key == " ":
                paused = not paused
                message = "Paused command output" if paused else "Resumed command output"
            elif key == "h":
                master_anchor, slave_anchor = _capture_anchors(
                    master,
                    slave,
                    args.damiao_wait,
                    master_last_raw,
                    master_continuous,
                )
                q_des_last = slave_anchor
                message = "Anchors recaptured at current positions"

            values = {item.name: item.value for item in params}
            kp = min(values["kp"], values["kp_limit"])
            kd = min(values["kd"], values["kd_limit"])
            tau_ff = max(-values["tau_limit"], min(values["tau_limit"], values["tau_ff"]))
            max_speed = max(0.01, values["max_speed_rad_s"])

            master_snapshot = master.read_snapshot(joint_names={JOINT_NAME})
            update_continuous_position_map(_position_map(master_snapshot), master_last_raw, master_continuous)
            master_now = master_continuous.get(JOINT_NAME)
            slave_now = _read_slave_joint3(slave, args.damiao_wait)

            if master_now is None:
                message = "Missing master joint3 position"
            else:
                raw_target = slave_anchor + mapping_sign * mapping_scale * (master_now - master_anchor)
                target_limited = max(limit.min_rad, min(limit.max_rad, raw_target))
                if q_des_last is None:
                    q_des_last = slave_now
                max_step = max_speed * period
                q_des = _rate_limit(q_des_last, target_limited, max_step)
                q_des_last = q_des
                if not paused:
                    _send_joint3_mit(slave, q_des, kp, kd, tau_ff)
                last_status = {
                    "master_now": master_now,
                    "master_delta": master_now - master_anchor,
                    "slave_now": slave_now,
                    "target_raw": raw_target,
                    "target_cmd": q_des,
                    "err": q_des - slave_now,
                    "kp_used": kp,
                    "kd_used": kd,
                    "tau_used": tau_ff,
                    "paused": paused,
                    "limit": _limit_status(target_limited, raw_target),
                }

            now = time.perf_counter()
            if now - last_draw > 0.08:
                _draw(params, cursor, last_status, limit, mapping_sign, mapping_scale, message)
                last_draw = now

            elapsed = time.perf_counter() - started
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        message = "KeyboardInterrupt"
    finally:
        print()
        print(message)
        try:
            for line in disable_arm_motion(slave):
                print(line)
        finally:
            master.close()
            slave.close()


def _make_params(args) -> list[TunableParam]:
    return [
        TunableParam("kp", args.kp, 0.5, 0.0, 80.0, "MIT position gain"),
        TunableParam("kp_limit", args.kp_limit, 0.5, 0.0, 80.0, "software cap for kp"),
        TunableParam("kd", args.kd, 0.1, 0.0, 5.0, "MIT damping gain"),
        TunableParam("kd_limit", args.kd_limit, 0.1, 0.0, 5.0, "software cap for kd"),
        TunableParam("tau_ff", args.tau_ff, 0.05, -5.0, 5.0, "feed-forward torque command"),
        TunableParam("tau_limit", args.tau_limit, 0.05, 0.0, 5.0, "absolute cap for tau_ff"),
        TunableParam("max_speed_rad_s", args.max_speed_rad_s, 0.05, 0.01, 3.0, "target slew-rate limit"),
    ]


def _mapping_defaults(sign: int, scale: float) -> tuple[int, float]:
    try:
        mapping = load_teleop_mapping("pink_master", "pink_slave")
        setting = mapping.to_joint_settings().get(JOINT_NAME)
        if setting is not None:
            return sign if sign is not None else setting.sign, scale if scale is not None else setting.scale
    except Exception:
        pass
    return sign, scale


def _capture_anchors(master, slave, damiao_wait: float, master_last_raw: dict[str, float], master_continuous: dict[str, float]) -> tuple[float, float]:
    master_snapshot = master.read_snapshot(joint_names={JOINT_NAME})
    update_continuous_position_map(_position_map(master_snapshot), master_last_raw, master_continuous)
    master_anchor = master_continuous.get(JOINT_NAME)
    slave_anchor = _read_slave_joint3(slave, damiao_wait)
    if master_anchor is None:
        raise RuntimeError("missing master joint3 position during anchor capture")
    return master_anchor, slave_anchor


def _read_slave_joint3(slave, damiao_wait: float) -> float:
    snapshot = slave.read_snapshot(joint_names={JOINT_NAME}, damiao_response_wait=damiao_wait)
    joint = next((item for item in snapshot.joints if item.name == JOINT_NAME), None)
    if joint is None or joint.position_rad is None:
        raise RuntimeError("missing slave joint3 position")
    return joint.position_rad


def _enable_joint3_mit(slave) -> None:
    item = _find_runtime_item(slave, JOINT_NAME)
    slave.controller.enable(item.motor)
    time.sleep(0.05)


def _send_joint3_mit(slave, target_rad: float, kp: float, kd: float, tau: float) -> None:
    item = _find_runtime_item(slave, JOINT_NAME)
    slave.controller.controlMIT(item.motor, kp, kd, target_rad, 0.0, tau)


def _find_runtime_item(connection, joint_name: str) -> Any:
    for item in connection.items:
        if item.config.name == joint_name:
            return item
    raise RuntimeError(f"{joint_name}: runtime item not found")


def _position_map(snapshot: ArmStatusSnapshot) -> dict[str, float | None]:
    return {joint.name: joint.position_rad for joint in snapshot.joints}


def _rate_limit(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def _relax_limit(limits: list[JointLimit], joint_name: str, relax_rad: float) -> list[JointLimit]:
    if relax_rad <= 0.0:
        return limits
    return [
        JointLimit(limit.name, limit.min_rad - relax_rad, limit.max_rad + relax_rad)
        if limit.name == joint_name
        else limit
        for limit in limits
    ]


def _find_limit(limits: list[JointLimit], joint_name: str) -> JointLimit:
    for limit in limits:
        if limit.name == joint_name:
            return limit
    raise RuntimeError(f"{joint_name}: missing joint limit")


def _limit_status(target_limited: float, raw_target: float) -> str:
    if target_limited < raw_target:
        return "LIMIT_MAX"
    if target_limited > raw_target:
        return "LIMIT_MIN"
    return "OK"


def _read_key() -> str | None:
    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getwch()
    if ch in ("\x00", "\xe0"):
        code = msvcrt.getwch()
        if code == "H":
            return "up"
        if code == "P":
            return "down"
        return None
    return ch.lower()


def _draw(params: list[TunableParam], cursor: int, status: dict[str, Any], limit: JointLimit, sign: int, scale: float, message: str) -> None:
    os.system("cls")
    print("Pink master joint3 -> pink slave joint3 MIT tuning console")
    print("Keys: Up/Down or c/n select | -/= adjust | [/ ] step | h recapture anchors | space pause | q quit")
    print("Safety: real MIT commands are being sent. Keep physical E-stop ready.")
    print()
    print(f"Mapping: sign={sign:+d}, scale={scale:.3f}")
    print(f"Software limit: [{limit.min_rad:+.4f}, {limit.max_rad:+.4f}] rad")
    print()
    print("Tunable parameters:")
    for index, item in enumerate(params):
        pointer = ">" if index == cursor else " "
        print(f"{pointer} {item.name:<16} value={item.value:+8.4f}  step={item.step:<7.4f}  {item.description}")
    print()
    if status:
        print("Live status:")
        print(
            f"  master={status['master_now']:+.4f}  "
            f"master_delta={status['master_delta']:+.4f}  "
            f"slave_now={status['slave_now']:+.4f}"
        )
        print(
            f"  raw_target={status['target_raw']:+.4f}  "
            f"cmd_target={status['target_cmd']:+.4f}  "
            f"err={status['err']:+.4f}  "
            f"limit={status['limit']}"
        )
        print(
            f"  MIT used: kp={status['kp_used']:.2f}, kd={status['kd_used']:.2f}, "
            f"tau={status['tau_used']:+.3f}, paused={status['paused']}"
        )
    print()
    print(f"Message: {message}")


if __name__ == "__main__":
    main()
