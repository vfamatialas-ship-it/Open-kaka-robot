"""Multi-joint zero-delta teleoperation test.

This script is the next step after all seven single-joint tests passed.

Safety:
- Motion is refused unless --enable-motion YES is provided.
- Uses saved software zeros and software joint limits.
- Optional extra test clamp can be enabled with --max-slave-delta-rad.
- If a master read misses briefly, the previous target is held.
- Ctrl+C disables the full pink slave arm.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import ArmStatusSnapshot, open_arm_connection
from robot_core.services.joint_limit_service import JointLimit, load_limits_text, parse_limits_text
from robot_core.services.single_joint_motion_service import command_single_joint_target
from robot_core.services.single_joint_motion_service import disable_arm_motion
from robot_core.services.teleop_gain_config import get_joint_gain
from robot_core.services.teleop_mapping_config import load_teleop_mapping
from robot_core.services.zero_service import load_zero_text


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink master -> pink slave multi-joint zero-delta follow.")
    parser.add_argument("--joints", nargs="*", default=list(JOINT_NAMES), choices=JOINT_NAMES)
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--alpha", type=float, default=None)
    parser.add_argument("--max-step-rad", type=float, default=None)
    parser.add_argument(
        "--max-target-speed-rad-s",
        type=float,
        default=None,
        help="optional target velocity limit in rad/s; converted to per-cycle max step",
    )
    parser.add_argument("--damiao-wait", type=float, default=None)
    parser.add_argument(
        "--master-deadband-rad",
        type=float,
        default=0.004,
        help="ignore tiny master position changes to reduce jitter",
    )
    parser.add_argument(
        "--target-deadband-rad",
        type=float,
        default=0.003,
        help="hold previous target when target change is tiny",
    )
    parser.add_argument("--test-scale", type=float, default=0.3, help="extra scale multiplier for small-range testing")
    parser.add_argument(
        "--joint-scale",
        action="append",
        default=[],
        help="optional per-joint extra scale override, e.g. joint3=1.0",
    )
    parser.add_argument(
        "--joint-tau",
        action="append",
        default=[],
        help="optional per-joint MIT tau override, e.g. joint3=-0.05",
    )
    parser.add_argument(
        "--joint-kp",
        action="append",
        default=[],
        help="optional per-joint MIT kp override, e.g. joint2=12",
    )
    parser.add_argument(
        "--joint-kd",
        action="append",
        default=[],
        help="optional per-joint MIT kd override, e.g. joint2=1.0",
    )
    parser.add_argument(
        "--max-slave-delta-rad",
        type=float,
        default=0.0,
        help="optional temporary clamp around each slave zero; 0 disables this extra test clamp",
    )
    parser.add_argument(
        "--motor-direction",
        action="append",
        default=[],
        help="optional real motor direction compensation, e.g. joint3=-1",
    )
    parser.add_argument("--slave-read-every", type=int, default=2)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument("--max-master-miss", type=int, default=5)
    parser.add_argument("--enable-each-command", action="store_true")
    parser.add_argument("--enable-motion", default="NO", help="must be exactly YES")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.enable_motion != "YES":
        raise SystemExit(
            "Refusing to move real slave joints.\n"
            "Use --enable-motion YES only after single-joint tests are correct."
        )

    joint_names = tuple(dict.fromkeys(args.joints))
    mapping = load_teleop_mapping("pink_master", "pink_slave")
    runtime_alpha = _clamp(args.alpha if args.alpha is not None else mapping.runtime.alpha, 0.0, 1.0)
    max_step = max(0.0, args.max_step_rad if args.max_step_rad is not None else mapping.runtime.max_step_rad)
    max_speed_step = _speed_to_step(args.max_target_speed_rad_s, args.hz)
    if max_speed_step is not None:
        max_step = min(max_step, max_speed_step) if max_step > 0.0 else max_speed_step
    damiao_wait = max(0.0, args.damiao_wait if args.damiao_wait is not None else mapping.runtime.damiao_response_wait)
    motor_directions = _parse_motor_directions(args.motor_direction)
    joint_scale_overrides = _parse_joint_float_overrides(args.joint_scale, "--joint-scale")
    joint_tau_overrides = _parse_joint_float_overrides(args.joint_tau, "--joint-tau")
    joint_kp_overrides = _parse_joint_float_overrides(args.joint_kp, "--joint-kp")
    joint_kd_overrides = _parse_joint_float_overrides(args.joint_kd, "--joint-kd")

    mapping_by_joint = {joint.slave_joint: joint for joint in mapping.joints}
    master_zero = {name: _load_zero("pink_master", name) for name in joint_names}
    slave_zero = {name: _load_zero("pink_slave", name) for name in joint_names}
    controls = {name: _joint_control(name) for name in joint_names}
    for name, tau in joint_tau_overrides.items():
        if name in controls:
            controls[name]["tau"] = tau
    for name, kp in joint_kp_overrides.items():
        if name in controls:
            controls[name]["kp"] = max(0.0, kp)
    for name, kd in joint_kd_overrides.items():
        if name in controls:
            controls[name]["kd"] = max(0.0, kd)
    saved_limits = parse_limits_text(load_limits_text("pink_slave"))
    saved_limit_by_joint = {limit.name: limit for limit in saved_limits}
    effective_limit_by_joint = {
        name: _relax_limit(saved_limit_by_joint[name], float(controls[name]["limit_relax_rad"]))
        for name in joint_names
        if name in saved_limit_by_joint
    }
    effective_limits = list(effective_limit_by_joint.values())

    for name in joint_names:
        setting = mapping_by_joint.get(name)
        if setting is None or not setting.enabled:
            raise SystemExit(f"{name} is disabled or missing in configs/teleop_mapping.yaml")
        if setting.mapping_mode != "zero_delta":
            raise SystemExit(f"{name} mapping_mode is {setting.mapping_mode!r}, expected 'zero_delta'")
        if name not in effective_limit_by_joint:
            raise SystemExit(f"{name}: missing pink_slave software limit")

    print("Pink master -> real pink slave multi-joint zero-delta follow")
    print("WARNING: real Damiao motion commands will be sent.")
    print(f"Joints: {', '.join(joint_names)}")
    print(f"Master: pink_master @ {args.master_port} {args.master_baudrate}")
    print(f"Slave:  pink_slave  @ {args.slave_port} {args.slave_baudrate}")
    print(
        "Runtime: "
        f"hz={args.hz:.1f}, alpha={runtime_alpha:.3f}, max_step_rad={max_step:.4f}, "
        f"test_scale={args.test_scale:.3f}, max_slave_delta={args.max_slave_delta_rad:.3f}, "
        f"damiao_wait={damiao_wait:.4f}s"
    )
    if args.max_target_speed_rad_s is not None:
        print(
            "Target speed limit: "
            f"{max(0.0, args.max_target_speed_rad_s):.3f} rad/s "
            f"(effective max_step_rad={max_step:.4f})"
        )
    if joint_scale_overrides:
        print(f"Per-joint scale overrides: {joint_scale_overrides}")
    if joint_tau_overrides:
        print(f"Per-joint tau overrides: {joint_tau_overrides}")
    if joint_kp_overrides:
        print(f"Per-joint kp overrides: {joint_kp_overrides}")
    if joint_kd_overrides:
        print(f"Per-joint kd overrides: {joint_kd_overrides}")
    if args.max_slave_delta_rad > 0.0:
        print("Extra test clamp: target is limited around each slave zero before software limit check.")
    else:
        print("Extra test clamp: disabled. Only saved pink_slave software limits are enforced.")
    print("Press Ctrl+C to stop and disable the slave arm.")
    print()

    master = open_arm_connection("pink_master", port=args.master_port, baudrate=args.master_baudrate)
    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)

    previous_targets: dict[str, float] = {}
    last_slave_positions: dict[str, float | None] = {name: None for name in joint_names}
    last_master_raw: dict[str, float] = {}
    master_continuous: dict[str, float] = {}
    last_used_master: dict[str, float] = {}
    master_misses: dict[str, int] = {name: 0 for name in joint_names}

    try:
        slave_snapshot = slave.read_snapshot(joint_names=set(joint_names), damiao_response_wait=damiao_wait)
        for name in joint_names:
            slave_now = _position_or_raise(slave_snapshot, name)
            last_slave_positions[name] = slave_now
            previous_targets[name] = slave_now

        print("Holding current slave positions before multi-joint follow...")
        for name in joint_names:
            control = controls[name]
            command_single_joint_target(
                slave,
                name,
                previous_targets[name],
                effective_limits,
                enable_motor=True,
                kp=control["kp"],
                kd=control["kd"],
                tau=control["tau"],
                control_mode=control["control_mode"],
                posvel_velocity=control["posvel_velocity"],
                enable_old_mode=control["enable_old_mode"],
                switch_mode=control["switch_mode"],
            )
            time.sleep(0.02)

        period = 1.0 / max(1.0, args.hz)
        cycle = 0
        while args.cycles <= 0 or cycle < args.cycles:
            cycle += 1
            started = time.perf_counter()

            master_snapshot = master.read_snapshot(joint_names=set(joint_names))
            if cycle == 1 or cycle % max(1, args.slave_read_every) == 0:
                slave_snapshot = slave.read_snapshot(joint_names=set(joint_names), damiao_response_wait=damiao_wait)
                for name in joint_names:
                    value = _position_or_none(slave_snapshot, name)
                    if value is not None:
                        last_slave_positions[name] = value

            statuses: dict[str, str] = {}
            for name in joint_names:
                master_raw = _position_or_none(master_snapshot, name)
                if master_raw is None:
                    master_misses[name] += 1
                    if master_misses[name] > max(1, args.max_master_miss):
                        raise RuntimeError(f"{name}: missing master position for {master_misses[name]} consecutive cycles")
                    target = previous_targets[name]
                    statuses[name] = "MISS"
                else:
                    master_misses[name] = 0
                    master_continuous[name], last_master_raw[name] = _update_continuous_master(
                        raw=master_raw,
                        last_raw=last_master_raw.get(name),
                        continuous=master_continuous.get(name),
                        zero=master_zero[name],
                    )
                    master_for_mapping = _deadband_master(
                        current=master_continuous[name],
                        previous=last_used_master.get(name),
                        deadband=max(0.0, args.master_deadband_rad),
                    )
                    last_used_master[name] = master_for_mapping
                    setting = mapping_by_joint[name]
                    effective_sign = setting.sign * motor_directions.get(name, 1)
                    raw_target = (
                        slave_zero[name]
                        + effective_sign
                        * setting.scale
                        * joint_scale_overrides.get(name, args.test_scale)
                        * (master_for_mapping - master_zero[name])
                        + setting.offset_rad
                    )
                    filtered = previous_targets[name] + runtime_alpha * (raw_target - previous_targets[name])
                    stepped = _limit_step(previous_targets[name], filtered, max_step)
                    stepped = _deadband_target(
                        previous=previous_targets[name],
                        target=stepped,
                        deadband=max(0.0, args.target_deadband_rad),
                    )
                    target, statuses[name] = _clamp_to_test_and_software_limits(
                        stepped,
                        slave_zero[name],
                        max(0.0, args.max_slave_delta_rad),
                        effective_limit_by_joint[name],
                    )
                    previous_targets[name] = target

                control = controls[name]
                command_single_joint_target(
                    slave,
                    name,
                    previous_targets[name],
                    effective_limits,
                    enable_motor=args.enable_each_command,
                    kp=control["kp"],
                    kd=control["kd"],
                    tau=control["tau"],
                    control_mode=control["control_mode"],
                    posvel_velocity=control["posvel_velocity"],
                    enable_old_mode=control["enable_old_mode"],
                    switch_mode=control["switch_mode"],
                )

            if args.print_every > 0 and cycle % args.print_every == 0:
                print(_status_line(joint_names, previous_targets, statuses), end="", flush=True)

            elapsed = time.perf_counter() - started
            time.sleep(max(0.0, period - elapsed))

    except KeyboardInterrupt:
        print("\nStopping and disabling slave arm...")
    finally:
        try:
            for line in disable_arm_motion(slave):
                print(line)
        except Exception as exc:  # noqa: BLE001
            print(f"Disable failed: {exc}")
        master.close()
        slave.close()


def _joint_control(joint_name: str) -> dict[str, float | str | bool]:
    gain = get_joint_gain(joint_name, "pink_slave")
    return {
        "kp": gain.kp,
        "kd": gain.kd,
        "tau": gain.tau,
        "control_mode": gain.control_mode,
        "posvel_velocity": gain.posvel_velocity,
        "enable_old_mode": gain.enable_old_mode,
        "switch_mode": gain.switch_mode,
        "limit_relax_rad": gain.limit_relax_rad,
    }


def _relax_limit(limit: JointLimit, relax_rad: float) -> JointLimit:
    relax = max(0.0, relax_rad)
    if relax <= 0.0:
        return limit
    return JointLimit(
        name=limit.name,
        min_rad=limit.min_rad - relax,
        max_rad=limit.max_rad + relax,
    )


def _load_zero(profile_key: str, joint_name: str) -> float:
    current_name: str | None = None
    for raw_line in load_zero_text(profile_key).splitlines():
        line = raw_line.strip()
        if line.startswith("- name:"):
            current_name = line.split(":", 1)[1].strip()
            continue
        if current_name == joint_name and line.startswith("zero_position_rad:"):
            value = line.split(":", 1)[1].strip()
            if value.lower() == "null":
                break
            return float(value)
    raise RuntimeError(f"{profile_key}: missing zero_position_rad for {joint_name}")


def _position_or_raise(snapshot: ArmStatusSnapshot, joint_name: str) -> float:
    value = _position_or_none(snapshot, joint_name)
    if value is None:
        raise RuntimeError(f"{joint_name}: missing position in {snapshot.profile_label}")
    return value


def _position_or_none(snapshot: ArmStatusSnapshot, joint_name: str) -> float | None:
    for joint in snapshot.joints:
        if joint.name == joint_name:
            return joint.position_rad
    return None


def _parse_motor_directions(items: list[str]) -> dict[str, int]:
    directions: dict[str, int] = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"bad --motor-direction {item!r}, expected jointN=-1 or jointN=1")
        name, value = item.split("=", 1)
        if name not in JOINT_NAMES or value not in {"-1", "1"}:
            raise SystemExit(f"bad --motor-direction {item!r}, expected jointN=-1 or jointN=1")
        directions[name] = int(value)
    return directions


def _parse_joint_float_overrides(items: list[str], option_name: str) -> dict[str, float]:
    overrides: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"bad {option_name} {item!r}, expected jointN=value")
        name, value = item.split("=", 1)
        if name not in JOINT_NAMES:
            raise SystemExit(f"bad {option_name} {item!r}, unknown joint name")
        try:
            overrides[name] = float(value)
        except ValueError as exc:
            raise SystemExit(f"bad {option_name} {item!r}, value must be a number") from exc
    return overrides


def _update_continuous_master(
    *,
    raw: float,
    last_raw: float | None,
    continuous: float | None,
    zero: float,
) -> tuple[float, float]:
    if last_raw is None or continuous is None:
        return _unwrap_near(raw, zero), raw

    delta = raw - last_raw
    while delta > math.pi:
        delta -= math.tau
    while delta < -math.pi:
        delta += math.tau
    return continuous + delta, raw


def _unwrap_near(raw: float, reference: float) -> float:
    value = raw
    while value - reference > math.pi:
        value -= math.tau
    while value - reference < -math.pi:
        value += math.tau
    return value


def _limit_step(previous: float, target: float, max_step: float) -> float:
    if max_step <= 0:
        return previous
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target


def _speed_to_step(speed_rad_s: float | None, hz: float) -> float | None:
    if speed_rad_s is None:
        return None
    if speed_rad_s <= 0.0:
        return 0.0
    return speed_rad_s / max(1.0, hz)


def _deadband_master(current: float, previous: float | None, deadband: float) -> float:
    if previous is None:
        return current
    if abs(current - previous) < deadband:
        return previous
    return current


def _deadband_target(previous: float, target: float, deadband: float) -> float:
    if abs(target - previous) < deadband:
        return previous
    return target


def _clamp_to_test_and_software_limits(
    target: float,
    slave_zero: float,
    max_delta: float,
    limit: JointLimit,
) -> tuple[float, str]:
    if max_delta <= 0.0:
        if target < limit.min_rad:
            return limit.min_rad, "LIMIT_MIN"
        if target > limit.max_rad:
            return limit.max_rad, "LIMIT_MAX"
        return target, "OK"

    test_min = max(limit.min_rad, slave_zero - max_delta)
    test_max = min(limit.max_rad, slave_zero + max_delta)
    if target < test_min:
        return test_min, "TEST_MIN"
    if target > test_max:
        return test_max, "TEST_MAX"
    return target, "OK"


def _status_line(
    joint_names: tuple[str, ...],
    targets: dict[str, float],
    statuses: dict[str, str],
) -> str:
    chunks = [
        f"{name[-1]}:{targets[name]:+.2f}/{statuses.get(name, '?')}"
        for name in joint_names
    ]
    return "\r\x1b[2K" + " ".join(chunks)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


if __name__ == "__main__":
    main()
