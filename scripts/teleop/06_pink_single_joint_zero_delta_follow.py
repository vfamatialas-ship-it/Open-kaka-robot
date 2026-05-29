"""Pink master to real pink slave single-joint zero-delta follow.

Safety:
- This script moves exactly one real Damiao slave joint.
- Motion is refused unless --enable-motion YES is provided.
- Startup reads the current slave position and commands a hold first.
- Mapping uses saved software zeros:

    q_slave_target = q_slave_zero
                   + sign * scale * (q_master_now - q_master_zero)
                   + offset

- Every command is clamped to the saved pink_slave software limits.
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink master -> pink slave zero-delta single-joint follow.")
    parser.add_argument("--joint", default="joint1", choices=[f"joint{i}" for i in range(1, 8)])
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--alpha", type=float, default=None, help="override teleop_mapping.yaml alpha")
    parser.add_argument("--max-step-rad", type=float, default=None, help="override teleop_mapping.yaml max_step_rad")
    parser.add_argument("--damiao-wait", type=float, default=None)
    parser.add_argument("--scale", type=float, default=None, help="temporary scale override")
    parser.add_argument("--sign", type=int, choices=[-1, 1], default=None, help="temporary sign override")
    parser.add_argument(
        "--motor-direction",
        type=int,
        choices=[-1, 1],
        default=1,
        help="real Damiao direction compensation; use -1 if real slave follows opposite to virtual sign",
    )
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--kd", type=float, default=None)
    parser.add_argument("--tau", type=float, default=None)
    parser.add_argument("--control-mode", choices=["mit", "posvel"], default=None)
    parser.add_argument("--posvel-velocity", type=float, default=None)
    parser.add_argument("--enable-old-mode", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--switch-mode", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--slave-read-every", type=int, default=None)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument(
        "--max-master-miss",
        type=int,
        default=5,
        help="disable after this many consecutive missing master reads",
    )
    parser.add_argument("--enable-each-command", action="store_true")
    parser.add_argument("--enable-motion", default="NO", help="must be exactly YES")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.enable_motion != "YES":
        raise SystemExit(
            "Refusing to move a real slave joint.\n"
            "Use --enable-motion YES only after virtual/Dry Run is correct."
        )

    mapping = load_teleop_mapping("pink_master", "pink_slave")
    runtime_alpha = _clamp(args.alpha if args.alpha is not None else mapping.runtime.alpha, 0.0, 1.0)
    max_step = max(0.0, args.max_step_rad if args.max_step_rad is not None else mapping.runtime.max_step_rad)
    damiao_wait = max(0.0, args.damiao_wait if args.damiao_wait is not None else mapping.runtime.damiao_response_wait)

    joint_map = {joint.slave_joint: joint for joint in mapping.joints}
    setting = joint_map[args.joint]
    if not setting.enabled:
        raise SystemExit(f"{args.joint} is disabled in configs/teleop_mapping.yaml")
    if setting.mapping_mode != "zero_delta":
        raise SystemExit(f"{args.joint} mapping_mode is {setting.mapping_mode!r}, expected 'zero_delta'")

    sign = args.sign if args.sign is not None else setting.sign
    effective_sign = sign * args.motor_direction
    scale = args.scale if args.scale is not None else setting.scale
    offset = setting.offset_rad

    master_zero = _load_zero("pink_master", args.joint)
    slave_zero = _load_zero("pink_slave", args.joint)
    limits = parse_limits_text(load_limits_text("pink_slave"))
    limit = _limit_for(args.joint, limits)
    control = _joint_control(args)

    print("Pink master -> real pink slave ZERO-DELTA single joint follow")
    print("WARNING: real Damiao motion commands will be sent.")
    print(f"Joint:  {args.joint}")
    print(f"Master: pink_master @ {args.master_port} {args.master_baudrate}")
    print(f"Slave:  pink_slave  @ {args.slave_port} {args.slave_baudrate}")
    print(
        "Mapping: "
        f"q_slave_zero + effective_sign({effective_sign:+d}) * "
        f"scale({scale:.3f}) * (q_master - q_master_zero)"
    )
    print(f"  config sign={sign:+d}, real motor_direction={args.motor_direction:+d}")
    print(f"Zeros: master={master_zero:+.6f} rad, slave={slave_zero:+.6f} rad")
    print(f"Limit: [{limit.min_rad:+.6f}, {limit.max_rad:+.6f}] rad")
    print(f"Runtime: hz={args.hz:.1f}, alpha={runtime_alpha:.3f}, max_step_rad={max_step:.4f}, damiao_wait={damiao_wait:.4f}s")
    print(
        "Damiao: "
        f"mode={control['control_mode']}, kp={control['kp']:.2f}, kd={control['kd']:.2f}, "
        f"tau={control['tau']:.3f}, posvel_velocity={control['posvel_velocity']:.3f}, "
        f"switch_mode={control['switch_mode']}, enable_old_mode={control['enable_old_mode']}"
    )
    print("Press Ctrl+C to stop and disable the slave arm.")
    print()

    master = open_arm_connection("pink_master", port=args.master_port, baudrate=args.master_baudrate)
    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)

    previous_target: float | None = None
    last_slave_position: float | None = None
    last_master_raw: float | None = None
    master_continuous: float | None = None
    master_miss_count = 0
    try:
        slave_snapshot = slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=damiao_wait)
        slave_now = _position_or_raise(slave_snapshot, args.joint)
        last_slave_position = slave_now
        previous_target = slave_now

        print("Holding current slave position before follow...")
        command_single_joint_target(
            slave,
            args.joint,
            slave_now,
            limits,
            enable_motor=True,
            kp=control["kp"],
            kd=control["kd"],
            tau=control["tau"],
            control_mode=control["control_mode"],
            posvel_velocity=control["posvel_velocity"],
            enable_old_mode=control["enable_old_mode"],
            switch_mode=control["switch_mode"],
        )
        time.sleep(0.1)

        period = 1.0 / max(1.0, args.hz)
        slave_read_every = max(1, int(args.slave_read_every or control["slave_read_every"]))
        cycle = 0
        while args.cycles <= 0 or cycle < args.cycles:
            cycle += 1
            started = time.perf_counter()

            master_snapshot = master.read_snapshot(joint_names={args.joint})
            master_raw = _position_or_none(master_snapshot, args.joint)
            if master_raw is None:
                master_miss_count += 1
                if master_miss_count > max(1, args.max_master_miss):
                    raise RuntimeError(f"{args.joint}: missing master position for {master_miss_count} consecutive cycles")
                command_single_joint_target(
                    slave,
                    args.joint,
                    previous_target,
                    limits,
                    enable_motor=False,
                    kp=control["kp"],
                    kd=control["kd"],
                    tau=control["tau"],
                    control_mode=control["control_mode"],
                    posvel_velocity=control["posvel_velocity"],
                    enable_old_mode=control["enable_old_mode"],
                    switch_mode=control["switch_mode"],
                )
                print(
                    "\r\x1b[2K"
                    f"{args.joint} master read miss {master_miss_count}/{max(1, args.max_master_miss)} "
                    f"holding target={previous_target:+.3f}",
                    end="",
                    flush=True,
                )
                elapsed = time.perf_counter() - started
                time.sleep(max(0.0, period - elapsed))
                continue
            master_miss_count = 0
            master_continuous, last_master_raw = _update_continuous_master(
                raw=master_raw,
                last_raw=last_master_raw,
                continuous=master_continuous,
                zero=master_zero,
            )
            master_now = master_continuous

            if cycle == 1 or cycle % slave_read_every == 0:
                slave_snapshot = slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=damiao_wait)
                read_slave = _position_or_none(slave_snapshot, args.joint)
                if read_slave is not None:
                    last_slave_position = read_slave

            raw_target = slave_zero + effective_sign * scale * (master_now - master_zero) + offset
            filtered = previous_target + runtime_alpha * (raw_target - previous_target)
            stepped = _limit_step(previous_target, filtered, max_step)
            target, status = _clamp_to_limit(stepped, limit)
            previous_target = target

            command_single_joint_target(
                slave,
                args.joint,
                target,
                limits,
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
                err = None if last_slave_position is None else target - last_slave_position
                print(
                    "\r\x1b[2K"
                    f"{args.joint} "
                    f"m_delta={master_now - master_zero:+.3f} "
                    f"slave={_fmt(last_slave_position)} "
                    f"target={target:+.3f} "
                    f"err={_fmt(err)} "
                    f"{status}",
                    end="",
                    flush=True,
                )

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


def _joint_control(args) -> dict[str, float | str | bool | int]:
    gain = get_joint_gain(args.joint, "pink_slave")
    return {
        "kp": max(0.0, args.kp if args.kp is not None else gain.kp),
        "kd": max(0.0, args.kd if args.kd is not None else gain.kd),
        "tau": args.tau if args.tau is not None else gain.tau,
        "control_mode": args.control_mode or gain.control_mode,
        "posvel_velocity": max(0.0, args.posvel_velocity if args.posvel_velocity is not None else gain.posvel_velocity),
        "enable_old_mode": gain.enable_old_mode if args.enable_old_mode is None else args.enable_old_mode,
        "switch_mode": gain.switch_mode if args.switch_mode is None else args.switch_mode,
        "slave_read_every": max(1, int(args.slave_read_every or gain.slave_read_every)),
    }


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


def _limit_for(joint_name: str, limits: list[JointLimit]) -> JointLimit:
    for limit in limits:
        if limit.name == joint_name:
            return limit
    raise RuntimeError(f"{joint_name}: missing software limit")


def _unwrap_near(raw: float, reference: float) -> float:
    value = raw
    while value - reference > math.pi:
        value -= math.tau
    while value - reference < -math.pi:
        value += math.tau
    return value


def _update_continuous_master(
    *,
    raw: float,
    last_raw: float | None,
    continuous: float | None,
    zero: float,
) -> tuple[float, float]:
    """Track master position continuously across the 0/2pi encoder wrap.

    The safety-critical point is that a hand-driven master may pass beyond the
    slave's software range. If we fold every sample back near zero, an angle past
    one limit can look like a valid target on the opposite side. Continuous
    tracking keeps the over-limit side over-limit, so the slave stays saturated
    at the boundary until the master moves back.
    """

    if last_raw is None or continuous is None:
        return _unwrap_near(raw, zero), raw

    delta = raw - last_raw
    while delta > math.pi:
        delta -= math.tau
    while delta < -math.pi:
        delta += math.tau
    return continuous + delta, raw


def _limit_step(previous: float, target: float, max_step: float) -> float:
    if max_step <= 0:
        return previous
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target


def _clamp_to_limit(target: float, limit: JointLimit) -> tuple[float, str]:
    if target < limit.min_rad:
        return limit.min_rad, "LIMIT_MIN"
    if target > limit.max_rad:
        return limit.max_rad, "LIMIT_MAX"
    return target, "OK"


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _fmt(value: float | None) -> str:
    if value is None:
        return "None"
    return f"{value:+.4f}"


if __name__ == "__main__":
    main()
