"""Probe Damiao MIT tau direction for one pink slave joint.

This script does not use the master arm. It holds the selected slave joint at
its current position and briefly applies +tau and -tau. Use it to understand
which tau sign helps support gravity for a loaded joint.

Safety:
- Refuses to run unless --enable-motion YES is provided.
- Reads current position first, then holds q_des=q_now.
- Stops and disables all pink slave motors on Ctrl+C or after the probe.
- Aborts if the joint moves more than --max-motion-rad during a probe.
"""

from __future__ import annotations

import argparse
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


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe MIT tau direction on one pink slave joint.")
    parser.add_argument("--joint", default="joint2", choices=JOINT_NAMES)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--kd", type=float, default=None)
    parser.add_argument("--tau", type=float, default=0.03, help="absolute tau magnitude to test")
    parser.add_argument("--seconds", type=float, default=1.2, help="duration for each +tau/-tau probe")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--damiao-wait", type=float, default=0.005)
    parser.add_argument("--max-motion-rad", type=float, default=0.12)
    parser.add_argument("--enable-motion", default="NO", help="must be exactly YES")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.enable_motion != "YES":
        raise SystemExit(
            "Refusing to send real Damiao commands.\n"
            "Use --enable-motion YES only with physical E-stop ready."
        )

    gain = get_joint_gain(args.joint, "pink_slave")
    kp = max(0.0, args.kp if args.kp is not None else min(gain.kp, 8.0))
    kd = max(0.0, args.kd if args.kd is not None else min(gain.kd, 0.8))
    tau_abs = abs(args.tau)
    limits = _relax_limits(parse_limits_text(load_limits_text("pink_slave")), args.joint, gain.limit_relax_rad)

    print("Pink slave MIT tau direction probe / 粉色从臂 MIT tau 方向探测")
    print("WARNING: real Damiao MIT commands will be sent.")
    print(f"Joint: {args.joint}")
    print(f"Slave: pink_slave @ {args.slave_port} {args.slave_baudrate}")
    print(f"Hold gains: kp={kp:.2f}, kd={kd:.2f}")
    print(f"Probe tau: +{tau_abs:.3f}, -{tau_abs:.3f}")
    print("Meaning: positive/negative drift shows motor torque direction in joint radians.")
    print("含义：漂移方向代表该 tau 符号对应的关节角方向。")
    print()

    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)
    try:
        q_hold = _read_position(slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=args.damiao_wait), args.joint)
        print(f"Hold target q_des=q_now={q_hold:+.6f} rad")

        _send_hold(slave, args.joint, q_hold, limits, kp, kd, tau=0.0, enable_motor=True)
        time.sleep(0.3)

        plus_result = _run_probe(slave, args, q_hold, limits, kp, kd, +tau_abs)
        _send_hold(slave, args.joint, q_hold, limits, kp, kd, tau=0.0, enable_motor=False)
        time.sleep(0.4)
        minus_result = _run_probe(slave, args, q_hold, limits, kp, kd, -tau_abs)

        print()
        print("Probe summary / 探测结果:")
        print(f"  +tau drift: {plus_result:+.6f} rad")
        print(f"  -tau drift: {minus_result:+.6f} rad")
        print()
        print("How to interpret / 如何判断:")
        print("  If +tau reduces sag in the loaded pose, use positive tau_ff.")
        print("  如果 +tau 在当前承重姿态下更能抵抗下垂，就用正 tau_ff。")
        print("  If -tau reduces sag, use negative tau_ff.")
        print("  如果 -tau 更能抵抗下垂，就用负 tau_ff。")
        print("  If both look similar, tau is too small or kp is masking the effect; lower kp or test a larger but safe tau.")
        print("  如果两边差不多，说明 tau 太小或 kp 掩盖了效果；可以降低 kp 或小幅增加 tau。")

    except KeyboardInterrupt:
        print("\nStopping and disabling slave arm...")
    finally:
        try:
            for line in disable_arm_motion(slave):
                print(line)
        except Exception as exc:  # noqa: BLE001
            print(f"Disable failed: {exc}")
        slave.close()


def _run_probe(
    slave,
    args,
    q_hold: float,
    limits: list[JointLimit],
    kp: float,
    kd: float,
    tau: float,
) -> float:
    period = 1.0 / max(1.0, args.hz)
    cycles = max(1, int(args.seconds / period))
    start = _read_position(slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=args.damiao_wait), args.joint)
    last = start
    print(f"Probe tau={tau:+.3f}: start={start:+.6f} rad")
    for _ in range(cycles):
        loop_start = time.perf_counter()
        _send_hold(slave, args.joint, q_hold, limits, kp, kd, tau=tau, enable_motor=False)
        snapshot = slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=args.damiao_wait)
        last = _read_position(snapshot, args.joint)
        drift = last - start
        print(f"\r\x1b[2K  q={last:+.6f} drift={drift:+.6f}", end="", flush=True)
        if abs(last - q_hold) > max(0.0, args.max_motion_rad):
            raise RuntimeError(
                f"{args.joint}: moved too far during tau probe: "
                f"q={last:+.6f}, hold={q_hold:+.6f}"
            )
        elapsed = time.perf_counter() - loop_start
        time.sleep(max(0.0, period - elapsed))
    print()
    return last - start


def _send_hold(
    slave,
    joint_name: str,
    q_hold: float,
    limits: list[JointLimit],
    kp: float,
    kd: float,
    *,
    tau: float,
    enable_motor: bool,
) -> None:
    command_single_joint_target(
        slave,
        joint_name,
        q_hold,
        limits,
        enable_motor=enable_motor,
        kp=kp,
        kd=kd,
        tau=tau,
        control_mode="mit",
        posvel_velocity=1.0,
        enable_old_mode=False,
        switch_mode=False,
    )


def _read_position(snapshot: ArmStatusSnapshot, joint_name: str) -> float:
    for joint in snapshot.joints:
        if joint.name == joint_name and joint.position_rad is not None:
            return joint.position_rad
    raise RuntimeError(f"{joint_name}: no valid position in snapshot")


def _relax_limits(limits: list[JointLimit], joint_name: str, relax_rad: float) -> list[JointLimit]:
    relaxed: list[JointLimit] = []
    relax = max(0.0, relax_rad)
    for limit in limits:
        if limit.name != joint_name or relax <= 0.0:
            relaxed.append(limit)
        else:
            relaxed.append(JointLimit(limit.name, limit.min_rad - relax, limit.max_rad + relax))
    return relaxed


if __name__ == "__main__":
    main()
