"""Single-joint real follow with software limit saturation.

This is the first real-motion teleoperation step after Dry Run.

Safety behavior:
- Default is not enough to move: you must pass ``--enable-motion YES``.
- Only one selected joint is allowed to follow.
- Startup reads current master/slave positions and uses them as anchors.
- The slave is commanded to hold its current position before following.
- Every target is clamped to the saved software joint limit before command.
- If the master moves beyond the slave range, the slave target stays at
  LIMIT_MIN or LIMIT_MAX instead of continuing past the algorithmic boundary.
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
from robot_core.services.single_joint_motion_service import disable_arm_motion, hold_current_position
from robot_core.services.teleop_gain_config import get_joint_gain
from robot_core.services.teleop_mapping_config import load_teleop_mapping
from robot_core.services.teleop_service import compute_mapping_preview
from robot_core.services.teleop_service import target_status_is_commandable
from robot_core.services.teleop_service import update_continuous_position_map
from robot_core.services.zero_service import load_zero_text


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pink master -> pink slave single-joint follow with limit saturation.")
    parser.add_argument("--joint", default="joint1", choices=[f"joint{i}" for i in range(1, 8)])
    parser.add_argument("--master-port", default="COM8")
    parser.add_argument("--master-baudrate", type=int, default=1000000)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--alpha", type=float, default=None, help="override teleop_mapping.yaml runtime alpha")
    parser.add_argument("--max-step-rad", type=float, default=None, help="override runtime max_step_rad")
    parser.add_argument("--damiao-wait", type=float, default=None, help="override runtime damiao_response_wait")
    parser.add_argument("--scale", type=float, default=None, help="temporary mapping scale override for selected joint")
    parser.add_argument("--sign", type=int, choices=[-1, 1], default=None, help="temporary mapping sign override")
    parser.add_argument("--limit-relax-rad", type=float, default=None, help="override configured temporary limit relax")
    parser.add_argument(
        "--hold-offset-rad",
        type=float,
        default=0.0,
        help="ignore master follow and command slave_anchor + offset as a fixed target",
    )
    parser.add_argument(
        "--slave-read-every",
        type=int,
        default=None,
        help="override configured slave feedback read cadence",
    )
    parser.add_argument("--print-every", type=int, default=5, help="print status every N control cycles")
    parser.add_argument("--debug-follow", action="store_true", help="print master delta and target delta while following")
    parser.add_argument("--kp", type=float, default=None, help="override Damiao MIT kp")
    parser.add_argument("--kd", type=float, default=None, help="override Damiao MIT kd")
    parser.add_argument("--tau", type=float, default=None, help="override Damiao MIT feed-forward torque")
    parser.add_argument("--control-mode", choices=["mit", "posvel"], default=None, help="override configured Damiao command mode")
    parser.add_argument("--posvel-velocity", type=float, default=None, help="override velocity for Damiao posvel mode")
    parser.add_argument(
        "--enable-old-mode",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="override configured Damiao old-firmware enable mode",
    )
    parser.add_argument(
        "--switch-mode",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="override configured Damiao internal mode switching",
    )
    parser.add_argument("--probe-step-rad", type=float, default=0.0, help="optional small safe slave probe before follow")
    parser.add_argument("--probe-seconds", type=float, default=1.0, help="duration for optional probe command streaming")
    parser.add_argument("--scan-probe", action="store_true", help="scan small kp/tau combinations, then exit")
    parser.add_argument("--enable-each-command", action="store_true", help="send Damiao enable before each target command")
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument(
        "--enable-motion",
        default="NO",
        help="must be exactly YES to send real Damiao motion commands",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.enable_motion != "YES":
        raise SystemExit(
            "Refusing to move. This script controls a real slave joint.\n"
            "Run again with --enable-motion YES only after Dry Run is correct."
        )

    mapping = load_teleop_mapping("pink_master", "pink_slave")
    _apply_runtime_overrides(mapping, args)
    joint_settings = mapping.to_joint_settings()
    for name, setting in joint_settings.items():
        setting.enabled = name == args.joint and setting.enabled
    if args.scale is not None:
        joint_settings[args.joint].scale = args.scale
    if args.sign is not None:
        joint_settings[args.joint].sign = args.sign
    if not joint_settings[args.joint].enabled:
        raise SystemExit(f"{args.joint} is disabled in configs/teleop_mapping.yaml")

    control = _select_joint_control(args.joint, args)
    limits = _relax_selected_joint_limit(
        parse_limits_text(load_limits_text("pink_slave")),
        args.joint,
        float(control["limit_relax_rad"]),
    )
    kp = float(control["kp"])
    kd = float(control["kd"])
    tau = float(control["tau"])

    print("Pink master -> real pink slave SINGLE JOINT FOLLOW")
    print("WARNING: real Damiao motion commands will be sent.")
    print(f"Joint:  {args.joint}")
    print(f"Master: pink_master @ {args.master_port} {args.master_baudrate}")
    print(f"Slave:  pink_slave  @ {args.slave_port} {args.slave_baudrate}")
    print(
        "Runtime: "
        f"hz={args.hz:.1f}, alpha={mapping.runtime.alpha:.3f}, "
        f"max_step_rad={mapping.runtime.max_step_rad:.4f}, "
        f"damiao_wait={mapping.runtime.damiao_response_wait:.4f}s, "
        f"slave_read_every={control['slave_read_every']}, "
        f"limit_relax={float(control['limit_relax_rad']):.4f}rad"
    )
    if control["control_mode"] == "mit":
        print(f"Damiao MIT gains: kp={kp:.2f}, kd={kd:.2f}, tau={tau:.3f}")
    else:
        print(f"Damiao PosVel: velocity={control['posvel_velocity']:.3f} rad/s")
    print(
        "Mode entry: "
        f"switch_mode={control['switch_mode']}, "
        f"enable_old_mode={control['enable_old_mode']}"
    )
    print("Safety: target is clamped to software limit before every command.")
    print("Press Ctrl+C for software disable. Keep physical E-stop ready.")
    print()

    master = open_arm_connection("pink_master", port=args.master_port, baudrate=args.master_baudrate)
    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)
    master_last_raw: dict[str, float] = {}
    master_continuous: dict[str, float] = {}
    previous_targets: dict[str, float] = {}

    try:
        print("Reading current positions as anchors...")
        master_anchor_snapshot = master.read_snapshot(joint_names={args.joint})
        slave_anchor_snapshot = slave.read_snapshot(
            joint_names={args.joint},
            damiao_response_wait=mapping.runtime.damiao_response_wait,
        )
        update_continuous_position_map(
            _position_map(master_anchor_snapshot),
            master_last_raw,
            master_continuous,
        )
        current_master_anchor = dict(master_continuous)
        current_slave_anchor = _required_anchor(slave_anchor_snapshot)
        if joint_settings[args.joint].mapping_mode == "zero_delta":
            master_anchor = _load_zero_anchor("pink_master", {args.joint})
            slave_anchor = _load_zero_anchor("pink_slave", {args.joint})
            print("Using software zero anchors for zero_delta mapping.")
        else:
            master_anchor = current_master_anchor
            slave_anchor = current_slave_anchor
            print("Using startup-current anchors for anchor_delta/range mapping.")

        print("Holding current slave position first...")
        result = hold_current_position(slave, args.joint, limits)
        print(f"  {args.joint}: q_now=q_des={result.target_rad:.6f} rad")
        print("Entering configured Damiao control mode at current position...")
        command_single_joint_target(
            slave,
            args.joint,
            result.target_rad,
            limits,
            enable_motor=True,
            kp=kp,
            kd=kd,
            tau=tau,
            control_mode=control["control_mode"],
            posvel_velocity=control["posvel_velocity"],
            enable_old_mode=control["enable_old_mode"],
            switch_mode=control["switch_mode"],
        )
        time.sleep(0.1)
        if args.scan_probe:
            _run_probe_scan(slave, args.joint, result.target_rad, limits, args.hz, args.probe_seconds)
            return
        if abs(args.probe_step_rad) > 0.0:
            _run_probe_step(
                slave,
                args.joint,
                result.target_rad,
                args.probe_step_rad,
                args.probe_seconds,
                args.hz,
                limits,
                kp,
                kd,
                tau,
                control["control_mode"],
                control["posvel_velocity"],
                control["enable_old_mode"],
                control["switch_mode"],
                args.enable_each_command,
            )

        period = 1.0 / max(1.0, args.hz)
        cycle = 0
        slave_snapshot = slave_anchor_snapshot
        slave_read_every = int(control["slave_read_every"])
        while args.cycles <= 0 or cycle < args.cycles:
            cycle += 1
            started = time.perf_counter()

            master_snapshot = master.read_snapshot(joint_names={args.joint})
            if cycle == 1 or cycle % slave_read_every == 0:
                new_slave_snapshot = slave.read_snapshot(
                    joint_names={args.joint},
                    damiao_response_wait=mapping.runtime.damiao_response_wait,
                )
                if _snapshot_has_joint_position(new_slave_snapshot, args.joint):
                    slave_snapshot = new_slave_snapshot
                elif args.debug_follow:
                    print(f"\n{args.joint}: slave feedback missing this cycle, reusing last valid sample")
            update_continuous_position_map(
                _position_map(master_snapshot),
                master_last_raw,
                master_continuous,
            )
            previews, previous_targets = compute_mapping_preview(
                master_snapshot=master_snapshot,
                slave_snapshot=slave_snapshot,
                joint_settings=joint_settings,
                runtime_settings=mapping.runtime,
                slave_limits=limits,
                master_anchor=master_anchor,
                slave_anchor=slave_anchor,
                previous_targets=previous_targets,
                master_continuous_positions=master_continuous,
            )
            preview = next(item for item in previews if item.name == args.joint)
            if abs(args.hold_offset_rad) > 0.0:
                target = _fixed_offset_target(args.joint, slave_anchor, limits, args.hold_offset_rad)
                preview = preview.__class__(
                    name=preview.name,
                    master_rad=preview.master_rad,
                    slave_current_rad=preview.slave_current_rad,
                    target_rad=target,
                    ratio=preview.ratio,
                    sign=preview.sign,
                    enabled=preview.enabled,
                    limit_status="OK_FIXED_OFFSET",
                )
            if preview.target_rad is None or not target_status_is_commandable(preview.limit_status):
                if preview.limit_status != "OK_FIXED_OFFSET":
                    raise RuntimeError(f"{args.joint}: target not commandable, status={preview.limit_status}")

            command_single_joint_target(
                slave,
                args.joint,
                preview.target_rad,
                limits,
                enable_motor=args.enable_each_command,
                kp=kp,
                kd=kd,
                tau=tau,
                control_mode=control["control_mode"],
                posvel_velocity=control["posvel_velocity"],
                enable_old_mode=control["enable_old_mode"],
                switch_mode=control["switch_mode"],
            )
            if args.print_every > 0 and cycle % args.print_every == 0:
                follow_error = None
                if preview.slave_current_rad is not None:
                    follow_error = preview.target_rad - preview.slave_current_rad
                master_delta = None
                if preview.master_rad is not None and args.joint in master_anchor:
                    master_delta = preview.master_rad - master_anchor[args.joint]
                target_delta = None
                if args.joint in slave_anchor:
                    target_delta = preview.target_rad - slave_anchor[args.joint]
                detail = ""
                if args.debug_follow:
                    detail = (
                        f" master_delta={_fmt(master_delta)}"
                        f" target_delta={_fmt(target_delta)}"
                    )
                print(
                    f"\r{args.joint} "
                    f"master={_fmt(preview.master_rad)} "
                    f"slave_now={_fmt(preview.slave_current_rad)} "
                    f"target={preview.target_rad:+.4f} "
                    f"err={_fmt(follow_error)} "
                    f"status={preview.limit_status:<9}"
                    f"{detail} ",
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


def _position_map(snapshot: ArmStatusSnapshot) -> dict[str, float | None]:
    return {joint.name: joint.position_rad for joint in snapshot.joints}


def _load_zero_anchor(profile_key: str, required_joints: set[str]) -> dict[str, float]:
    """Load software zero positions for mapping anchors."""

    zeros: dict[str, float] = {}
    current_name: str | None = None
    for raw_line in load_zero_text(profile_key).splitlines():
        line = raw_line.strip()
        if line.startswith("- name:"):
            current_name = line.split(":", 1)[1].strip()
            continue
        if current_name is not None and line.startswith("zero_position_rad:"):
            value = line.split(":", 1)[1].strip()
            if value and value.lower() != "null":
                zeros[current_name] = float(value)
            current_name = None

    missing = sorted(joint for joint in required_joints if joint not in zeros)
    if missing:
        raise RuntimeError(f"{profile_key}: missing software zero for {', '.join(missing)}")
    return {joint: zeros[joint] for joint in required_joints}


def _apply_runtime_overrides(mapping, args) -> None:
    """Apply conservative runtime overrides from command-line arguments."""

    if args.alpha is not None:
        mapping.runtime.alpha = max(0.0, min(1.0, args.alpha))
    if args.max_step_rad is not None:
        mapping.runtime.max_step_rad = max(0.0, args.max_step_rad)
    if args.damiao_wait is not None:
        mapping.runtime.damiao_response_wait = max(0.0, args.damiao_wait)


def _select_joint_control(joint_name: str, args) -> dict[str, float | str | bool]:
    """Load configured joint command mode, then apply command-line overrides."""

    gain = get_joint_gain(joint_name, "pink_slave")
    kp = gain.kp
    kd = gain.kd
    tau = gain.tau
    control_mode = gain.control_mode
    posvel_velocity = gain.posvel_velocity
    enable_old_mode = gain.enable_old_mode
    switch_mode = gain.switch_mode
    slave_read_every = gain.slave_read_every
    if args.kp is not None:
        kp = max(0.0, args.kp)
    if args.kd is not None:
        kd = max(0.0, args.kd)
    if args.tau is not None:
        tau = args.tau
    if args.control_mode is not None:
        control_mode = args.control_mode
    if args.posvel_velocity is not None:
        posvel_velocity = max(0.0, args.posvel_velocity)
    if args.enable_old_mode is not None:
        enable_old_mode = args.enable_old_mode
    if args.switch_mode is not None:
        switch_mode = args.switch_mode
    if args.slave_read_every is not None:
        slave_read_every = max(1, args.slave_read_every)
    limit_relax_rad = gain.limit_relax_rad
    if args.limit_relax_rad is not None:
        limit_relax_rad = max(0.0, args.limit_relax_rad)
    return {
        "kp": kp,
        "kd": kd,
        "tau": tau,
        "control_mode": control_mode,
        "posvel_velocity": posvel_velocity,
        "enable_old_mode": enable_old_mode,
        "switch_mode": switch_mode,
        "slave_read_every": slave_read_every,
        "limit_relax_rad": limit_relax_rad,
    }


def _required_anchor(snapshot: ArmStatusSnapshot) -> dict[str, float]:
    anchor = {}
    for joint in snapshot.joints:
        if joint.position_rad is None:
            raise RuntimeError(f"{joint.name}: missing slave position, cannot build follow anchor")
        anchor[joint.name] = joint.position_rad
    return anchor


def _snapshot_has_joint_position(snapshot: ArmStatusSnapshot, joint_name: str) -> bool:
    joint = next((item for item in snapshot.joints if item.name == joint_name), None)
    return joint is not None and joint.position_rad is not None


def _relax_selected_joint_limit(limits: list[JointLimit], joint_name: str, relax_rad: float) -> list[JointLimit]:
    """Return limits with a temporary debug relax applied to one joint only."""

    if relax_rad <= 0.0:
        return limits
    relaxed: list[JointLimit] = []
    for limit in limits:
        if limit.name == joint_name:
            relaxed.append(JointLimit(limit.name, limit.min_rad - relax_rad, limit.max_rad + relax_rad))
        else:
            relaxed.append(limit)
    return relaxed


def _fixed_offset_target(joint_name: str, slave_anchor: dict[str, float], limits, offset_rad: float) -> float:
    limit = next((item for item in limits if item.name == joint_name), None)
    if limit is None:
        raise RuntimeError(f"{joint_name}: missing software limit for fixed offset test")
    target = slave_anchor[joint_name] + offset_rad
    return max(limit.min_rad, min(limit.max_rad, target))


def _run_probe_step(
    slave,
    joint_name: str,
    current_rad: float,
    step_rad: float,
    seconds: float,
    hz: float,
    limits,
    kp: float,
    kd: float,
    tau: float,
    control_mode: str,
    posvel_velocity: float,
    enable_old_mode: bool,
    switch_mode: bool,
    enable_each_command: bool,
) -> None:
    """Stream one small saturated target before follow to test motor response."""

    limit = next((item for item in limits if item.name == joint_name), None)
    if limit is None:
        raise RuntimeError(f"{joint_name}: missing software limit for probe")
    target = max(limit.min_rad, min(limit.max_rad, current_rad + step_rad))
    if abs(target - current_rad) < 1e-5:
        raise RuntimeError(f"{joint_name}: probe target is clipped to current position, choose opposite sign")
    print(f"Probe step: {joint_name} q_now={current_rad:.6f} -> q_probe={target:.6f} rad")
    period = 1.0 / max(1.0, hz)
    deadline = time.perf_counter() + max(0.05, seconds)
    command_count = 0
    while time.perf_counter() < deadline:
        command_single_joint_target(
            slave,
            joint_name,
            target,
            limits,
            enable_motor=enable_each_command or command_count == 0,
            kp=kp,
            kd=kd,
            tau=tau,
            control_mode=control_mode,
            posvel_velocity=posvel_velocity,
            enable_old_mode=enable_old_mode,
            switch_mode=switch_mode,
        )
        command_count += 1
        time.sleep(period)
    snapshot = slave.read_snapshot(joint_names={joint_name}, damiao_response_wait=0.005)
    joint = next((item for item in snapshot.joints if item.name == joint_name), None)
    measured = joint.position_rad if joint is not None else None
    moved = None if measured is None else measured - current_rad
    print(f"Probe readback: {joint_name} q_after={_fmt(measured)} rad, moved={_fmt(moved)} rad, commands={command_count}")


def _run_probe_scan(slave, joint_name: str, current_rad: float, limits, hz: float, seconds: float) -> None:
    """Try conservative kp/tau combinations and print measured movement."""

    candidates = [
        (18.0, 1.5, 0.20, 0.05),
        (22.0, 1.8, 0.25, 0.05),
        (26.0, 2.0, 0.30, 0.05),
        (30.0, 2.2, 0.35, 0.05),
        (22.0, 1.8, 0.25, -0.05),
        (26.0, 2.0, 0.30, -0.05),
    ]
    print("Probe scan: conservative kp/kd/tau combinations")
    print("Stop immediately with Ctrl+C if the joint moves too much or feels unsafe.")
    for kp, kd, tau, step in candidates:
        before = _read_one_joint_rad(slave, joint_name)
        print()
        print(f"Scan item: kp={kp:.1f}, kd={kd:.1f}, tau={tau:+.2f}, step={step:+.3f}, q_before={before:.6f}")
        _run_probe_step(slave, joint_name, before, step, seconds, hz, limits, kp, kd, tau, "mit", 1.0, False, False, True)
        after = _read_one_joint_rad(slave, joint_name)
        print(f"Scan result: moved={after - before:+.6f} rad")
        time.sleep(0.4)


def _read_one_joint_rad(slave, joint_name: str) -> float:
    snapshot = slave.read_snapshot(joint_names={joint_name}, damiao_response_wait=0.005)
    joint = next((item for item in snapshot.joints if item.name == joint_name), None)
    if joint is None or joint.position_rad is None:
        raise RuntimeError(f"{joint_name}: failed to read current position")
    return joint.position_rad


def _fmt(value: float | None) -> str:
    if value is None:
        return "--"
    return f"{value:+.4f}"


if __name__ == "__main__":
    main()
