"""Interactive MIT tau_ff tuning console for one pink slave joint.

Use this when a loaded joint needs gravity support:
- Manually place the arm in a loaded pose.
- Keep one hand lightly supporting the arm segment.
- Run this script and slowly adjust tau with the keyboard.
- The sign/magnitude that reduces your hand support is the first tau_ff guess.

Safety:
- Refuses to run unless --enable-motion YES is provided.
- Holds current position q_des=q_now; it does not follow the master arm.
- Every hold command is checked against saved software limits.
- Space pauses command output, h recaptures the current hold position.
- q exits and disables all pink slave motors.
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
    parser = argparse.ArgumentParser(description="Interactive pink slave MIT tau tuning console.")
    parser.add_argument("--joint", default="joint2", choices=JOINT_NAMES)
    parser.add_argument("--slave-port", default="COM7")
    parser.add_argument("--slave-baudrate", type=int, default=921600)
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--kd", type=float, default=None)
    parser.add_argument("--tau", type=float, default=0.0)
    parser.add_argument("--tau-step", type=float, default=0.01)
    parser.add_argument("--tau-limit", type=float, default=0.25)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--damiao-wait", type=float, default=0.005)
    parser.add_argument("--max-error-rad", type=float, default=0.18)
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
    kp = max(0.0, args.kp if args.kp is not None else gain.kp)
    kd = max(0.0, args.kd if args.kd is not None else gain.kd)
    tau = _clamp(args.tau, -abs(args.tau_limit), abs(args.tau_limit))
    tau_step = abs(args.tau_step)
    limits = _relax_limits(parse_limits_text(load_limits_text("pink_slave")), args.joint, gain.limit_relax_rad)

    print("Pink slave interactive MIT tau tuning / 粉色从臂 MIT tau 交互调参")
    print("WARNING: real Damiao MIT commands will be sent.")
    print(f"Joint: {args.joint}")
    print(f"Slave: pink_slave @ {args.slave_port} {args.slave_baudrate}")
    print(f"kp={kp:.2f}, kd={kd:.2f}, tau_limit=±{abs(args.tau_limit):.3f}, tau_step={tau_step:.3f}")
    print()
    print("Keys / 按键:")
    print("  = / + : increase tau / 增大 tau")
    print("  - / _ : decrease tau / 减小 tau")
    print("  ]     : increase tau step / 增大 tau 步长")
    print("  [     : decrease tau step / 减小 tau 步长")
    print("  0     : set tau = 0 / tau 归零")
    print("  h     : hold current position / 重新捕获当前位置保持")
    print("  space : pause/resume command output / 暂停或恢复命令输出")
    print("  q     : quit and disable / 退出并失能")
    print()
    print("Use one hand to lightly support the loaded arm segment. Keep physical E-stop ready.")
    print("用手轻托承重段，手边保持物理急停。")
    print()

    slave = open_arm_connection("pink_slave", port=args.slave_port, baudrate=args.slave_baudrate)
    paused = False
    try:
        q_hold = _read_position(slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=args.damiao_wait), args.joint)
        q_now = q_hold
        print(f"Initial hold target q_des=q_now={q_hold:+.6f} rad")

        command_single_joint_target(
            slave,
            args.joint,
            q_hold,
            limits,
            enable_motor=True,
            kp=kp,
            kd=kd,
            tau=tau,
            control_mode="mit",
            posvel_velocity=1.0,
            enable_old_mode=False,
            switch_mode=False,
        )
        time.sleep(0.2)

        period = 1.0 / max(1.0, args.hz)
        while True:
            started = time.perf_counter()

            key = _read_key()
            if key:
                if key in {"q", "Q"}:
                    print("\nQuit requested.")
                    break
                if key in {"=", "+"}:
                    tau = _clamp(tau + tau_step, -abs(args.tau_limit), abs(args.tau_limit))
                elif key in {"-", "_"}:
                    tau = _clamp(tau - tau_step, -abs(args.tau_limit), abs(args.tau_limit))
                elif key == "]":
                    tau_step = min(abs(args.tau_limit), tau_step * 2.0)
                elif key == "[":
                    tau_step = max(0.001, tau_step / 2.0)
                elif key == "0":
                    tau = 0.0
                elif key in {"h", "H"}:
                    q_hold = _read_position(
                        slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=args.damiao_wait),
                        args.joint,
                    )
                elif key == " ":
                    paused = not paused

            snapshot = slave.read_snapshot(joint_names={args.joint}, damiao_response_wait=args.damiao_wait)
            q_now = _read_position(snapshot, args.joint)
            err = q_hold - q_now
            if abs(err) > max(0.0, args.max_error_rad):
                raise RuntimeError(
                    f"{args.joint}: moved too far from hold target: "
                    f"q_now={q_now:+.6f}, q_hold={q_hold:+.6f}, err={err:+.6f}"
                )

            if not paused:
                command_single_joint_target(
                    slave,
                    args.joint,
                    q_hold,
                    limits,
                    enable_motor=False,
                    kp=kp,
                    kd=kd,
                    tau=tau,
                    control_mode="mit",
                    posvel_velocity=1.0,
                    enable_old_mode=False,
                    switch_mode=False,
                )

            print(
                "\r\x1b[2K"
                f"{args.joint} q={q_now:+.4f} hold={q_hold:+.4f} "
                f"err={err:+.4f} tau={tau:+.3f} step={tau_step:.3f} "
                f"{'PAUSED' if paused else 'ACTIVE'}",
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
        slave.close()


def _read_key() -> str | None:
    if sys.platform.startswith("win"):
        import msvcrt

        if not msvcrt.kbhit():
            return None
        char = msvcrt.getwch()
        # Arrow/function keys arrive as a two-character sequence. Ignore them.
        if char in {"\x00", "\xe0"}:
            if msvcrt.kbhit():
                msvcrt.getwch()
            return None
        return char
    return None


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


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


if __name__ == "__main__":
    main()
