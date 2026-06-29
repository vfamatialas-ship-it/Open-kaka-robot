#!/usr/bin/env python3
"""
00_selftest.py — FIRST-RUN self test. Run once on any (new) arm before control.
At the zero pose it checks two things, WITHOUT ever driving the joints:

  Test 1: are all joint angles inside the OBJECTIVE joint limits?
  Test 2: are the gravity-compensation VALUES sane at the zero pose?
          (computes G(q0) and checks it against each joint's torque cap —
           NO torque is ever applied to the motors, so the arm cannot jerk.)

On a DIFFERENT arm you MUST calibrate first (02_set_zero, then tune
direction / gravity_scale).

Usage: python3 00_selftest.py [config.yaml]
The motors are only ever commanded ZERO torque (to solicit feedback); the arm
stays passive/free the whole time. Ctrl-C disables the motors.
"""
import sys, time
import numpy as np
from kaka_arm_lab import Robot

cfg = sys.argv[1] if len(sys.argv) > 1 else "config/arm_example.yaml"
NOT_ZERO = 0.15            # rad — a "zero pose" joint should be within this of 0

r = Robot(cfg).connect()
try:
    print("=" * 64)
    print("  kaka_arm_lab — first-run self test  (no joint is ever driven)")
    print("=" * 64)
    input("\n[setup] Move the arm to the ZERO pose, hold it, then press Enter ...")

    r.enable_all()
    z = np.zeros(r.n)
    t0 = time.time()                          # solicit feedback (zero torque = free)
    while time.time() - t0 < 0.6:
        r.send_mit_all(z, z, z, z, z)
        time.sleep(0.02)
    q = r.positions()

    # ── Test 1: joint limits ──────────────────────────────────────────────────
    print("\n--- Test 1: joint limits ---")
    n_out = 0
    for name, qi, lo, up, margin, status in r.check_limits(q):
        flag = "   <== OUT OF LIMIT" if status == "OUT" else ""
        print(f"  {name}: q={qi:+.3f}  [{lo:+.2f}, {up:+.2f}]  "
              f"margin={margin:+.3f}  {status}{flag}")
        if status == "OUT":
            n_out += 1
    far = [r.motors[i].name for i in range(r.n) if abs(q[i]) > NOT_ZERO]
    if far:
        print(f"  ! not near 0 at the 'zero' pose: {far}  "
              f"-> re-calibrate zero (02_set_zero) or you weren't at the zero pose")
    print("  RESULT:", "FAIL (out of limits)" if n_out else "PASS")

    # ── Test 2: gravity-compensation VALUES (computed only, no torque) ────────
    print("\n--- Test 2: gravity-compensation values at zero (NO torque applied) ---")
    if r.gravity is None:
        print("  (no urdf_path in config -> gravity disabled, skipped)")
    else:
        G = r.compute_gravity(q)
        n_over = 0
        print("  joint :    G(q0)[Nm]   tau_limit   status")
        for i, m in enumerate(r.motors):
            over = abs(G[i]) > r.tau_limit[i]
            print(f"   {m.name}  : {G[i]:+9.3f}     {r.tau_limit[i]:6.1f}    "
                  f"{'OVER tau_limit <==' if over else 'ok'}")
            if over:
                n_over += 1
        if n_over:
            print("  RESULT: FAIL — a joint's computed gravity torque exceeds its "
                  "cap; the gravity_scale is too large or the model is wrong.")
        else:
            print("  RESULT: values within torque caps. Sanity-check the magnitudes:")
            print("          load-bearing joints should be non-zero, the rest ~0.")
        print("  (computed from the model only — nothing was sent to the motors.)")

    print("\nSelf test complete.")
finally:
    r.disconnect()
