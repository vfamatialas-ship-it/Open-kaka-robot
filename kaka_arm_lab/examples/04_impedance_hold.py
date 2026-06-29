#!/usr/bin/env python3
"""Impedance + gravity feed-forward: capture the current pose and hold it.
   tau = kp*(q_des - q) + kd*(0 - qd) + G(q)
Hold the arm where you want, then start this — it locks onto that pose.
kp/kd/tau_limit come from the config (fallback defaults if absent).
Usage: python3 04_impedance_hold.py [config.yaml]
SAFETY: keep a hand on the arm; Ctrl-C disables the motors."""
import sys, time
import numpy as np
from kaka_arm_lab import Robot

cfg = sys.argv[1] if len(sys.argv) > 1 else "config/arm_example.yaml"

with Robot(cfg) as r:
    r.enable_all()
    time.sleep(0.4)                       # let feedback arrive before latching
    q_des = r.positions().copy()          # hold the current (hand-placed) pose
    print("Holding pose:", np.round(q_des, 3))
    print("kp:", r.kp_hold, " kd:", r.kd_hold)
    print("Impedance + gravity hold. Ctrl-C to stop.")
    try:
        while True:
            r.impedance_hold(q_des, r.kp_hold, r.kd_hold, r.tau_limit)
            time.sleep(0.005)             # 200 Hz
    except KeyboardInterrupt:
        print("\nstopping")
