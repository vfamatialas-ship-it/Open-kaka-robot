#!/usr/bin/env python3
"""Set the CURRENT pose as q=0 for every motor (persists to flash).
Pose the arm at the URDF zero configuration FIRST, then run.
Usage: python3 02_set_zero.py [config.yaml]"""
import sys, time
import numpy as np
from kaka_arm_lab import Robot

cfg = sys.argv[1] if len(sys.argv) > 1 else "config/arm_example.yaml"

with Robot(cfg) as r:
    r.enable_all()
    time.sleep(0.3)
    z = np.zeros(r.n)
    print("Hold the arm at the zero pose. Zeroing in 5 s ...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        r.send_mit_all(z, z, z, z, z)             # passive (zero torque)
        time.sleep(0.02)
    r.set_zero_all()
    time.sleep(0.3)
    print("Done. Joint angles now read:", np.round(r.positions(), 3))
