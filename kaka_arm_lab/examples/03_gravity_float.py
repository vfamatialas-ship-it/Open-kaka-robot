#!/usr/bin/env python3
"""Pure gravity compensation: the arm floats weightless (tau = G(q), kp=kd=0).
Push it and it stays where you leave it (if the gravity model is well tuned).
Needs urdf_path in the config. Usage: python3 03_gravity_float.py [config.yaml]
SAFETY: keep a hand on the arm; start from a stable pose."""
import sys, time
from kaka_arm_lab import Robot

cfg = sys.argv[1] if len(sys.argv) > 1 else "config/arm_example.yaml"

with Robot(cfg) as r:
    r.enable_all()
    time.sleep(0.3)
    print("Gravity float at 200 Hz. Ctrl-C to stop.")
    try:
        while True:
            r.gravity_float()
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopping")
