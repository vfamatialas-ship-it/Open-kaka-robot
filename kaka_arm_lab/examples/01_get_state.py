#!/usr/bin/env python3
"""Read and print joint state. Enables motors with zero torque (free) to solicit
feedback. Usage: python3 01_get_state.py [config.yaml]"""
import sys, time
import numpy as np
from kaka_arm_lab import Robot

cfg = sys.argv[1] if len(sys.argv) > 1 else "config/arm_example.yaml"

with Robot(cfg) as r:
    r.enable_all()
    time.sleep(0.3)
    z = np.zeros(r.n)
    try:
        while True:
            r.send_mit_all(z, z, z, z, z)          # zero torque → free, solicits feedback
            time.sleep(0.05)
            p = np.round(r.positions(), 3)
            print("\rpos[rad]:", p, end="", flush=True)
    except KeyboardInterrupt:
        print("\nbye")
