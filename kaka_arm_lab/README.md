# kaka_arm_lab

A minimal, dependency-light Python SDK for **DaMiao-motor robot arms** driven
through the **DaMiao USB-to-CAN adapter**. Provides:

- **MIT-mode control** (position / velocity / kp / kd / torque feed-forward)
- **Gravity compensation** via Pinocchio (RNEA), with per-joint direction &
  magnitude calibration and gripper-mimic handling
- **Fully config-driven** — run the same code on any same-type arm by editing a
  YAML (serial port, motor CAN ids, types, calibration)

> The DaMiao USB-to-CAN debugger (HDSC chip) enumerates as a USB-CDC serial port
> (`/dev/ttyACM*`) and speaks DaMiao's own `0x55/0xAA` framing — **not** Linux
> SocketCAN. This SDK talks that serial protocol directly, so no `ip link` /
> `can0` setup is needed.

## Install

Dependencies: `pyserial`, `numpy`, `pyyaml`, and (for gravity comp) Pinocchio
Python bindings.

**Option 1 — no install, just run (simplest; recommended if deps already present):**
```bash
git clone <your-repo-url> kaka_arm_lab
cd kaka_arm_lab
# run from the repo root with PYTHONPATH:
PYTHONPATH=.:$PYTHONPATH python3 examples/01_get_state.py config/arm_example.yaml
```
Get the deps via apt (works around Ubuntu 24.04 PEP-668):
```bash
sudo apt install python3-serial python3-numpy python3-yaml
# Pinocchio: either  sudo apt install ros-jazzy-pinocchio  (then source it)
#            or       pipx install / a venv with `pip install pin`
source /opt/ros/jazzy/setup.bash      # if using the ROS pinocchio
```

**Option 2 — venv (clean, isolated):**
```bash
python3 -m venv --system-site-packages .venv   # inherit system/ROS pinocchio
source .venv/bin/activate
pip install -e .
```

**Option 3 — pip into the user site (if your distro allows it):**
```bash
pip install -e . --break-system-packages        # Ubuntu 24.04 needs this flag
```

Serial permission: add yourself to the `dialout` group once, then re-login:
```bash
sudo usermod -aG dialout $USER
```

## Configure

Copy `config/arm_example.yaml` and edit it for your arm:

```yaml
robot:
  serial_port: "/dev/ttyACM0"
  urdf_path:   "/abs/path/to/your_arm.urdf"   # for gravity comp (optional)
  motors:
    - { name: J1, type: DM4340, can_id: 0x01, feedback_id: 0x11, direction: 1, gravity_scale: 1.0 }
    # ...
```

**Different (same-type) arm?** Just change `can_id` / `feedback_id` to the new
motors' ids and re-run the calibration below. Nothing in the code changes.

## First run / self test (do this on every new arm)

Joint limits are **objective hardware constraints**; gravity comp must be
**calibrated per arm**. Before using the arm, run the self test — it prompts you
to move to the zero pose, then checks (1) all joints are inside their limits and
(2) the gravity-compensation values at zero are sane (computed only — NO torque
is applied, so the arm never jerks):

```bash
python3 examples/00_selftest.py config/kaka_arm.yaml
```

A **different same-type arm** (different CAN ids) must be calibrated first:
1. set the ids in a copy of `config/arm_example.yaml`;
2. run `02_set_zero.py` at the zero pose;
3. tune `direction` / `gravity_scale` until `00_selftest.py` Test 2 passes.

## Quick start

```bash
python3 examples/00_selftest.py      config/arm_example.yaml   # first-run checks
python3 examples/01_get_state.py     config/arm_example.yaml   # read joint angles
python3 examples/02_set_zero.py      config/arm_example.yaml   # calibrate zero
python3 examples/03_gravity_float.py config/arm_example.yaml   # weightless float
python3 examples/04_impedance_hold.py config/arm_example.yaml  # spring-hold a pose
```

Minimal program:

```python
import time, numpy as np
from kaka_arm_lab import Robot

with Robot("config/arm_example.yaml") as r:
    r.enable_all()
    time.sleep(0.4)
    while True:
        r.gravity_float()        # tau = G(q); arm floats
        time.sleep(0.005)        # 200 Hz
```

## Calibration (per arm)

1. **Zero**: pose the arm at the URDF zero configuration, run `02_set_zero.py`
   (sets the current pose as q=0 for all motors; persists to flash).
2. **Direction**: if a joint "sags worse" under gravity comp, set `direction: -1`
   for it in the YAML.
3. **gravity_scale**: if a link still droops, raise its `gravity_scale`
   (>1); if it flings up, lower it (<1).

## API

| Call | Meaning |
|---|---|
| `Robot(cfg).connect()` / `with Robot(cfg) as r:` | open serial + RX thread |
| `r.enable_all()` / `disable_all()` / `set_zero_all()` | motor lifecycle |
| `r.send_mit(i, pos, vel, kp, kd, torq)` | raw MIT command, one joint |
| `r.send_mit_all(pos, vel, kp, kd, torq)` | MIT command, all joints (arrays) |
| `r.positions()` / `velocities()` / `torques()` | latest state (numpy) |
| `r.check_limits(q=None)` | per-joint OK / NEAR / OUT vs objective limits |
| `r.compute_gravity()` | gravity torques G(q) [Nm] |
| `r.gravity_float()` | pure gravity comp (weightless) |
| `r.impedance_hold(q_des, kp, kd, tau_limit)` | software impedance + gravity |
| `r.hold_with_native_pd(q_des, kp, kd)` | motor-native PD + gravity FF |

## Push to GitHub

First time (turn this folder into a repo and push):
```bash
cd kaka_arm_lab
git init
git add .
git commit -m "kaka_arm_lab: DaMiao arm MIT + gravity-compensation SDK"
git branch -M main
git remote add origin https://github.com/<you>/<repo>.git
git push -u origin main
# if the remote already has a commit (e.g. an auto-created README) and push is
# rejected, overwrite it: git push -u origin main --force
```

Update an EXISTING repo later (after editing files):
```bash
git add -A
git commit -m "describe your change"
git push
```

Auth: HTTPS asks for a **Personal Access Token** (GitHub → Settings → Developer
settings → Tokens) as the password — not your account password. Or use SSH
(`git remote set-url origin git@github.com:<you>/<repo>.git`) or the `gh` CLI.

## Safety

- Start from a stable pose, keep a hand ready, and `Ctrl-C` disables the motors.
- Gravity comp quality depends on the URDF mass model; tune `gravity_scale`
  conservatively first.

## License

MIT — see [LICENSE](LICENSE).
