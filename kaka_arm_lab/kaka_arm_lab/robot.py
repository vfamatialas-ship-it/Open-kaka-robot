"""
robot.py — high-level arm control: MIT control + gravity compensation.

Config-driven, so the same code runs on any same-type DaMiao arm: just point it
at a different YAML (different serial port, motor ids, types, calibration).
"""
import os
import threading
import numpy as np
import yaml

from .damiao_can import DamiaoCAN
from .motor import DmMotor, CMD_ENABLE, CMD_DISABLE, CMD_SET_ZERO
from .gravity import GravityModel


class Robot:
    def __init__(self, config_path):
        self._cfg_dir = os.path.dirname(os.path.abspath(config_path))
        with open(config_path) as f:
            cfg = yaml.safe_load(f)["robot"]

        self.port = cfg.get("serial_port", "/dev/ttyACM0")
        self.motors = []
        for m in cfg["motors"]:
            self.motors.append(DmMotor(
                name=m["name"], motor_type=m["type"],
                can_id=int(m["can_id"]), feedback_id=int(m["feedback_id"]),
                direction=m.get("direction", 1.0),
                gravity_scale=m.get("gravity_scale", 1.0),
                kp=m.get("kp"), kd=m.get("kd"), tau_limit=m.get("tau_limit"),
                limit_lower=m.get("limit_lower"), limit_upper=m.get("limit_upper")))
        self.n = len(self.motors)

        # Impedance-hold gains (per joint from config; sensible fallbacks).
        self.kp_hold = np.array([m.kp if m.kp is not None else 20.0
                                 for m in self.motors])
        self.kd_hold = np.array([m.kd if m.kd is not None else 0.5
                                 for m in self.motors])
        self.tau_limit = np.array([m.tau_limit if m.tau_limit is not None
                                   else 0.8 * m.t_max for m in self.motors])
        self._by_fid = {m.feedback_id: m for m in self.motors}
        self._lock = threading.Lock()

        self.can = DamiaoCAN(self.port, on_frame=self._on_frame)

        # optional gravity model. A relative urdf_path is resolved against the
        # config file's directory, so the SDK is portable (no absolute paths).
        self.gravity = None
        urdf = cfg.get("urdf_path")
        if urdf and not os.path.isabs(urdf):
            urdf = os.path.join(self._cfg_dir, urdf)
        if urdf:
            self.gravity = GravityModel(
                urdf_path=urdf,
                joint_names=[m.name for m in self.motors],
                directions=[m.direction for m in self.motors],
                scales=[m.gravity_scale for m in self.motors],
                mimics=cfg.get("mimics"))

        # Objective joint limits [rad]: config override, else URDF, else ±pi.
        ulo = uup = None
        if self.gravity is not None:
            ulo, uup = self.gravity.joint_limits()
        self.limit_lower = np.array([
            m.limit_lower if m.limit_lower is not None
            else (ulo[i] if ulo is not None else -np.pi)
            for i, m in enumerate(self.motors)])
        self.limit_upper = np.array([
            m.limit_upper if m.limit_upper is not None
            else (uup[i] if uup is not None else np.pi)
            for i, m in enumerate(self.motors)])

    # ── connection ─────────────────────────────────────────────────────────────
    def connect(self):
        self.can.open()
        return self

    def disconnect(self):
        try:
            self.disable_all()
        finally:
            self.can.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.disconnect()

    def _on_frame(self, can_id, data):
        m = self._by_fid.get(can_id)
        if m:
            with self._lock:
                m.decode_feedback(data)

    # ── lifecycle commands ─────────────────────────────────────────────────────
    def enable_all(self):
        for m in self.motors:
            self.can.send(m.can_id, CMD_ENABLE)

    def disable_all(self):
        for m in self.motors:
            self.can.send(m.can_id, CMD_DISABLE)

    def set_zero_all(self):
        """Set the CURRENT physical pose as q=0 for every motor (persists to flash)."""
        for m in self.motors:
            self.can.send(m.can_id, CMD_SET_ZERO)

    # ── MIT control ────────────────────────────────────────────────────────────
    def send_mit(self, i, pos, vel, kp, kd, torq):
        m = self.motors[i]
        self.can.send(m.can_id, m.mit_cmd(pos, vel, kp, kd, torq))

    def send_mit_all(self, pos, vel, kp, kd, torq):
        for i in range(self.n):
            self.send_mit(i, pos[i], vel[i], kp[i], kd[i], torq[i])

    # ── state ──────────────────────────────────────────────────────────────────
    def positions(self):
        with self._lock:
            return np.array([m.state.position for m in self.motors])

    def velocities(self):
        with self._lock:
            return np.array([m.state.velocity for m in self.motors])

    def torques(self):
        with self._lock:
            return np.array([m.state.torque for m in self.motors])

    # ── limits ───────────────────────────────────────────────────────────────
    def check_limits(self, q=None, near=0.05):
        """Per joint: (name, q, lower, upper, margin, status). status ∈
        OK / NEAR (margin<near) / OUT (outside the objective limit)."""
        q = self.positions() if q is None else q
        rows = []
        for i, m in enumerate(self.motors):
            lo, up = self.limit_lower[i], self.limit_upper[i]
            margin = min(q[i] - lo, up - q[i])
            if q[i] < lo or q[i] > up:
                status = "OUT"
            elif margin < near:
                status = "NEAR"
            else:
                status = "OK"
            rows.append((m.name, float(q[i]), float(lo), float(up),
                         float(margin), status))
        return rows

    # ── high-level modes ───────────────────────────────────────────────────────
    def compute_gravity(self, q=None):
        if self.gravity is None:
            raise RuntimeError("No urdf_path in config → gravity model unavailable")
        q = self.positions() if q is None else q
        return self.gravity.compute(q)

    def gravity_float(self):
        """Pure gravity compensation: tau = G(q), kp=kd=0. Arm floats weightless.
        Torque is clamped to tau_limit for safety against a bad gravity model."""
        G = np.clip(self.compute_gravity(), -self.tau_limit, self.tau_limit)
        z = np.zeros(self.n)
        self.send_mit_all(z, z, z, z, G)

    def impedance_hold(self, q_des, kp, kd, tau_limit=None):
        """tau = kp*(q_des-q) + kd*(0-qd) + G(q), sent as motor torque feed-forward."""
        q = self.positions()
        qd = self.velocities()
        G = self.compute_gravity(q)
        tau = np.asarray(kp) * (np.asarray(q_des) - q) + np.asarray(kd) * (-qd) + G
        if tau_limit is not None:
            tau = np.clip(tau, -np.asarray(tau_limit), np.asarray(tau_limit))
        z = np.zeros(self.n)
        self.send_mit_all(z, z, z, z, tau)

    def hold_with_native_pd(self, q_des, kp, kd, gravity_ff=True):
        """Native motor PD + optional gravity feed-forward (kp/kd run on the motor)."""
        G = self.compute_gravity() if gravity_ff and self.gravity else np.zeros(self.n)
        z = np.zeros(self.n)
        self.send_mit_all(q_des, z, kp, kd, G)
