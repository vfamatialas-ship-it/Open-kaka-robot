"""
motor.py — DaMiao motor MIT-mode protocol (encode command / decode feedback).

Control law executed inside the motor (MIT mode):
    tau = kp*(p_des - p) + kd*(v_des - v) + tau_ff
"""

# Per-type encoding ranges. kp_max=500, kd_max=5 for all DaMiao MIT motors.
MOTOR_LIMITS = {
    # type   : (p_max[rad], v_max[rad/s], t_max[Nm], kp_max, kd_max)
    "DM4340": (12.5, 10.0, 28.0, 500.0, 5.0),
    "DM4310": (12.5, 30.0, 10.0, 500.0, 5.0),
    "DM4310_48": (12.5, 30.0, 10.0, 500.0, 5.0),
    "DM6006": (12.5, 45.0, 12.0, 500.0, 5.0),
    "DM8009": (12.5, 45.0, 54.0, 500.0, 5.0),
}

# MIT-mode special commands (8 data bytes).
CMD_ENABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
CMD_DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
CMD_SET_ZERO = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])


def _f2u(x, xmin, xmax, bits):
    x = max(xmin, min(xmax, x))
    span = xmax - xmin
    return int((x - xmin) * ((1 << bits) - 1) / span) & ((1 << bits) - 1)


def _u2f(u, xmin, xmax, bits):
    span = xmax - xmin
    return u * span / ((1 << bits) - 1) + xmin


class MotorState:
    __slots__ = ("position", "velocity", "torque")

    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0


class DmMotor:
    def __init__(self, name, motor_type, can_id, feedback_id,
                 direction=1.0, gravity_scale=1.0,
                 kp=None, kd=None, tau_limit=None,
                 limit_lower=None, limit_upper=None):
        if motor_type not in MOTOR_LIMITS:
            raise ValueError(f"Unknown motor type '{motor_type}'. "
                             f"Known: {list(MOTOR_LIMITS)}")
        self.name = name
        self.type = motor_type
        self.can_id = can_id
        self.feedback_id = feedback_id
        self.direction = float(direction)
        self.gravity_scale = float(gravity_scale)
        self.kp = kp              # impedance-hold gains (None → use defaults)
        self.kd = kd
        self.tau_limit = tau_limit
        self.limit_lower = limit_lower   # objective joint limits [rad] (None → URDF)
        self.limit_upper = limit_upper
        self.p_max, self.v_max, self.t_max, self.kp_max, self.kd_max = \
            MOTOR_LIMITS[motor_type]
        self.state = MotorState()

    # ── encode ──────────────────────────────────────────────────────────────────
    def mit_cmd(self, pos, vel, kp, kd, torq):
        pu = _f2u(pos, -self.p_max, self.p_max, 16)
        vu = _f2u(vel, -self.v_max, self.v_max, 12)
        ku = _f2u(kp, 0.0, self.kp_max, 12)
        du = _f2u(kd, 0.0, self.kd_max, 12)
        tu = _f2u(torq, -self.t_max, self.t_max, 12)
        return bytes([
            (pu >> 8) & 0xFF,
            pu & 0xFF,
            (vu >> 4) & 0xFF,
            ((vu & 0xF) << 4) | ((ku >> 8) & 0xF),
            ku & 0xFF,
            (du >> 4) & 0xFF,
            ((du & 0xF) << 4) | ((tu >> 8) & 0xF),
            tu & 0xFF,
        ])

    # ── decode ──────────────────────────────────────────────────────────────────
    def decode_feedback(self, data):
        pu = (data[1] << 8) | data[2]
        vu = (data[3] << 4) | (data[4] >> 4)
        tu = ((data[4] & 0xF) << 8) | data[5]
        self.state.position = _u2f(pu, -self.p_max, self.p_max, 16)
        self.state.velocity = _u2f(vu, -self.v_max, self.v_max, 12)
        self.state.torque = _u2f(tu, -self.t_max, self.t_max, 12)
