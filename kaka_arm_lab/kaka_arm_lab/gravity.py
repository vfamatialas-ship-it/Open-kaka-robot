"""
gravity.py — Pinocchio gravity compensation, generic over arm topology.

G(q) = RNEA(model, q, v=0, a=0)  →  the joint torques that hold the arm still.

Per-joint calibration (from config):
  direction      : +1 / -1, motor axis vs URDF joint axis
  gravity_scale  : magnitude trim (1.0 = model as-is)

Mimic joints (e.g. gripper fingers driven by one motor) are handled via virtual
work: their gravity torque is projected back onto the driving joint.
"""
import numpy as np
import pinocchio as pin


class GravityModel:
    def __init__(self, urdf_path, joint_names, directions=None, scales=None,
                 mimics=None):
        """
        joint_names : actuated joints in motor order, e.g. ['J1',...,'J6']
        directions  : list, +1/-1 per actuated joint (default all +1)
        scales      : list, gravity_scale per actuated joint (default all 1.0)
        mimics      : list of dicts {name, mimic, multiplier} for passive joints
        """
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.names = list(joint_names)
        self.n = len(self.names)
        self.dir = np.array(directions if directions else [1.0] * self.n)
        self.scale = np.array(scales if scales else [1.0] * self.n)

        self.idx_q = [self.model.joints[self.model.getJointId(j)].idx_q
                      for j in self.names]
        self.idx_v = [self.model.joints[self.model.getJointId(j)].idx_v
                      for j in self.names]

        # mimic joints: {q_idx, v_idx, driver_index (into names), multiplier}
        self.mimics = []
        for m in (mimics or []):
            if not self.model.existJointName(m["name"]):
                continue
            jid = self.model.getJointId(m["name"])
            self.mimics.append({
                "q_idx": self.model.joints[jid].idx_q,
                "v_idx": self.model.joints[jid].idx_v,
                "driver": self.names.index(m["mimic"]),
                "mult": float(m.get("multiplier", 1.0)),
            })

    def joint_limits(self):
        lo = np.array([self.model.lowerPositionLimit[i] for i in self.idx_q])
        up = np.array([self.model.upperPositionLimit[i] for i in self.idx_q])
        return lo, up

    def compute(self, q_motor, apply_scale=True):
        """Gravity torque per actuated joint, in MOTOR convention [Nm]."""
        q = np.zeros(self.model.nq)
        for i in range(self.n):
            q[self.idx_q[i]] = self.dir[i] * q_motor[i]
        for m in self.mimics:
            q[m["q_idx"]] = self.dir[m["driver"]] * q_motor[m["driver"]] * m["mult"]

        v = np.zeros(self.model.nv)
        tau = pin.rnea(self.model, self.data, q, v, v)   # a = v = 0

        G = np.zeros(self.n)
        for i in range(self.n):
            s = self.scale[i] if apply_scale else 1.0
            G[i] = s * self.dir[i] * tau[self.idx_v[i]]
        # project mimic torques onto their driver (virtual work)
        for m in self.mimics:
            s = self.scale[m["driver"]] if apply_scale else 1.0
            G[m["driver"]] += s * self.dir[m["driver"]] * m["mult"] * tau[m["v_idx"]]
        return G
