"""The measured R1 arm in a MuJoCo scene: gravity, friction with memory, inertia and delay from
calibration/arm_model_fit.py.
"""
import collections

import mujoco
import numpy as np

EPS_V = 0.005                                               # rad/s: the smooth sign() of the explicit friction terms


class R1ArmModel(object):
    def __init__(self, m, side, fit, joints):
        self.m, self.side, self.fit = m, side, fit
        self.jid = [m.joint(j).id for j in joints]
        self.dof = np.array([m.jnt_dofadr[j] for j in self.jid])
        g = fit["gravity"]
        self.seg = [m.body(s).id for s in g["segments"]]
        self.k = float(g.get("torque_scale") or 1.0)
        self.chosen = g["chosen"]
        self.pi = np.array(g["pi_true"], float)
        self.bias = np.array(g["bias_nm"], float) / self.k
        self.spring = np.array(g["spring_nm_per_rad"], float) / self.k
        self.scale, self.sbias = np.array(g["scale"], float), np.array(g["scale_bias_nm"], float)
        # the bodies of each segment: in the subtree of its link, not of the next segment's
        sub = [self._subtree(b) for b in self.seg]
        self.members = [sorted(set(sub[i]) - (set(sub[i + 1]) if i < 4 else set())) for i in range(5)]
        self.fr = fit["friction"]["joints"]
        act = fit.get("actuator", {})
        self.delay = float(act.get("delay_ms", 0.0)) / 1000.0
        self.gain = np.array(act.get("gain", [1.0] * 5), float)
        self.buf = collections.deque()
        self.z = np.zeros(5)                                    # LuGre friction state per joint (model "L")

    def _subtree(self, b):
        out = [b]
        for c in range(self.m.nbody):
            p = c
            while p > 0 and p != b:
                p = self.m.body_parentid[p]
            if p == b and c != b:
                out.append(c)
        return out

    # ------------------------------------------------------------------ once
    def setup(self, d, actuators=None):
        m = self.m
        for i in range(5):
            for b in self.members[i]:
                m.body_gravcomp[b] = 0.0
        for j, f in enumerate(self.fr):
            if not f.get("model"):
                continue
            n = self.dof[j]
            if f["model"] == "L":                               # all of it in friction_extra, from the friction state
                m.dof_frictionloss[n], m.dof_damping[n] = 0.0, 0.0
                m.dof_armature[n] += max(0.0, f.get("armature_kgm2", 0.0)) / self.k
                continue
            m.dof_frictionloss[n] = 0.5 * (f["coulomb_pos_nm"] + f["coulomb_neg_nm"]) / self.k
            m.dof_damping[n] = max(0.0, f["viscous_nm_s"]) / self.k
            m.dof_armature[n] += max(0.0, f.get("armature_kgm2", 0.0)) / self.k
        if actuators is not None:                               # position actuators: force = kp (ctrl - q) - kv dq
            for j, a in enumerate(actuators):
                m.actuator_gainprm[a, 0] *= self.gain[j]
                m.actuator_biasprm[a, 1] *= self.gain[j]
                m.actuator_biasprm[a, 2] *= self.gain[j]
        mujoco.mj_forward(m, d)

    # ------------------------------------------------------------------ gravity
    def Y(self, d):
        """5 x 20: the torque each arm joint needs against gravity per segment parameter [mass, mass x COM (segment frame)]."""
        gv = self.m.opt.gravity
        Y = np.zeros((5, 20))
        for i, j in enumerate(self.jid):
            o, ax = d.xanchor[j], d.xaxis[j]
            for gi in range(i, 5):
                b = self.seg[gi]
                R, p = d.xmat[b].reshape(3, 3), d.xpos[b]
                Y[i, 4 * gi] = -ax @ np.cross(p - o, gv)
                for k in range(3):
                    Y[i, 4 * gi + 1 + k] = -ax @ np.cross(R[:, k], gv)
        return Y

    def pi_mujoco(self, d):
        """MuJoCo's own segment parameters now (finger bodies move with the hand shape)."""
        pi = np.zeros(20)
        for gi in range(5):
            b0 = self.seg[gi]
            R, p = d.xmat[b0].reshape(3, 3), d.xpos[b0]
            for b in self.members[gi]:
                mb = self.m.body_mass[b]
                pi[4 * gi] += mb
                pi[4 * gi + 1:4 * gi + 4] += mb * (R.T @ (d.xipos[b] - p))
        return pi

    def gravity_extra(self, d):
        """Torque the robot needs against gravity minus what MuJoCo's own masses need, per arm joint."""
        Y = self.Y(d)
        pm = self.pi_mujoco(d)
        q = d.qpos[[self.m.jnt_qposadr[j] for j in self.jid]]
        if self.chosen == "scale":
            g = Y @ pm
            return (self.scale - 1.0) * g + self.sbias
        if self.chosen == "urdf":
            return np.zeros(5)
        return Y @ (self.pi - pm) + self.bias + self.spring * q

    # ------------------------------------------------------------------ friction beyond Coulomb + viscous
    def friction_extra(self, d):
        v = d.qvel[self.dof]
        q = d.qpos[[self.m.jnt_qposadr[j] for j in self.jid]]
        out = np.zeros(5)
        dt = self.m.opt.timestep
        for j, f in enumerate(self.fr):
            if not f.get("model"):
                continue
            if f["model"] == "L":                               # friction with memory, as arm_model_fit.lugre_run
                s0 = f["lugre_s0_nm_per_rad"]
                g = f["coulomb_nm"] + f.get("stribeck_extra_nm", 0.0) * np.exp(-(v[j] / f.get("stribeck_v", 0.02)) ** 2)
                a = s0 * abs(v[j]) / max(g, 1e-6)
                zs = np.sign(v[j]) * g / s0
                self.z[j] = zs + (self.z[j] - zs) * np.exp(-a * dt)
                out[j] = (s0 * self.z[j] + f.get("viscous_nm_s", 0.0) * v[j]) / self.k
                continue
            s = np.tanh(v[j] / EPS_V)
            F = 0.5 * (f["coulomb_pos_nm"] - f["coulomb_neg_nm"]) * abs(s)
            F += f.get("stribeck_extra_nm", 0.0) * np.exp(-(v[j] / f.get("stribeck_v", 0.02)) ** 2) * s
            F += (f.get("angle_cos_nm", 0.0) * np.cos(q[j]) + f.get("angle_sin_nm", 0.0) * np.sin(q[j])) * s
            out[j] = F / self.k
        return out

    def apply(self, d):
        """Call before every mj_step: sets qfrc_applied on the arm's joints."""
        d.qfrc_applied[self.dof] = -self.gravity_extra(d) - self.friction_extra(d)

    # ------------------------------------------------------------------ delay
    def delayed(self, t, target):
        """The position target as the motor sees it: what was asked `delay` seconds ago."""
        self.buf.append((t, np.array(target, float)))
        while len(self.buf) > 1 and self.buf[1][0] <= t - self.delay:
            self.buf.popleft()
        return self.buf[0][1]
