"""The R1 control chain between a planned motion and the simulated motors: the server's safety filter, gravity offset,
the measured arm and hand, and the command delay.
"""
import collections
import json
import math
import os
import sys

import mujoco
import numpy as np

from platepick.r1_arm_model import R1ArmModel

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(ROOT)
DEPLOY = os.path.join(PROJECT, "robot")
ARM_JSON = os.path.join(PROJECT, "models", "sim_arm_model_%s.json")
FULL_SIM = np.array([1.55334, 1.0472, 1.41, 1.41, 1.41, 1.41])   # sim order: rad at command 1
SIM2SRV = [1, 0, 2, 3, 4, 5]                                    # service channel i = sim channel SIM2SRV[i]
V_MAX, A_MAX, V_HAND, LEAD = 0.8, 3.0, 1.5, 0.12
HAND_DELAY, HAND_LAG = 0.020, 0.030
SIDES = ("left", "right")


def _deploy():
    if DEPLOY not in sys.path:
        sys.path.insert(0, DEPLOY)
    from pk import config as cfg        # noqa: E402
    from pk.model import GravityComp    # noqa: E402
    return cfg, GravityComp


class ServerSim(object):
    def __init__(self, m, d, meta, A, HA, C, hhi, torque_scale=1.0, payload=None):
        self.m, self.d, self.A, self.HA, self.C, self.hhi = m, d, A, HA, C, hhi
        self.cfg, GC = _deploy()
        self.comp = GC()
        self.payload = payload
        self.arm = {}
        for s in SIDES:
            fit = json.load(open(ARM_JSON % s))
            fit["gravity"]["torque_scale"] = torque_scale      # doc 1.0 until a second load test
            self.arm[s] = R1ArmModel(m, s, fit, meta["arm"][s])
            self.arm[s].setup(d, A[s])
        self.adr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][s]] for s in SIDES}
        # measured finger channel (sim order) = its proximal joint
        self.hadr = {}
        for s in SIDES:
            rows = []
            for c in range(6):
                i = int(np.where(np.isclose(C[s][:, c], 1.0))[0][0])
                rows.append(m.jnt_qposadr[m.joint(meta["hand"][s][i]).id])
            self.hadr[s] = rows
        q11 = np.zeros(11)
        for k, s in ((0, "left"), (5, "right")):
            q11[k:k + 5] = d.qpos[self.adr[s]]
        self.tgt = q11.copy()
        self.des = q11.copy()
        self.vel = np.zeros(11)
        self.last_tgt, self.tgt_t, self.v_tgt = q11.copy(), d.time, np.zeros(11)
        self.htgt = {s: self._hmeas(s) for s in SIDES}           # 0..1, sim order
        self.hands = {s: self.htgt[s].copy() for s in SIDES}
        self.hbuf = {s: collections.deque() for s in SIDES}
        self.hout = {s: self.hands[s].copy() for s in SIDES}
        self.sent = self.des.copy()
        self.t_next = d.time
        self.k = 0
        self.lag_max = np.zeros(11)
        self.tau_max = 0.0
        self.log = []

    def _hmeas(self, s):
        return np.clip(self.d.qpos[self.hadr[s]] / FULL_SIM, 0, 1)

    def target(self, s, q5, hand_rad6=None):
        k = 0 if s == "left" else 5
        self.tgt[k:k + 5] = q5
        if hand_rad6 is not None:
            self.htgt[s] = np.clip(np.asarray(hand_rad6, float) / FULL_SIM, 0, 1)

    # -------------------------------------------------- server, 100 Hz (as server_core.step / track)
    def _track(self, tgt, dt, now):
        if np.max(np.abs(tgt - self.last_tgt)) > 1e-7:
            v = (tgt - self.last_tgt) / max(now - self.tgt_t, dt)
            jump = np.abs(v) > 2 * V_MAX
            v[jump] = 0.0
            self.v_tgt = np.clip(0.5 * self.v_tgt + 0.5 * v, -V_MAX, V_MAX)
            self.v_tgt[jump] = 0.0
            self.last_tgt, self.tgt_t = tgt.copy(), now
        elif now - self.tgt_t > 0.06:
            self.v_tgt = np.zeros(11)
        tgt = tgt + self.v_tgt * min(now - self.tgt_t, 0.06)
        dd = tgt - self.des
        v_stop = np.sqrt(2 * A_MAX * np.abs(dd)) + np.abs(self.v_tgt)
        v_ref = np.clip(self.v_tgt + 10.0 * dd, -np.minimum(V_MAX, v_stop), np.minimum(V_MAX, v_stop))
        self.vel = self.vel + np.clip(v_ref - self.vel, -A_MAX * dt, A_MAX * dt)
        st = self.vel * dt
        over = (np.abs(st) > np.abs(dd)) & (np.abs(self.v_tgt) < 1e-6)
        st[over] = dd[over]
        self.vel[over] = 0.0
        return self.cfg.clamp_arm(self.des + st)

    def _server(self):
        dt, now = 0.01, self.d.time
        tgt = self.cfg.clamp_arm(self.tgt)
        tgt[10] = 0.0
        self.des = self._track(tgt, dt, now)
        if self.k % 2 == 0:                                   # hands at 50 Hz
            for s in SIDES:
                hm = self._hmeas(s)
                dh = np.clip(self.htgt[s] - self.hands[s], -V_HAND * 0.02, V_HAND * 0.02)
                h_new = np.clip(self.hands[s] + dh, 0, 1)
                closing = h_new > self.hands[s]
                cap = hm + LEAD
                h_new = np.where(closing & (h_new > cap), np.maximum(cap, np.minimum(self.hands[s], h_new)), h_new)
                self.hands[s] = h_new
        h12 = np.r_[self.hands["left"][SIM2SRV], self.hands["right"][SIM2SRV]]
        grav = self.comp.offset(self.des, h12, self.cfg.KP, self.payload)
        self.sent = self.cfg.clamp_arm(self.des + grav)
        self.k += 1

    # -------------------------------------------------- physics step
    def step(self):
        m, d = self.m, self.d
        if d.time >= self.t_next - 1e-9:
            self._server()
            self.t_next += 0.01
            q11 = np.r_[d.qpos[self.adr["left"]], d.qpos[self.adr["right"]], 0.0]
            self.lag_max = np.maximum(self.lag_max, np.abs(self.des - q11))
        for k, s in ((0, "left"), (5, "right")):
            d.ctrl[self.A[s]] = self.arm[s].delayed(d.time, self.sent[k:k + 5])
            self.arm[s].apply(d)
            self.hbuf[s].append((d.time, self.hands[s].copy()))
            while len(self.hbuf[s]) > 1 and self.hbuf[s][1][0] <= d.time - HAND_DELAY:
                self.hbuf[s].popleft()
            a = 1.0 - math.exp(-m.opt.timestep / HAND_LAG)
            self.hout[s] += (self.hbuf[s][0][1] - self.hout[s]) * a
            d.ctrl[self.HA[s]] = np.minimum(self.C[s] @ (self.hout[s] * FULL_SIM), self.hhi[s])
        mujoco.mj_step(m, d)
        self.tau_max = max(self.tau_max, float(np.max(np.abs(d.actuator_force[np.r_[self.A["left"], self.A["right"]]]))))

    def summary(self):
        return dict(lag_max_rad=[round(float(x), 3) for x in self.lag_max[:10]], motor_tau_max_Nm=round(self.tau_max, 2),
                    trip_lag=bool(np.max(self.lag_max[:10]) > 0.25), trip_tau=bool(self.tau_max > 15.0))
