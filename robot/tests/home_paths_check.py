"""Check the server's joint-space moves to and from HOME in MuJoCo: self-collision, table and plate, along every move a
test makes. Uses the URDF collision shapes, not the real robot.

    python tests/home_paths_check.py [out.png]
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
ROOT = os.path.join(os.path.dirname(PICK), "sim")
sys.path.insert(0, PICK)
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402

from pk import config  # noqa: E402
from pk.model import Robot, hand_points  # noqa: E402
from pk.plate import TASK  # noqa: E402
from pk.stage_base import load_params  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402

PELVIS, TABLE = 0.71, 0.755
CLEAR = 0.02
FULL = [1.0472, 1.55334, 1.41, 1.41, 1.41, 1.41]          # service order: thumb bend, thumb rotation, index.. little


class FakeCtx(object):
    def __init__(self, rob):
        self.rob = rob

    def params(self, name):
        return load_params(name)


def neutral():
    return np.array(json.load(open(os.path.join(config.PARAMS_DIR, "home_neutral.json")))["q"], float)


def pregrip():
    h = json.load(open(os.path.join(config.PARAMS_DIR, "premotion.json")))
    return np.array(h["q"], float), np.array(h["hands"], float)


def lone_high(rob, grip, q0):
    from pk.stages.lone_grip import LoneGrip
    ctx = FakeCtx(rob)
    lg = LoneGrip.__new__(LoneGrip)
    LoneGrip.__init__(lg, type("C", (), dict(params=lambda s, n: dict(load_params(n), grip=grip), rob=rob))(), "full")
    st = dict(des=list(q0), hands_cmd=[0.0] * 12)
    lg.plan(st, (0.33, 0.0), TABLE - PELVIS, 0.0)
    return lg.q11_at("high", corr=False), lg.hands12("pre")


def push10_high(rob, q0):
    from pk.stages.push10 import Push10
    ctx = type("C", (), dict(params=lambda s, n: dict(load_params(n), table_m=TABLE), rob=rob))()
    p = Push10(ctx, "full")
    st = dict(des=list(q0), hands_cmd=[0.0] * 12)
    p.build(st)
    p.cx, p.cy = config.STACK_XY_SIM
    rim_z = TABLE - PELVIS + 2 * 0.0068 + TASK.H
    p.z_edge = float(rim_z - TASK.H + TASK.z_top(TASK.R))
    p.yE = p.cy + TASK.R
    pts = dict((n, pt) for n, pt, _ in p.plan_points())
    q5, e_mm, _ = p.ik(pts["high"], p.q_plan, p.P["w_rot"])
    q = np.array(q0, float)
    q[config.LEFT] = q5
    return q, p.hands.copy(), e_mm


class Check(object):
    def __init__(self, clear=CLEAR):
        sc = RobotScene(n=1, table=TABLE, xy=(0.33, 0.0), pelvis=PELVIS, fr=Friction())
        xml, self.meta = sc.build()
        self.m = m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(m)
        self.robot = m.geom_group == 3
        m.geom_margin[:] = 0.0
        for g in range(m.ngeom):
            if self.robot[g]:
                m.geom_margin[g] = clear
        self.adr = {s: [m.jnt_qposadr[m.joint(j).id] for j in self.meta["arm"][s]] for s in ("left", "right")}
        self.hadr = {s: [m.jnt_qposadr[m.joint(j).id] for j in self.meta["hand"][s]] for s in ("left", "right")}
        self.C = {s: np.array(self.meta["coupling"][s]) for s in ("left", "right")}
        self.hhi = {s: np.array([m.jnt_range[m.joint(j).id][1] for j in self.meta["hand"][s]]) for s in ("left", "right")}
        self.plate = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("plate")}

    def put(self, q11, h12):
        for k, s in enumerate(("left", "right")):
            self.d.qpos[self.adr[s]] = q11[config.LEFT if s == "left" else config.RIGHT]
            svc = np.asarray(h12[6 * k:6 * k + 6], float)
            sim = np.array([svc[1] * FULL[1], svc[0] * FULL[0]] + [svc[i] * FULL[i] for i in range(2, 6)])   # tr, tf, ci.. cp
            self.d.qpos[self.hadr[s]] = np.minimum(self.C[s] @ sim, self.hhi[s])
        mujoco.mj_forward(self.m, self.d)

    def contacts(self):
        rr, env = set(), set()
        m, d = self.m, self.d
        for i in range(d.ncon):
            a, b = d.contact[i].geom1, d.contact[i].geom2
            na, nb = m.body(m.geom_bodyid[a]).name, m.body(m.geom_bodyid[b]).name
            if self.robot[a] and self.robot[b]:
                if d.contact[i].dist < CLEAR:
                    rr.add(tuple(sorted((na, nb))))
            elif self.robot[a] or self.robot[b]:
                r, o = (a, b) if self.robot[a] else (b, a)
                if d.contact[i].dist < 0.0:
                    env.add((m.body(m.geom_bodyid[r]).name, "plate" if o in self.plate else (m.body(m.geom_bodyid[o]).name or "table")))
        return rr, env


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PICK, "logs", "home_paths_check.png")
    rob = Robot(offsets=None)
    ck = Check()
    qn = neutral()
    hn = np.zeros(12)
    qp, hp = pregrip()
    moves = [("neutral -> pre-grip pose", qn, hn, qp, hp)]
    for g in ("v2_thumb", "v2_slot2", "slot2_thumbclose"):
        qh, hh = lone_high(rob, g, qn)
        moves.append(("lone grip %s 'high' -> neutral" % g, qh, hh, qn, hh))
    q10, h10, e10 = push10_high(rob, qn)
    moves.append(("push10 'high' (IK %.0f mm) -> neutral" % e10, q10, h10, qn, h10))
    # at rest: the neutral pose itself
    ck.put(qn, hn)
    rr0, env0 = ck.contacts()
    print("MuJoCo %s, the plan's R1 scene (URDF collision shapes), table %.3f, pelvis %.2f, clearance %.0f mm" % (
        mujoco.__version__, TABLE, PELVIS, 1000 * CLEAR))
    print("neutral HOME itself: robot-robot within %.0f mm %s; table / plate %s" % (1000 * CLEAR, sorted(rr0) or "none", sorted(env0) or "none"))
    table_top = TABLE - PELVIS
    paths = []
    for name, qa, ha, qb, hb in moves:
        rr, env = set(), set()
        low, low_x = 9.0, None
        P = []
        for u in np.linspace(0.0, 1.0, 61):
            s = u * u * (3 - 2 * u)
            q = qa + s * (qb - qa)
            h = ha + s * (hb - ha)
            ck.put(q, h)
            a, b = ck.contacts()
            rr |= a
            env |= b
            T = rob.fk(q, h)
            pts = np.vstack([np.vstack(list(hand_points(T, sd).values())) for sd in ("left", "right")])
            k = int(np.argmin(pts[:, 2]))
            if pts[k, 2] < low:
                low, low_x = float(pts[k, 2]), pts[k, :2]
            P.append(np.vstack([T["%s_base_link" % sd][:3, 3] for sd in ("left", "right")]))
        paths.append((name, np.array(P)))
        print("%-46s robot-robot within %.0f mm: %s | touches table / plate: %s | lowest hand point %+.0f mm vs the table top (at x %.2f y %+.2f)" % (
            name, 1000 * CLEAR, sorted(rr) or "none", sorted(env) or "none", 1000 * (low - table_top), low_x[0], low_x[1]))
    draw(paths, table_top, out)


def draw(paths, table_top, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
    for i, (name, P) in enumerate(paths):
        c = "C%d" % i
        for k, ls in ((0, "-"), (1, "--")):
            ax[0].plot(P[:, k, 0], P[:, k, 2], ls, color=c, label=name if k == 0 else None)
            ax[1].plot(P[:, k, 1], P[:, k, 0], ls, color=c)
            ax[0].plot(P[-1, k, 0], P[-1, k, 2], "o", color=c, ms=4)
    ax[0].axhline(table_top, color="k", lw=1)
    ax[0].fill_between([0.33 - TASK.R - 0.015, 0.7], table_top - 0.03, table_top, color="#ccc")
    ax[0].text(0.6, table_top + 0.005, "table 0.755 (pelvis 0.71)", fontsize=8)
    ax[0].set_xlabel("x forward (m, pelvis)")
    ax[0].set_ylabel("z up (m, pelvis)")
    ax[0].set_title("side view: palm origin of each hand (left solid, right dashed); dot = end", fontsize=9, loc="left")
    ax[1].add_patch(plt.Circle((0.0, 0.33), TASK.R, fill=False, color="gray"))
    ax[1].set_xlabel("y left (m)")
    ax[1].set_ylabel("x forward (m)")
    ax[1].invert_xaxis()
    ax[1].set_title("top view (plate at x 0.33 for the lone grip)", fontsize=9, loc="left")
    ax[0].legend(fontsize=7, loc="upper left")
    for a in ax:
        a.grid(alpha=0.3)
        a.set_aspect("equal")
    fig.suptitle("Straight joint-space moves to / from the neutral HOME (MuJoCo, the plan's R1 scene; not the R1)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("figure:", out)


if __name__ == "__main__":
    main()
