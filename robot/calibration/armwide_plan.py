"""Plan the poses and motions of pk/stages/armwide.py for both arms, each checked in MuJoCo against the robot's collision shapes.

    python calibration/armwide_plan.py                 # writes params/armwide_plan.json and logs/armwide_plan_<side>.png
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
ROOT = os.path.join(os.path.dirname(PICK), "sim")
sys.path.insert(0, ROOT)
sys.path.insert(0, PICK)
import mujoco  # noqa: E402

from pk import config  # noqa: E402
from platepick.ik import PAD_POINT  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402

BOX = dict(x=(0.12, 0.48), y=(0.02, 0.42), z=(-0.03, 0.32))     # hand, pelvis frame; y mirrored for the right arm
PELVIS = 0.71
N_POSES = 30
N_CENTRES = 3
CLEAR_M = 0.03
FREQ = np.array([0.11, 0.19, 0.31, 0.47, 0.71])
MOTION_S = 40.0
BOX_MOTION = np.array([0.25, 0.25, 0.3, 0.3, 0.3])
A_MAX = 2.0                                                     # rad/s^2, under the server's 3
R_MAX = 0.6                                                     # rad, half the widest single-joint sweep


class Scene(object):
    def __init__(self, side, home):
        sc = RobotScene(n=1, table=0.747, xy=(2.36, 0.06), pelvis=PELVIS, fr=Friction(glaze_glaze=0.375, foot_glaze=0.575, pad_glaze=0.25, table=0.45))
        xml, self.meta = sc.build()
        self.m = m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(m)
        self.side = side
        self.sl = config.LEFT if side == "left" else config.RIGHT
        sl_other = config.RIGHT if side == "left" else config.LEFT
        other = "right" if side == "left" else "left"
        self.adr = [m.jnt_qposadr[m.joint(j).id] for j in self.meta["arm"][side]]
        self.d.qpos[[m.jnt_qposadr[m.joint(j).id] for j in self.meta["arm"][other]]] = home[sl_other]
        self.lo, self.hi = config.ARM_LO[self.sl] + config.LIMIT_MARGIN + 0.03, config.ARM_HI[self.sl] - config.LIMIT_MARGIN - 0.03
        self.idx_b = m.body("%s_index_touch_link" % side).id
        self.thb_b = m.body("%s_thumb_touch_link" % side).id
        self.mid_b = m.body("%s_middle_touch_link" % side).id
        self.elb_b = m.body("%s_elbow_link" % side).id
        self.wri_b = m.body("%s_wrist_roll_link" % side).id
        self.robot_geoms = m.geom_group == 3
        m.geom_margin[:] = 0.0
        for g in range(m.ngeom):
            if self.robot_geoms[g] and m.body(m.geom_bodyid[g]).name.startswith(side + "_") and (m.geom_contype[g] & 10):
                m.geom_margin[g] = CLEAR_M
        self.sy = 1.0 if side == "left" else -1.0

    def pads(self, q):
        self.d.qpos[self.adr] = q
        mujoco.mj_kinematics(self.m, self.d)
        return [self.d.xpos[b] + self.d.xmat[b].reshape(3, 3) @ PAD_POINT - np.array([0, 0, PELVIS]) for b in (self.idx_b, self.thb_b)]

    def frames(self, q):
        """Index and middle pad normals (the *_touch_link x axis), the wrist (pelvis frame) and the forearm's direction."""
        self.d.qpos[self.adr] = q
        mujoco.mj_kinematics(self.m, self.d)
        n = [self.d.xmat[b].reshape(3, 3)[:, 0].copy() for b in (self.idx_b, self.mid_b)]
        w = self.d.xpos[self.wri_b] - np.array([0, 0, PELVIS])
        f = self.d.xpos[self.wri_b] - self.d.xpos[self.elb_b]
        return n, w, f / np.linalg.norm(f)

    def hits(self, q):
        """The pairs of the robot's links within CLEAR_M of each other, at q."""
        d, m = self.d, self.m
        d.qpos[self.adr] = q
        mujoco.mj_forward(m, d)
        out = []
        for i in range(d.ncon):
            a, b = d.contact[i].geom1, d.contact[i].geom2
            if self.robot_geoms[a] and self.robot_geoms[b]:
                out.append((m.body(m.geom_bodyid[a]).name, m.body(m.geom_bodyid[b]).name))
        return out

    def in_box(self, q):
        for p in self.pads(q):
            if not (BOX["x"][0] < p[0] < BOX["x"][1] and BOX["y"][0] < self.sy * p[1] < BOX["y"][1] and BOX["z"][0] < p[2] < BOX["z"][1]):
                return False
        return True

    def ok(self, q, allowed=(), box=True):
        if np.any(q < self.lo) or np.any(q > self.hi) or (box and not self.in_box(q)):
            return False
        return not [h for h in self.hits(q) if h not in allowed]

    def path_ok(self, a, b, step=0.03, allowed=(), box=True):
        n = max(2, int(np.max(np.abs(b - a)) / step))
        return all(self.ok(a + u * (b - a), allowed, box) for u in np.linspace(0.0, 1.0, n + 1)[1:-1])


def motion_q(amp, phase, t, T):
    """The motion's offset from its centre at times t: sum of sines, faded in and out over 2 s. As pk/stages/armwide.py."""
    f = np.minimum(1.0, np.minimum(t, T - t) / 2.0)
    f = f * f * (3 - 2 * f)
    w = 2 * np.pi * FREQ
    return f[:, None] * np.sum(amp[None] * np.sin(w[None, None] * t[:, None, None] + phase[None]), axis=2)


def motion(rng, c, sc, v, allowed):
    """Random sines around c, scaled to peak speed v inside BOX_MOTION; None if too fast or it leaves the safe set."""
    a = rng.uniform(-1, 1, (5, len(FREQ))) / (1.0 + FREQ)
    ph = rng.uniform(0, 2 * np.pi, (5, len(FREQ)))
    t = np.arange(0.0, MOTION_S, 1.0 / 30)
    Q = motion_q(a, ph, t, MOTION_S)
    s = min(np.min(BOX_MOTION / np.max(np.abs(Q), axis=0)), v / np.max(np.abs(np.gradient(Q, t, axis=0))))
    Q = motion_q(a * s, ph, t, MOTION_S)
    V = np.gradient(Q, t, axis=0)
    acc = np.max(np.abs(np.gradient(V, t, axis=0)))
    if acc > A_MAX or not all(sc.ok(c + x, allowed) for x in Q[::6]):
        return None
    return dict(centre=np.round(c, 4).tolist(), amp=np.round(a * s, 5).tolist(), phase=np.round(ph, 4).tolist(), speed=v, peak_v=round(float(np.max(np.abs(V))), 3),
                peak_a=round(float(acc), 3), range=np.round(np.max(np.abs(Q), axis=0), 3).tolist())


def plan_side(side, home):
    sc = Scene(side, home)
    qh = home[sc.sl].copy()
    allowed = tuple(sc.hits(qh))
    if allowed:
        print("%s: within %.0f mm already at the pre-grip pose, allowed: %s" % (side, 1000 * CLEAR_M, sorted(set(allowed))))
    rng = np.random.RandomState(7 if side == "left" else 8)
    cand = []
    tried = 0
    while tried < 40000 and len(cand) < 1200:
        tried += 1
        q = rng.uniform(sc.lo, sc.hi)
        if sc.ok(q, allowed):
            cand.append(q)
    print("%s: %d safe poses of %d tried" % (side, len(cand), tried), flush=True)
    span = sc.hi - sc.lo
    d = 0.05                                                    # the approach goes +-0.04 on all joints together, both ways
    cand = [q for q in cand if sc.ok(q + d, allowed) and sc.ok(q - d, allowed) and sc.path_ok(qh, q, allowed=allowed, box=False)]
    print("%s: %d keep room for the approach and are reached straight from the pre-grip pose" % (side, len(cand)), flush=True)
    dist = [float(np.max(np.abs((q - qh) / span))) for q in cand]
    chosen = []
    while len(chosen) < N_POSES and cand:
        k = int(np.argmax(dist))
        q = cand.pop(k)
        dist.pop(k)
        chosen.append(q)
        dist = [min(x, float(np.max(np.abs((c - q) / span)))) for x, c in zip(dist, cand)]
    # the right thumb is at x 0.086 at the pre-grip pose, so moves to and from there are not held to BOX. No straight move to any pose left: go by the pre-grip pose
    order, via, cur = [], {}, qh
    rest = chosen
    while rest and len(order) < N_POSES:
        rest.sort(key=lambda q: float(np.max(np.abs(q - cur))))
        for q in rest:
            if sc.path_ok(cur, q, allowed=allowed, box=len(order) > 0):
                break
        else:
            q = rest[0]
            via[str(len(order))] = True
        order.append(q)
        rest = [x for x in rest if x is not q]
        cur = q
    print("%s: %d poses in order, %d by way of the pre-grip pose" % (side, len(order), len(via)), flush=True)
    motions = []
    centres = 0
    for i in rng.permutation(len(order)):                       # poses at the edge of the box have no room: go on until N_CENTRES have
        found = []                                              # all reached straight from the pre-grip pose (rest above)
        for v in (0.3, 0.6):
            for _ in range(60):
                mo = motion(rng, order[i], sc, v, allowed)
                if mo:
                    mo["pose"] = int(i)
                    found.append(mo)
                    break
        if len(found) == 2:
            motions += found
            centres += 1
        if centres == N_CENTRES:
            break
    order = np.array(order)
    rng2 = np.random.RandomState(17 if side == "left" else 18)
    joints = plan_joints(sc, order, allowed)
    palm = plan_palm_up(sc, qh, allowed, rng2)
    load = []
    for k, q in enumerate(order):
        _, w, f = sc.frames(q)
        if abs(f[2]) < 0.5 and w[0] > 0.25:
            load.append(k)
    print("%s: %d poses keep a bag on the wrist (forearm within 30 deg of level, wrist 0.25 m forward)" % (side, len(load)), flush=True)
    pads = np.array([sc.pads(q)[0] for q in order])
    print("%s: index pad x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f; joint range covered %s of %s; %d motions" % (
        side, pads[:, 0].min(), pads[:, 0].max(), pads[:, 1].min(), pads[:, 1].max(), pads[:, 2].min(), pads[:, 2].max(),
        np.round(np.max(order, axis=0) - np.min(order, axis=0), 2), np.round(span, 2), len(motions)), flush=True)
    return dict(poses=np.round(order, 4).tolist(), pads=np.round(pads, 3).tolist(), motions=motions, via_home=via, allowed=sorted(set(allowed)),
                joints=joints, palm_up=palm, load_poses=load), sc, order, pads


def plan_joints(sc, order, allowed):
    """For each joint, the pose (of `order`, all reached from the pre-grip pose) where that joint alone sweeps the widest +-r, every 0.03 rad checked."""
    out = []
    for j in range(5):
        best = (None, 0.0)
        for k, c in enumerate(order):
            r_ok = 0.0
            for r in np.arange(0.1, R_MAX + 1e-9, 0.05):
                a, b = c.copy(), c.copy()
                a[j], b[j] = c[j] - r, c[j] + r
                if not (sc.ok(a, allowed) and sc.ok(b, allowed) and sc.path_ok(a, b, allowed=allowed)):
                    break
                r_ok = float(r)
            if r_ok > best[1]:
                best = (k, r_ok)
        out.append(dict(joint=j, pose=best[0], centre=np.round(order[best[0]], 4).tolist(), range=round(best[1], 3)))
        print("%s: joint %d sweeps +-%.2f rad around pose %s" % (sc.side, j, best[1], best[0]), flush=True)
    return out


def plan_palm_up(sc, qh, allowed, rng):
    """The pose nearest the pre-grip pose with the index and middle pads facing up (within 30 deg), 0.25-0.45 m forward, +-0.05 rad of room."""
    best = None
    for _ in range(60000):
        q = rng.uniform(sc.lo, sc.hi)
        n, w, f = sc.frames(q)
        if min(n[0][2], n[1][2]) < 0.87:
            continue
        p = sc.pads(q)[0]
        if not (0.25 < p[0] < 0.45 and 0.05 < p[2] < 0.25):
            continue
        dist = float(np.max(np.abs(q - qh)))
        if best is not None and dist >= best[0]:
            continue
        if sc.ok(q, allowed) and sc.ok(q + 0.05, allowed) and sc.ok(q - 0.05, allowed) and sc.path_ok(qh, q, allowed=allowed, box=False):
            best = (dist, q, p, n)
    if best is None:
        print("%s: no palm-up pose found" % sc.side)
        return None
    dist, q, p, n = best
    print("%s: palm-up pose %s, index pad %s, pad normals up %.2f %.2f, %.2f rad from the pre-grip pose" % (sc.side, np.round(q, 2), np.round(p, 3), n[0][2], n[1][2], dist), flush=True)
    return dict(q=np.round(q, 4).tolist(), pad=np.round(p, 3).tolist())


def draw(side, order, pads, sc, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a0, a1, a2) = plt.subplots(1, 3, figsize=(12, 4.2), gridspec_kw=dict(width_ratios=[1, 1, 1.4]))
    a0.plot(pads[:, 0], pads[:, 1], "-o", ms=3, lw=0.5, color="#2a6fb0")
    a0.set(xlabel="x forward, m", ylabel="y left, m", title="R1 %s arm plan: index pad from above" % side)
    a1.plot(pads[:, 0], pads[:, 2], "-o", ms=3, lw=0.5, color="#2a6fb0")
    a1.axhline(0.037, color="#a33", lw=1)
    a1.text(pads[:, 0].min(), 0.043, "height of the 0.747 m table (pull the robot back)", fontsize=7, color="#a33")
    a1.set(xlabel="x forward, m", ylabel="z up from the pelvis, m", title="from the side")
    x = (order - sc.lo) / (sc.hi - sc.lo)
    for j in range(5):
        a2.plot(np.full(len(x), j) + np.linspace(-0.25, 0.25, len(x)), x[:, j], "o", ms=3, color="#2a6fb0")
    a2.set_xticks(range(5))
    a2.set_xticklabels(["pitch", "roll", "yaw", "elbow", "wrist"])
    a2.set(ylabel="position within the limits, 0..1", title="joint range covered", ylim=(-0.05, 1.05))
    for a in (a0, a1, a2):
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def main():
    home = np.array(json.load(open(os.path.join(PICK, "params", "premotion.json")))["q"], float)
    plan = dict(_about="Made by calibration/armwide_plan.py, MuJoCo %s. Do not edit by hand: the checks hold for these numbers." % mujoco.__version__, home=np.round(home, 4).tolist(),
                clear_m=CLEAR_M, box=BOX, freq=FREQ.tolist(), motion_s=MOTION_S)
    os.makedirs(os.path.join(PICK, "logs"), exist_ok=True)
    for side in ("left", "right"):
        res, sc, order, pads = plan_side(side, home)
        plan[side] = res
        draw(side, order, pads, sc, os.path.join(PICK, "logs", "armwide_plan_%s.png" % side))
    json.dump(plan, open(os.path.join(PICK, "params", "armwide_plan.json"), "w"), indent=0)
    print("written")


if __name__ == "__main__":
    main()
