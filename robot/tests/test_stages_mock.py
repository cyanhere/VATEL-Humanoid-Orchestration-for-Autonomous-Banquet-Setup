"""Offline run of the test stages against the simulated arms and a simple plate stack (logic only, no contact physics).

    python3 tests/test_stages_mock.py [--stages push grip1 carry grip2] [--rim-err 12]
    python3 tests/test_stages_mock.py --stages edge --profile --cam-xy-err 20 -42     # the proximity edge search alone
    python3 tests/test_stages_mock.py --stages push --profile --cam-xy-err 20 -42 --set push.edge_search=true
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from mock_world import FakePerception, MockStack  # noqa: E402
from pk import config  # noqa: E402
from pk.client import Client, operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server, load_home  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402

PORT = 9721


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stages", nargs="+", default=["push", "grip1", "carry", "grip2"])
    ap.add_argument("--rim-err", type=float, default=12.0, help="the camera's rim height error, mm (+ = reads too high)")
    ap.add_argument("--friction", type=float, default=0.3, help="the simulated joints' Coulomb friction, N m (the real fit: 0.3-0.9)")
    ap.add_argument("--no-touch", action="store_true", help="no fingertip touch data (as the R1 on 2026-10-20)")
    ap.add_argument("--stiction", type=float, default=1.0, help="breakaway / sliding friction of the simulated joints (> 1: stick-slip)")
    ap.add_argument("--grav-scale", type=float, default=1.2, help="the simulated arm's true gravity x URDF (the server's fit: 1.1-2.5)")
    ap.add_argument("--profile", action="store_true", help="the plate's measured cross-section (slope to the rim) and proximity")
    ap.add_argument("--cam-xy-err", type=float, nargs=2, default=(0.0, 0.0), help="the camera's plate centre error, mm (x y)")
    ap.add_argument("--set", action="append", default=[], help="stage.key=value parameter override (JSON value)")
    ap.add_argument("--from-push-end", action="store_true",
                    help="skip the mock push: replay the reference push into place, plate where the simulation's push left it")
    a = ap.parse_args()
    stack = MockStack(center=(0.366, 0.063), rim_z=0.058, profile=a.profile)
    io = MockBackend(grav_scale=a.grav_scale, friction=a.friction)
    io.stiction = a.stiction
    io.touch_alive = not a.no_touch
    io.contact = stack.contact
    if a.profile:
        io.proximity = stack.proximity
    # HOME = the pre-grip pose, where these stages were planned (the server's default is the neutral
    # pose; run_test.py moves there first, these tests call the stages directly)
    srv = Server(io, home=load_home("pregrip"), port=PORT, verbose=True, log_dir=os.path.join(os.path.dirname(HERE), "logs", "test"))
    run = dict(stop=False, trace=[])

    def loop():
        nxt = time.time()
        while not run["stop"]:
            srv.step()
            if srv.des is not None:
                run["trace"].append((time.time(), srv.mode, srv.tag, srv.des.copy(), io.s.q.copy(), srv.hands.copy(), io.s.hand_q.copy(),
                                     None if io.last_force is None else io.last_force.copy(), stack.c.copy()))
            nxt += 0.01
            time.sleep(max(0.0, nxt - time.time()))
    threading.Thread(target=loop, daemon=True).start()
    time.sleep(0.3)
    operator("take", port=PORT)
    while srv.mode != "HELD":
        time.sleep(0.05)
    operator("home", port=PORT)
    while srv.mode != "HOME":
        time.sleep(0.05)
    per = FakePerception(stack, port=PORT, rim_err_mm=a.rim_err, xy_err_mm=a.cam_xy_err)
    time.sleep(0.5)
    # the right hand's fingers meet the rim at these values in the grips (the mock's "plate" for the fingers)
    io.finger_stop[6 + 2:6 + 6] = [0.25, 0.45, 0.85, 0.9]
    c = Client("test mock", port=PORT)
    out = os.path.join(os.path.dirname(HERE), "logs", "test", "stages_" + time.strftime("%H%M%S"))
    import json as _json
    overrides = {"push.debug": True} if os.environ.get("PUSH_DEBUG") else {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        overrides[k.strip()] = _json.loads(v)
    ctx = Ctx(c, out, overrides=overrides or None)
    ok = True
    pushed_mm = None
    t_start = time.time()
    if a.from_push_end:
        from pk.refs import cached
        from pk.plate import TASK
        with c.busy(20):
            ref = cached()
        seg = ref.stage("1push")
        # the arms along the reference push (open loop, the mock plate out of the way), plate placed where it ends up
        stack.c[:] = (10.0, 10.0)
        c.move_to(seg["q"][0], seg["h"][0], v=0.3)
        ctx.play(seg["q"], seg["h"], float(np.median(np.diff(seg["t"]))), speed=0.25)
        p1 = ref.stage("2grip1")["plate0"]
        stack.c[:] = p1[:2] + np.array([0.004, -0.003])          # a few mm off the simulation's spot
        stack.c0[:] = stack.c + np.array([0.0, 0.042])
        per.locate()
        ctx.memory["rim_z"] = p1[2] + TASK.H
        ctx.memory["push_end"] = dict(q=list(c.target()[0]))
        a.stages = [x for x in a.stages if x != "push"]
        print("arms at the reference push end; plate at %s" % np.round(stack.c, 4))
    try:
        for s in a.stages:
            if s == "push":
                from pk.stages.push import Push
                st = Push(ctx)
            elif s == "edge":
                from pk.stages.push import Edge
                st = Edge(ctx)
            else:
                from pk.stages.replay import Replay
                st = Replay(ctx, s)
                if s == "carry":
                    io.payload = ("right", 0.5225, "right_base_link", np.array([-0.2556, 3.198, 101.5]) / 1000)
                if s == "grip2":
                    io.finger_stop[:6] = [0.6, 0.9, 0.3, 0.5, 2, 2]
            res = st.run()
            print("== %s: %s" % (s, res))
            if s == "push":                   # the later stages nudge the crude mock plate too: judge the push here
                pushed_mm = 1000 * np.linalg.norm(stack.c - stack.c0)
    except Abort as ex:
        ok = False
        print("ABORT: %s" % ex)
    c.bye()
    t0 = time.time()
    while srv.mode != "HOME" and time.time() - t0 < 120:
        time.sleep(0.1)
    home = srv.mode == "HOME"
    run["stop"] = True
    per.stop = True
    time.sleep(0.1)
    srv.close()
    push = ctx.results.get("push", {})
    checks = []
    if "push" in a.stages:
        checks.append(("rim found by touch (true %.1f mm, camera said %.1f)" % (1000 * stack.rim_z, 1000 * (stack.rim_z + a.rim_err / 1000)),
                       abs(push.get("rim_z", 0) - stack.rim_z) < 0.004))
        moved = pushed_mm if pushed_mm is not None else 1000 * np.linalg.norm(stack.c - stack.c0)
        checks.append(("plate pushed %.1f mm by the push (stop at 42; %.1f mm by the end of the run)" % (
            moved, 1000 * np.linalg.norm(stack.c - stack.c0)), 38 <= moved <= 50))
    edge = ctx.results.get("edge") or ctx.memory.get("edge")
    if "edge" in a.stages or edge:
        from pk.plate import TASK
        if edge is None:
            checks.append(("edge search finished", False))
        else:
            e = np.asarray(edge["edge"], float)
            c0 = stack.c0
            r_true = float(np.linalg.norm(e[:2] - c0))
            out_v = (e[:2] - c0) / max(r_true, 1e-9)
            err_r = 1000 * (r_true - TASK.R)
            checks.append(("edge found %.1f mm from the true edge (radially; the camera was %.0f %+.0f mm off)" % (
                err_r, a.cam_xy_err[0], a.cam_xy_err[1]), abs(err_r) < 3.0))
            checks.append(("rim top by proximity %.1f mm vs true %.1f (camera said %.1f)" % (
                1000 * edge["rim_z"], 1000 * stack.rim_z, 1000 * stack.rim_z + a.rim_err), abs(edge["rim_z"] - stack.rim_z) < 0.002))
    for s in a.stages[1:]:
        checks.append(("%s ran" % s, s in ctx.results))
    checks.append(("arms back home after the test", home))
    for what, good in checks:
        print(("PASS " if good else "FAIL ") + what)
    plot(run["trace"], stack, os.path.join(os.path.dirname(HERE), "logs", "test_stages_mock.png"), t_start)
    return 0 if ok and all(g for _, g in checks) else 1


def plot(trace, stack, path, t0):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    from pk.model import PINKY_END, Robot
    rob = Robot()
    tr = [x for x in trace if x[0] >= t0 - 1][::3]
    t = np.array([x[0] for x in tr]) - t0
    pz, py = [], []
    for x in tr:
        T = rob.fk(x[4], x[6])
        p = rob.point(T, "left_pinky_distal_link", PINKY_END["left"])
        pz.append(1000 * p[2])
        py.append(1000 * p[1])
    F = np.array([x[7] if x[7] is not None else np.zeros(3) for x in tr])
    pc = np.array([x[8] for x in tr])
    hands = np.array([x[6] for x in tr])
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(t, pz, label="pinky end z (measured)")
    ax[0].axhline(1000 * stack.rim_z, color="k", ls="--", lw=0.8, label="true rim top")
    ax[0].axhline(1000 * stack.rim_z - 6.8, color="gray", ls=":", lw=0.8, label="plate below")
    ax[0].set_ylabel("mm (pelvis frame)")
    ax[0].legend(fontsize=7)
    ax[1].plot(t, py, label="pinky end y")
    ax[1].plot(t, 1000 * (pc[:, 1] + stack.R), label="plate's left edge y")
    ax[1].set_ylabel("mm")
    ax[1].legend(fontsize=7)
    ax[2].plot(t, F[:, 0], label="Fx")
    ax[2].plot(t, F[:, 1], label="Fy")
    ax[2].plot(t, F[:, 2], label="Fz")
    ax[2].set_ylabel("N on the fingertip (mock)")
    ax[2].legend(fontsize=7)
    for j, nm in ((8, "R index"), (9, "R middle"), (10, "R ring"), (2, "L index"), (3, "L middle")):
        ax[3].plot(t, hands[:, j], label=nm)
    ax[3].set_ylabel("finger (measured, 0..1)")
    ax[3].legend(fontsize=7, ncol=5)
    modes = [x[1] + ":" + (x[2] or "") for x in tr]
    start = 0
    for i in range(1, len(modes) + 1):
        if i == len(modes) or modes[i] != modes[start]:
            ax[3].text((t[start] + t[i - 1]) / 2, 1.02, modes[start][:18], fontsize=5, rotation=60, ha="left",
                       transform=ax[3].get_xaxis_transform())
            start = i
    ax[3].set_xlabel("s")
    fig.suptitle("Stages against the simulated arms + a crude plate stack (tests/mock_world.py; NOT MuJoCo, NOT physics)", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("wrote", path)


if __name__ == "__main__":
    sys.exit(main())
