#!/usr/bin/env python3
"""Run test stages, closed loop. Needs server.py at the pre-grip pose and perception.py.

    python3 run_test.py push                     # stage 1 only: touch-down, edge, push 42 mm; then the arms go home
    python3 run_test.py push grip1               #... and the first grip (your LEFT hand, robot right)
    python3 run_test.py push grip1 carry         #... and the lift / carry
    python3 run_test.py all                      # push grip1 carry grip2
    python3 run_test.py push --set push.press_mm=6 --set push.speed_mm_s=8     # tune without editing params/*.json
    python3 run_test.py push grip1 --hold        # at the end HOLD where it is (look); `python3 monitor.py home` after
    python3 run_test.py push --dry               # plan only (perception, geometry, IK); nothing moves
    python3 run_test.py push10                   # push10, the hooked-middle side push
    python3 run_test.py push10_air               #... its reference path 40 mm up in FREE AIR (no plate): tracking only
    python3 run_test.py push10_shape             #... the hand shape at the pre pose, held 15 s to look at
    python3 run_test.py push10_edge              #... search, touch and edge only, then lift off
    python3 run_test.py push10 --set push10.table_m=0.762 --dry
    python3 run_test.py push10_grip              # push10, then the left holds and the RIGHT hand grips the side that sticks out (demo, not simulated)
    python3 run_test.py lone_grip                # lone plate, both hands, grip params/lone_grip.json "grip"
    python3 run_test.py lone_grip_air            #... the whole path 60 mm higher, NO plate: tracking and the aim correction
    python3 run_test.py lone_grip_shape          #... the hand shapes pre / close / close2 at the approach pose, to look at
    python3 run_test.py lone_grip --set lone_grip.grip=v2_slot2      # the backups: v2_slot2, slot2_thumbclose
    python3 run_test.py armid                    # the left arm in FREE AIR, no plate in reach: how it follows commands (about 3 min)
    python3 run_test.py armid_load               #... holds one pose while you hang known weights from the wrist
    python3 run_test.py armwide_poses            # either arm (--set armwide.side=right), FREE AIR, across its working range; also armwide_motion
    python3 run_test.py armwide_joint            # either arm, each joint alone: sweeps, back-and-forth, steps
    python3 run_test.py handid_track             # both hands, arms still at the pre-grip pose: fingers against their commands; handid_load: palm up, hung weights
    python3 run_test.py armid_pose               #... armid at the contact pose; also armid_grav, armid_fric, armid_chirp, armid_hand
    python3 run_test.py nearforce_shape          # near-rim force test: hover and look; also _touch, _tip, _slip, _press
    python3 run_test.py nearforce --set nearforce.press_N=5 --set nearforce.n=3    # one recorded try
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402
from pk.client import Client, Denied, ServerGone  # noqa: E402
from pk.stage_base import Abort, Ctx, _js  # noqa: E402

ORDER = ["push", "grip1", "carry", "grip2"]
ANYWHERE = ("push10", "lone_grip")     # stages that plan from wherever the arms are (no move to the pre-grip pose first)
ALONE = ["edge", "push10_grip", "lone_grip", "lone_grip_air", "lone_grip_shape", "push10", "push10_air", "push10_shape", "push10_edge", "armid", "armid_load", "armid_pose", "armid_grav", "armid_fric",
         "armid_chirp", "armid_hand", "nearforce", "nearforce_shape", "nearforce_touch", "nearforce_press", "nearforce_tip", "nearforce_slip", "armwide_poses", "armwide_motion", "armwide_joint", "handid_track", "handid_load"]     # run on their own, not in ORDER


def parse_set(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        try:
            v = json.loads(v)
        except ValueError:
            pass
        out[k.strip()] = v
    return out


def make(ctx, name):
    if name == "push":
        from pk.stages.push import Push
        return Push(ctx)
    if name == "edge":
        from pk.stages.push import Edge
        return Edge(ctx)
    if name == "push10_grip":
        from pk.stages.push10_grip import Push10Grip
        return Push10Grip(ctx)
    if name.startswith("push10"):
        from pk.stages.push10 import Push10
        return Push10(ctx, name[7:] or "full")
    if name.startswith("lone_grip"):
        from pk.stages.lone_grip import LoneGrip
        return LoneGrip(ctx, name[10:] or "full")
    if name.startswith("armid"):
        from pk.stages.armid import ArmId
        return ArmId(ctx, name[6:] or "full")
    if name.startswith("armwide"):
        from pk.stages.armwide import ArmWide
        return ArmWide(ctx, name[8:])
    if name.startswith("nearforce"):
        from pk.stages.nearforce import NearForce
        return NearForce(ctx, name[10:] or "full")
    if name.startswith("handid"):
        from pk.stages.handid import HandId
        return HandId(ctx, name[7:])
    from pk.stages.replay import Replay
    return Replay(ctx, name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stages", nargs="+", help="push | grip1 | carry | grip2 | all (each needs the ones before it) | edge "
                    "(only the proximity edge search: where the finger finds the edge vs the camera) | push10 | push10_air | "
                    "push10_shape | push10_edge (the side push and its tests, each on its own)")
    ap.add_argument("--set", action="append", help="override a parameter: stage.key=value (JSON value)")
    ap.add_argument("--hold", action="store_true", help="at the end hold where the arms are instead of going home")
    ap.add_argument("--dry", action="store_true", help="plan only, nothing moves")
    ap.add_argument("--port", type=int, default=config.SERVER_PORT, help="the arm server's port (offline tests use another)")
    a = ap.parse_args()
    st = ORDER if a.stages == ["all"] else a.stages
    if len(st) == 1 and st[0] in ALONE:
        pass
    elif any(s in ALONE for s in st):
        ap.error("%s run on their own" % " / ".join(s for s in st if s in ALONE))
    else:
        bad = [s for s in st if s not in ORDER]
        if bad:
            ap.error("unknown stage %s" % bad)
        want = ORDER[:max(ORDER.index(s) for s in st) + 1]
        if st != want:
            print("stages %s need the ones before them: running %s" % (" ".join(st), " ".join(want)))
        st = want
    name = "_".join(st) if len(st) < 4 else "all"
    out = os.path.join(config.LOG_DIR, "tests", time.strftime("%Y%m%d_%H%M%S_") + name)
    overrides = parse_set(a.set)
    if a.dry:
        return dry(st, out, overrides, a.port)
    try:
        c = Client("test " + name, port=a.port)
    except (Denied, ServerGone) as ex:
        print("cannot start: %s" % ex)
        return 1
    summary = dict(stages=st, overrides=overrides, started=time.strftime("%Y-%m-%d %H:%M:%S"), results={}, ended="")
    then = "hold" if a.hold else "home"
    ctx = None

    def note(what, **kw):                                     # the test's log, or the terminal when it has none yet
        if ctx is not None:
            ctx.log(what, **kw)
        else:
            print("%s: %s" % (what, kw), flush=True)
    # from here on the arms are this test's: whatever happens below, the finally hands them back
    try:
        with c.busy(20):                                      # building the model takes a moment
            ctx = Ctx(c, out, overrides)
        if not st[0].startswith(ANYWHERE):
            ctx.log("start", note="moving to the pre-grip pose (params/premotion.json), where this test was planned")
            why = c.to_pregrip()
            if why:
                raise Abort("could not get to the pre-grip pose: %s" % why)
        ctx.log("test", stages=st, overrides=overrides, params={s: ctx.params("lone_grip" if s.startswith("lone_grip") else s.split("_")[0]) for s in st})
        for s in st:
            ctx.log("stage", name=s, note="start")
            res = make(ctx, s).run()
            summary["results"][s] = res
            ctx.log("stage", name=s, note="done", result=res)
        summary["ended"] = "completed"
    except Abort as ex:
        summary["ended"] = "aborted: %s" % ex
        note("ABORT", why=str(ex))
        then = "hold" if a.hold else "path"                   # where the stage stopped is not known to be clear: back the way it came
    except Denied as ex:
        summary["ended"] = "the server took the arms: %s" % ex
        note("STOPPED", why=str(ex))
        then = None
    except KeyboardInterrupt:
        summary["ended"] = "Ctrl-C"
        note("STOPPED", why="Ctrl-C")
        then = "path"
    except Exception as ex:                                   # a bug: still hand the arms back home
        summary["ended"] = "error: %r" % ex
        note("ERROR", why=repr(ex))
        import traceback
        traceback.print_exc()
        then = "path"
    finally:
        try:
            c.request("head_lock", on=False)
        except Exception:                                     # noqa: BLE001
            pass
        if then:
            c.bye(then=then, why=summary["ended"])
        c.close()
        os.makedirs(out, exist_ok=True)
        try:
            txt = json.dumps(_js(summary), indent=1, default=str)
        except (TypeError, ValueError):
            txt = json.dumps(summary, indent=1, default=str)
        open(os.path.join(out, "summary.json"), "w").write(txt)
        print("\n%s -> %s\nlog: %s" % (summary["ended"], {"home": "the arms go straight back HOME",
                                                          "path": "the arms go back HOME along the path they came",
                                                          "hold": "the arms HOLD here: `python3 monitor.py home` when done",
                                                          None: "the server has the arms"}[then], out))
    return 0 if summary["ended"] == "completed" else 2


def dry(stages, out, overrides, port=config.SERVER_PORT):
    """Plan without moving: perception's plate, where the push would touch down, whether the paths are reachable."""
    import numpy as np
    from pk.ik import ArmChain, shift_path, solve_point_dir
    from pk.model import PINKY_END
    from pk.plate import TASK
    from pk.refs import Reference
    try:
        c = Client("dry run", role="monitor", port=port)
    except ServerGone as ex:
        print(ex)
        return 1
    ctx = Ctx(c, out, overrides, dry=True)
    st = c.state()
    if not stages[0].startswith(ANYWHERE):                   # planned from the pre-grip pose, which the run moves to first
        pre = json.load(open(config.PREGRIP_FILE))
        st = dict(st, des=list(pre["q"][:10]) + [st["des"][config.WAIST]], hands_cmd=list(pre["hands"]))
    p = ctx.plate(st, max_age=2.0)
    if stages[0].startswith("push10"):
        from pk.stages.push10 import Push10
        Push10(ctx, stages[0][7:] or "full").dry(st, p)
        c.close()
        return 0
    if stages[0].startswith("lone_grip"):
        from pk.stages.lone_grip import LoneGrip
        LoneGrip(ctx, stages[0][10:] or "full").dry(st, p)
        c.close()
        return 0
    if stages[0].startswith("armwide"):
        from pk.stages.armwide import ArmWide
        ArmWide(ctx, stages[0][8:]).dry(st, p)
        c.close()
        return 0
    if stages[0].startswith("handid"):
        from pk.stages.handid import HandId
        HandId(ctx, stages[0][7:]).dry(st, p)
        c.close()
        return 0
    if stages[0].startswith("armid"):
        from pk.stages.armid import ArmId
        ArmId(ctx, stages[0][6:] or "full").dry(st, p)
        c.close()
        return 0
    if stages[0].startswith("nearforce"):
        from pk.stages.nearforce import NearForce
        NearForce(ctx, stages[0][10:] or "full").dry(st, p)
        c.close()
        return 0
    P = ctx.params("push")
    if p and p.get("center"):
        cx, cy, rim_z = p["center"]
        print("plate (perception): centre x %.3f y %+.3f rim z %.3f, %s, located %s plates" % (cx, cy, rim_z, p.get("status"),
                                                                                             (p.get("located") or {}).get("n_plates")))
    else:
        cx, cy = P["plate_xy"]
        rim_z = P["rim_z"]
        print("NO plate from perception: using params/push.json plate_xy / rim_z")
    off = np.hypot(cx - config.STACK_XY_SIM[0], cy - config.STACK_XY_SIM[1])
    print("offset from the reference spot (x %.2f y %.2f): %.0f mm (limit %.0f)" % (config.STACK_XY_SIM[0], config.STACK_XY_SIM[1],
                                                                                   1000 * off, 1000 * P["max_offset_m"]))
    h = ctx.pelvis(st)
    print("pelvis from the leg joints: %.3f m above the floor (the reference runs: 0.71)" % h)
    q = np.array(st["des"])
    hands = np.array(st["hands_cmd"])
    hands[5] = P["pinky_curl"]
    ch = ArmChain(ctx.rob, "left", "left_pinky_distal_link", q, hands)
    import math
    out_v = np.array([math.sin(math.radians(P["phi_deg"])), math.cos(math.radians(P["phi_deg"])), 0.0])
    g = math.radians(P["gamma_deg"])
    nrm = -math.cos(g) * out_v - math.sin(g) * np.array([0, 0, 1.0])
    top = np.array([cx, cy, rim_z - TASK.H])
    rc = TASK.R - P["cal_r_mm"] / 1000
    C = top + rc * out_v + np.array([0, 0, TASK.z_top(rc)])
    for name, pt in (("touch-down hover", C + np.array([0, 0, P["search_above_mm"] / 1000])), ("touch-down point", C),
                     ("search bottom", C - np.array([0, 0, (P["search_depth_mm"] - P["search_above_mm"]) / 1000])),
                     ("push start", top + TASK.edge_c[0] * out_v + np.array([0, 0, TASK.edge_c[1]]) - 0.025 * nrm),
                     ("push end", top + TASK.edge_c[0] * out_v + np.array([0, 0, TASK.edge_c[1]]) + np.array([0, -0.045, -0.006]))):
        q5, info = solve_point_dir(ch, q[config.LEFT], pt, PINKY_END["left"], [0, 0, 1], nrm, w=0.05)
        print("  %-18s (%.3f %+.3f %.3f): IK %.1f mm, %.1f deg %s" % (name, pt[0], pt[1], pt[2], info["pos_mm"], info.get("ang_deg", 0),
                                                                     "OK" if info["pos_mm"] < 2 else "OUT OF REACH"))
    if len(stages) > 1:
        ref = Reference()
        seg = ref.stage("2grip1")
        delta = np.array([cx, cy, rim_z - TASK.H]) - ref.stage("1push")["plate0"]      # the stack's offset carries over
        q_path, worst = shift_path(ctx.rob, "right", "right_base_link", np.array([-0.2556, 3.198, 101.5]) / 1000, seg["q"][::10],
                                   seg["h"][::10], delta)
        print("first grip, moved by the stack's offset %s mm: worst IK %.1f mm" % (np.round(1000 * delta, 1), worst))
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
