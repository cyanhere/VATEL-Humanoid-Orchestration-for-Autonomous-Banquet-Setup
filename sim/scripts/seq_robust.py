"""Robustness of the two-hand sequence (right grip, carry to centre, left pinch, lift) over post-push states, camera
error, friction and arm gains. Each condition chains grip_slot.py, grip_centre.py and grip_left.py.

    python scripts/seq_robust.py OUT.jsonl WORKERS [--gains stiff,real] [--quick]
"""
import argparse
import itertools
import json
import os
import subprocess
import sys
import tempfile
import time
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable
RIGHT = ("--psi 0 --head -15 --appr 30 --tm 0.3 --tm_close 1.55 --tp_close 1.04 --sy 20.6 --sx 1 --sz 100 --others 1 --back 60 "
         "--in_T 3 --push_in 0 --up 10 --left_away 1 --left_retreat 15 --flex 0.15 --allow 4 --din 2 --pitch 7 --mflex 0.55 --relax 0.08 --relax_thumb 0.1")
CARRY = "--rigid --x 0.29 --dz 0.04 --enc_fb " + __import__("os").environ.get("ENC_FB", "0")
LEFT = ("--psi 10 --head -15 --appr 30 --din 2 --pitch 3 --sz 100 --sy -20.6 --flex 0.0 --mflex 0.55 --tm 0.3 --tm_close 1.55 "
        "--tp_close 1.04 --push_in 0 --relax 0.08 --relax_thumb 0.1 --allow 4")
GAINS = {"stiff": {}, "real": {"DEXPICK_KP_ARM": "50,50,40,40,30", "DEXPICK_KV_ARM": "2"}}


def last_json(txt):
    i, j = txt.find("{"), txt.rfind("}")
    return json.loads(txt[i:j + 1]) if i >= 0 else None


def chain(c):
    env = dict(os.environ, OMP_NUM_THREADS="1", **GAINS[c["gains"]])
    out = dict(cond=c)
    t0 = time.time()
    with tempfile.TemporaryDirectory(dir=os.path.join(ROOT, "data", "tmp")) as td:
        sR, sC = os.path.join(td, "R.npz"), os.path.join(td, "C.npz")
        e = c["err"]
        mu = f"--mu_pad {c['mu'][0]} --mu_glaze {c['mu'][1]}"
        extra = c.get("right_extra", "")
        cmd = (f"{PY} scripts/grip_slot.py --state {c['state']} {RIGHT} {extra} {mu} --pos_err_x {e[0]} --pos_err_y {e[1]} "
               f"--pos_err_z {e[2]} --save {sR}")
        r = subprocess.run(cmd, shell=True, cwd=ROOT, env=env, capture_output=True, text=True)
        try:
            out["right"] = last_json(r.stdout.split("wall time")[0])
        except Exception as ex:                          # noqa: BLE001
            out["right"] = dict(error=repr(ex), tail=r.stdout[-300:] + r.stderr[-300:])
        if "ERROR" in r.stdout and isinstance(out["right"], dict):     # keep the planner's reason (grip_search)
            out["right"]["error"] = r.stdout[r.stdout.find("ERROR"):].split("{")[0].strip()
        if not os.path.exists(sR):
            out["stage"] = "right"
            return out
        r = subprocess.run(f"{PY} scripts/grip_centre.py {sR} {CARRY} --save {sC}", shell=True, cwd=ROOT, env=env, capture_output=True, text=True)
        lines = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
        out["carry"] = json.loads(lines[-1]) if lines else dict(error=r.stderr[-300:])
        if not os.path.exists(sC):
            out["stage"] = "carry"
            return out
        le = c.get("left_err", (0, 0, 0))
        r = subprocess.run(f"{PY} scripts/grip_left.py --state {sC} {LEFT} {c.get('left_extra', '')} --pos_err_x {le[0]} --pos_err_y {le[1]} --pos_err_z {le[2]}",
                           shell=True, cwd=ROOT, env=env, capture_output=True, text=True)
        try:
            out["left"] = last_json(r.stdout.split("wall time")[0])
        except Exception as ex:                          # noqa: BLE001
            out["left"] = dict(error=repr(ex), tail=r.stdout[-300:] + r.stderr[-300:])
        # further left-hand variants from the same carried state ("name=args;name=args")
        for item in filter(None, c.get("left_variants", "").split(";")):
            name, args = item.split("=", 1)
            r = subprocess.run(f"{PY} scripts/grip_left.py --state {sC} {LEFT} {args} --pos_err_x {le[0]} --pos_err_y {le[1]} --pos_err_z {le[2]}",
                               shell=True, cwd=ROOT, env=env, capture_output=True, text=True)
            try:
                out.setdefault("lefts", {})[name] = last_json(r.stdout.split("wall time")[0])
            except Exception as ex:                      # noqa: BLE001
                out.setdefault("lefts", {})[name] = dict(error=repr(ex), tail=r.stdout[-300:] + r.stderr[-300:])
        out["stage"] = "done"
    out["wall_s"] = round(time.time() - t0)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("workers", type=int)
    ap.add_argument("--gains", default="stiff")
    ap.add_argument("--states", default="data/results/robot/state_push_best_*.npz")
    ap.add_argument("--right_extra", default="")
    ap.add_argument("--left_extra", default="", help="extra grip_left.py arguments (2026-10-16)")
    ap.add_argument("--left_variants", default="", help="'name=args;name=args': more grip_left.py runs from the same carried state")
    ap.add_argument("--quick", action="store_true", help="no camera / friction variation (one run per state)")
    a = ap.parse_args()
    import glob
    states = sorted(glob.glob(os.path.join(ROOT, a.states)))
    errs = [(0, 0, 0)] if a.quick else [(0, 0, 0), (3, 0, 0), (-3, 0, 0), (0, 3, 0), (0, -3, 0), (0, 0, 2), (0, 0, -2)]
    mus = [(0.25, 0.375)] if a.quick else [(0.25, 0.375), (0.2, 0.35), (0.3, 0.4)]
    conds = [dict(state=os.path.relpath(s, ROOT), err=e, mu=mu, gains=g, right_extra=a.right_extra, left_extra=a.left_extra, left_variants=a.left_variants)
             for s, e, mu, g in itertools.product(states, errs, mus, a.gains.split(","))]
    os.makedirs(os.path.join(ROOT, "data", "tmp"), exist_ok=True)
    print(f"{len(conds)} conditions", flush=True)
    with Pool(a.workers) as pool, open(a.out, "w") as f:
        for k, r in enumerate(pool.imap_unordered(chain, conds)):
            f.write(json.dumps(r, default=float) + "\n")
            f.flush()
            if k % 10 == 0:
                print(k + 1, "done", flush=True)


if __name__ == "__main__":
    main()
