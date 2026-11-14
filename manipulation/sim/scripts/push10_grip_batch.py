"""Run push10_grip_sim.py in batches, one process per run. Writes OUT/results.jsonl and OUT/<name>/.

    python scripts/push10_grip_batch.py --sweep variants --out data/results/push10_grip_sim/sweep_variants --workers 8
"""
import argparse
import json
import multiprocessing as mp
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FIX = dict(height="plane", tap=1)
VARIANTS = {
    "v0_as_run": dict(height="median"),
    "v1_fixed_sq_hook3": dict(FIX, hook=3),
    "v2_sq0": dict(FIX, sq=0),
    "v3_sq0_hook8": dict(FIX, sq=0, hook=8),
    "v4_hook8": dict(FIX, hook=8),
    "v5_sq5": dict(FIX, sq=5),
    "v6_sq0_out55": dict(FIX, sq=0, out=55),
}
# robustness: each kept variant against camera error (tap-corrected), push distance, pad / table friction, rim-plane error, table
CONDS = {
    "base": {},
    "cam-12": dict(cam_y=-12, tap_err=-2), "cam+12": dict(cam_y=12, tap_err=2),
    "slide35": dict(slide=35), "slide48": dict(slide=48),
    "pad0.5": dict(pad=0.5), "tbl0.3": dict(table_mu=0.3), "tbl0.6": dict(table_mu=0.6),
    "plane-2": dict(plane_err=-2), "plane+2": dict(plane_err=2),
    "glaze0.35": dict(glaze=0.35), "foot0.5": dict(foot=0.5),
    "table0.747": dict(table=0.747), "table0.762": dict(table=0.762),
}


def one(job):
    name, par, out, frames = job
    import push10_grip_sim as g
    try:
        r = g.run(par, os.path.join(out, name), frames)
    except Exception:                                     # noqa: BLE001
        r = dict(error=traceback.format_exc())
    r.pop("par", None)
    return dict(name=name, set=par, **r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", default="variants")
    ap.add_argument("--keep", default="v2_sq0,v5_sq5", help="variants for --sweep robust")
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--frames", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.sweep == "variants":
        jobs = [(k, dict(v, lift=30), a.out, a.frames) for k, v in VARIANTS.items()]
    else:
        jobs = [("%s__%s" % (k, c), dict(VARIANTS[k], **cv, lift=30), a.out, a.frames) for k in a.keep.split(",") for c, cv in CONDS.items()]
    with mp.Pool(a.workers) as pool, open(os.path.join(a.out, "results.jsonl"), "a") as f:
        for r in pool.imap_unordered(one, jobs):
            f.write(json.dumps(r, default=float) + "\n")
            f.flush()
            print(r["name"], r.get("result", r.get("error", "")[-300:]), flush=True)


if __name__ == "__main__":
    main()
