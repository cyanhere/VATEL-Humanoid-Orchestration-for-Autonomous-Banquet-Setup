"""Run friction_replay.py over runs, stack shifts and frictions in parallel.

    python scripts/friction_batch.py OUT_DIR WORKERS --reg data/results/friction_replay/registration.json \
"""
import argparse
import itertools
import json
import os
import subprocess
import sys
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def one(job):
    run, pad, fg, tb, dy, out = job
    cmd = [sys.executable, os.path.join(ROOT, "scripts", "friction_replay.py"), run, "-o", os.path.join(out, run),
           "--pad", str(pad), "--fg", str(fg), "--table_mu", str(tb), "--dy", str(dy)]
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    return run, pad, fg, tb, dy, r.returncode, (r.stdout.strip().splitlines() or [""])[-1][:300], r.stderr[-400:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("workers", type=int)
    ap.add_argument("--reg", required=True)
    ap.add_argument("--pad", default="0.25,0.5,0.75,1.0,1.3")
    ap.add_argument("--fg", default="0.45,0.575,0.7")
    ap.add_argument("--table_mu", default="0.9")
    ap.add_argument("--runs", default=None)
    ap.add_argument("--dy_n", type=int, default=3)
    a = ap.parse_args()
    reg = json.load(open(a.reg))
    runs = a.runs.replace(":", ",").split(",") if a.runs else sorted(reg)   # ":" also separates (sbatch --export splits on commas)
    jobs = []
    for run in runs:
        ok = sorted(reg[run]["dy_ok"])
        dys = sorted({reg[run]["dy_best"], ok[0], ok[-1]})[:a.dy_n]
        for pad, fg, tb, dy in itertools.product(*(map(float, v.replace(":", ",").split(",")) for v in (a.pad, a.fg, a.table_mu)), dys):
            jobs.append((run, pad, fg, tb, dy, a.out))
    print("jobs:", len(jobs), flush=True)
    os.makedirs(a.out, exist_ok=True)
    with Pool(a.workers) as p, open(os.path.join(a.out, "batch.log"), "a") as lg:
        for k, res in enumerate(p.imap_unordered(one, jobs)):
            lg.write(json.dumps(res) + "\n")
            lg.flush()
            print("%d/%d" % (k + 1, len(jobs)), res[:6], flush=True)
    print("BATCH_DONE", flush=True)


if __name__ == "__main__":
    main()
