"""Join the armid_grav, armid_fric, armid_chirp, armid_hand and nearforce fits into one file for the simulation.

    python calibration/sim_arm_join.py logs/tests/<time>_armid_grav logs/tests/<time>_armid_fric... --out../models/sim_arm_R1.json
"""
import argparse
import json
import os
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = dict(robot="R1 left arm", made=time.strftime("%Y-%m-%d %H:%M"), joints=["sh pitch", "sh roll", "sh yaw", "elbow", "wrist roll"], parts={})
    for d in a.dirs:
        p = os.path.join(d, "sim_arm.json")
        if not os.path.exists(p):
            print("%s: no sim_arm.json (run calibration/armid_fit.py on it)" % d)
            continue
        for k, v in json.load(open(p)).items():
            out["parts"][k] = dict(v, run=os.path.basename(os.path.normpath(d)))
    missing = [k for k in ("grav", "fric", "chirp", "hand") if k not in out["parts"]]
    if missing:
        print("not measured yet: %s" % missing)
    json.dump(out, open(a.out, "w"), indent=1)
    print(a.out)


if __name__ == "__main__":
    main()
