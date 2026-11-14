"""The simulation's reference sequence: arm and hand targets per stage, phases inside each stage, and the top plate pose."""
import json
import os

import numpy as np

from . import config

STAGES = ["1push", "2grip1", "3carry", "4grip2"]
_CACHE = {}


def cached():
    """The default Reference, loaded once per process (it reads several JSON logs: ~1 s)."""
    if "ref" not in _CACHE:
        _CACHE["ref"] = Reference()
    return _CACHE["ref"]


class Reference(object):
    def __init__(self, csv_path=None, sim_dir=None, pelvis=config.PELVIS_SIM, name="n3_t0.747"):
        csv_path = csv_path or os.path.join(config.REFS_DIR, "p071_n3mr_t0.747.csv")
        sim_dir = sim_dir or os.path.join(config.REFS_DIR, "n3mr")
        rows = [ln.strip().split(",") for ln in open(csv_path) if ln[:1].isdigit()]
        self.t = np.array([float(r[0]) for r in rows])
        self.stage_of = np.array([r[1] for r in rows])
        self.q = np.array([[float(x) for x in r[2:13]] for r in rows])
        self.h = np.array([[float(x) for x in r[13:25]] for r in rows])
        self.phase = np.array(self.stage_of, dtype=object)
        self.pelvis = pelvis
        self.plate_t, self.plate_p = [], []
        self.left_ready = None
        for st in STAGES:
            lp = os.path.join(sim_dir, "%s_%s_log.json" % (name, st))
            if os.path.exists(lp):
                L = json.load(open(lp))
                for s in L["samples"]:
                    self.plate_t.append(s["t"])
                    p = np.array(s["plates"][-1][:3], float)
                    p[2] -= pelvis
                    self.plate_p.append(p)
            rp = os.path.join(sim_dir, "%s_%s_rec.json" % (name, st))
            if os.path.exists(rp):
                R = json.load(open(rp))
                t0 = self.start(st)
                dt = R.get("dt", 1 / 30.0)
                tt = t0 + dt * np.arange(len(R["steps"]))
                ph = [s["phase"] for s in R["steps"]]
                m = self.stage_of == st
                idx = np.searchsorted(tt, self.t[m], side="right") - 1
                self.phase[m] = [ph[max(0, min(i, len(ph) - 1))] for i in idx]
        o = np.argsort(self.plate_t)
        self.plate_t = np.array(self.plate_t)[o]
        self.plate_p = np.array(self.plate_p)[o]

    def start(self, st):
        return float(self.t[np.nonzero(self.stage_of == st)[0][0]])

    def end(self, st):
        return float(self.t[np.nonzero(self.stage_of == st)[0][-1]])

    def plate_at(self, t):
        return np.array([np.interp(t, self.plate_t, self.plate_p[:, k]) for k in range(3)])

    def stage(self, st):
        m = self.stage_of == st
        t = self.t[m]
        return dict(t=t - t[0], t_ref=t, q=self.q[m], h=self.h[m], phase=self.phase[m], name=st,
                    plate0=self.plate_at(t[0]), plate1=self.plate_at(t[-1]))

    def phases(self, st):
        seg = self.stage(st)
        out, start = [], 0
        for i in range(1, len(seg["phase"]) + 1):
            if i == len(seg["phase"]) or seg["phase"][i] != seg["phase"][start]:
                out.append((seg["phase"][start], float(seg["t"][start]), float(seg["t"][i - 1])))
                start = i
        return out
