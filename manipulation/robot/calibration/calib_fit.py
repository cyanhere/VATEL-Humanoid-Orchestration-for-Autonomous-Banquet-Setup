"""Fit where the camera sits on the head from calib_cam.py frames and write model/camera_correction.json.

    python calibration/calib_fit.py DIR [DIR...]          # npz frames; laptop or robot (numpy + scipy)
    python calibration/calib_fit.py DIR --dry              # fit and report, write nothing
    python calibration/calib_fit.py DIR --model 6dof       # 3dof (default) | 3dof_head | 6dof
"""
import argparse
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
sys.path.insert(0, PICK)

from pk import config  # noqa: E402
from pk.model import Robot, hand_points  # noqa: E402
from pk.tracker import deproject, project  # noqa: E402

MIRROR = os.path.join(PICK, "..", "robot_mirror", "R1_Plate_Perception")      # laptop: read-only copy


def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


NPAR = {"3dof": 4, "3dof_head": 6, "6dof": 7}           #... + the depth range bias on the hand, last


def bounds(model):
    """Physical limits: a few degrees and centimetres, never a turn that a head offset cancels."""
    t, s, h, b = np.radians(15), 0.06, 0.10, 0.02
    lo = {"3dof": [-t, -s, -s, -b], "3dof_head": [-t, -s, -s, -h, -h, -b], "6dof": [-t, -t, -t, -s, -s, -s, -b]}[model]
    return np.array(lo), -np.array(lo)


def correction(x, model):
    """(4x4 C in head_yaw_link, head offset (pitch, yaw), depth range bias b) from the parameter vector."""
    C = np.eye(4)
    if model in ("3dof", "3dof_head"):
        C[:3, :3] = rot("z", x[0])
        C[:2, 3] = x[1:3]
        off = np.array(x[3:5]) if model == "3dof_head" else np.zeros(2)
    else:
        C[:3, :3] = rot("z", x[2]) @ rot("y", x[1]) @ rot("x", x[0])
        C[:3, 3] = x[3:6]
        off = np.zeros(2)
    return C, off, float(x[-1])


def biased(o, b):
    """Depth points moved b along their rays (the camera reads the dark hand's surface b too close: b > 0)."""
    return o + b * o / np.linalg.norm(o, axis=1, keepdims=True)


class Frames(object):
    def __init__(self, dirs, kin, rob, min_in_view=0.4, first_only=False):
        self.kin, self.rob, self.fr = kin, rob, []
        files = sorted(f for d in dirs for f in glob.glob(os.path.join(d, "*.npz")))
        if first_only:
            files = [f for f in files if not f.endswith("_1.npz") or "_" not in os.path.basename(f)]
        for f in files:
            z = np.load(f)
            st = json.loads(str(z["state"]))
            side = str(z["side"]) if "side" in z.files else "left"
            if "in_view" in z.files and float(z["in_view"]) < min_in_view:
                continue
            ang = json.loads(str(z["angles"])) if "angles" in z.files else kin.angles(st)
            intr, T_cd, scale = json.loads(str(z["intr"])), z["T_cd"], float(z["depth_scale"])
            depth = z["depth"]
            H, W = depth.shape
            P = deproject(depth, intr, scale, (0, W, 0, H), 2)
            P = P[(P[:, 2] > 0.15) & (P[:, 2] < 1.2)]
            P = P @ T_cd[:3, :3].T + T_cd[:3, 3]                      # depth optical -> colour optical
            T = rob.fk(np.array(st["q"]), np.array(st["hands"]))
            model = np.vstack(list(hand_points(T, side).values()))
            plate = st.get("plate") if isinstance(st.get("plate"), dict) else None
            self.fr.append(dict(name=os.path.basename(f), cloud_o=P, model=model, ang=ang, side=side,
                                intr_c=json.loads(str(z["intr_c"])), plate=plate, color=z["color"] if z["color"].ndim == 3 else None,
                                T_po_raw=z["T_po"]))

    def T_po(self, fr, C, off):
        ang = dict(fr["ang"])
        ang["head_pitch_joint"] += off[0]
        ang["head_yaw_joint"] += off[1]
        cc = self.kin.cc
        if hasattr(cc, "T_pelvis_headyaw"):          # R1_Plate_Perception >= the yaw axis' tilt (neck_tilt_rad)
            T_hy = cc.T_pelvis_headyaw(self.kin.joints, ang, self.kin.cal.get("neck_tilt_rad") or (0.0, 0.0))
        else:
            T_hy = self.kin.fk.T(self.kin.joints, self.kin.rcfg.ROOT_LINK, "head_yaw_link", ang)
        return T_hy @ C @ np.array(self.kin.cal["T_headyaw_optical"], float)


def visible(P, T, intr, cell=4):
    Po = (P - T[:3, 3]) @ T[:3, :3]
    uv = (project(Po, intr) / cell).astype(int)
    order = np.argsort(Po[:, 2])
    _, first = np.unique(uv[order, 0] * 100000 + uv[order, 1], return_index=True)
    return order[first]


def crop(fr, P):
    """Real points near the model hand, without the table under it and the plate (if the frame knows one)."""
    m = fr["model"]
    lo, hi = m.min(axis=0) - np.array([0.06, 0.06, 0.03]), m.max(axis=0) + 0.06
    keep = np.all((P > lo) & (P < hi), axis=1)
    pl = fr["plate"]
    if pl and pl.get("center"):
        c = pl["center"]
        r = np.hypot(P[:, 0] - c[0], P[:, 1] - c[1])
        keep &= ~((r < 0.1275 + 0.006) & (P[:, 2] < c[2] + 0.004))
    return keep


def coarse_offset(fr, T, b=0.0, span=0.08, step=0.01):
    """Translation (pelvis) that best puts the model's visible hand on the real points: grid +-span, then ICP."""
    from scipy.spatial import cKDTree
    P = biased(fr["cloud_o"], b) @ T[:3, :3].T + T[:3, 3]
    m = fr["model"]
    lo, hi = m.min(axis=0) - 0.09, m.max(axis=0) + 0.09
    P = P[np.all((P > lo) & (P < hi), axis=1)]
    if len(P) < 200:
        return None
    tree = cKDTree(P)
    M = m[visible(m, T, fr["intr_c"])]
    M = M[:: max(1, len(M) // 600)]
    best, sh = 1e9, np.zeros(3)
    g = np.arange(-span, span + 1e-4, step)
    for dx in g:
        for dy in g:
            for dz in np.arange(-min(span, 0.04), min(span, 0.04) + 1e-4, step):
                s = np.array([dx, dy, dz])
                dd, _ = tree.query(M + s, distance_upper_bound=0.015)
                cost = np.mean(np.minimum(dd, 0.015))
                if cost < best:
                    best, sh = cost, s
    for cap in (0.015, 0.012, 0.010, 0.008, 0.008):
        dd, ii = tree.query(M + sh, distance_upper_bound=cap)
        ok = np.isfinite(dd)
        if ok.sum() < 50:
            break
        sh = sh + np.median(P[ii[ok]] - (M[ok] + sh), axis=0)
    return sh


def fit(F, model, x0, caps=(0.03, 0.02, 0.012, 0.008, 0.006, 0.006), per_frame=300, verbose=True):
    from scipy.optimize import least_squares
    from scipy.spatial import cKDTree
    x = np.array(x0, float)
    for cap in caps:
        pairs = []
        C, off, b = correction(x, model)
        for k, fr in enumerate(F.fr):
            T = F.T_po(fr, C, off)
            P = biased(fr["cloud_o"], b) @ T[:3, :3].T + T[:3, 3]
            keep = crop(fr, P)
            if keep.sum() < 100:
                continue
            idx_c = np.nonzero(keep)[0]
            tree = cKDTree(P[idx_c])
            vis = visible(fr["model"], T, fr["intr_c"])
            dd, ii = tree.query(fr["model"][vis], distance_upper_bound=cap)
            ok = np.isfinite(dd)
            sel = np.nonzero(ok)[0]
            if len(sel) > per_frame:
                sel = sel[np.linspace(0, len(sel) - 1, per_frame).astype(int)]
            pairs.append((k, fr["model"][vis[sel]], fr["cloud_o"][idx_c[ii[sel]]]))

        def resid(xx):
            Cx, offx, bx = correction(xx, model)
            r = []
            for k, Pm, Po in pairs:
                T = F.T_po(F.fr[k], Cx, offx)
                r.append((biased(Po, bx) @ T[:3, :3].T + T[:3, 3] - Pm).ravel())
            return np.concatenate(r) if r else np.zeros(1)
        sol = least_squares(resid, x, loss="soft_l1", f_scale=0.003, bounds=bounds(model))
        x = sol.x
        if verbose:
            r = resid(x).reshape(-1, 3)
            print("   cap %4.0f mm: %5d pairs, median |residual| %.1f mm" % (1000 * cap, len(r), 1000 * np.median(np.linalg.norm(r, axis=1))))
    return x


def describe(x, model):
    C, off, b = correction(x, model)
    yaw = np.degrees(np.arctan2(C[1, 0], C[0, 0]))
    return "turn %+.2f deg about the head's yaw axis, shift %+.1f %+.1f %+.1f mm (head_yaw_link)%s; depth reads the hand %.1f mm close" % (
        yaw, *(1000 * C[:3, 3]), (", head encoder offsets pitch %+.2f yaw %+.2f deg" % tuple(np.degrees(off))) if np.any(off) else "", 1000 * b)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--model", default="3dof", choices=["3dof", "3dof_head", "6dof"])
    ap.add_argument("--perception", default=config.PERCEPTION_DIR if os.path.isdir(config.PERCEPTION_DIR) else MIRROR)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--plot", default=None, help="write a figure of the result here (png)")
    ap.add_argument("--first-frame-only", action="store_true", help="one frame per head aim (calib_cam saves 2, near-identical)")
    a = ap.parse_args()
    import perception as per
    kin = per.Kin(a.perception, correct=False)
    rob = Robot()
    F = Frames(a.dirs, kin, rob, first_only=a.first_frame_only)
    print("%d frames (hand in view) from %s" % (len(F.fr), ", ".join(a.dirs)))
    if len(F.fr) < 3:
        print("too few frames")
        return 1
    heads = np.array([[fr["ang"]["head_pitch_joint"], fr["ang"]["head_yaw_joint"]] for fr in F.fr])
    spread = np.degrees(heads.max(axis=0) - heads.min(axis=0))
    print("head poses: pitch spread %.1f deg, yaw spread %.1f deg%s" % (
        spread[0], spread[1], "  -- TOO SIMILAR: the camera's mount and the head's encoders cannot be told apart" if max(spread) < 5 else ""))
    # 1. per frame, as R1_Plate_Perception has the camera
    I, z2 = np.eye(4), np.zeros(2)
    d0 = []
    for fr in F.fr:
        sh = coarse_offset(fr, F.T_po(fr, I, z2))
        d0.append(sh if sh is not None else np.full(3, np.nan))
    d0 = np.array(d0)
    ok0 = np.all(np.isfinite(d0), axis=1)
    print("before: where the camera sees the hand minus where the arm puts it, per frame (mm): median x %+.1f y %+.1f z %+.1f, "
          "spread %.1f %.1f %.1f" % (*(1000 * np.nanmedian(d0, axis=0)), *(1000 * np.nanstd(d0, axis=0))))
    # 2. a start for the fit: the shifts as a turn + shift of the camera (centroid correspondences), then ICP
    models = [a.model] if a.model != "3dof" else ["3dof", "3dof_head", "6dof"]
    results = {}
    for model in models:
        x0 = np.zeros(NPAR[model])
        from scipy.optimize import least_squares

        def r0(xx):
            C, off, b = correction(xx, model)
            out = []
            for fr, d in zip(F.fr, d0):
                if not np.all(np.isfinite(d)):
                    continue
                c = fr["model"].mean(axis=0)
                T_raw = F.T_po(fr, I, z2)
                o = (c + d - T_raw[:3, 3]) @ T_raw[:3, :3]            # the hand as the camera saw it (optical)
                T = F.T_po(fr, C, off)
                out.append(biased(o[None, :], b)[0] @ T[:3, :3].T + T[:3, 3] - c)
            return np.concatenate(out)
        x0 = least_squares(r0, x0, bounds=bounds(model)).x
        print("%s: start %s" % (model, describe(x0, model)))
        x = fit(F, model, x0)
        C, off, b = correction(x, model)
        d1 = []
        for fr in F.fr:
            sh = coarse_offset(fr, F.T_po(fr, C, off), b, span=0.024, step=0.006)
            d1.append(sh if sh is not None else np.full(3, np.nan))
        d1 = np.array(d1)
        res = np.linalg.norm(d1, axis=1)
        results[model] = (x, d1)
        print("%s: %s\n   after: per-frame offset median |d| %.1f mm, worst %.1f mm, (x %+.1f y %+.1f z %+.1f)" % (
            model, describe(x, model), 1000 * np.nanmedian(res), 1000 * np.nanmax(res), *(1000 * np.nanmedian(d1, axis=0))))
    # the simplest model unless a richer one is clearly better (by > 1.5 mm median); never one stuck on a limit
    def at_bound(m):
        lo, hi = bounds(m)
        return bool(np.any(np.isclose(results[m][0], lo, atol=1e-4) | np.isclose(results[m][0], hi, atol=1e-4)))
    order = [m for m in ("3dof", "3dof_head", "6dof") if m in results]
    fine = [m for m in order if not at_bound(m)] or order
    for m in order:
        if at_bound(m):
            print("%s: a parameter sits on its limit -- not trusted" % m)
    best = fine[0]
    for m in fine[1:]:
        if np.nanmedian(np.linalg.norm(results[m][1], axis=1)) < np.nanmedian(np.linalg.norm(results[best][1], axis=1)) - 0.0015:
            best = m
    x, d1 = results[best]
    C, off, b = correction(x, best)
    summary = "%s: %s; per-frame residual median %.1f mm (was %.1f), %d frames" % (
        best, describe(x, best), 1000 * np.nanmedian(np.linalg.norm(d1, axis=1)), 1000 * np.nanmedian(np.linalg.norm(d0[ok0], axis=1)), len(F.fr))
    print("CHOSEN " + summary)
    if a.plot:
        plot(F, d0, d1, C, off, a.plot, summary)
    if a.dry:
        return 0
    out = dict(T_hy_corr=C.tolist(), head_offset=[float(v) for v in off], model=best, params=[float(v) for v in x],
               hand_depth_bias_m=b,
               summary=summary, date=time.strftime("%Y-%m-%d %H:%M"), frames=[fr["name"] for fr in F.fr], dirs=a.dirs,
               note="T_pelvis_optical = T_pelvis_headyaw(head angles + head_offset) @ T_hy_corr @ T_headyaw_optical (R1_Plate_Perception's)")
    path = os.path.join(config.MODEL_DIR, "camera_correction.json")
    json.dump(out, open(path, "w"), indent=1)
    print("wrote %s -- sync it to the robot; perception.py applies it from its next start" % path)
    return 0


def plot(F, d0, d1, C, off, path, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import cv2
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_subplot(2, 3, 1)
    for k, (lab, col) in enumerate((("x", "#2a78d6"), ("y", "#eb6834"), ("z", "#1baf7a"))):
        ax.plot(1000 * d0[:, k], "o", color=col, ms=4, label="%s before" % lab, alpha=0.5)
        ax.plot(1000 * d1[:, k], "s", color=col, ms=4, label="%s after" % lab)
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("frame")
    ax.set_ylabel("camera sees the hand - arm puts it (mm)")
    ax.legend(fontsize=7, ncol=2)
    ax.set_title("per-frame offset, before / after", fontsize=9, loc="left")
    picks = [i for i in np.linspace(0, len(F.fr) - 1, 5).astype(int)]
    for j, i in enumerate(picks):
        fr = F.fr[i]
        if fr["color"] is None:
            continue
        img = np.ascontiguousarray(fr["color"][:, :, ::-1]) if fr["color"].ndim == 3 else None
        for T, col in ((F.T_po(fr, np.eye(4), np.zeros(2)), (220, 60, 255)), (F.T_po(fr, C, off), (60, 255, 60))):
            uv = project((fr["model"] - T[:3, 3]) @ T[:3, :3], fr["intr_c"]).astype(int)
            for u, v in uv[::4]:
                if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                    cv2.circle(img, (u, v), 1, col, -1)
        ax = fig.add_subplot(2, 3, 2 + j)
        ax.imshow(img[:, :, ::-1])
        ax.axis("off")
        ax.set_title(fr["name"], fontsize=8)
    fig.suptitle("Camera calibration on the real hand: magenta = the hand where the arm puts it, drawn with the old camera pose;"
                 " green = with the fitted one (should lie on the real hand)\n" + summary, fontsize=9, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, dpi=90)
    print("wrote", path)


if __name__ == "__main__":
    sys.exit(main())
