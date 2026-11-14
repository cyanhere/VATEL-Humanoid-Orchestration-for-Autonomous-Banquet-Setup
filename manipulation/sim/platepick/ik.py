"""Small inverse kinematics for the MuJoCo robot scene: place a finger pad at a point, facing a given direction."""
import numpy as np

import mujoco

PAD_POINT = np.array([0.0, 0.0, 0.001])      # centre of the pad face in the *_touch_link frame (face is the plane x = 0)
PAD_NORMAL = np.array([1.0, 0.0, 0.0])       # outward normal of the pad face
FINGER_AXIS = np.array([0.0, 0.0, 1.0])      # toward the fingertip


class PadIK:
    def __init__(self, m, meta, side="left", finger="index", mode="pad"):
        """mode "pad": the fingertip pad and the direction it faces.
        mode "tip": the end of the fingertip and the direction it points (a curled finger that pokes or hooks the rim).
        mode "side+" / "side-": one side of the last finger segment (+y / -y of the segment), for a curled finger moved
        sideways.
        """
        self.m, self.d = m, mujoco.MjData(m)
        self.side, self.finger, self.mode = side, finger, mode
        self.body = m.body(f"{side}_{finger}_touch_link").id
        self.pt, self.nrm = PAD_POINT, PAD_NORMAL
        # the END of the fingertip (in the last finger segment), whatever the contact mode: used to hold its height
        self.end_body = m.body(f"{side}_{finger}_distal_link").id
        g = next(g for g in range(m.ngeom) if m.geom_bodyid[g] == self.end_body and m.geom_group[g] == 3)
        mid = m.geom_dataid[g]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.geom_quat[g])
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]] @ R.reshape(3, 3).T + m.geom_pos[g]
        top = v[v[:, 2] > v[:, 2].max() - 0.0015]
        self.end_pt = np.array([top[:, 0].mean(), top[:, 1].mean(), v[:, 2].max()])
        self.end_verts = v[v[:, 2] > v[:, 2].max() - 0.012]      # the last 12 mm of the finger, for its lowest point
        if mode == "tip":
            self.body = m.body(f"{side}_{finger}_distal_link").id
            g = next(g for g in range(m.ngeom) if m.geom_bodyid[g] == self.body and m.geom_group[g] == 3)
            mid = m.geom_dataid[g]
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[g])
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]] @ R.reshape(3, 3).T + m.geom_pos[g]
            top = v[v[:, 2] > v[:, 2].max() - 0.0015]
            self.pt = np.array([top[:, 0].mean(), top[:, 1].mean(), v[:, 2].max()])
            self.nrm = FINGER_AXIS
        if mode in ("side+", "side-"):
            sgn = 1.0 if mode == "side+" else -1.0
            self.body = m.body(f"{side}_{finger}_distal_link").id
            g = next(g for g in range(m.ngeom) if m.geom_bodyid[g] == self.body and m.geom_group[g] == 3)
            mid = m.geom_dataid[g]
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[g])
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]] @ R.reshape(3, 3).T + m.geom_pos[g]
            zc = v[:, 2].max() - 0.010                       # 10 mm back from the fingertip end
            band = v[np.abs(v[:, 2] - zc) < 0.004]
            self.pt = np.array([band[:, 0].mean(), sgn * np.abs(band[:, 1]).max(), zc])
            self.nrm = np.array([0.0, sgn, 0.0])
        self.wrist = m.body(f"{side}_wrist_roll_link").id
        self.palm = m.body(f"{side}_base_link").id
        self.z_keep = None            # if set: prefer the wrist and the palm above this height (soft)
        self.hand_dir = None          # if set: prefer the hand (wrist to knuckles) pointing this way (soft)
        self.arm = [m.joint(j).id for j in meta["arm"][side]]
        hj, C = meta["hand"][side], np.array(meta["coupling"][side])
        act = meta["hand_act"][side]
        k = act.index(f"{side}_{finger}_proximal_joint")
        # physical joints driven by this finger's command, with their coupling gains
        self.flex = [(m.joint(hj[i]).id, C[i, k]) for i in range(len(hj)) if C[i, k] != 0]
        self.lo = np.array([m.jnt_range[j][0] for j in self.arm] + [m.jnt_range[self.flex[0][0]][0]])
        self.hi = np.array([m.jnt_range[j][1] for j in self.arm] + [m.jnt_range[self.flex[0][0]][1]])

    def set(self, d, x):
        for j, v in zip(self.arm, x[:5]):
            d.qpos[self.m.jnt_qposadr[j]] = v
        for j, g in self.flex:
            d.qpos[self.m.jnt_qposadr[j]] = min(g * x[5], self.m.jnt_range[j][1])

    def pad(self, d):
        R = d.xmat[self.body].reshape(3, 3)
        return d.xpos[self.body] + R @ self.pt, R @ self.nrm, R @ FINGER_AXIS

    def end(self, d):
        """World position of the fingertip end, and the height of the lowest point of the finger's last 12 mm."""
        R = d.xmat[self.end_body].reshape(3, 3)
        return d.xpos[self.end_body] + R @ self.end_pt, float((d.xpos[self.end_body] + self.end_verts @ R.T)[:, 2].min())

    def residual(self, x, p, n, axis=None, w_axis=0.0):
        self.set(self.d, x)
        mujoco.mj_kinematics(self.m, self.d)
        pp, nn, aa = self.pad(self.d)
        r = [pp - p, 0.05 * (nn - n)]
        if axis is not None:
            r.append(w_axis * (aa - axis))
        if self.z_keep is not None:
            r.append(0.3 * np.array([max(0.0, self.z_keep - self.d.xpos[b][2]) for b in (self.wrist, self.palm)]))
        if self.hand_dir is not None:
            r.append(0.03 * (self.d.xmat[self.palm].reshape(3, 3)[:, 2] - self.hand_dir))
        return np.concatenate(r)

    def solve(self, p, n, seeds, axis=None, w_axis=0.0, base=None):
        """Bounded least squares from several seeds. `base` is a full qpos to start from (the other joints stay as
        they are there). Returns the best solution and, under "all", every seed's result."""
        from scipy.optimize import least_squares
        best, cands = None, []
        lo, hi = self.lo + 1e-4, self.hi - 1e-4
        for x0 in seeds:
            if base is not None:
                self.d.qpos[:] = base
            r = least_squares(lambda x: self.residual(x, p, n, axis, w_axis), np.clip(np.array(x0, float), lo, hi),
                              bounds=(self.lo, self.hi), xtol=1e-10, ftol=1e-12)
            x = r.x
            if self.z_keep is not None or self.hand_dir is not None:   # preferences only steer: finish on the pose alone
                keep, self.z_keep = (self.z_keep, self.hand_dir), None
                self.hand_dir = None
                x = least_squares(lambda x: self.residual(x, p, n, axis, w_axis), np.clip(x, lo, hi),
                                  bounds=(self.lo, self.hi), xtol=1e-10, ftol=1e-12).x
                self.z_keep, self.hand_dir = keep
            self.set(self.d, x)
            mujoco.mj_kinematics(self.m, self.d)
            pp, nn, aa = self.pad(self.d)
            err = (float(np.linalg.norm(pp - p)), float(np.degrees(np.arccos(np.clip(nn @ n, -1, 1)))))
            cost = err[0] * 1000 + err[1]
            cands.append(dict(x=x.copy(), pos_err_mm=err[0] * 1000, ang_err_deg=err[1], axis=aa.copy()))
            if best is None or cost < best[0]:
                best = (cost, x.copy(), err, aa.copy())
        return dict(x=best[1], pos_err_mm=best[2][0] * 1000, ang_err_deg=best[2][1], axis=best[3], all=cands)
