"""Plate model from its measured cross-section, as convex primitives, with per-surface-pair friction."""
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class PlateSpec:
    """Cross-section of the task plate, from measurements.

    Underside: foot ring, a first slope out to the 186 mm circle, then the wide rim out to the edge. Top side: flat
    floor 164 mm across, then a gentle rise into the rim. The first slope height (13.25 mm) is taken as measured with
    the plate upside down (8.75 mm right way up); this gives a plate no thinner than about 2.5 mm.
    """
    name: str = "task255"
    R: float = 0.1275               # outer radius (diameter 255 mm)
    H: float = 0.022                # total height
    rim_t: float = 0.0055           # rim thickness at the edge (vertical)
    mass: float = 0.5225
    foot_out: float = 0.0775        # foot ring outer radius ("foot circle 155 mm")
    foot_w: float = 0.005           # foot ring radial width (about 5 mm)
    foot_h: float = 0.002           # foot ring height (about 2 mm)
    floor_top: float = 0.0068       # well floor above the table = stacking pitch. One plate is
                                    # 21.8-22.5 mm tall, two stacked 28.5-29.3 mm, so the pitch is about 6.8 mm and
                                    # varies between plates (6.0-7.5 at the extremes); rims are then 0.5-2 mm apart
    floor_r: float = 0.082          # radius of the flat floor on the top side (164 mm across)
    fillet: float = 0.008           # ASSUMED radius of the rounding where the floor turns up into the slope
    kink_r: float = 0.093           # underside: the first slope ends here (186 mm circle)
    kink_z: float = 0.00875         #... at this height right way up (22 - 13.25, see the class note)
    shell: float = 0.0018           # thickness of the collision boxes that carry the two surfaces
    n_seg: int = 40

    # ---- derived cross-section
    @property
    def well_r(self):
        return self.floor_r

    @property
    def play(self):                  # flat floor beyond the foot ring, per side
        return self.floor_r - self.foot_out

    @property
    def rim_ang(self):               # slope of the rim (underside from the kink to the edge)
        return math.atan2((self.H - self.rim_t) - self.kink_z, self.R - self.kink_r)

    @property
    def wall_ang(self):              # slope of the first slope (underside from the foot to the kink)
        return math.atan2(self.kink_z - self.foot_h, self.kink_r - self.foot_out)

    @property
    def fillet_end(self):            # top side: where the rounding ends and the straight slope begins
        a, p = self.wall_ang, self.fillet
        return self.floor_r + p * math.sin(a), self.floor_top + p * (1 - math.cos(a))

    @property
    def top_kink(self):              # top side: where the slope meets the rim
        tw, tr = math.tan(self.wall_ang), math.tan(self.rim_ang)
        rf, zf = self.fillet_end
        r = (self.H - self.R * tr - zf + rf * tw) / (tw - tr)
        return r, zf + (r - rf) * tw

    @property
    def edge_r(self):                # radius of the rounded rim edge
        return self.rim_t * math.cos(self.rim_ang) / 2

    @property
    def edge_c(self):                # centre of the rounded rim edge (r, z)
        re = self.edge_r
        r = self.R - re
        z_top = self.H - (self.R - r) * math.tan(self.rim_ang)
        return r, z_top - re / math.cos(self.rim_ang)

    # ---- analytic surfaces (edge rounding ignored), plate frame: z = 0 at the foot bottom
    def z_top(self, r):
        rf, zf = self.fillet_end
        rk, zk = self.top_kink
        if r <= self.floor_r:
            return self.floor_top
        if r <= rf:
            return self.floor_top + self.fillet - math.sqrt(max(self.fillet ** 2 - (r - self.floor_r) ** 2, 0.0))
        if r <= rk:
            return zf + (r - rf) * math.tan(self.wall_ang)
        return zk + (r - rk) * math.tan(self.rim_ang)

    def z_under(self, r):
        if r < self.foot_out - self.foot_w:
            return self.foot_h
        if r <= self.foot_out:
            return 0.0
        if r <= self.kink_r:
            return self.foot_h + (r - self.foot_out) * math.tan(self.wall_ang)
        return self.kink_z + (r - self.kink_r) * math.tan(self.rim_ang)

    def top_line(self, n_fillet=3):
        """Top surface as a polyline (r, z) from the floor edge to the start of the rounded rim edge."""
        a = self.wall_ang
        arc = [(self.floor_r + self.fillet * math.sin(u), self.floor_top + self.fillet * (1 - math.cos(u)))
               for u in np.linspace(0, a, n_fillet + 1)]
        ce = self.edge_c
        return [*arc, self.top_kink, (ce[0], self.z_top(ce[0]))]

    def profile(self, n_arc=9):
        """Closed cross-section polygon of the right half, (r, z), counter-clockwise from the centre underside."""
        ce, re, a = self.edge_c, self.edge_r, self.rim_ang
        arc = [(ce[0] + re * math.cos(u), ce[1] + re * math.sin(u))
               for u in np.linspace(a - math.pi / 2, a + math.pi / 2, n_arc)]
        top = self.top_line(8)[:-1]
        return np.array([(0, self.foot_h), (self.foot_out - self.foot_w, self.foot_h), (self.foot_out - self.foot_w, 0),
                         (self.foot_out, 0), (self.foot_out, self.foot_h), (self.kink_r, self.kink_z), *arc, *top[::-1],
                         (0, self.floor_top)])

    def inertia(self, n=400):
        """(com_z, I_xx, I_zz) of the solid of revolution with the spec's mass, by sampling the cross-section."""
        r = (np.arange(n) + 0.5) / n * self.R
        zs = (np.arange(n) + 0.5) / n * self.H
        top = np.array([self.z_top(x) for x in r])
        und = np.array([self.z_under(x) for x in r])
        inside = (zs[None, :] <= top[:, None]) & (zs[None, :] >= und[:, None])
        w = inside * r[:, None]                       # dm ~ r dr dz
        m = w.sum()
        zc = (w * zs[None, :]).sum() / m
        izz = (w * r[:, None] ** 2).sum() / m * self.mass
        ixx = (w * (r[:, None] ** 2 / 2 + (zs[None, :] - zc) ** 2)).sum() / m * self.mass
        return zc, ixx, izz


TASK = PlateSpec()


@dataclass
class Friction:
    """Sliding friction per surface pair, through MuJoCo geom priorities (the higher-priority geom's value is used).

    glaze (0) < foot ring (1) < hand pad (2) < table (3) gives glaze-glaze, foot-glaze, pad-glaze and anything-table.
    Pad on the unglazed foot ring cannot be set separately and gets pad_glaze.
    """
    glaze_glaze: float = 0.375
    foot_glaze: float = 0.575
    pad_glaze: float = 0.25
    table: float = 0.45
    extra: dict = field(default_factory=dict)

    def attrs(self, kind):
        mu, prio = {"glaze": (self.glaze_glaze, 0), "foot": (self.foot_glaze, 1), "pad": (self.pad_glaze, 2),
                    "table": (self.table, 3)}[kind]
        return {"friction": f"{mu:.4g} 0.002 0.0001", "priority": str(prio)}


def _f(v):
    return " ".join(f"{x:.6g}" for x in v)


def _quat(R):
    """Rotation matrix -> quaternion (w x y z), without numpy.linalg."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]


def _rz(a):
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def plate_xml(spec: PlateSpec, name: str, pos, fr: Friction, rgba="0.95 0.95 0.93 1", contact="contype='1' conaffinity='1'"):
    """MJCF <body> string for one free plate. Geom names: {name}_floor, _wall{k}, _rim{k}, _edge{k}, _foot{k}."""
    s, n = spec, spec.n_seg
    dth = 2 * math.pi / n
    zc, ixx, izz = s.inertia()
    sol = "solref='0.004 1' solimp='0.95 0.99 0.001' condim='3'"
    gl, ft = fr.attrs("glaze"), fr.attrs("foot")
    g = []

    def geom(kind, gname, typ, size, p, R=None, fromto=None):
        a = gl if kind == "glaze" else ft
        col = rgba if kind == "glaze" else "0.80 0.74 0.62 1"
        place = f"fromto='{_f(fromto)}'" if fromto is not None else f"pos='{_f(p)}' quat='{_f(_quat(R))}'"
        g.append(f"<geom name='{name}_{gname}' type='{typ}' size='{_f(size)}' {place} friction='{a['friction']}' "
                 f"priority='{a['priority']}' rgba='{col}' {contact} {sol}/>")

    # well floor: a solid disc out to the foot ring; beyond it the two surfaces are carried by thin box rings
    th = s.floor_top - s.foot_h
    geom("glaze", "floor", "cylinder", [s.foot_out, th / 2], [0, 0, s.foot_h + th / 2], np.eye(3))

    def ring(label, p_in, p_out, thick, ext_in=0.0, below=True):
        """Boxes with one face on the segment p_in -> p_out (r, z): the top face if `below` (the box hangs under the
        segment, for the top surface), the bottom face otherwise (for the underside). ext_in lengthens the box inward
        along the slope, to bury its end inside the neighbouring piece."""
        slope = math.atan2(p_out[1] - p_in[1], p_out[0] - p_in[0])
        d = np.array([math.cos(slope), math.sin(slope)])
        a_in = np.array(p_in) - ext_in * d
        L = math.hypot(p_out[0] - a_in[0], p_out[1] - a_in[1])
        down = np.array([math.sin(slope), -math.cos(slope)])
        mid = (a_in + np.array(p_out)) / 2 + (down if below else -down) * thick / 2
        half_w = p_out[0] * math.tan(dth / 2) * 1.05
        Ry = np.array([[math.cos(-slope), 0, math.sin(-slope)], [0, 1, 0], [-math.sin(-slope), 0, math.cos(-slope)]])
        for k in range(n):
            a = k * dth
            geom("glaze", f"{label}{k}", "box", [L / 2, half_w, thick / 2],
                 [mid[0] * math.cos(a), mid[0] * math.sin(a), mid[1]], _rz(a) @ Ry)

    rk, zk = s.top_kink
    ce, re = s.edge_c, s.edge_r
    top = s.top_line()
    ring("flat", (s.foot_out - 0.003, s.floor_top), top[0], s.shell)            # flat floor beyond the foot ring
    for i_ in range(len(top) - 2):                                              # rounding, then the slope
        ring(f"top{i_}_", top[i_], top[i_ + 1], s.shell, ext_in=0.0005)
    ring("rim", top[-2], top[-1], 2 * re)                                       # the rim, full thickness
    ring("uwall", (s.foot_out, s.foot_h), (s.kink_r, s.kink_z), s.shell, ext_in=0.003, below=False)
    u_end = min(rk + 0.004, ce[0])
    ring("urim", (s.kink_r, s.kink_z), (u_end, s.z_under(u_end)), s.shell, ext_in=0.0005, below=False)

    def chain(kind, label, rad, rc, z):
        """Ring of capsules of radius `rad` whose axis lies on the circle of radius rc at height z."""
        for k in range(n):
            a0, a1 = (k - 0.5) * dth, (k + 0.5) * dth
            geom(kind, f"{label}{k}", "capsule", [rad], None,
                 fromto=[rc * math.cos(a0), rc * math.sin(a0), z, rc * math.cos(a1), rc * math.sin(a1), z])

    chain("glaze", "edge", re, ce[0], ce[1])
    rf = s.foot_h / 2
    chain("foot", "foot", rf, s.foot_out - rf, rf)
    chain("foot", "footin", rf, s.foot_out - s.foot_w + rf, rf)
    body = (f"<body name='{name}' pos='{_f(pos)}'>\n  <freejoint name='{name}_free'/>\n"
            f"  <inertial pos='0 0 {zc:.6g}' mass='{s.mass:.6g}' diaginertia='{ixx:.6g} {ixx:.6g} {izz:.6g}'/>\n  "
            + "\n  ".join(g) + f"\n  <site name='{name}_c' pos='0 0 {s.floor_top:.6g}' size='0.003' group='4'/>\n</body>")
    return body
