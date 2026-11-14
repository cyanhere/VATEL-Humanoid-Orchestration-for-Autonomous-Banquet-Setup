"""Task plate geometry, the same as the simulation's platepick/plate.py. Plate frame: origin at the bottom centre of
the foot ring, z up.
"""
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
    R: float = 0.1265               # outer radius: diameter 253 mm (measured)
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

