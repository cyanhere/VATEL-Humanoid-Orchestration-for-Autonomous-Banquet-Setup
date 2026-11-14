"""Stage 1 scene: a table, a plate stack and one free fingertip (no robot)."""
import math
from dataclasses import dataclass

from .plate import Friction, PlateSpec, plate_xml

TIP_R = 0.007                # fingertip radius (Revo2 fingertip is about 14 mm thick)
PAD = (0.020, 0.007, 0.005)   # flat pad half sizes: along the finger, across the pad (14 mm wide), thickness
KP, KV, FMAX = 1500.0, 40.0, 20.0     # N/m, N s/m, N: stiffness of a finger on the arm, roughly


@dataclass
class ProbeScene:
    spec: PlateSpec
    n: int = 3                   # plates in the stack
    fr: Friction = None
    timestep: float = 0.001
    kz: float = KP                # vertical stiffness of the fingertip, N/m
    pad_deg: float = None         # None: round fingertip. A number: a flat finger pad whose face looks toward the
                                  # plate and this many degrees downward (finger lying along the rim, hand rolled)

    def xml(self):
        fr = self.fr or Friction()
        s = self.spec
        tb, pd = fr.attrs("table"), fr.attrs("pad")
        plates = "\n".join(plate_xml(s, f"plate{k}", [0, 0, 0.0003 + k * (s.floor_top + 0.0006)], fr)
                           for k in range(self.n))
        lean = math.radians(40)                               # the finger body leans outward, as a real finger would
        ax = (0, math.sin(lean) * 0.03, math.cos(lean) * 0.03)
        tip_geom = f'<geom name="tip" type="capsule" size="{TIP_R}" fromto="0 0 0 {ax[0]} {ax[1]:.5f} {ax[2]:.5f}"'
        if self.pad_deg is not None:                          # body origin = centre of the pad face
            g = math.radians(self.pad_deg)
            phi = g - math.pi / 2
            c = (0, math.cos(g) * PAD[2], math.sin(g) * PAD[2])
            tip_geom = (f'<geom name="tip" type="box" size="{PAD[0]} {PAD[1]} {PAD[2]}" pos="0 {c[1]:.5f} {c[2]:.5f}" '
                        f'quat="{math.cos(phi / 2):.6f} {math.sin(phi / 2):.6f} 0 0"')
        return f"""<mujoco model="push_probe">
<compiler angle="radian" autolimits="true"/>
<option timestep="{self.timestep}" integrator="implicitfast" cone="elliptic" impratio="10"/>
<visual><global offwidth="1280" offheight="720"/><headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/>
<quality shadowsize="2048"/></visual>
<worldbody>
<light pos="0.3 -0.4 1.0" dir="-0.2 0.3 -1" diffuse="0.7 0.7 0.7"/>
<geom name="table" type="box" size="0.4 0.4 0.02" pos="0 0 -0.02" rgba="0.62 0.45 0.32 1" friction="{tb['friction']}"
      priority="{tb['priority']}" condim="3" solref="0.004 1" solimp="0.95 0.99 0.001"/>
{plates}
<body name="tip" pos="0 0.25 0.15" gravcomp="1">
  <joint name="tip_x" type="slide" axis="1 0 0" damping="1"/>
  <joint name="tip_y" type="slide" axis="0 1 0" damping="1"/>
  <joint name="tip_z" type="slide" axis="0 0 1" damping="1"/>
  <inertial pos="0 0 0" mass="0.15" diaginertia="1e-4 1e-4 1e-4"/>
  {tip_geom} rgba="0.85 0.33 0.31 1"
        friction="{pd['friction']}" priority="{pd['priority']}" condim="3" solref="0.004 1" solimp="0.95 0.99 0.001"/>
</body>
<camera name="side" pos="0.42 -0.02 0.07" xyaxes="0 1 0 -0.10 0 1" fovy="30"/>
<camera name="lead" pos="0.20 -0.33 0.10" xyaxes="0.8 0.6 0 -0.13 0.17 1" fovy="34"/>
</worldbody>
<actuator>
  <position name="a_x" joint="tip_x" kp="{KP}" kv="{KV}" forcerange="-{FMAX} {FMAX}"/>
  <position name="a_y" joint="tip_y" kp="{KP}" kv="{KV}" forcerange="-{FMAX} {FMAX}"/>
  <position name="a_z" joint="tip_z" kp="{self.kz}" kv="{KV}" forcerange="-{FMAX} {FMAX}"/>
</actuator>
</mujoco>"""
