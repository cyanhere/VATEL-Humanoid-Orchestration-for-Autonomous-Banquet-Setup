"""Robot scene: the R1 upper body (both 5-DoF arms, both Revo2 hands, waist yaw), a table and a plate stack."""
import json
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

from .plate import Friction, PlateSpec, TASK, _f, _quat, plate_xml
from .urdf import URDF, rpy_to_matrix

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KP_ARM, KV_ARM, F_ARM = 200.0, 20.0, 60.0
KP_HAND, KV_HAND, F_HAND = 20.0, 0.5, 2.0
# the real arm_sdk example uses kp 50/50/40/40/30, kd 2 (shoulder pitch.. wrist). DEXPICK_KP_ARM / DEXPICK_KV_ARM
# (one number, or five comma-separated per joint) override the arm gains to test the scripted motions at real stiffness.
def _arm_gains(var, default):
    v = os.environ.get(var, "")
    if not v:
        return [default] * 5
    x = [float(t) for t in v.split(",")]
    return x * 5 if len(x) == 1 else x
ARM_WORDS = ("shoulder", "elbow_link", "wrist", "base_link", "thumb", "index", "middle", "ring", "pinky")


def _pose(T):
    return {"pos": _f(T[:3, 3]), "quat": _f(_quat(T[:3, :3]))}


@dataclass
class RobotScene:
    spec: PlateSpec = field(default_factory=lambda: TASK)
    n: int = 3                      # plates in the stack
    table: float = 0.762            # table height (0.747 test table, 0.762 final)
    pelvis: float = 0.76            # pelvis height (legacy scenes: 0.75-0.76; the real standing height is unverified)
    xy: tuple = (0.30, 0.0)         # stack centre in front of the robot
    fr: Friction = None
    timestep: float = 0.001
    edge_gap: float = 0.015         # table front edge this far in front of the stack's rim
    table_right: float = 0.0        # > 0: the table's right edge (robot's right, -y) this far from the stack centre (m)
    wrist_cams: dict = None         # {side: dict(body, name, pos, quat, fovy)} from platepick.wrist_cam_sim.scene_cams()

    def build(self):
        """Returns (xml string, info dict)."""
        fr = self.fr or Friction()
        info = json.load(open(os.path.join(ROOT, "assets", "robots", "robot_info.json")))
        both = info["r1_revo2_upper_both"]
        urdf = URDF(os.path.join(ROOT, "assets", "robots", both["file"]))
        pd, tb = fr.attrs("pad"), fr.attrs("table")
        mj = ET.Element("mujoco", model="r1_plate_push")
        ET.SubElement(mj, "compiler", angle="radian", autolimits="true", balanceinertia="true")
        ET.SubElement(mj, "option", timestep=str(self.timestep), integrator="implicitfast", cone="elliptic", impratio="10")
        vis = ET.SubElement(mj, "visual")
        ET.SubElement(vis, "global", offwidth="1280", offheight="720")
        ET.SubElement(vis, "headlight", ambient="0.45 0.45 0.45", diffuse="0.6 0.6 0.6")
        asset = ET.SubElement(mj, "asset")
        world = ET.SubElement(mj, "worldbody")
        ET.SubElement(world, "light", pos="0.5 -0.5 2.5", dir="-0.2 0.2 -1", diffuse="0.8 0.8 0.8")
        ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", rgba="0.25 0.27 0.3 1", contype="1",
                      conaffinity="3")
        s, th = self.spec, self.table
        edge = self.xy[0] - s.R - self.edge_gap
        y_lo, y_hi = self.xy[1] - (self.table_right if self.table_right > 0 else 0.9), self.xy[1] + 0.9
        ET.SubElement(world, "geom", name="table", type="box", size=_f([0.38, (y_hi - y_lo) / 2, th / 2]),
                      pos=_f([edge + 0.38, (y_hi + y_lo) / 2, th / 2]), rgba="0.62 0.45 0.32 1", contype="1", conaffinity="3",
                      friction=tb["friction"], priority=tb["priority"], condim="3", solref="0.004 1",
                      solimp="0.95 0.99 0.001")
        self._robot(world, asset, urdf, both["root_link"], [0, 0, self.pelvis], pd)
        if self.wrist_cams:                     # cameras fixed to the hand base links (the R1's wrist cameras)
            bodies = {b.get("name"): b for b in world.iter("body")}
            for wc in self.wrist_cams.values():
                ET.SubElement(bodies[wc["body"]], "camera", name=wc["name"], pos=_f(wc["pos"]), quat=_f(wc["quat"]),
                              fovy="%.4f" % wc["fovy"])
        world.append(ET.Comment("PLATES"))
        cam = [self.xy[0], self.xy[1], th]
        ET.SubElement(world, "camera", name="front", pos=_f([cam[0] + 0.75, cam[1] + 0.05, cam[2] + 0.30]),
                      xyaxes="0 1 0 -0.35 0 1", fovy="38")
        ET.SubElement(world, "camera", name="left", pos=_f([cam[0] + 0.05, cam[1] + 0.55, cam[2] + 0.16]),
                      xyaxes="-1 0 0 0 -0.25 1", fovy="36")
        ztop = th + (self.n - 1) * s.floor_top + s.H           # close-up of the left rim edge of the top plate, from the left
        ET.SubElement(world, "camera", name="close", pos=_f([cam[0], cam[1] + s.R + 0.30, ztop + 0.035]),
                      xyaxes="-1 0 0 0 -0.15 1", fovy="13")
        ET.SubElement(world, "camera", name="closefront", pos=_f([cam[0] + 0.34, cam[1] + s.R - 0.01, ztop + 0.03]),
                      xyaxes="0 1 0 -0.12 0 1", fovy="13")
        ET.SubElement(world, "camera", name="closeright", pos=_f([cam[0] - 0.05, cam[1] - s.R - 0.45, ztop + 0.10]),
                      xyaxes="1 0 0 0 0.2 1", fovy="30")
        for nm in ("grip", "gripside"):          # placeholders; scripts aim them at a grip point at run time
            ET.SubElement(world, "camera", name=nm, pos=_f([cam[0], cam[1] - 0.4, ztop + 0.1]), xyaxes="1 0 0 0 0 1", fovy="32")
        ET.SubElement(world, "camera", name="top", pos=_f([cam[0], cam[1], cam[2] + 0.75]), xyaxes="0 1 0 -1 0 0", fovy="40")
        con = ET.SubElement(mj, "contact")
        for j in urdf.joints.values():          # shoulder housings sit inside the torso shell
            if "shoulder_pitch" in j.child:
                ET.SubElement(con, "exclude", body1=j.parent, body2=j.child)
        act = ET.SubElement(mj, "actuator")
        for side in ("left", "right"):
            kps, kvs = _arm_gains("DEXPICK_KP_ARM", KP_ARM), _arm_gains("DEXPICK_KV_ARM", KV_ARM)
            for ia, jn in enumerate(both[side]["arm_joints"]):
                ET.SubElement(act, "position", name=f"a_{jn}", joint=jn, kp=str(kps[ia]), kv=str(kvs[ia]),
                              forcerange=_f([-F_ARM, F_ARM]), ctrlrange=_f([urdf.joints[jn].lower, urdf.joints[jn].upper]))
            for jn in info[f"revo2_{side}_hand"]["movable_joints"]:
                ET.SubElement(act, "position", name=f"a_{jn}", joint=jn, kp=str(KP_HAND), kv=str(KV_HAND),
                              forcerange=_f([-F_HAND, F_HAND]), ctrlrange=_f([urdf.joints[jn].lower, urdf.joints[jn].upper]))
        ET.SubElement(act, "position", name="a_waist_yaw_joint", joint="waist_yaw_joint", kp="3000", kv="150",
                      forcerange="-60 60", ctrlrange="-2.618 2.618")
        ET.indent(mj)
        xml = ET.tostring(mj, encoding="unicode")
        plates = "\n".join(plate_xml(s, f"plate{k}", [self.xy[0], self.xy[1], th + 0.0003 + k * (s.floor_top + 0.0006)], fr,
                                     contact="contype='1' conaffinity='7'") for k in range(self.n))
        xml = xml.replace("<!--PLATES-->", plates)
        meta = dict(arm={sd: both[sd]["arm_joints"] for sd in ("left", "right")},
                    hand={sd: info[f"revo2_{sd}_hand"]["movable_joints"] for sd in ("left", "right")},
                    hand_act={sd: info[f"revo2_{sd}_hand"]["actuated_joints"] for sd in ("left", "right")},
                    coupling={sd: info[f"revo2_{sd}_hand"]["coupling"] for sd in ("left", "right")},
                    ready={sd: both[sd]["ready_pose"] for sd in ("left", "right")},
                    table=th, xy=list(self.xy), n=self.n, pelvis=self.pelvis)
        return xml, meta

    @staticmethod
    def _robot(world, asset, urdf, root, base_pos, pd):
        raw = {l.get("name"): l for l in urdf.root.findall("link")}
        children = {}
        for j in urdf.joints.values():
            children.setdefault(j.parent, []).append(j)
        meshes = {}

        def mesh_name(g):
            key = (g.filename, tuple(np.round(g.scale, 6)) if g.scale is not None else (1, 1, 1))
            if key not in meshes:
                name = f"m{len(meshes)}_{os.path.splitext(os.path.basename(g.filename))[0]}"
                meshes[key] = name
                ET.SubElement(asset, "mesh", name=name, file=os.path.abspath(g.filename).replace("\\", "/"), scale=_f(key[1]))
            return meshes[key]

        def add_geoms(body_el, link):
            for g in link.visuals:
                if g.kind == "mesh":
                    ET.SubElement(body_el, "geom", type="mesh", mesh=mesh_name(g), contype="0", conaffinity="0", group="1",
                                  density="0", rgba="0.85 0.85 0.87 1", **_pose(g.origin))
            # collision bits: 1 environment, 2 left arm, 4 robot body, 8 right arm. Arms collide with the environment,
            # the body and each other, not with themselves; the body collides with nothing but the arms and plates.
            arm = any(k in link.name for k in ARM_WORDS) and link.name.startswith(("left_", "right_"))
            body = not arm and "imu" not in link.name
            ct, ca = (("2", "13") if link.name.startswith("left_") else ("8", "7")) if arm else ("4", "0")
            for g in (link.collisions if (arm or body) else []):
                a = dict(_pose(g.origin), contype=ct, conaffinity=ca, group="3", rgba="0.4 0.6 0.9 0.5", density="0",
                         friction=pd["friction"], priority=pd["priority"], condim="3", solref="0.004 1",
                         solimp="0.95 0.99 0.001", name=f"{link.name}_c{len(body_el)}")
                if g.kind == "mesh":
                    ET.SubElement(body_el, "geom", type="mesh", mesh=mesh_name(g), **a)
                elif g.kind == "box":
                    ET.SubElement(body_el, "geom", type="box", size=_f(np.asarray(g.size) / 2), **a)
                elif g.kind == "cylinder":
                    ET.SubElement(body_el, "geom", type="cylinder", size=_f([g.size[0], g.size[1] / 2]), **a)
                elif g.kind == "sphere":
                    ET.SubElement(body_el, "geom", type="sphere", size=_f(g.size), **a)
            il = raw[link.name].find("inertial")
            if il is not None and il.find("mass") is not None and float(il.find("mass").get("value")) > 0:
                o = il.find("origin")
                xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
                R = rpy_to_matrix(rpy)
                i = il.find("inertia")
                I = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                              [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                              [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
                I = R @ I @ R.T + np.eye(3) * 1e-7
                ET.SubElement(body_el, "inertial", pos=_f(xyz), mass=_f([float(il.find("mass").get("value"))]),
                              fullinertia=_f([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
            else:
                ET.SubElement(body_el, "inertial", pos="0 0 0", mass="0.001", diaginertia="1e-7 1e-7 1e-7")

        def build(parent_el, link_name):
            for j in children.get(link_name, []):
                b = ET.SubElement(parent_el, "body", name=j.child, gravcomp="1", **_pose(j.origin))
                if j.movable:
                    ET.SubElement(b, "joint", name=j.name, type="hinge" if j.type != "prismatic" else "slide",
                                  axis=_f(j.axis), range=_f([j.lower, j.upper]), limited="true", damping="0.05",
                                  armature="0.005")
                elif j.name == "waist_yaw_joint":               # fixed in the upper-body URDF; +-150 deg on the R1
                    ET.SubElement(b, "joint", name=j.name, type="hinge", axis="0 0 1", range="-2.618 2.618",
                                  limited="true", damping="2.0", armature="0.05")
                add_geoms(b, urdf.links[j.child])
                build(b, j.child)

        rb = ET.SubElement(world, "body", name=root, pos=_f(base_pos), gravcomp="1")
        add_geoms(rb, urdf.links[root])
        build(rb, root)
