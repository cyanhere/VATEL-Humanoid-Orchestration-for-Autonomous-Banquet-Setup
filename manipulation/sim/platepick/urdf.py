"""Minimal URDF tools: parsing, mimic coupling, merging (R1 + Revo2) and writing."""
from __future__ import annotations

import copy
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


# ----------------------------------------------------------------------------- math

def rpy_to_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-9:
        p = np.arcsin(sp)
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        p = np.pi / 2 * np.sign(sp)
        r = 0.0
        y = np.arctan2(-R[0, 1], R[1, 1])
    return np.array([r, p, y])


def make_tf(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _vec(s: str | None, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    return np.array([float(v) for v in s.split()]) if s else np.array(default, dtype=float)


# ----------------------------------------------------------------------------- model

@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray                   # 4x4 parent->joint frame
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0
    mimic: tuple[str, float, float] | None = None   # (master joint, multiplier, offset)

    @property
    def movable(self) -> bool:
        return self.type in ("revolute", "continuous", "prismatic")


@dataclass
class Geometry:
    kind: str                            # mesh | box | cylinder | sphere
    origin: np.ndarray
    filename: str | None = None          # absolute path after resolve
    scale: np.ndarray | None = None
    size: np.ndarray | None = None


@dataclass
class Link:
    name: str
    visuals: list[Geometry] = field(default_factory=list)
    collisions: list[Geometry] = field(default_factory=list)


class URDF:
    def __init__(self, path: str, package_dirs: dict[str, str] | None = None):
        self.path = os.path.abspath(path)
        self.base_dir = os.path.dirname(self.path)
        self.package_dirs = package_dirs or {}
        self.tree = ET.parse(self.path)
        self.root = self.tree.getroot()
        self.name = self.root.get("name", "robot")
        self.links: dict[str, Link] = {}
        self.joints: dict[str, Joint] = {}
        self._parse()

    # -- parsing
    def resolve_mesh(self, filename: str) -> str:
        if filename.startswith("package://"):
            pkg, rel = filename[len("package://"):].split("/", 1)
            if pkg in self.package_dirs:
                return os.path.normpath(os.path.join(self.package_dirs[pkg], rel))
            # fall back: assume the URDF lives in <pkg>/urdf/
            guess = os.path.join(os.path.dirname(self.base_dir), rel)
            return os.path.normpath(guess)
        if filename.startswith("file://"):
            filename = filename[len("file://"):]
        return os.path.normpath(filename if os.path.isabs(filename) else os.path.join(self.base_dir, filename))

    def _geom(self, el) -> Geometry | None:
        o = el.find("origin")
        origin = make_tf(_vec(o.get("xyz") if o is not None else None), _vec(o.get("rpy") if o is not None else None))
        g = el.find("geometry")
        if g is None or len(g) == 0:
            return None
        s = g[0]
        if s.tag == "mesh":
            return Geometry("mesh", origin, filename=self.resolve_mesh(s.get("filename")),
                            scale=_vec(s.get("scale"), (1, 1, 1)))
        if s.tag == "box":
            return Geometry("box", origin, size=_vec(s.get("size")))
        if s.tag == "cylinder":
            return Geometry("cylinder", origin, size=np.array([float(s.get("radius")), float(s.get("length"))]))
        if s.tag == "sphere":
            return Geometry("sphere", origin, size=np.array([float(s.get("radius"))]))
        return None

    def _parse(self):
        for l in self.root.findall("link"):
            link = Link(l.get("name"))
            for v in l.findall("visual"):
                g = self._geom(v)
                if g:
                    link.visuals.append(g)
            for c in l.findall("collision"):
                g = self._geom(c)
                if g:
                    link.collisions.append(g)
            self.links[link.name] = link
        for j in self.root.findall("joint"):
            o = j.find("origin")
            origin = make_tf(_vec(o.get("xyz") if o is not None else None), _vec(o.get("rpy") if o is not None else None))
            ax = j.find("axis")
            lim = j.find("limit")
            mim = j.find("mimic")
            joint = Joint(
                name=j.get("name"), type=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                origin=origin,
                axis=_vec(ax.get("xyz")) if ax is not None else np.array([1.0, 0.0, 0.0]),
            )
            if lim is not None:
                joint.lower = float(lim.get("lower", 0))
                joint.upper = float(lim.get("upper", 0))
                joint.effort = float(lim.get("effort", 0))
                joint.velocity = float(lim.get("velocity", 0))
            if mim is not None:
                joint.mimic = (mim.get("joint"), float(mim.get("multiplier", 1)), float(mim.get("offset", 0)))
            n = np.linalg.norm(joint.axis)
            if n > 0:
                joint.axis = joint.axis / n
            self.joints[joint.name] = joint

    # -- queries
    @property
    def root_link(self) -> str:
        children = {j.child for j in self.joints.values()}
        roots = [l for l in self.links if l not in children]
        assert len(roots) == 1, f"expected one root link, got {roots}"
        return roots[0]

    def child_joints(self, link: str) -> list[Joint]:
        return [j for j in self.joints.values() if j.parent == link]

    def topo_joints(self) -> list[Joint]:
        """Joints in parent-before-child order."""
        out, stack = [], [self.root_link]
        while stack:
            l = stack.pop(0)
            for j in self.child_joints(l):
                out.append(j)
                stack.append(j.child)
        return out

    def movable_joints(self) -> list[Joint]:
        return [j for j in self.topo_joints() if j.movable]

    def actuated_joints(self) -> list[Joint]:
        """Movable joints that are not mimics (these are the policy / optimizer DoF)."""
        return [j for j in self.movable_joints() if j.mimic is None]

    def coupling_matrix(self, actuated: list[str] | None = None):
        """q_physical = C @ q_actuated + b, over ``movable_joints()`` order."""
        actuated = actuated or [j.name for j in self.actuated_joints()]
        movable = self.movable_joints()
        C = np.zeros((len(movable), len(actuated)))
        b = np.zeros(len(movable))
        for i, j in enumerate(movable):
            name, mult, off = j.name, 1.0, 0.0
            # follow mimic chains
            while self.joints[name].mimic is not None:
                m, k, o = self.joints[name].mimic
                off = off + mult * o
                mult = mult * k
                name = m
            if name in actuated:
                C[i, actuated.index(name)] = mult
                b[i] = off
        return C, b, [j.name for j in movable], actuated

    def link_world_transforms(self, q: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        """Numpy FK (for asset tools / tests). Mimic joints are filled from their masters."""
        q = dict(q or {})
        for j in self.movable_joints():
            if j.mimic is not None and j.name not in q:
                m, k, o = j.mimic
                q[j.name] = k * q.get(m, 0.0) + o
        T = {self.root_link: np.eye(4)}
        for j in self.topo_joints():
            M = T[j.parent] @ j.origin
            v = q.get(j.name, 0.0)
            if j.type in ("revolute", "continuous"):
                M = M @ _axis_angle_tf(j.axis, v)
            elif j.type == "prismatic":
                D = np.eye(4)
                D[:3, 3] = j.axis * v
                M = M @ D
            T[j.child] = M
        return T

    def write(self, path: str):
        ET.indent(self.tree)
        self.tree.write(path, encoding="utf-8", xml_declaration=True)


def _axis_angle_tf(axis, angle) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    return T


# ----------------------------------------------------------------------------- editing helpers

def _fmt(v) -> str:
    return " ".join(f"{float(x):.10g}" for x in v)


def absolutize_mesh_paths(urdf: URDF, relative_to: str | None = None):
    """Rewrite every mesh filename to an absolute path (or relative to ``relative_to``)."""
    for mesh in urdf.root.iter("mesh"):
        p = urdf.resolve_mesh(mesh.get("filename"))
        if relative_to:
            p = os.path.relpath(p, relative_to)
        mesh.set("filename", p.replace("\\", "/"))


def prefix_names(urdf: URDF, prefix: str):
    if not prefix:
        return
    for l in urdf.root.findall("link"):
        l.set("name", prefix + l.get("name"))
    for j in urdf.root.findall("joint"):
        j.set("name", prefix + j.get("name"))
        for tag in ("parent", "child"):
            e = j.find(tag)
            e.set("link", prefix + e.get("link"))
        m = j.find("mimic")
        if m is not None:
            m.set("joint", prefix + m.get("joint"))


def strip_link_mesh_beyond(urdf_path: str, out_urdf: str, link: str, axis: int, cut: float, mesh_out_dir: str,
                           package_dirs: dict | None = None) -> dict:
    """Remove the part of a link's meshes beyond `cut` along a link-frame axis and write a URDF that uses the trimmed meshes.

    Mass and COM are re-estimated from the kept surface-area fraction (the vendor STLs are open shells); inertia is
    scaled with mass. These are estimates.
    """
    import trimesh

    u = URDF(urdf_path, package_dirs)
    absolutize_mesh_paths(u, None)
    os.makedirs(mesh_out_dir, exist_ok=True)
    el = [l for l in u.root.findall("link") if l.get("name") == link][0]
    normal = np.zeros(3)
    normal[axis] = -1.0                        # keep the side below the cut
    origin = np.zeros(3)
    origin[axis] = cut
    info = {}
    for tag in ("visual", "collision"):
        for g in el.findall(tag):
            mesh_el = g.find("geometry/mesh")
            if mesh_el is None:
                continue
            src = u.resolve_mesh(mesh_el.get("filename"))
            m = trimesh.load(src, force="mesh")
            kept = trimesh.intersections.slice_mesh_plane(m, plane_normal=normal, plane_origin=origin, cap=False)
            dst = os.path.join(mesh_out_dir, os.path.splitext(os.path.basename(src))[0] + f"_nohand_cut{int(round(cut * 1000))}mm.stl")
            kept.export(dst)
            mesh_el.set("filename", os.path.abspath(dst).replace("\\", "/"))
            info = dict(src=src, dst=dst, area_fraction=float(kept.area / m.area),
                        kept_centroid=kept.area_weighted_centroid if hasattr(kept, "area_weighted_centroid") else
                        (kept.triangles_center * kept.area_faces[:, None]).sum(0) / kept.area)
    inert = el.find("inertial")
    if inert is not None and info:
        f = info["area_fraction"]
        mass_el = inert.find("mass")
        old_mass = float(mass_el.get("value"))
        mass_el.set("value", f"{old_mass * f:.6g}")
        inert.find("origin").set("xyz", _fmt(info["kept_centroid"]))
        I = inert.find("inertia")
        for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            I.set(k, f"{float(I.get(k)) * f:.6g}")
        info.update(old_mass=old_mass, new_mass=old_mass * f)
    u.write(out_urdf)
    return info


def merge_urdf(base_path: str, attach_path: str, parent_link: str, mount_xyz, mount_rpy,
               out_path: str, robot_name: str, attach_prefix: str = "",
               base_packages: dict | None = None, attach_packages: dict | None = None,
               drop_links: list[str] | None = None) -> URDF:
    """Attach the root link of `attach_path` to `parent_link` of `base_path` with a fixed joint.
    Mesh paths are rewritten relative to `out_path`.
    """
    out_dir = os.path.dirname(os.path.abspath(out_path))
    base = URDF(base_path, base_packages)
    att = URDF(attach_path, attach_packages)
    absolutize_mesh_paths(base, out_dir)
    absolutize_mesh_paths(att, out_dir)
    att_root = att.root_link
    prefix_names(att, attach_prefix)
    att_root = attach_prefix + att_root

    root = copy.deepcopy(base.root)
    root.set("name", robot_name)
    # drop links (and joints that reference them), e.g. stock end-effector stubs
    for name in drop_links or []:
        for l in list(root.findall("link")):
            if l.get("name") == name:
                root.remove(l)
        for j in list(root.findall("joint")):
            if j.find("child").get("link") == name:
                root.remove(j)
    link_names = {l.get("name") for l in root.findall("link")}
    assert parent_link in link_names, f"{parent_link} not in base URDF"

    # materials must be unique by name at top level
    have_mats = {m.get("name") for m in root.findall("material")}
    for el in att.root:
        if el.tag == "material" and el.get("name") in have_mats:
            continue
        if el.tag in ("link", "joint", "material"):
            root.append(copy.deepcopy(el))
    mount = ET.SubElement(root, "joint", {"name": f"{att_root}_mount_joint", "type": "fixed"})
    ET.SubElement(mount, "origin", {"xyz": _fmt(mount_xyz), "rpy": _fmt(mount_rpy)})
    ET.SubElement(mount, "parent", {"link": parent_link})
    ET.SubElement(mount, "child", {"link": att_root})

    tree = ET.ElementTree(root)
    ET.indent(tree)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return URDF(out_path)


def lock_joints(path_in: str, path_out: str, keep_movable: list[str], locked_values: dict[str, float] | None = None):
    """Turn every movable joint not in `keep_movable` into a fixed joint (mimics of kept joints stay).
    `locked_values`: angle (rad) or offset (m) each joint is frozen at, baked into the fixed joint origin (default 0).
    """
    u = URDF(path_in)
    keep = set(keep_movable)
    locked_values = locked_values or {}
    for j in u.root.findall("joint"):
        name = j.get("name")
        if j.get("type") == "fixed":
            continue
        m = j.find("mimic")
        master = m.get("joint") if m is not None else None
        if name in keep or (master in keep):
            continue
        v = locked_values.get(name)
        if v is None and master is not None and master in locked_values:
            v = float(m.get("multiplier", 1)) * locked_values[master] + float(m.get("offset", 0))
        if v:
            jj = u.joints[name]
            if jj.type in ("revolute", "continuous"):
                M = jj.origin @ _axis_angle_tf(jj.axis, v)
            else:
                D = np.eye(4)
                D[:3, 3] = jj.axis * v
                M = jj.origin @ D
            o = j.find("origin")
            if o is None:
                o = ET.SubElement(j, "origin")
            o.set("xyz", _fmt(M[:3, 3]))
            o.set("rpy", _fmt(matrix_to_rpy(M[:3, :3])))
        j.set("type", "fixed")
        for tag in ("limit", "mimic", "dynamics", "axis"):
            e = j.find(tag)
            if e is not None:
                j.remove(e)
    u.write(path_out)
    return URDF(path_out)


def add_floating_wrist(hand_path: str, out_path: str, robot_name: str, trans_limit: float = 2.0):
    """Hand-only variant with 6 virtual joints (x, y, z prismatic; roll, pitch, yaw revolute)
    from a ``world`` link, so the wrist can be driven directly as joint targets in sim."""
    out_dir = os.path.dirname(os.path.abspath(out_path))
    u = URDF(hand_path)
    absolutize_mesh_paths(u, out_dir)
    hand_root = u.root_link
    root = copy.deepcopy(u.root)
    root.set("name", robot_name)
    specs = [("x", "prismatic", "1 0 0"), ("y", "prismatic", "0 1 0"), ("z", "prismatic", "0 0 1"),
             ("roll", "revolute", "1 0 0"), ("pitch", "revolute", "0 1 0"), ("yaw", "revolute", "0 0 1")]
    new = [ET.Element("link", {"name": "world"})]
    prev = "world"
    for i, (n, t, ax) in enumerate(specs):
        child = hand_root if i == len(specs) - 1 else f"virtual_{n}_link"
        if child != hand_root:
            l = ET.Element("link", {"name": child})
            inert = ET.SubElement(l, "inertial")
            ET.SubElement(inert, "mass", {"value": "0.01"})
            ET.SubElement(inert, "inertia", {"ixx": "1e-6", "iyy": "1e-6", "izz": "1e-6", "ixy": "0", "ixz": "0", "iyz": "0"})
            new.append(l)
        j = ET.Element("joint", {"name": f"virtual_{n}_joint", "type": t})
        ET.SubElement(j, "parent", {"link": prev})
        ET.SubElement(j, "child", {"link": child})
        ET.SubElement(j, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(j, "axis", {"xyz": ax})
        lim = (trans_limit if t == "prismatic" else 2 * np.pi)   # ±2π: interpolated Euler angles may unwrap past ±π
        ET.SubElement(j, "limit", {"lower": f"{-lim}", "upper": f"{lim}", "effort": "500", "velocity": "5"})
        new.append(j)
        prev = child
    for i, el in enumerate(new):
        root.insert(i, el)
    tree = ET.ElementTree(root)
    ET.indent(tree)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return URDF(out_path)
