"""Settle the plates of a new RobotScene without the robot touching them."""
import mujoco


def settle_plates(m, d, steps=1500):
    robot = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(("left_", "right_"))]
    keep = [(g, int(m.geom_contype[g]), int(m.geom_conaffinity[g])) for g in robot]
    for g in robot:
        m.geom_contype[g] = 0
        m.geom_conaffinity[g] = 0
    for _ in range(steps):
        mujoco.mj_step(m, d)
    for g, ct, ca in keep:
        m.geom_contype[g] = ct
        m.geom_conaffinity[g] = ca
    mujoco.mj_forward(m, d)
