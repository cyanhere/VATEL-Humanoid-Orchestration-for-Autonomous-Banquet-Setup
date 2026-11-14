"""Actuator command log: with DEXPICK_CTRL_LOG=FILE.json set, the actuator targets, joint angles and plate poses are
recorded at 30 Hz of simulated time and written when the script exits.
"""
import atexit
import json
import os

import mujoco

DT = 1 / 30


def install(path):
    logs = {}
    real_step = mujoco.mj_step

    def step(m, d, *a, **k):
        real_step(m, d, *a, **k)
        L = logs.get(id(d))
        if L is None:
            hinge = [j for j in range(m.njnt) if m.jnt_type[j] in (2, 3)]
            free = [j for j in range(m.njnt) if m.jnt_type[j] == 0]
            L = logs[id(d)] = dict(m=m, d=d, next=d.time, steps=0, hinge=hinge, free=free, samples=[],
                                   actuators=[m.actuator(i).name for i in range(m.nu)],
                                   joints=[m.joint(j).name for j in hinge], bodies=[m.joint(j).name for j in free])
        L["steps"] += 1
        if d.time >= L["next"] - 1e-9:
            L["next"] += DT
            L["samples"].append(dict(t=round(float(d.time), 4), ctrl=[round(float(v), 5) for v in d.ctrl],
                                     q=[round(float(d.qpos[m.jnt_qposadr[j]]), 5) for j in L["hinge"]],
                                     plates=[[round(float(v), 5) for v in d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 7]] for j in L["free"]]))

    def write():
        if not logs:
            return
        L = max(logs.values(), key=lambda v: v["steps"])
        json.dump(dict(dt=DT, actuators=L["actuators"], joints=L["joints"], plates=L["bodies"], samples=L["samples"],
                       timestep=float(L["m"].opt.timestep), arm_kp=os.environ.get("DEXPICK_KP_ARM", "stiff 200"),
                       arm_kv=os.environ.get("DEXPICK_KV_ARM", "stiff 20")), open(path, "w"))

    mujoco.mj_step = step
    atexit.register(write)
