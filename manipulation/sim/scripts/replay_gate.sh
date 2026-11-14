#!/bin/bash
# MuJoCo replay of held-out R1 random-motion runs, open loop, driven by what the motors got (`sent`).
# Five arm models per side: A fitted (gravity + friction + delay + torque scale from the bag),
# B as A with the torque scale forced to 1 (the bag's scale is not trusted), E as B with friction WITHOUT memory (the steady curve only),
# C fitted gravity without friction, D the URDF without friction.
# --fric 0: no legacy joint friction under the models (C and D have none). First 95 s of each run (motion 0 at 0.3 and motion 1 at 0.6 of the speed limit).
cd "$(dirname "$0")/.." || exit 1   # run from sim/
PY=python
M=../models; NF=data/results/replay_arm
L=../data/robot_pick_logs/logs/tests
mkdir -p data/results/replay_arm
$PY - <<'EOF'
import json, copy
for side in ("left", "right"):
    F = json.load(open("../models/sim_arm_model_%s.json" % side))
    B = copy.deepcopy(F); B["gravity"]["torque_scale"] = 1.0
    C = copy.deepcopy(B)
    for f in C["friction"]["joints"]:
        f["model"] = None
    D = copy.deepcopy(C); D["gravity"]["chosen"] = "urdf"
    E = copy.deepcopy(B)
    for f in E["friction"]["joints"]:
        if f.get("model") == "L":
            f["model"] = "S"
    for k, v in (("B", B), ("C", C), ("D", D), ("E", E)):
        json.dump(v, open("data/results/replay_arm/gate_%s_%s.json" % (side, k), "w"))
EOF
run() {   # side run model tag
  $PY scripts/replay_real.py $L/$2/armwide_cmd.csv --no_plate --drive sent --side $1 --arm_model $3 --fric 0 --t_end 95 \
      -o data/results/replay_arm/gate_$1_$4.npz > data/results/replay_arm/gate_$1_$4.log 2>&1
  echo "done $1 $4 exit $?"
}
run left 20261105_135848_armwide_motion $M/sim_arm_model_left.json A &
run left 20261105_135848_armwide_motion $NF/gate_left_B.json B &
run left 20261105_135848_armwide_motion $NF/gate_left_C.json C &
run left 20261105_135848_armwide_motion $NF/gate_left_D.json D &
run left 20261105_135848_armwide_motion $NF/gate_left_E.json E &
run right 20261105_151004_armwide_motion $M/sim_arm_model_right.json A &
run right 20261105_151004_armwide_motion $NF/gate_right_B.json B &
run right 20261105_151004_armwide_motion $NF/gate_right_C.json C &
run right 20261105_151004_armwide_motion $NF/gate_right_D.json D &
run right 20261105_151004_armwide_motion $NF/gate_right_E.json E &
wait
echo GATE_DONE
