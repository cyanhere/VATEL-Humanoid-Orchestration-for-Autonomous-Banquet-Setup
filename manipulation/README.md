# Manipulation (Step 4)

Takes one plate at a time off the AGV stack with the BrainCo Revo 2 hands: the left hand pushes the
top plate sideways with a hooked middle finger until it overhangs, the right hand grips the
overhanging rim, then both hands lift, carry and place it.

- `robot/` - runs on the R1 (Python 3.8, numpy). Copy it to the robot as `~/project/grasp/pick`.
- `sim/` - MuJoCo models and stage controllers.
- `models/` - fitted arm and hand models shared by both.
- `robot/calibration/` - fitting scripts for the head camera, joint offsets and arm/hand models.

## Run

Robot, from `robot/`, one terminal each:

```bash
./server.sh                  # hand + arm server; type take, then home
python3 perception.py        # head camera: stack and rim tracking
python3 run_test.py push10   # or push10_grip, lone_grip
```

`python3 monitor.py f` freezes the arms; `./stop.sh` stops everything.

Simulation, from `sim/` (MuJoCo 3.13+, numpy, scipy, opencv):

```bash
python scripts/push10_grip_sim.py --set height=plane,tap=1 -o data/results/example --frames
```

Offline tests (no robot): `python3 robot/tests/test_stages_mock.py`.
