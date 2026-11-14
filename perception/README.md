# Perception (Step 4)

Finds the plate stack with the R1's head camera (RealSense D435if) and reports the top plate's
position in the robot frame, so the robot can pick one plate off the stack.

- `check_stack.py` - detect the plate stack and write its position (`stack.json`)
- `calibrate_sweep.py`, `check_mount.py` - calibrate the head camera
- `reach_plate.py` - reach-and-grasp test to verify the result on the robot
- `run_tests.py` - offline tests
