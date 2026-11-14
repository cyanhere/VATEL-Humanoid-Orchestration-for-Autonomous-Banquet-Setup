#!/usr/bin/env bash
# Real-time pose tracking: Livox driver (CustomMsg) + FAST-LIO2.
#   ~/.scuba/lio_ws/run_lio.sh
# Publishes: /Odometry (nav_msgs/Odometry), /path, /cloud_registered
#
# NOT -u: ROS setup.bash references unset vars and dies under nounset.
set -o pipefail

# Domain 0 by default: the control stack (/api/sport/request, nav_bridge) lives
# there, and MPPI has to see /Odometry. livox_port/env.sh defaults to 99 to keep
# the point cloud off the control domain - that isolation is deliberate, but it
# also hides /Odometry from the navigation stack, so we override it here.
export LIO_DOMAIN=${LIO_DOMAIN:-0}

source /home/unitree/.scuba/livox_port/env.sh > /tmp/lio_env.log 2>&1
source /home/unitree/.scuba/lio_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=$LIO_DOMAIN

# Domain 0 carries the robot's Unitree CycloneDDS traffic. Foxy's default RMW is
# FastRTPS, which tries to parse it and dies with "bad_alloc caught". Speak
# CycloneDDS instead - same as sdk_teleop/env.sh.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null   # read-only
# unitree_sdk2py's bindings need libddsc 0.10.2 (ddsi_sertype_v0); Foxy ships
# 0.7.0. Force 0.10.2 to the front regardless of what earlier sourcing did.
CDDS_LIB=/home/unitree/cyclonedds_ws/install/cyclonedds/lib
[ -d "$CDDS_LIB" ] && export LD_LIBRARY_PATH="$CDDS_LIB:${LD_LIBRARY_PATH:-}"

CFG=/home/unitree/.scuba/livox_port/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
PARAMS=/home/unitree/.scuba/lio_ws/src/fast_lio/config/mid360_g1.yaml

for f in "$CFG" "$PARAMS"; do
  [ -f "$f" ] || { echo "!! missing: $f"; exit 1; }
done
command -v ros2 >/dev/null || { echo "!! ros2 not on PATH - see /tmp/lio_env.log"; exit 1; }

echo ">>> env"
echo "    ROS_DOMAIN_ID = $ROS_DOMAIN_ID   (must match nav_bridge / MPPI)"
echo "    python3       = $(command -v python3)"
echo "    rmw           = $RMW_IMPLEMENTATION"

CALIB=/home/unitree/.scuba/lio_ws/mount_calib.json
MOUNT=/home/unitree/.scuba/lio_ws/mount_calib.py
DRV=""; LIO=""; TFP=""
cleanup(){ [ -n "$TFP" ] && kill $TFP 2>/dev/null; [ -n "$LIO" ] && kill $LIO 2>/dev/null;
           [ -n "$DRV" ] && kill $DRV 2>/dev/null; sleep 1; }
trap cleanup EXIT INT TERM

echo ">>> livox driver (xfer_format=1 / CustomMsg - FAST-LIO needs this)"
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
  -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:="$CFG" \
  > /tmp/lio_driver.log 2>&1 & DRV=$!
for i in $(seq 1 25); do grep -q "publish use" /tmp/lio_driver.log 2>/dev/null && break; sleep 1; done
if ! grep -q "publish use" /tmp/lio_driver.log 2>/dev/null; then
  echo "!! driver did not start:"; tail -8 /tmp/lio_driver.log; exit 1
fi
grep -E "successfully change work mode|publish use" /tmp/lio_driver.log | sed 's/^/    /'

# The mount angle is MEASURED every startup from gravity, not hardcoded: the
# bracket gets adjusted, and every consumer (relocalizer seed, path viewer,
# base_link below) should follow the mount rather than a stale constant.
echo ">>> mount calibration (gravity from the lidar IMU)"
/usr/bin/python3 "$MOUNT" --save "$CALIB" 2>&1 | sed 's/^/    /'
CAL_RC=${PIPESTATUS[0]}
case "$CAL_RC" in
  2) echo "    !! the robot was MOVING - that angle is unreliable, restart when still" ;;
  1) echo "    !! no IMU reading - base_link will not be published this run" ;;
esac

echo ">>> FAST-LIO2"
echo "    KEEP THE ROBOT STILL until the first /Odometry."
echo "    Gravity is estimated at init; gait accel (0.49 g vs 0.05 g static)"
echo "    corrupts it - that produced the 0.99 deg tilt in ilounge_02."
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$PARAMS" \
  > /tmp/lio_fastlio.log 2>&1 & LIO=$!
sleep 8
if ! kill -0 $LIO 2>/dev/null; then
  echo "!! fastlio exited:"; tail -15 /tmp/lio_fastlio.log; exit 1
fi
tail -6 /tmp/lio_fastlio.log | sed 's/^/    /'

if [ -f "$CALIB" ]; then
  echo ">>> static body -> base_link, levelled from the measured mount"
  /usr/bin/python3 "$MOUNT" --publish --from-file "$CALIB" > /tmp/lio_mount_tf.log 2>&1 & TFP=$!
  sleep 2
  kill -0 $TFP 2>/dev/null && sed -n '1,2p' /tmp/lio_mount_tf.log | sed 's/^/    /' \
    || { echo "    !! base_link publisher exited:"; tail -3 /tmp/lio_mount_tf.log | sed 's/^/    /'; }
fi

echo
# NOT `ros2 topic hz`: measured 2026-10-29 at load 16, it printed "NO MESSAGES
# in 12s" for topics publishing at 9-26 Hz, because its own process was not
# scheduled often enough to see its own subscription. A startup check that
# lies is worse than no check.
echo ">>> checking /Odometry ..."
/usr/bin/python3 "$HOME/.scuba/nav/waitfor.py" --topic /Odometry --count 5 --timeout 60 \
  | sed 's/^/    /'
echo
echo ">>> in another shell (SAME domain):"
echo "      source ~/.scuba/livox_port/env.sh; export ROS_DOMAIN_ID=$LIO_DOMAIN"
echo "      source ~/.scuba/lio_ws/install/setup.bash"
echo "      ros2 topic echo /Odometry --field pose.pose.position"
echo ">>> NOTE: this has RE-ANCHORED the map frame."
echo "    camera_init is now wherever the robot is standing, and the"
echo "    relocalizer seeds the map frame onto it. Localization MUST be"
echo "    restarted, and every previously noted map coordinate is void."
echo "        ~/.scuba/livemap/start_localization.sh"
echo "    Or use the merged terminal, which sequences both for you:"
echo "        ~/.scuba/lio_ws/start_sensing.sh"
echo ">>> Ctrl+C stops both."
wait
