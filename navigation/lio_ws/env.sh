# Env for ANY shell that talks to the FAST-LIO stack (monitoring, MPPI, rviz).
# Source this instead of hand-setting variables - the ordering matters.
#
#   source ~/.scuba/lio_ws/env.sh
#
# Why each piece:
#  * cyclonedds_ws supplies libddsc 0.10.2 AND an rmw_cyclonedds_cpp built
#    against it. Foxy's own rmw is built against 0.7.0, which does not
#    understand the <Interfaces> element in cyclonedds.xml and fails with
#    "Interfaces: unknown element". They must be a matched pair.
#  * Domain 0 is where the robot's control stack and nav_bridge live, so
#    /Odometry has to be published there for MPPI to see it.
conda deactivate 2>/dev/null; conda deactivate 2>/dev/null
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniforge | grep -v -i conda | paste -sd:)

source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash          # read-only: 0.10.2 + matching rmw
source /home/unitree/.scuba/livox_port/livox_ws/install/setup.bash 2>/dev/null
source /home/unitree/.scuba/lio_ws/install/setup.bash 2>/dev/null

CDDS_LIB=/home/unitree/cyclonedds_ws/install/cyclonedds/lib
[ -d "$CDDS_LIB" ] && export LD_LIBRARY_PATH="$CDDS_LIB:${LD_LIBRARY_PATH:-}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
export ROS_DOMAIN_ID=${LIO_DOMAIN:-0}

echo "lio env | domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
echo "  libddsc: $(ls $CDDS_LIB/libddsc.so.0.* 2>/dev/null | head -1)"
if timeout 10 ros2 topic list >/dev/null 2>&1; then
  echo "  ros2 graph: OK"
else
  echo "  !! ros2 graph FAILED - check the cyclonedds config/rmw pair"
fi
