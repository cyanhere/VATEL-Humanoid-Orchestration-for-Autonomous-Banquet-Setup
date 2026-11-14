# Env for the head-camera tools. They are ROS 1 (Noetic) because that is where
# realsense2_camera is installed; there is no ROS 2 realsense package and
# pyrealsense2 is not installed.
#
#   source ~/.scuba/camview/env.sh
#
# 1) Strip conda FIRST. An interactive shell on this robot activates conda
#    base, whose python3 has no cv2, rospy or cv_bridge -> "ModuleNotFoundError:
#    No module named 'cv2'". The same strip is in lio_ws/env.sh and
#    sdk_teleop/env.sh; /opt/ros/noetic/setup.bash does NOT do it, and conda
#    only auto-activates in INTERACTIVE shells, so the failure looks random.
conda deactivate 2>/dev/null; conda deactivate 2>/dev/null
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniforge | grep -v -i conda | paste -sd:)

# 2) Refuse to stack on top of ROS 2. Sourcing foxy THEN noetic puts ROS 1's
#    tf2_ros ahead on the path, and any ROS 2 node launched from that shell
#    dies in xmlrpc trying to reach a ROS 1 master. A shell cannot be
#    un-sourced, so the only fix is a new terminal.
if [ "${ROS_DISTRO:-}" = "foxy" ]; then
  echo "!! this shell already has ROS 2 (foxy) sourced."
  echo "!! open a NEW terminal for the camera tools - do not stack the two."
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/noetic/setup.bash

echo "camview env | ROS_DISTRO=$ROS_DISTRO"
echo "  python3 : $(command -v python3)"
if err=$(python3 -c "import cv2, rospy, cv_bridge, numpy" 2>&1); then
  echo "  cv2 $(python3 -c 'import cv2;print(cv2.__version__)') + rospy + cv_bridge : OK"
else
  echo "  !! imports FAILED:"; echo "$err" | tail -2 | sed 's/^/     /'
fi
echo "  master  : $ROS_MASTER_URI"
