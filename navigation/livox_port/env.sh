# Livox build/run env — sanitizes conda out of the way. Source, never add to .bashrc.
conda deactivate 2>/dev/null; conda deactivate 2>/dev/null
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniforge | paste -sd:)
source /opt/ros/foxy/setup.bash
unset DISTRO_ROS
export CMAKE_PREFIX_PATH=$HOME/.scuba/livox_port/sdk2_install:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.scuba/livox_port/sdk2_install/lib:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=99
[ -f $HOME/.scuba/livox_port/livox_ws/install/setup.bash ] && \
  source $HOME/.scuba/livox_port/livox_ws/install/setup.bash
echo "livox env ready | python3=$(which python3) | ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# --- DDS consistency (added) -------------------------------------------------
# Domain 99 stays: it keeps 200k pts/s of point cloud OFF the robot control
# domain (0) during mapping. That isolation is deliberate.
# But use CycloneDDS rather than Foxy default FastRTPS, so all three ~/.scuba
# environments speak the same DDS. FastRTPS on domain 0 dies parsing Unitree
# traffic ("bad_alloc caught"); pinning cyclonedds here removes that variable.
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null   # read-only
_CDDS=/home/unitree/cyclonedds_ws/install/cyclonedds/lib
[ -d "$_CDDS" ] && export LD_LIBRARY_PATH="$_CDDS:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
echo "  dds: cyclonedds, domain $ROS_DOMAIN_ID (isolated from control domain 0)"
