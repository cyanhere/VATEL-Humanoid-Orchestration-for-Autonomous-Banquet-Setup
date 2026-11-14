#!/usr/bin/env bash
# MERGE A: one terminal for the whole navigation layer.
#
#     ~/.scuba/nav/start_nav.sh
#     ~/.scuba/nav/start_nav.sh --check     # verify preconditions, start nothing
#
# Replaces FOUR terminals: odom_2d, costmap_decay, the Nav2 launch, nav_bridge.
#
# WHY THESE FOUR, AND IN THIS ORDER
# The order is not cosmetic, it is a dependency chain:
#   odom_2d        needs /Odometry from FAST-LIO
#   costmap_decay  needs map->base_link, which only the relocalizer publishes
#   Nav2           its memory_layer has NO source but /memory_grid; without it
#                  the planner sees only obstacle_layer (obstacle_range 4.0)
#                  and will happily plan through walls beyond 4 m
#   nav_bridge     has no upstream dependency, so it goes last
#
# WHY ONE SHELL CAN HOLD ALL FOUR
# lio_ws/env.sh and sdk_teleop/env.sh are both Foxy, both domain 0, both
# rmw_cyclonedds_cpp, same CYCLONEDDS_URI, same libddsc 0.10.2 forced to the
# front. Verified 2026-10-30 in BOTH orders: g1_teleop, fast_lio and
# livox_ros_driver2 all visible, unitree_sdk2py imports, ros2 graph OK. The two
# were separate terminals by history, not necessity.
#   This does NOT extend to camview/env.sh (Noetic). That stays in its own
#   shell and refuses to load on top of Foxy.
set -o pipefail

PY=/usr/bin/python3
WAIT="$PY $HOME/.scuba/nav/waitfor.py"
NAV2_LAUNCH=$HOME/.scuba/livemap/launch/nav_livemap.launch.py
NAV2_YAML=$HOME/.scuba/livemap/config/nav2_livemap.yaml
DECAY_ARGS=${DECAY_ARGS:---box 0 --max-age-s 0 --raytrace-range 15}
BRIDGE_ARGS=${BRIDGE_ARGS:--p enable:=true -p max_vy:=0.50 -p max_lin_acc:=2.0 -p max_ang_acc:=2.0}

CHECK=0
BRIDGE=1
for arg in "$@"; do
  case "$arg" in
    --check)     CHECK=1 ;;
    --no-bridge) BRIDGE=0 ;;
  esac
done

# ---- env: lio first, then sdk_teleop for the g1_teleop package
source "$HOME/.scuba/lio_ws/env.sh"      >/dev/null 2>&1 || { echo "!! lio_ws/env.sh failed"; exit 1; }
source "$HOME/.scuba/sdk_teleop/env.sh"  >/dev/null 2>&1 || { echo "!! sdk_teleop/env.sh failed"; exit 1; }
command -v ros2 >/dev/null || { echo "!! ros2 not on PATH"; exit 1; }

echo "=== env ==="
echo "    ROS_DOMAIN_ID=$ROS_DOMAIN_ID  rmw=$RMW_IMPLEMENTATION"
for pkg in fast_lio g1_teleop; do
  if ros2 pkg list 2>/dev/null | grep -qx "$pkg"; then echo "    $pkg: visible"
  else echo "    !! $pkg NOT visible - the overlay did not source"; exit 1; fi
done

echo
echo "=== preconditions (this layer cannot work without them) ==="
fail=0
if $WAIT --topic /Odometry --count 3 --timeout 15; then :; else
  echo "    !! no /Odometry. Start the sensing terminal first:"
  echo "       ~/.scuba/lio_ws/start_sensing.sh"
  fail=1
fi
if $WAIT --tf map base_link --timeout 15; then :; else
  echo "    !! no map->base_link. The relocalizer is not up (or still seeding)."
  echo "       It comes from start_localization.sh, inside start_sensing.sh."
  fail=1
fi
[ "$fail" -ne 0 ] && { echo; echo "refusing to start a half-working nav layer."; exit 1; }

# A live invariant worth catching at startup rather than mid-mission: DWB will
# plan yaw rates the bridge then clips, so the robot turns slower than planned.
if [ -f "$NAV2_YAML" ]; then
  mvt=$(grep -E "^\s*max_vel_theta:" "$NAV2_YAML" | head -1 | awk '{print $2}')
  mvy=$(echo "$BRIDGE_ARGS" | grep -oE "max_vyaw:=[0-9.]+" | cut -d= -f3)
  mvy=${mvy:-0.40}
  if [ -n "$mvt" ] && $PY -c "import sys; sys.exit(0 if float('$mvt')>float('$mvy') else 1)"; then
    echo
    echo "    !! max_vel_theta $mvt (DWB) > max_vyaw $mvy (bridge):"
    echo "       DWB will plan turns the bridge clips. Known open item."
  fi
fi

OWNED_NODES="odom_2d.py costmap_decay.py planner_server controller_server nav_bridge"

# ---- refuse to start a SECOND copy -----------------------------------------
# See nav/dupcheck.py for why this is a hard refusal and not a warning.
if ! /usr/bin/python3 "$HOME/.scuba/nav/dupcheck.py" $OWNED_NODES; then
  echo
  if [ "${FORCE:-0}" = "1" ]; then
    echo "   FORCE=1 set - continuing anyway, on your head."
  else
    echo "   re-check with:  bash $0 --check-dups"
    exit 1
  fi
fi
if [ "${1:-}" = "--check-dups" ]; then
  echo "    no duplicates of: $OWNED_NODES"
  exit 0
fi

PIDS=(); NAMES=()
REAPED=0
cleanup() {
  [ "$REAPED" = 1 ] && return
  REAPED=1
  echo
  echo "=== stopping (reverse order) ==="
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    p=${PIDS[$i]}
    [ -z "$p" ] && continue
    kill -0 "$p" 2>/dev/null || continue
    echo "    SIGINT ${NAMES[$i]} (pid $p)"
    # SIGINT, not killpg: `ros2 launch` gives each node its own process group,
    # so killpg would orphan the servers. SIGINT makes launch shut them down.
    kill -INT "$p" 2>/dev/null
  done
  sleep 3
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    p=${PIDS[$i]}
    [ -n "$p" ] && kill -0 "$p" 2>/dev/null && { echo "    SIGKILL ${NAMES[$i]}"; kill -9 "$p" 2>/dev/null; }
  done
}
# see the note in start_sensing.sh: HUP is how a closed terminal orphans these
trap 'cleanup; exit 130' INT TERM HUP
trap cleanup EXIT

start() {                       # start <label> <prefix> <cmd...>
  local label="$1" prefix="$2"; shift 2
  echo
  echo "=== $label ==="
  if [ "$CHECK" -eq 1 ]; then echo "    [--check] would run: $*"; return 0; fi
  # process substitution, NOT a pipe: with `cmd | sed &` the $! is SED's pid,
  # so cleanup would kill the log prefixer and leave the node running.
  "$@" > >(sed "s/^/$prefix /") 2>&1 &
  local p=$!
  PIDS+=("$p"); NAMES+=("$label")
  echo "    pid $p"
  sleep 1
  kill -0 "$p" 2>/dev/null || { echo "    !! $label exited immediately"; cleanup; exit 1; }
}

ready() {                       # ready <label> <waitfor args...>
  # shift the label off FIRST: leaving it in "$@" hands waitfor.py a
  # positional argument it has none of, argparse exits 2, and every readiness
  # check "fails" - tearing down a stack that was coming up perfectly well.
  # --check cannot catch this, because --check returns before the call.
  local label="$1"; shift
  [ "$CHECK" -eq 1 ] && return 0
  if ! $WAIT "$@"; then
    echo "    !! $label never became ready - stopping, rather than leaving a"
    echo "       stack that looks up but cannot plan."
    cleanup
    exit 1
  fi
}

start "odom_2d" "[odom]" $PY "$HOME/.scuba/nav/odom_2d.py"
ready "/odom_2d" --topic /odom_2d --count 3 --timeout 25

start "costmap_decay" "[decay]" $PY -u "$HOME/.scuba/livemap/costmap_decay.py" $DECAY_ARGS
ready "/memory_grid" --topic /memory_grid --count 1 --timeout 60

start "Nav2" "[nav2]" ros2 launch "$NAV2_LAUNCH"
ready "global costmap" --topic /global_costmap/costmap_raw --count 1 --timeout 60

if [ "$BRIDGE" -eq 1 ]; then
  start "nav_bridge" "[bridge]" ros2 run g1_teleop nav_bridge --ros-args $BRIDGE_ARGS
  ready "/g1_nav_bridge" --node g1_nav_bridge --timeout 25
else
  echo
  echo "=== nav_bridge: SKIPPED (--no-bridge) ==="
  echo "    nav_bridge is the ONLY route from ROS 2 to the legs, so with it"
  echo "    down the robot physically cannot move. Planning, costmaps and the"
  echo "    viewer all work; /cmd_vel is published to nobody."
fi

if [ "$CHECK" -eq 1 ]; then
  echo
  echo "=== --check done: preconditions OK, nothing started ==="
  exit 0
fi

echo
if [ "$BRIDGE" -eq 1 ]; then
  echo "=== nav layer UP (4 terminals merged into this one) ==="
  echo "    enable:=true is in BRIDGE_ARGS - without it nothing reaches the robot"
else
  echo "=== nav layer UP, NO BRIDGE - the robot cannot move ==="
  echo "    plan with:  ros2 action send_goal /compute_path_to_pose ..."
  echo "    do NOT use send_goal.py here: it publishes /cmd_vel itself"
fi
echo "    Ctrl-C stops all four, in reverse order"
echo
while true; do
  for ((i=0; i<${#PIDS[@]}; i++)); do
    kill -0 "${PIDS[$i]}" 2>/dev/null || { echo "!! ${NAMES[$i]} exited"; cleanup; exit 1; }
  done
  sleep 2
done
