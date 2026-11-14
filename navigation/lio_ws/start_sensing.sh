#!/usr/bin/env bash
# MERGE C: one terminal for sensing AND localization.
#
#     ~/.scuba/lio_ws/start_sensing.sh
#     ~/.scuba/lio_ws/start_sensing.sh --ros-args -p seed_only:=true
#     NO_LOC=1 ~/.scuba/lio_ws/start_sensing.sh     # lidar + FAST-LIO only
#
# Runs run_lio.sh (livox driver + FAST-LIO + mount_calib) and then
# start_localization.sh (accumulator + relocalizer), in that order, in one
# terminal. Both scripts are left intact and still work on their own.
#
# WHY MERGE THESE TWO
# They are coupled by physics, not convenience. Restarting FAST-LIO RE-ANCHORS
# camera_init to wherever the robot is standing, and since the relocalizer
# seeds at seed_x/y/yaw = 0 against a cleared map, the whole `map` frame moves
# with it. Verified: map->camera_init measures ~identity in translation. So you
# can never restart FAST-LIO alone and keep a valid map - localization must
# follow. One terminal makes that automatic instead of remembered.
#
# WHAT IT COSTS YOU
# Restarting localization ALONE is still a common operation (a bad map, a
# re-seed) and you do NOT want the lidar to bounce for that. So do that with
# start_localization.sh directly, exactly as before. This script is for the
# case where FAST-LIO needs to come up.
#
# AFTER THIS, EVERY MAP-FRAME COORDINATE IS VOID
# Table centres, noted goals - all of them refer to the previous anchor.
# table_registry keeps its entries in memory, so restarting it re-detects.
set -o pipefail

LIO=$HOME/.scuba/lio_ws/run_lio.sh
LOC=$HOME/.scuba/livemap/start_localization.sh
PY=/usr/bin/python3

# waitfor.py needs rclpy, and this wrapper deliberately does NOT source ROS:
# run_lio.sh documents "sources its own env; do not pre-source", and
# start_localization.sh sources its own too. So the env is sourced inside a
# SUBSHELL for the duration of each check, leaving the parent - and therefore
# both children - exactly as they were.
WAIT() {
  ( source "$HOME/.scuba/lio_ws/env.sh" >/dev/null 2>&1 || exit 97
    exec "$PY" "$HOME/.scuba/nav/waitfor.py" "$@" )
}

for f in "$LIO" "$LOC"; do
  [ -x "$f" ] || [ -f "$f" ] || { echo "!! missing $f"; exit 1; }
done

OWNED_NODES="relocalizer_live.py accumulator.py fastlio_mapping livox_ros_driver2_node"

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

LIO_PID=""; LOC_PID=""
cleanup() {
  [ -z "$LOC_PID$LIO_PID" ] && return    # already reaped; idempotent by state,
                                         # not by a latch that outlives it
  echo
  echo "=== stopping (localization first, then the lidar) ==="
  # Reverse order deliberately: the relocalizer and accumulator consume
  # FAST-LIO's output, and tearing the producer down first makes them log
  # errors on the way out for no reason.
  #
  # SIGINT, not SIGKILL and not killpg: each child script installs its own
  # trap and cleans up ITS children. Killing the script outright would orphan
  # the livox driver, FAST-LIO, the accumulator and the relocalizer - which is
  # exactly the mess this is meant to avoid.
  # Escalate to the process GROUP, not just the supervisor. Each child was
  # setsid'd, so its pid IS its group leader and `kill -- -PID` reaps the whole
  # tree. MEASURED 2026-11-03: SIGKILLing the supervisor alone left the livox
  # driver and FAST-LIO running as orphans, because SIGKILL cannot be trapped
  # and so run_lio.sh's own cleanup never ran. 20 s before escalating, since
  # run_lio.sh can be mid-`sleep 8` when the signal lands.
  for p in "$LOC_PID" "$LIO_PID"; do
    [ -z "$p" ] && continue
    kill -0 "$p" 2>/dev/null || continue
    echo "    SIGINT pid $p (letting its own trap reap its children)"
    kill -INT "$p" 2>/dev/null
    for i in $(seq 1 20); do
      kill -0 "$p" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$p" 2>/dev/null; then
      echo "    did not stop - SIGTERM to the process group"
      kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null
      sleep 3
    fi
    if kill -0 "$p" 2>/dev/null; then
      echo "    SIGKILL to the process group"
      kill -9 -- "-$p" 2>/dev/null || kill -9 "$p" 2>/dev/null
    fi
  done
  LIO_PID=""; LOC_PID=""
}
# HUP matters as much as INT: closing the terminal sends HUP, and because the
# children are setsid they are in another session and never see the terminal's
# own HUP. Without this they outlive the script as orphans - which is exactly
# how the duplicate stack above was created. EXIT covers every other path.
trap 'cleanup; exit 130' INT TERM HUP
trap cleanup EXIT

echo "================================================================"
echo "  sensing + localization   (merge C: 2 terminals in 1)"
echo "================================================================"
echo "  KEEP THE ROBOT STILL until the first /Odometry - FAST-LIO"
echo "  estimates gravity at init and gait accel corrupts it."
echo

# setsid: give each child its own process group so Ctrl-C at the terminal does
# NOT race us by hitting them directly. We signal them in a deliberate order.
setsid bash "$LIO" > >(sed 's/^/[lio] /') 2>&1 &
LIO_PID=$!
echo "[sensing] run_lio.sh pid $LIO_PID"

if ! kill -0 "$LIO_PID" 2>/dev/null; then
  echo "!! run_lio.sh exited immediately"; exit 1
fi

echo
echo "[sensing] waiting for /Odometry before starting localization ..."
echo "          (not ros2 topic hz, which reports NO MESSAGES on a busy box"
echo "           for topics that are in fact publishing)"
if ! WAIT --topic /Odometry --count 5 --timeout 90; then
  echo "!! /Odometry never arrived."
  echo "   Starting the relocalizer now would have it register against"
  echo "   nothing, so stopping instead. Check /tmp/lio_fastlio.log."
  cleanup
  exit 1          # cleanup RETURNS now that EXIT is trapped; without this the
fi                # script carried on and started localization anyway
if ! WAIT --tf map camera_init --timeout 5 --quiet; then
  echo "[sensing] (map->camera_init absent, as expected: the relocalizer"
  echo "          publishes it and has not started yet)"
fi

if [ "${NO_LOC:-0}" = "1" ]; then
  echo
  echo "[sensing] NO_LOC=1 - stopping here. Start localization yourself with:"
  echo "            ~/.scuba/livemap/start_localization.sh"
else
  echo
  echo "[sensing] starting localization (clears the map, by design)"
  setsid bash "$LOC" "$@" > >(sed 's/^/[loc] /') 2>&1 &
  LOC_PID=$!
  echo "[sensing] start_localization.sh pid $LOC_PID"
  sleep 2
  kill -0 "$LOC_PID" 2>/dev/null || { echo "!! localization exited immediately"; cleanup; exit 1; }
  echo
  echo "[sensing] waiting for the relocalizer's map->base_link ..."
  if WAIT --tf map base_link --timeout 90; then
    echo "[sensing] localization is live"
  else
    echo "[sensing] !! map->base_link never resolved. The relocalizer may still"
    echo "          be seeding, or it is not registering. Leaving both up so"
    echo "          you can read the [rel] lines; Ctrl-C stops everything."
  fi
fi

echo
echo "================================================================"
echo "  UP. Next terminal:   ~/.scuba/nav/start_nav.sh"
echo "================================================================"
echo "  Remember: the map frame has just been RE-ANCHORED. Every"
echo "  previously noted map coordinate (table centres, goals) is void."
echo "  Ctrl-C stops localization and the lidar, in that order."
echo

while true; do
  kill -0 "$LIO_PID" 2>/dev/null || { echo "!! run_lio.sh exited"; cleanup; exit 1; }
  if [ -n "$LOC_PID" ]; then
    kill -0 "$LOC_PID" 2>/dev/null || { echo "!! localization exited"; cleanup; exit 1; }
  fi
  sleep 2
done
