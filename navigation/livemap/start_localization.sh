#!/usr/bin/env bash
# Start the accumulator and the relocalizer TOGETHER, from a cleared map.
#
#     ~/.scuba/livemap/start_localization.sh
#     ~/.scuba/livemap/start_localization.sh --ros-args -p seed_only:=true
#     ACC_ARGS="--pub-voxel 0.05" ~/.scuba/livemap/start_localization.sh
#
# WHY THIS EXISTS
# These two share one file, ~/.scuba/livemap/maps/live/live_cloud.npy: the
# accumulator writes it, the relocalizer registers against it and reloads it
# every map_reload_s. Start them in the wrong order, or start the relocalizer
# alone, and it registers today's scans against a PREVIOUS RUN's map of a room
# that has changed. Measured 2026-10-23, that produced inliers of 0.41-0.44 -
# inside the "ambiguous, do not insert" band - so the gate skipped 1749 scans,
# the accumulator stored nothing, and it wrote the near-empty store back over
# the file. A self-reinforcing collapse that does not recover on its own.
# A separate occasion left the relocalizer on a day-old 418k-point map,
# wandering 4.8 cm per ICP cycle on a stationary robot.
#
# The mission is designed to start from zero every run because the room changes
# between runs, so clearing is correct - it is doing it reliably that matters.
#
# WHAT IT DOES
#   1. rolls the existing map to .prev  (we have destroyed a map by accident
#      once: 76,294 points -> 26,696; one rolling backup is cheap insurance)
#   2. removes it and VERIFIES it is gone, rather than trusting the
#      accumulator's constructor to do it
#   3. starts the accumulator, waits for /accumulated_cloud to appear
#   4. only then starts the relocalizer, so it comes up with no prior map and
#      sits in its documented gravity-levelled-seed state
#   5. Ctrl-C stops both
set -o pipefail

MAP="$HOME/.scuba/livemap/maps/live/live_cloud.npy"
ACC="$HOME/.scuba/livemap/accumulator.py"
REL="$HOME/.scuba/livemap/relocalizer_live.py"
PY=/usr/bin/python3

source "$HOME/.scuba/lio_ws/env.sh" || { echo "!! env.sh failed"; exit 1; }
command -v ros2 >/dev/null || { echo "!! ros2 not on PATH"; exit 1; }

echo
echo "=== 1. clearing the previous map ==="
if [ -f "$MAP" ]; then
  n=$("$PY" - "$MAP" <<'EOF' 2>/dev/null || echo "?"
import sys, numpy
print(len(numpy.load(sys.argv[1])))
EOF
)
  echo "    existing: $MAP  ($n points)"
  mv -f "$MAP" "$MAP.prev"
  echo "    rolled to $(basename "$MAP").prev"
else
  echo "    no existing map"
fi
if [ -f "$MAP" ]; then
  echo "!! could not remove $MAP - refusing to start, the relocalizer would"
  echo "   register against it."
  exit 1
fi
echo "    verified gone"

# kill by PID only. `pkill -f <path>` matches THIS script's own command line and
# has killed the shell running it.
cleanup() {
  echo
  echo "=== stopping ==="
  for p in $ACC_PID $REL_PID; do
    [ -n "$p" ] && kill "$p" 2>/dev/null && echo "    sent TERM to $p"
  done
  sleep 2
  for p in $ACC_PID $REL_PID; do
    [ -n "$p" ] && kill -9 "$p" 2>/dev/null
  done
  exit 0
}
trap cleanup INT TERM

echo
echo "=== 2. accumulator ==="
# process substitution, NOT a pipe: with `cmd | sed &` the $! is SED's pid,
# so cleanup would kill the prefixer and leave the node running.
$PY -u "$ACC" ${ACC_ARGS:-} > >(sed 's/^/[acc] /') 2>&1 &
ACC_PID=$!
echo "    pid $ACC_PID"

# Wait for the NODE, not for /accumulated_cloud. The accumulator cannot
# publish until it has TF map->camera_init, and only the relocalizer provides
# that - so waiting on the topic would deadlock the very ordering we want.
# The node existing is enough: the map removal happens in its constructor, and
# all we need is for it to be alive before the relocalizer comes up.
echo -n "    waiting for the /map_accumulator node "
ok=0
for i in $(seq 1 15); do
  if timeout 3 ros2 node list 2>/dev/null | grep -q '/map_accumulator'; then
    ok=1; echo " up after ${i}s"; break
  fi
  if ! kill -0 "$ACC_PID" 2>/dev/null; then
    echo; echo "!! the accumulator exited. Not starting the relocalizer - it"
    echo "   would register against whatever map appears next."
    exit 1
  fi
  echo -n "."
  sleep 1
done
[ "$ok" -ne 1 ] && echo " not seen in 15s - continuing, the process is alive"

echo
echo "=== 3. relocalizer ==="
$PY -u "$REL" "$@" > >(sed 's/^/[rel] /') 2>&1 &
REL_PID=$!
echo "    pid $REL_PID"
echo
echo "both up. Ctrl-C stops both. Watch for:"
echo "  [rel] auto_z: floor measured at ...   the relocalizer has its floor"
echo "  [acc] ... stored N                    N climbing from 0"
echo "  [acc] inliers                         should rise ABOVE gate_hi 0.50;"
echo "                                        0.41-0.44 is the ambiguous band"
echo

# if either dies, stop the other rather than leaving a half-started pair
while true; do
  kill -0 "$ACC_PID" 2>/dev/null || { echo "!! accumulator exited"; cleanup; }
  kill -0 "$REL_PID" 2>/dev/null || { echo "!! relocalizer exited"; cleanup; }
  sleep 2
done
