#!/bin/bash
# STOP -- any terminal on the robot. Stops the pick programs even when their own terminal is lost or stuck.
#   ./stop.sh          the tests, tune.py and perception.py stop, then the server RELEASES the arms (3 s ramp, hands
#                      open) and exits: THE ROBOT LOWERS ITS ARMS -- table away first
#   ./stop.sh tests    only the tests, tune.py and perception.py: the server takes the arms back to the pre-grip pose
#                      along their path and keeps holding them
#   ./stop.sh other    also the other programs that command the arms (dp_play, check_stack.py, r1_head.py): a Ctrl-C
#                      to each, which they answer by giving the arms back
#   ./stop.sh -9       KILL our programs and those others at once: no ramp, the arm stream just stops (the
#                      arms then hold ~10 s and drop to the robot's own pose; the hands stay as they were ->
#                      python3 hands.py open). Last resort, when ./stop.sh did not work
#   ./stop.sh list     only show what runs
#   ./stop.sh hands    only the hand server started by ./server.sh (refused while server.py runs; asks for sudo)
cd "$(dirname "$0")" || exit 1
PICK="$(pwd)"

ours() {   # PIDs of `python3 <script>` run from this folder (not R1_Plate_Perception's plate_perception.py)
  for pid in $(pgrep -f "(^|[ /])$1( |\$)"); do
    [ "$pid" = "$$" ] && continue
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null)
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    case "$cwd/ $cmd" in *"$PICK/"*|*"pick/$1"*) echo "$pid" ;; esac
  done
}
others() {  # other programs that publish rt/arm_sdk
  pgrep -x dp_play
  pgrep -f '(^|[ /])(check_stack|r1_head)\.py( |$)'
}
show() {
  for pid in "$@"; do echo "    $pid  $(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)"; done
}
alive() {   # those of the given PIDs still running (also another user's; an exited one not yet reaped is gone)
  local st
  for pid in "$@"; do
    st=$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -n "$st" ] && [ "${st:0:1}" != "Z" ] && echo "$pid"
  done
}
wait_gone() {   # wait_gone SECONDS PID... -> prints the survivors, fails if any
  local n=$(( $1 * 10 )) t=0 left
  shift
  while :; do
    left=$(alive "$@")
    [ -z "$left" ] && return 0
    [ "$t" -ge "$n" ] && { echo $left; return 1; }
    sleep 0.1
    t=$((t + 1))
  done
}

SERVER=$(ours 'server\.py')
CLIENTS=$(for n in 'run_test\.py' 'tune\.py' 'perception\.py'; do ours "$n"; done)
MONITORS=$(ours 'monitor\.py')
OTHER=$(others)
echo "server:              ${SERVER:-none}";   show $SERVER
echo "tests / perception:  ${CLIENTS:-none}";  show $CLIENTS
echo "monitors:            ${MONITORS:-none}"
echo "other arm programs:  ${OTHER:-none}";    show $OTHER

HANDSRV=$(pgrep -f "brainco_hand_server --network_interface")
echo "hand server:         ${HANDSRV:-none}"

case "$1" in
  list)
    exit 0 ;;
  hands)
    [ -n "$SERVER" ] && { echo "server.py runs: stop it first (./stop.sh), the hands need the hand server while it holds them"; exit 1; }
    [ -z "$HANDSRV" ] && { echo "no hand server running"; exit 0; }
    sudo kill -INT $HANDSRV
    left=$(wait_gone 3 $HANDSRV) || { sudo kill -TERM $left; left=$(wait_gone 2 $left); }
    [ -n "$left" ] && { echo "hand server did not stop: $left  (sudo kill -9 $left)"; exit 1; }
    echo "hand server stopped (the hands keep their last shape)"
    exit 0 ;;
  -9|--kill)
    ALL="$SERVER $CLIENTS $MONITORS $OTHER"
    [ -z "${ALL// /}" ] && { echo "nothing to kill"; exit 0; }
    echo "KILL -9: $ALL  (the arm stream stops without a ramp)"
    kill -9 $ALL 2>/dev/null
    left=$(wait_gone 2 $ALL) || { echo "still running (another user's? try sudo kill -9): $left"; exit 1; }
    echo "all gone"
    echo "measured 2026-10-20: the arms keep the last command for ~10 s, then drop to the robot's own pose -- keep"
    echo "clear, and start the server again only after they dropped. The hands keep their last command:"
    echo "  python3 hands.py open      (or restart brainco_hand_server)"
    exit 0 ;;
esac

rc=0
if [ -n "$CLIENTS" ]; then
  echo "stopping the tests / perception (Ctrl-C to each; the server takes the arms back along the path) ..."
  kill -INT $CLIENTS 2>/dev/null
  left=$(wait_gone 3 $CLIENTS) || { kill -TERM $left 2>/dev/null; left=$(wait_gone 2 $left); }
  [ -n "$left" ] && { echo "did not stop: $left  (./stop.sh -9)"; rc=1; }
fi
if [ "$1" = "other" ] && [ -n "$OTHER" ]; then
  echo "Ctrl-C to the other arm programs: $OTHER"
  kill -INT $OTHER 2>/dev/null
  left=$(wait_gone 5 $OTHER) || { echo "did not stop: $left  (sudo? or ./stop.sh -9)"; rc=1; }
elif [ -n "$OTHER" ]; then
  echo "NOTE: other arm programs run ($OTHER): ./stop.sh other stops them too"
fi
if [ "$1" = "tests" ]; then
  exit $rc
fi
if [ -n "$SERVER" ]; then
  echo "server: RELEASE (3 s ramp, hands open; the robot lowers its arms), then it exits ..."
  kill -TERM $SERVER 2>/dev/null
  left=$(wait_gone 8 $SERVER) || { echo "the server did not stop: $left  -> ./stop.sh -9 kills it (no ramp)"; rc=1; }
  [ -z "$left" ] && echo "server stopped"
fi
exit $rc
