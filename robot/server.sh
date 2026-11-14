#!/bin/bash
# Hand server + arm server in one terminal.
#   ./server.sh            starts brainco_hand_server (with touch) in the background if it is not running yet (asks for
#                          the sudo password once, here), checks it reports both hands' touch sensors, then runs
#                          python3 server.py in this terminal (take / home / release ...)
#   ./server.sh --mock     no hand server; server.py --mock (simulated arm)
#   any other arguments go to server.py
#   ./server.sh --keep-hands   the same, but the hand server keeps running when server.py ends
# When server.py ends (after `release`, `quit`, a kill or a crash) the hands are opened (hands.py open) and the hand
# server is stopped; --keep-hands keeps it.
# Its output goes to logs/hand_server/hand_server_<time>.log. Stop it on its own with: ./stop.sh hands
cd "$(dirname "$0")" || exit 1
HS_DIR="$HOME/project/hand_service/bin"         # a build with touch sensor support
HS_IF="eth10"
hs_pids() { pgrep -f "brainco_hand_server --network_interface"; }

KEEP=""
ARGS=()
for x in "$@"; do
  if [ "$x" = "--keep-hands" ]; then KEEP=1; else ARGS+=("$x"); fi
done
set -- "${ARGS[@]}"

if [ "$1" = "--mock" ]; then
  exec python3 server.py "$@"
fi

if [ -n "$(hs_pids)" ]; then
  echo "hand server: already running (pid $(hs_pids | tr '\n' ' '))"
else
  [ -x "$HS_DIR/brainco_hand_server" ] || { echo "no $HS_DIR/brainco_hand_server: start the hand server yourself"; exit 1; }
  mkdir -p logs/hand_server
  LOG="$(pwd)/logs/hand_server/hand_server_$(date +%Y%m%d_%H%M%S).log"
  echo "hand server: starting (sudo password once) ..."
  sudo -v || { echo "no sudo: hand server not started"; exit 1; }
  (cd "$HS_DIR" && nohup sudo -n ./brainco_hand_server --network_interface "$HS_IF" > "$LOG" 2>&1 &)
  ok=""
  for _ in $(seq 1 50); do                       # up to 10 s for both hands
    n=$(grep -c "has touch sensors" "$LOG" 2>/dev/null)
    [ "${n:-0}" -ge 2 ] && { ok=1; break; }
    sleep 0.2
  done
  grep -i "hand" "$LOG" | head -6 | sed 's/^/  hand server: /'
  if [ -z "$ok" ]; then
    echo "!! hand server: not both 'left/right hand has touch sensors' lines after 10 s (log: $LOG)."
    echo "   pid now: $(hs_pids | tr '\n' ' ')   -- check the hands' power / cable, then ./stop.sh hands and start again."
    echo "   Enter: start server.py anyway   Ctrl-C: stop here"
    read -r _
  fi
  echo "hand server log: $LOG"
fi
echo
python3 server.py "$@"
rc=$?
echo
if [ -n "$KEEP" ]; then
  echo "server.py ended (exit $rc); the hand server keeps running (--keep-hands). Stop it: ./stop.sh hands"
  exit $rc
fi
if [ -n "$(hs_pids)" ]; then
  echo "server.py ended (exit $rc): opening the hands, then stopping the hand server ..."
  timeout 8 python3 hands.py open || echo "  (hands.py open did not finish: the hands keep their last shape)"
  ./stop.sh hands
fi
exit $rc
