#!/usr/bin/env bash
# Bring up the DRY-RUN navigation stack (terminals A..E) in one shell, each
# stage gated on the previous one actually being ready.
#
#   ~/.scuba/nav/start_dryrun.sh --seed-z 1.55 --seed-yaw 1.5708
#
# NO MOTION. nav_bridge is started with enable=false, so /cmd_vel is logged and
# never forwarded, and this script never touches the FSM services (~/start,
# ~/damp, ~/estop). Nothing is published to /api/sport/request - a watchdog
# verifies that continuously and shouts if anything ever appears there.
# Mode changing stays manual, by hand, as it should.
#
# Stages, in order:
#   A  run_lio.sh      livox driver + FAST-LIO + gravity mount calib + base_link
#   B  relocalizer     map -> camera_init   (needs the robot STILL to seed)
#   C  path_viewer     http://<robot>:8090
#   D  nav2            map_server + NavFn A* + DWB + odom_2d
#   E  nav_bridge      DRY RUN: logs what it WOULD send
#
# It deliberately does NOT send a goal. That is your call; the command is
# printed at the end, along with the logger and the jitter diagnostic.
#
# Ctrl-C once stops everything, in reverse order.
set -o pipefail

SEED_X=0.0; SEED_Y=0.0; SEED_Z=1.55; SEED_YAW=1.5708
PORT=8090
DO_VIEWER=1; DO_NAV=1; DO_BRIDGE=1
ASSUME_YES=0
TAIL_SPEC=""
LOGD=/tmp/scuba_dry
ALLARGS="$*"
# hostname -I lists the G1's internal 192.168.123.x first; the operator
# reaches the robot on the shared 172.20.10.x network, so prefer that.
IPADDR=$(hostname -I | tr ' ' '\n' | grep -m1 '^172\.20\.' \
         || hostname -I | awk '{print $1}')

# A dry run that can move the robot is not a dry run. Checked on the ORIGINAL
# arguments, before the parse loop shifts them away.
case "$ALLARGS" in *enable:=true*)
  echo "!! this script is the DRY RUN; it will not enable motion." >&2; exit 2 ;;
esac

usage() {
  sed -n '2,28p' "$0" | sed 's/^# \{0,1\}//'
  cat <<EOF

options
  --seed-x F --seed-y F --seed-z F   metres, map frame (floor at z=0)
  --seed-yaw F                       RADIANS; +y is 1.5708, +x is 0
  --port N                           path_viewer port (default $PORT)
  --no-viewer --no-nav --no-bridge   skip that stage
  -y, --yes                          do not prompt before starting
  --tail LIST                        stream these stage logs into THIS
                                     terminal, comma separated, any of
                                       lio reloc viewer nav bridge all
                                     e.g. --tail bridge,reloc
                                     (every stage is logged to $LOGD
                                      regardless; this only echoes it here)
EOF
}

num() {   # reject the placeholder mistake: seed_z:=Z used to reach python
  case "$2" in
    ''|*[!0-9.eE+-]*) echo "!! $1 must be a number, got '$2'" >&2; exit 2 ;;
  esac
}

while [ $# -gt 0 ]; do
  case "$1" in
    --seed-x)   num seed-x "$2";   SEED_X=$2;   shift 2 ;;
    --seed-y)   num seed-y "$2";   SEED_Y=$2;   shift 2 ;;
    --seed-z)   num seed-z "$2";   SEED_Z=$2;   shift 2 ;;
    --seed-yaw) num seed-yaw "$2"; SEED_YAW=$2; shift 2 ;;
    --port)     num port "$2";     PORT=$2;     shift 2 ;;
    --no-viewer) DO_VIEWER=0; shift ;;
    --no-nav)    DO_NAV=0;    shift ;;
    --no-bridge) DO_BRIDGE=0; shift ;;
    -y|--yes)    ASSUME_YES=1; shift ;;
    --tail)      TAIL_SPEC=$2; shift 2 ;;
    -h|--help)   usage; exit 0 ;;
    *) echo "!! unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

TAIL_STAGES=""
if [ -n "$TAIL_SPEC" ]; then
  for w in $(echo "$TAIL_SPEC" | tr ',' ' '); do
    case "$w" in
      lio)    TAIL_STAGES="$TAIL_STAGES A_lio" ;;
      reloc)  TAIL_STAGES="$TAIL_STAGES B_reloc" ;;
      viewer) TAIL_STAGES="$TAIL_STAGES C_viewer" ;;
      nav)    TAIL_STAGES="$TAIL_STAGES D_nav2" ;;
      bridge) TAIL_STAGES="$TAIL_STAGES E_bridge" ;;
      all)    TAIL_STAGES="A_lio B_reloc C_viewer D_nav2 E_bridge" ;;
      *) echo "!! --tail: unknown stage '$w' (lio reloc viewer nav bridge all)" >&2
         exit 2 ;;
    esac
  done
fi

LIOENV=/home/unitree/.scuba/lio_ws/env.sh
SDKENV=/home/unitree/.scuba/sdk_teleop/env.sh
for f in "$LIOENV" "$SDKENV" /home/unitree/.scuba/lio_ws/run_lio.sh \
         /home/unitree/.scuba/reloc/relocalizer.py \
         /home/unitree/.scuba/viz/path_viewer.py \
         /home/unitree/.scuba/nav/launch/nav_dwb.launch.py; do
  [ -e "$f" ] || { echo "!! missing: $f" >&2; exit 1; }
done

mkdir -p "$LOGD"
PIDS=(); NAMES=(); LOGS=()

CLEANED=0
cleanup() {
  [ "$CLEANED" -eq 1 ] && return 0
  CLEANED=1
  printf '\n>>> stopping, reverse order\n'
  local i p
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    p=${PIDS[$i]}
    kill -0 "$p" 2>/dev/null || continue
    printf '    TERM %s\n' "${NAMES[$i]}"
    kill -TERM -"$p" 2>/dev/null
  done
  sleep 3
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    p=${PIDS[$i]}
    if kill -0 "$p" 2>/dev/null; then
      printf '    KILL %s\n' "${NAMES[$i]}"
      kill -KILL -"$p" 2>/dev/null
    fi
  done
  printf '>>> logs kept in %s\n' "$LOGD"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# setsid so each stage leads its own process group: run_lio.sh spawns the livox
# driver, fastlio and the base_link publisher, and killing the script must take
# all of them down, not orphan them holding the lidar socket.
start_stage() {   # name  shell-command
  # NOT one `local`: bash expands every argument to `local` BEFORE it performs
  # any of the assignments, so "$LOGD/$name.log" would see an empty $name and
  # every stage would share one /tmp/scuba_dry/.log.
  local name="$1" cmd="$2"
  local log="$LOGD/$name.log"
  local pid
  printf '\n>>> %s  (log: %s)\n' "$name" "$log"
  setsid bash -c "$cmd" > "$log" 2>&1 &
  pid=$!
  PIDS+=("$pid"); NAMES+=("$name"); LOGS+=("$log")
  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    printf '    !! %s exited immediately:\n' "$name"
    tail -15 "$log" | sed 's/^/       /'
    return 1
  fi
  return 0
}

# One message on a topic is the only honest proof a stage is up: a node can be
# alive and still publishing nothing (that was the whole /Odometry-twist bug).
wait_msg() {   # topic  field-args  timeout-s
  local topic="$1" fld="$2" lim="$3" waited=0 tmp
  tmp=$(mktemp)
  while [ "$waited" -lt "$lim" ]; do
    : > "$tmp"
    # shellcheck disable=SC2086
    timeout 6 ros2 topic echo "$topic" $fld > "$tmp" 2>/dev/null
    if [ -s "$tmp" ]; then
      printf '\r    %-34s up after %ds\n' "$topic" "$waited"; rm -f "$tmp"; return 0
    fi
    waited=$((waited + 6))
    printf '\r    %-34s waiting %ds/%ds' "$topic" "$waited" "$lim"
  done
  printf '\n    !! %s never published\n' "$topic"; rm -f "$tmp"; return 1
}

wait_log() {   # log  pattern  timeout-s
  local log="$1" pat="$2" lim="$3" waited=0
  while [ "$waited" -lt "$lim" ]; do
    grep -qE "$pat" "$log" 2>/dev/null && {
      printf '    %-34s ready after %ds\n' "$(basename "$log")" "$waited"; return 0; }
    sleep 2; waited=$((waited + 2))
  done
  printf '    !! %s never printed /%s/\n' "$log" "$pat"; return 1
}

die() { printf '\n!! %s\n!! stage logs are in %s\n' "$1" "$LOGD" >&2; exit 1; }

cat <<EOF

================================================================
 DRY RUN - the robot will NOT be commanded to move
================================================================
 seed      x=$SEED_X  y=$SEED_Y  z=$SEED_Z m  yaw=$SEED_YAW rad
 viewer    http://$IPADDR:$PORT
 logs      $LOGD
 bridge    enable=false  (mode changing stays manual)

 The robot must be STILL on the rack until stage B reports a
 pose: FAST-LIO estimates gravity at init, and the relocalizer
 seeds from the same IMU.
================================================================
EOF
if [ "$ASSUME_YES" -eq 0 ]; then
  printf 'robot still and on the rack? [Enter to start, Ctrl-C to abort] '
  read -r _
fi

# ---- the gate shell needs the same DDS setup as the stages
# shellcheck disable=SC1090
source "$LIOENV" > "$LOGD/gate_env.log" 2>&1 || die "could not source $LIOENV"

# ---- A ---------------------------------------------------------------------
start_stage A_lio "source '$LIOENV' >/dev/null 2>&1; exec /home/unitree/.scuba/lio_ws/run_lio.sh" \
  || die "stage A failed to start"
wait_msg /Odometry "--field pose.pose.position" 90 || die "FAST-LIO produced no /Odometry"
grep -E "from up|mount|tilt" "$LOGD/A_lio.log" | head -4 | sed 's/^/    /'

# ---- B ---------------------------------------------------------------------
start_stage B_reloc "source '$LIOENV' >/dev/null 2>&1; exec /usr/bin/python3 \
  /home/unitree/.scuba/reloc/relocalizer.py --ros-args \
  -p seed_x:=$SEED_X -p seed_y:=$SEED_Y -p seed_z:=$SEED_Z -p seed_yaw:=$SEED_YAW" \
  || die "stage B failed to start"
wait_msg /relocalizer/fitness "--field data" 120 || die "relocalizer never registered a scan"
grep -E "seed:|icp:" "$LOGD/B_reloc.log" | head -3 | sed 's/^/    /'

# ---- C ---------------------------------------------------------------------
if [ "$DO_VIEWER" -eq 1 ]; then
  start_stage C_viewer "source '$LIOENV' >/dev/null 2>&1; exec /usr/bin/python3 \
    /home/unitree/.scuba/viz/path_viewer.py --port $PORT" || die "stage C failed to start"
  waited=0
  until curl -sf -o /dev/null "http://localhost:$PORT/"; do
    [ "$waited" -ge 40 ] && die "path_viewer did not serve on port $PORT"
    sleep 2; waited=$((waited + 2))
  done
  printf '    %-34s serving after %ds\n' "port $PORT" "$waited"
fi

# ---- D ---------------------------------------------------------------------
if [ "$DO_NAV" -eq 1 ]; then
  start_stage D_nav2 "source '$LIOENV' >/dev/null 2>&1; exec ros2 launch \
    /home/unitree/.scuba/nav/launch/nav_dwb.launch.py" || die "stage D failed to start"
  # a costmap message proves lifecycle reached ACTIVE *and* map->body TF resolves
  wait_msg /local_costmap/costmap "--field header.stamp" 120 \
    || die "nav2 never activated (check TF map->camera_init->body)"
fi

# ---- E ---------------------------------------------------------------------
if [ "$DO_BRIDGE" -eq 1 ]; then
  start_stage E_bridge "source '$SDKENV' >/dev/null 2>&1; exec ros2 run g1_teleop nav_bridge" \
    || die "stage E failed to start"
  wait_log "$LOGD/E_bridge.log" "ENABLE=False" 30 \
    || die "bridge did not confirm ENABLE=False - refusing to continue"
fi

# ---- watchdog: nothing may ever reach the robot ----------------------------
# ---- live followers -------------------------------------------------------
# The stage logs are the source of truth either way; these just echo them here
# with a prefix so one terminal can show the bridge's DRY lines next to the
# relocalizer's fitness and controller_server's warnings.
for st in $TAIL_STAGES; do
  [ -f "$LOGD/$st.log" ] || continue
  short=$(echo "$st" | cut -d_ -f2)
  setsid bash -c "tail -n 0 -F '$LOGD/$st.log' 2>/dev/null | sed -u 's/^/[$short] /'" &
  PIDS+=("$!"); NAMES+=("tail_$short"); LOGS+=("$LOGD/$st.log")
done

APILOG=$LOGD/api_watchdog.log
: > "$APILOG"
setsid bash -c "source '$SDKENV' >/dev/null 2>&1; exec ros2 topic echo /api/sport/request" \
  > "$APILOG" 2>/dev/null &
PIDS+=("$!"); NAMES+=("api_watchdog"); LOGS+=("$APILOG")

cat <<EOF

================================================================
 UP. Nothing is being sent to the robot.
================================================================
 viewer      http://$IPADDR:$PORT

 F  send a goal (in a NEW terminal):
      source ~/.scuba/lio_ws/env.sh
      /usr/bin/python3 ~/.scuba/nav/send_goal.py X Y YAW_DEG --max-aborts 99

 G  log positions and commands:
      source ~/.scuba/sdk_teleop/env.sh
      /usr/bin/python3 ~/.scuba/nav/nav_log.py

 H  diagnose the jitter WHILE it is happening:
      source ~/.scuba/lio_ws/env.sh
      /usr/bin/python3 ~/.scuba/nav/diag_jitter.py --seconds 60

 every stage is logged; follow any of them from another terminal:
      tail -f $LOGD/E_bridge.log     # [DRY] DWB .. | would .. | robot ..
      tail -f $LOGD/B_reloc.log      # relocalizer rmse / inliers
      tail -f $LOGD/D_nav2.log       # controller_server + DWB warnings
      tail -f $LOGD/A_lio.log        # FAST-LIO + mount angle
      tail -f $LOGD/C_viewer.log     # path_viewer
 or restart with --tail bridge,reloc to see them in THIS terminal.

 Push the rack at roughly 0.2 m/s, and turn it when the plan turns:
 DWB samples a window around the CURRENT velocity, so a stationary
 robot measures the wrong thing.

 Ctrl-C here stops all stages.
================================================================
EOF

# ---- monitor ---------------------------------------------------------------
HB=0
while :; do
  sleep 5
  HB=$((HB + 1))
  line=""
  for ((i=0; i<${#PIDS[@]}; i++)); do
    if kill -0 "${PIDS[$i]}" 2>/dev/null; then
      line="$line ${NAMES[$i]}:ok"
    else
      line="$line ${NAMES[$i]}:DEAD"
    fi
  done
  if grep -q "api_id" "$APILOG" 2>/dev/null; then
    printf '\n!! SOMETHING WAS SENT TO THE ROBOT - /api/sport/request is not empty\n'
    printf '!! see %s. Stopping.\n' "$APILOG"
    grep -m5 -B2 -A6 "api_id" "$APILOG" | sed 's/^/   /'
    exit 1
  fi
  if [ -n "$TAIL_STAGES" ]; then
    # followers own the terminal: a \r status line would be chopped up by
    # their output, so print a full line every 30 s instead
    [ $((HB % 6)) -eq 0 ] && printf -- '--- %s  [api clean] ---\n' "$line"
  else
    printf '\r%s  [api clean]' "$line"
  fi
done
