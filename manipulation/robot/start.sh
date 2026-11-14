#!/bin/bash
# Opens the pick programs in one tmux session (window "pick", 4 panes). Nothing is started that moves the robot:
# the server waits for `take` / `home`, perception waits for the server; tests are typed by you.
#   ./start.sh            then:  tmux attach -t pick   (Ctrl-b + arrow keys between panes; Ctrl-b d detaches)
#   ./start.sh --mock     the same with the simulated arm (no robot)
cd "$(dirname "$0")"
MOCK=""
[ "$1" = "--mock" ] && MOCK="--mock"
if ! command -v tmux >/dev/null; then
  echo "tmux is not installed: open 4 terminals in $(pwd) and run"
  echo "  1: python3 server.py $MOCK      2: python3 perception.py      3: (tests)      4: python3 monitor.py"
  exit 1
fi
if tmux has-session -t pick 2>/dev/null; then
  echo "a 'pick' session already exists: tmux attach -t pick"
  exit 0
fi
tmux new-session -d -s pick -n pick "python3 server.py $MOCK; bash"
tmux split-window -h -t pick "sleep 2; echo 'perception: type  python3 perception.py --expect 0.36 0.06'; bash"
tmux split-window -v -t pick:0.0 "echo 'tests: python3 run_test.py push   |   python3 tune.py check'; bash"
tmux split-window -v -t pick:0.1 "sleep 3; python3 monitor.py; bash"
tmux select-pane -t pick:0.0
echo "started: tmux attach -t pick   (server in the top-left pane: type take, then home with the table away)"
