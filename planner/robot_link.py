"""
The robot's end of VATEL's "Send plan" and "Start mission" buttons: a small HTTP service
that runs on the robot (or on the computer driving it).

    python robot_link.py                       # listens on :8780, keeps plans in ./missions/
    python robot_link.py --on-start "ros2 launch vatel_mission run.launch.py map:={yaml} goals:={goals}"

Then in the planner: Settings -> Robot link -> http://<robot>:8780.

    POST /plan            vatel-plan/0.1: the static map JSON plus the three files map_server
                          loads (map .pgm as base64, its .yaml, the named goals)
    POST /mission/start   start laying plates with the plan last received
    POST /mission/stop
    GET  /status          {"state": "idle" | "ready" | "running", "plan": {...}}

Each plan is written to missions/<time>_<name>/ as name.pgm, name.yaml, name_goals.yaml and
plan.json -- exactly what `ros2 run nav2_map_server map_server` and a mission runner read.

WHAT THIS DOES NOT DO
    Drive the robot. Starting a mission records it and, if --on-start is given, runs that
    command with {dir}, {yaml}, {goals} and {plan} filled in. That command -- the nav stack
    plus the plate-laying behaviour -- is where the real robot plugs in. Until it exists,
    this is the seam: the planner's side of the hand-over is complete and testable now.
"""

import argparse
import base64
import datetime
import json
import re
import shlex
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

SCHEMA = "vatel-plan/0.1"
MAX_BODY = 64 * 1024 * 1024


class Link:
    def __init__(self, out, on_start=None, robot="Unitree R1"):
        self.out = Path(out)
        self.on_start = on_start
        self.robot = robot
        self.lock = threading.Lock()
        self.state = "idle"
        self.plan = None          # summary of the last plan received
        self.proc = None

    def receive(self, doc):
        if doc.get("schema") != SCHEMA:
            raise ValueError("expected schema %s, got %r" % (SCHEMA, doc.get("schema")))
        ros = doc.get("ros") or {}
        for k in ("name", "pgm_base64", "yaml", "goals_yaml"):
            if not ros.get(k):
                raise ValueError("plan has no ros.%s" % k)
        name = re.sub(r"[^a-z0-9_-]+", "-", str(ros["name"]).lower()).strip("-") or "plan"
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        d = self.out / ("%s_%s" % (stamp, name))
        d.mkdir(parents=True, exist_ok=True)
        pgm = base64.b64decode(ros["pgm_base64"])
        if not pgm.startswith(b"P5"):
            raise ValueError("map image is not a binary PGM")
        (d / (name + ".pgm")).write_bytes(pgm)
        (d / (name + ".yaml")).write_text(ros["yaml"])
        (d / (name + "_goals.yaml")).write_text(ros["goals_yaml"])
        (d / "plan.json").write_text(json.dumps(doc.get("static_map", {}), indent=2))
        totals = (doc.get("static_map") or {}).get("totals", {})
        with self.lock:
            if self.state == "running":
                raise RuntimeError("a mission is running; stop it before sending a new plan")
            self.plan = {"dir": str(d), "name": name, "received_at": stamp,
                         "event": doc.get("event"), "tables": totals.get("tables"),
                         "covers": totals.get("covers"), "unreachable": totals.get("unreachable"),
                         "yaml": str(d / (name + ".yaml")), "goals": str(d / (name + "_goals.yaml")),
                         "plan": str(d / "plan.json")}
            self.state = "ready"
        print("plan received: %s tables, %s covers -> %s" % (totals.get("tables"), totals.get("covers"), d),
              flush=True)
        return {"state": self.state, "saved_to": str(d)}

    def start(self):
        with self.lock:
            if not self.plan:
                raise RuntimeError("no plan received yet")
            if self.state == "running":
                return {"state": self.state}
            self.state = "running"
            plan = dict(self.plan)
        print("MISSION START: %s (%s covers)" % (plan["name"], plan["covers"]), flush=True)
        if self.on_start:
            cmd = self.on_start.format(dir=plan["dir"], yaml=plan["yaml"], goals=plan["goals"],
                                       plan=plan["plan"])
            print("running: " + cmd, flush=True)
            self.proc = subprocess.Popen(cmd if sys.platform == "win32" else shlex.split(cmd))
            threading.Thread(target=self._wait, daemon=True).start()
        return {"state": "running"}

    def _wait(self):
        code = self.proc.wait()
        with self.lock:
            self.state = "ready"
        print("mission command finished with code %s" % code, flush=True)

    def stop(self):
        with self.lock:
            proc, self.proc = self.proc, None
            self.state = "ready" if self.plan else "idle"
        if proc and proc.poll() is None:
            proc.terminate()
        print("MISSION STOP", flush=True)
        return {"state": self.state}

    def status(self):
        with self.lock:
            return {"service": "vatel-robot-link", "robot": self.robot, "state": self.state,
                    "plan": self.plan}


def make_handler(link, verbose=False):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.end_headers()

        def do_GET(self):
            if self.path.split("?")[0] == "/status":
                return self._json(link.status())
            self._json({"error": "not found"}, 404)

        def do_POST(self):
            path = self.path.split("?")[0]
            try:
                n = int(self.headers.get("Content-Length") or 0)
                if n > MAX_BODY:
                    return self._json({"error": "plan too large"}, 413)
                body = json.loads(self.rfile.read(n) or b"{}")
                if path == "/plan":
                    return self._json(link.receive(body))
                if path == "/mission/start":
                    return self._json(link.start())
                if path == "/mission/stop":
                    return self._json(link.stop())
                self._json({"error": "not found"}, 404)
            except (ValueError, RuntimeError) as e:
                self._json({"error": str(e), "state": link.status()["state"]}, 409)

        def log_message(self, fmt, *a):
            if verbose:
                sys.stderr.write("%s  %s\n" % (self.address_string(), fmt % a))

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("--port", type=int, default=8780)
    ap.add_argument("--host", default="0.0.0.0", help="127.0.0.1 to accept this machine only")
    ap.add_argument("--out", default="missions", help="where received plans are kept")
    ap.add_argument("--on-start", help="command to run on Start mission; {dir} {yaml} {goals} {plan}")
    ap.add_argument("--robot", default="Unitree R1")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)
    link = Link(args.out, args.on_start, args.robot)
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(link, args.verbose))
    print("VATEL robot link: http://%s:%d  (plans -> %s)" % (args.host, args.port, Path(args.out).resolve()),
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
