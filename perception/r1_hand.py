"""The BrainCo Revo 2 hands, through Unitree's brainco_hand_service.

    python3 r1_hand.py eth10 right                 # read the hand (moves nothing)
    python3 r1_hand.py eth10 right open --go       # open it
    python3 r1_hand.py eth10 right close --go      # close it (the default grasp)
    python3 r1_hand.py eth10 right 0.5 1 0.7 0.7 0.7 0.7 --go     # any six positions

The service (github.com/unitreerobotics/brainco_hand_service) turns each hand's serial
link into DDS: rt/brainco/<left|right>/cmd and .../state, unitree_go MotorCmds_ /
MotorStates_, six motors in the order

    thumb, thumb_aux (the thumb's swing across the palm), index, middle, ring, pinky

with position q and speed dq both 0..1. Unitree's own test client cycles all-0, then
{0,1,1,1,1,1}, then all-1, and its README says the hand "makes a fist and opens" -- so 0 is
open and 1 is closed. That is read off their code, not seen on this robot: the first
thing to do with a hand is `open` and `close` and look.

The service must be running on the robot computer (once per boot):
    sudo ~/brainco_hand_service/bin/brainco_hand_server --network eth10
"""
import argparse
import sys
import threading
import time

FINGERS = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]
OPEN = [0.0] * 6
# A rim pinch: the thumb swung across and bent under, the fingers curled over. A start,
# to be tuned on the plate with `r1_hand.py ... <six numbers>`.
GRASP = [0.55, 1.0, 0.75, 0.75, 0.75, 0.75]


def topic(side, kind):
    return "rt/brainco/%s/%s" % (side, kind)


class Hand:
    """One Revo 2. `publisher`/`new_cmds` are injectable for tests."""

    def __init__(self, side, publisher=None, new_cmds=None, subscribe=True):
        if side not in ("left", "right"):
            raise ValueError("side is left or right")
        self.side = side
        self._lock = threading.Lock()
        self._state, self._t = None, 0.0
        if publisher is None:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
            publisher = ChannelPublisher(topic(side, "cmd"), MotorCmds_)
            publisher.Init()
        if new_cmds is None:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_, MotorCmds_
            try:
                from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_ as one
            except ImportError:                       # older SDKs: build it by hand
                def one():
                    return MotorCmd_(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0,
                                     reserve=[0, 0, 0])

            def new_cmds():
                return MotorCmds_(cmds=[one() for _ in range(6)])
        self.pub, self.new_cmds = publisher, new_cmds
        if subscribe:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
            self._sub = ChannelSubscriber(topic(side, "state"), MotorStates_)
            self._sub.Init(self._feed, 10)

    def _feed(self, msg):
        with self._lock:
            self._state, self._t = msg, time.time()

    def state(self):
        """The six finger positions 0..1, or None if the service has said nothing."""
        with self._lock:
            if self._state is None:
                return None
            return [float(m.q) for m in self._state.states[:6]]

    def age(self):
        with self._lock:
            return None if self._state is None else time.time() - self._t

    def build(self, q, speed=1.0):
        q = [min(max(float(v), 0.0), 1.0) for v in q]
        if len(q) != 6:
            raise ValueError("six finger positions, in the order %s" % ", ".join(FINGERS))
        msg = self.new_cmds()
        for m, v in zip(msg.cmds, q):
            m.q = v
            m.dq = float(min(max(speed, 0.05), 1.0))
        return msg

    def set(self, q, speed=1.0, repeat=5):
        """Send a hand pose a few times (it is one message per pose, not a stream)."""
        msg = self.build(q, speed)
        for _ in range(repeat):
            self.pub.Write(msg)
            time.sleep(0.02)

    def open(self, speed=1.0):
        self.set(OPEN, speed)

    def close(self, q=None, speed=0.6):
        self.set(q or GRASP, speed)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("interface", help="the robot computer's 192.168.123.x port (eth10)")
    ap.add_argument("side", choices=("left", "right"))
    ap.add_argument("pose", nargs="*", help="open | close | six numbers 0..1")
    ap.add_argument("--go", action="store_true", help="actually move the hand")
    args = ap.parse_args(argv)

    import r1_head
    r1_head.init_dds(args.interface)
    hand = Hand(args.side)
    t0 = time.time()
    while hand.state() is None and time.time() - t0 < 3.0:
        time.sleep(0.05)
    st = hand.state()
    if st is None:
        print("no state from %s in 3 s -- is the hand service running?\n"
              "    sudo ~/brainco_hand_service/bin/brainco_hand_server --network %s"
              % (topic(args.side, "state"), args.interface))
    else:
        print("%s hand now: %s" % (args.side, "  ".join("%s %.2f" % (f, v)
                                                      for f, v in zip(FINGERS, st))))
    if not args.pose:
        return 0
    if args.pose == ["open"]:
        q = OPEN
    elif args.pose == ["close"]:
        q = GRASP
    else:
        q = [float(v) for v in args.pose]
    print("target:   %s" % "  ".join("%s %.2f" % (f, v) for f, v in zip(FINGERS, q)))
    if not args.go:
        print("nothing moved -- add --go")
        return 0
    hand.set(q, speed=0.6)
    time.sleep(1.5)
    st = hand.state()
    if st:
        print("%s hand now: %s" % (args.side, "  ".join("%s %.2f" % (f, v)
                                                      for f, v in zip(FINGERS, st))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
