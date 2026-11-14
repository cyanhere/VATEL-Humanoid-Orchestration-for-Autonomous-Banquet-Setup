#!/usr/bin/env python3
"""Camera-closed-loop edge servoing: search, square up, stop at a standoff.

Extracted from edge_approach_test.py after that script was validated on the
robot, so the mission and the stage test run the SAME motion code.

Everything here was forced by measurements on this robot:

  * nav_bridge slews toward the commanded velocity at max_lin_acc, so a SHORT
    burst never reaches a speed the gait honours: at 0.35 m/s^2 a 0.25 s burst
    peaks at 0.0875 m/s, the gait ignores anything under ~0.20, and the robot
    twitches without moving. Hence continuous drives, not bursts.
  * a discrete move cannot be smaller than v^2/accel (0.138 m at the default
    accel, 0.024 m at 2.0), so the approach stops SHORT by the coast distance
    v^2/(2a) and lets the ramp-down finish the move.
  * turn-per-command varies 40-60% with gait phase (the same 0.40 s command
    gave 1.68, 1.91 and 4.65 deg), so alignment steps toward a zero crossing
    instead of sizing a correction. Measured residual: 0.67 deg.
  * the camera lags the motion and the robot keeps settling afterwards (an
    in-loop reading and one straight after differed by 2.25 deg with nothing
    commanded), so nothing is believed until it has settled.
  * edge_x is recomputed each frame from the drop the camera MEASURES, so it
    tracks stance changes; a fixed depth target does not (drop was seen at
    0.407, 0.424 and 0.455 in one session - 10 cm of standoff).
"""
import math
import inspect
import time


def adaptive_step(rate=10.2, min_dur=0.35, max_dur=1.2, aim=1.0):
    """Step duration from the error still to cover: dur = |err| / rate.

    Measured on this robot with max_ang_acc 2.0, over 22 steps of 0.40 s at
    0.40 rad/s commanded:

        per step   mean 4.08 deg, sd 1.43, min 0.74, max 5.98
        rate       10.21 deg/s  (commanded 22.9 deg/s, so gain 0.45)

    MIN_DUR exists because a short command may do nothing at all - 0.15 s
    measured 0.12 deg - so every step has to be long enough to actually
    move the robot. It also bounds the overshoot past zero, since a small
    error still costs one minimum step: 0.35 s x 10.2 deg/s = 3.6 deg.

    MAX_DUR exists because the rate FALLS as the command lengthens -
    turn_calib measured 11.6 deg/s at 0.60 s but 9.6 deg/s at 0.90 s - so
    one huge command is both slower per second and far less repeatable than
    a few capped ones.
    """
    def f(err):
        return max(min_dur, min(max_dur, abs(err) * aim / max(rate, 1e-6)))
    return f


def step_to_zero(read, turn, tol, step_dur, settle, max_steps=12,
                 log=print, abort=None, autoflip=True, flip=False,
                 flip_min_move=0.3, flip_window=3, wait=None,
                 stop_on_cross=True, fine_after_cross=False, fine_dur=None,
                 fine_min_move=0.5):
    """Step toward zero until the signed error CHANGES SIGN.

    read()        -> the signed error now, or None if unavailable
    turn(d, dur)  -> turn for `dur` seconds in direction d (+1 / -1)

    Deliberately dumb, and that is the point. Turn-per-command on this robot
    varies 40-60% with gait phase - the same 0.40 s command measured 1.68,
    1.91 and 4.65 deg - so anything that tries to SIZE a correction is
    guessing. Crossing zero only needs the direction to be right.

    Two things learned the hard way, both of which this gets right:
      * CROSSING IS CHECKED FIRST. Overshooting through zero also grows
        |error|, so testing "did it get worse" first mistakes a successful
        crossing for an inverted sign convention. Seen on the robot:
        -2.66 -> +4.23 was flagged INVERTED when it had in fact crossed.
      * Nothing is believed until `settle` after the robot stops. A reading
        taken immediately and one taken a moment later differed by 2.25 deg
        with nothing commanded in between.

    Returns True if it reached tolerance or crossed zero.

    fine_after_cross: once the sign HAS flipped, stop sizing the step from the
    error and take the smallest committed motion available instead - `fine_dur`
    seconds at the caller's fine rate. Sizing from the error is only sensible
    while the error is large; past the crossing the job is "nudge back inside
    the window", and a proportional step there just overshoots the other way.
    Needs a turn() that accepts a third `fine` argument; a 2-argument turn()
    disables it with a log line rather than raising.
    """
    abort = abort or (lambda: False)
    if fine_after_cross:
        try:
            nargs = len(inspect.signature(turn).parameters)
        except (TypeError, ValueError):
            nargs = 2
        if nargs < 3:
            log('    turn() takes %d args, not 3 - fine-after-cross OFF'
                % nargs)
            fine_after_cross = False
    if fine_dur is None:
        fine_dur = step_dur if not callable(step_dur) else 0.25
    # A plain sleep is wrong for a ROS caller: nothing services callbacks
    # while it blocks, so the TF buffer never updates and read() keeps
    # returning the SAME heading. The sign then never flips and it steps
    # until max_steps. Callers that need spinning pass their own wait.
    wait = wait or time.sleep
    err = read()
    if err is None:
        log('    nothing to align to')
        return False
    s0 = 1.0 if err > 0 else -1.0
    sign = -1.0 if flip else 1.0
    flipped = False
    crossed = False
    adaptive = callable(step_dur)
    log('    start %+.2f, stepping %s until the sign flips'
        % (err, 'adaptively' if adaptive else '%.2f s' % step_dur))
    for i in range(max_steps):
        if abort():
            return False
        if abs(err) <= tol:
            log('    %+.2f is inside %.2f - not turning' % (err, tol))
            return True
        d = sign * (1.0 if err > 0 else -1.0)
        fine = crossed and fine_after_cross
        dur = (fine_dur if fine
               else (step_dur(err) if adaptive else step_dur))
        log('      step %d: %+.2f -> turning %+.0f for %.2f s%s'
            % (i + 1, err, d, dur, '  [fine]' if fine else ''))
        if fine:
            turn(d, dur, True)
        else:
            turn(d, dur)
        wait(settle)
        new = read()
        if new is None:
            log('    lost the reference while turning')
            return False
        log('      -> %+.2f  (moved %+.2f)' % (new, new - err))
        if fine and abs(new - err) < fine_min_move:
            # the fine rate is BELOW the measured gait floor in duration terms,
            # so it can fail to commit at all. Noticing costs one step;
            # not noticing burns every remaining step turning nothing.
            log('    fine step moved only %+.2f (< %.2f) - the gait did not '
                'commit at the fine rate; reverting to the normal rate'
                % (new - err, fine_min_move))
            fine_after_cross = False
        if (1.0 if new > 0 else -1.0) != s0:
            log('    crossed zero: %+.2f -> %+.2f' % (err, new))
            if stop_on_cross or abs(new) <= tol:
                return True
            # stop_on_cross False: a crossing alone is not success. Keep
            # stepping until the error is actually inside tol.
            #
            # This works because the STEP IS SMALLER THAN THE WINDOW: tol 2.0
            # is a 4 deg window and the shortest step that moves is ~2.5-3.5
            # deg, so a step cannot jump the window - it must land inside it,
            # if not on this step then after one oscillation. d follows the
            # sign of err, so the direction reverses on its own.
            #
            # The convention is now PROVEN (it crossed), so suppress autoflip:
            # an overshoot also grows |error| and would otherwise be mistaken
            # for an inverted sign.
            flipped = True
            crossed = True
            err = new
            continue
        # The flip test compares against the move that actually happened,
        # not a fixed margin. A fixed 1.0 deg silently failed whenever a step
        # moved less than that - at 2 deg/s a 0.4 s step is 0.8 deg, so a
        # wrong direction was never detected and it walked away for ever.
        # The weakest steps measured on this robot were 0.25-1.57 deg, right
        # in that band. Also allow the check for the first few steps, because
        # step 1 can do nothing at all depending on gait phase.
        moved = abs(new - err)
        if (autoflip and not flipped and i < flip_window
                and moved >= flip_min_move and abs(new) > abs(err)):
            sign = -sign
            flipped = True
            log('      wrong direction (moved %.2f, error %+.2f -> %+.2f) '
                '- flipping the sign convention' % (moved, err, new))
        err = new
    log('    !! %d steps without crossing (now %+.2f)' % (max_steps, err))
    return False


class Params(object):
    """Defaults matching the validated edge_approach_test run."""
    v_fwd = 0.22
    w_turn = 0.40
    accel = 2.0            # nav_bridge max_lin_acc, MUST match the launch
    ang_accel = 2.0        # nav_bridge max_ang_acc
    tol = 0.03             # m
    ang_tol = 1.0          # deg
    step_dur = 0.4
    cam_delay = 0.5
    max_steps = 12
    refine = 0
    poll = 0.05
    settle = 0.3
    angle_avg = 8
    settle_tol = 0.6
    settle_timeout = 15.0
    max_travel = 1.20      # per side
    search_cap = 1.00
    stall_after = 0.20

    def __init__(self, **kw):
        for k, v in kw.items():
            if not hasattr(Params, k):
                raise KeyError('unknown servo param %r' % k)
            setattr(self, k, v)

    def min_move(self):
        return self.v_fwd ** 2 / max(self.accel, 1e-6)

    def coast(self):
        return self.v_fwd ** 2 / (2.0 * max(self.accel, 1e-6))

    def min_turn_deg(self):
        return math.degrees(self.w_turn ** 2 / max(self.ang_accel, 1e-6))


class EdgeServo(object):
    """Drives one side of one table, closing the loop on the plane detector."""

    def __init__(self, send, spin, plane, log, p=None, abort=None,
                 dry_run=False):
        self.send = send
        self.spin = spin
        self.plane = plane
        self.log = log
        self.p = p or Params()
        self.abort = abort or (lambda: False)
        self.dry = dry_run
        self.travel = 0.0
        self.seen = 0.0

    def reset(self):
        self.travel = 0.0
        self.seen = 0.0

    # ---------------------------------------------------------------- reads
    def _field(self, key, n=1):
        got = []
        for _ in range(max(1, n)):
            d = self.plane.read()
            if d and d.get('ok') and d.get(key) is not None:
                got.append(float(d[key]))
            if n > 1:
                time.sleep(0.1)
        return (sum(got) / len(got)) if got else None

    def edge_x(self):
        return self._field('edge_x')

    def angle(self, n=None):
        return self._field('edge_angle',
                           self.p.angle_avg if n is None else n)

    def visible(self):
        return self.plane.fresh_ok() is not None

    def settled_angle(self):
        t0 = time.time()
        buf = []
        while time.time() - t0 < self.p.settle_timeout:
            if self.abort():
                break
            v = self.angle(1)
            if v is not None:
                buf.append(v)
                if len(buf) > 6:
                    buf.pop(0)
                if len(buf) == 6 and max(buf) - min(buf) <= self.p.settle_tol:
                    return sum(buf) / len(buf)
            time.sleep(0.15)
        return (sum(buf[-6:]) / len(buf[-6:])) if buf else None

    # --------------------------------------------------------------- motion
    def halt(self):
        for _ in range(10):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def drive(self, vx, wz, why, pred=None, cap=None):
        """Continuous motion until pred() or cap metres.
        ALWAYS returns (reason, commanded_m, measured_m_or_None)."""
        cap = min(cap if cap is not None else 1e9,
                  max(0.0, self.p.max_travel - self.travel))
        if cap <= 0.0:
            self.log('      %s REFUSED: travel budget spent (%.2f of %.2f m)'
                     % (why, self.travel, self.p.max_travel))
            return 'cap', 0.0, None
        self.log('      drive %-18s vx %+.2f wz %+.2f  cap %.3f m'
                 % (why, vx, wz, cap))
        if self.dry:
            self.log('      [dry run] nothing published')
            return 'dry', 0.0, None
        x0 = self.edge_x()
        t0 = last = time.time()
        moved = 0.0
        reason = 'cap'
        try:
            while True:
                if self.abort():
                    reason = 'abort'
                    break
                self.send(vx, wz)
                self.spin(0.005)
                time.sleep(0.02)
                now = time.time()
                moved = abs(vx) * (now - t0)
                if moved >= cap:
                    break
                if now - last >= self.p.poll:
                    last = now
                    if pred is not None and pred():
                        reason = 'target'
                        break
        finally:
            self.halt()
            time.sleep(self.p.settle)
        self.travel += moved
        x1 = self.edge_x()
        seen = None
        if x0 is not None and x1 is not None:
            seen = abs(x1 - x0)
            self.seen += seen
            self.log('      commanded %.3f m   edge_x %.3f -> %.3f (%.3f)'
                     % (moved, x0, x1, seen))
        else:
            self.log('      commanded %.3f m   (edge_x not in frame)' % moved)
        return reason, moved, seen

    def turn_step(self, wz, dur):
        if self.dry:
            self.log('      [dry run] would turn wz %+.2f for %.2f s'
                     % (wz, dur))
            return
        t0 = time.time()
        while time.time() - t0 < dur and not self.abort():
            self.send(0.0, wz)
            self.spin(0.005)
            time.sleep(0.02)
        self.halt()

    # --------------------------------------------------------------- stages
    def search(self, cap=None):
        if self.visible():
            self.log('    tabletop already in frame')
            return True
        why, cmd, _ = self.drive(self.p.v_fwd, 0.0, 'search',
                                 pred=self.visible,
                                 cap=cap if cap is not None
                                 else self.p.search_cap)
        if self.visible():
            self.log('    acquired after %.2f m commanded' % cmd)
            return True
        self.log('    !! no tabletop after %.2f m - stopping rather than '
                 'driving on' % cmd)
        return False

    def align(self, flip=False, autoflip=True):
        ok = self._align_once(self.p.step_dur, flip, autoflip)
        dur = self.p.step_dur
        for r in range(self.p.refine):
            if not ok:
                break
            dur /= 2.0
            ang = self.settled_angle()
            if ang is None or abs(ang) <= self.p.ang_tol:
                break
            self.log('    refine %d/%d: step %.2f s, now %+.2f deg'
                     % (r + 1, self.p.refine, dur, ang))
            ok = self._align_once(dur, flip, autoflip)
        return ok

    def _align_once(self, step_dur, flip, autoflip):
        return step_to_zero(
            read=lambda: self.angle(),
            turn=lambda d, dur: self.turn_step(d * self.p.w_turn, dur),
            tol=self.p.ang_tol, step_dur=step_dur,
            settle=self.p.cam_delay, max_steps=self.p.max_steps,
            log=self.log, abort=self.abort, autoflip=autoflip, flip=flip)

    def approach(self, target):
        lead = self.p.coast()
        for i in range(self.p.max_steps):
            if self.abort():
                return False
            x = self.edge_x()
            if x is None:
                self.log('    lost the plane, re-acquiring')
                if not self.search(cap=0.20):
                    return False
                continue
            err = x - target
            self.log('    %d  edge_x %.3f  err %+.3f' % (i + 1, x, err))
            if abs(err) <= self.p.tol:
                self.log('    within %.0f mm' % (1000 * self.p.tol))
                return True
            if abs(err) <= lead + self.p.tol:
                self.log('    remaining %.3f m is inside the %.3f m coast - '
                         'as close as this accel allows' % (abs(err), lead))
                return True
            v = self.p.v_fwd if err > 0 else -self.p.v_fwd
            stop_at = target + (lead if v > 0 else -lead)
            if v > 0:
                pred = lambda: (self.edge_x() or 1e9) <= stop_at
            else:
                pred = lambda: (self.edge_x() or -1e9) >= stop_at
            why, cmd, seen = self.drive(v, 0.0, 'approach %+.3f' % err,
                                        pred=pred, cap=abs(err) + 0.15)
            if why == 'cap' and cmd <= 0.0:
                self.log('    !! out of travel budget (%.2f of %.2f m)'
                         % (self.travel, self.p.max_travel))
                return False
            if (cmd >= self.p.stall_after and seen is not None
                    and seen < 0.03):
                self.log('    !! STALL: commanded %.2f m, edge_x moved %.3f.'
                         % (cmd, seen))
                self.log('    !! Raise nav_bridge max_lin_acc.')
                return False
        self.log('    !! approach did not converge')
        return False

    def do_side(self, target, flip=False):
        """creep until the plane appears -> align on it -> optionally trim.

        The heading is NOT set here. It is set by send_goal.py --finish-yaw on
        the way in, which is the first of the two alignments: Nav2 ignores a
        pose's yaw (goal_checker.yaw_goal_tolerance is 3.00 rad), so the yaw
        has to be stepped separately, and send_goal already does exactly that.

        target None skips the final trim, so the robot stops wherever the
        tabletop first came into the detector's band. target set trims to that
        measured distance, which is the only way to actually land on the
        standoff rather than on whatever range the camera happened to acquire
        at.
        """
        if not self.search():
            return False
        if not self.align(flip=flip):
            self.log('    align did not converge - continuing off-square')
        if target is None:
            x = self.edge_x()
            self.log('    no trim requested - stopping at edge_x %s'
                     % ('unknown' if x is None else '%.3f m' % x))
            return True
        return self.approach(target)
