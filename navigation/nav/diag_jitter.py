#!/usr/bin/env python3
"""Decide WHICH of the four suspected jitter causes is actually firing.

SUBSCRIBE-ONLY. This node creates no publisher, no action client and no service
client, so it cannot move the robot. Safe next to a live run or a dry run
(nav_bridge with enable=false).

    /usr/bin/python3 ~/.scuba/nav/diag_jitter.py --seconds 60

Run it WHILE the robot is jittering in front of the unmapped obstacle. Each
hypothesis gets REAL / not seen / no data plus the number that decided it, so a
config change is only made for a cause that measured true.

  H1 churn        the global plan keeps being replaced, alternating sides
  H2 tortuosity   the plan is a 45 deg staircase and PathAlign chases it
  H3 saturation   BaseObstacle scores nearly the same for every sample, so it
                  decides nothing and some other critic picks the direction
  H4 slew         the bridge cannot execute a decision that flips this fast

H4 is simulated from the RECORDED /cmd_vel using the bridge's own constants, so
it needs no change to the bridge and works whether or not the bridge is running.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np

# ---- nav_bridge_node.py constants, mirrored (keep in sync if that file changes)
BR_MAX_VX = 0.30
BR_MAX_VY = 0.20
BR_MAX_VYAW = 0.40
BR_DEADBAND = 0.02
BR_MAX_LIN_ACC = 0.35
BR_MAX_ANG_ACC = 0.70
BR_RATE_HZ = 20.0
BR_CMD_TIMEOUT = 0.4
BR_STALE_DECEL = 3.0

# DWB validity thresholds, from nav2_dwb.yaml
MIN_SPEED_XY = 0.20
MIN_SPEED_THETA = 0.30


def clamp(v, lim):
    return max(-lim, min(lim, v))


def approach(cur, target, max_step):
    d = target - cur
    if d > max_step:
        return cur + max_step
    if d < -max_step:
        return cur - max_step
    return target


def arclen(xy):
    if len(xy) < 2:
        return np.zeros(len(xy))
    d = np.hypot(*np.diff(xy, axis=0).T)
    return np.concatenate([[0.0], np.cumsum(d)])


def point_at(xy, s_target):
    """Linear interpolation along the polyline at arc length s_target."""
    s = arclen(xy)
    if s[-1] <= 1e-9:
        return xy[0]
    s_target = min(max(s_target, 0.0), s[-1])
    i = int(np.searchsorted(s, s_target))
    i = min(max(i, 1), len(xy) - 1)
    f = (s_target - s[i - 1]) / max(s[i] - s[i - 1], 1e-9)
    return xy[i - 1] + f * (xy[i] - xy[i - 1])


def resample(xy, n):
    s = arclen(xy)
    if s[-1] <= 1e-9:
        return np.repeat(xy[:1], n, axis=0)
    return np.array([point_at(xy, t) for t in np.linspace(0.0, s[-1], n)])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ----------------------------------------------------------------- H2 geometry
def tortuosity(xy, max_len=2.5):
    """Heading change per metre, direction reversals per metre, staircase share.

    Computed on the RAW planner output: the 45 deg quantization is exactly what
    is under test, so smoothing here first would hide it.
    """
    if len(xy) < 4:
        return None
    seg = np.diff(xy, axis=0)
    L = np.hypot(seg[:, 0], seg[:, 1])
    keep = L > 1e-6
    seg, L = seg[keep], L[keep]
    if len(seg) < 3:
        return None
    cum = np.cumsum(L)
    n = min(len(seg), int(np.searchsorted(cum, max_len)) + 1)
    seg, L = seg[:n], L[:n]
    used = float(L.sum())
    if used < 0.3:
        return None
    th = np.arctan2(seg[:, 1], seg[:, 0])
    d = wrap(np.diff(th))
    turn_density = float(np.abs(d).sum() / used)
    big = d[np.abs(d) > math.radians(5.0)]
    rev = int(np.count_nonzero(np.diff(np.sign(big)))) if len(big) > 1 else 0
    stair = (float(np.mean(np.abs(np.abs(d) - math.radians(45.0))
                           < math.radians(10.0))) if len(d) else 0.0)
    return turn_density, rev / used, stair, used


def side_offset(xy, s_at=1.0, s_chord=2.0):
    """Signed lateral offset of the plan at s_at, about the start->s_chord chord.

    Sign tells which side of its own chord the plan bulges. A sign flip between
    consecutive plans is the left/right detour alternation in H1.
    """
    s = arclen(xy)
    if len(xy) < 3 or s[-1] < 0.5:
        return None
    c = point_at(xy, min(s_chord, s[-1])) - xy[0]
    n = math.hypot(c[0], c[1])
    if n < 1e-6:
        return None
    c = c / n
    p = point_at(xy, min(s_at, s[-1])) - xy[0]
    return float(c[0] * p[1] - c[1] * p[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0,
                    help='0 = until Ctrl-C')
    ap.add_argument('--plan-topic', default='/received_global_plan',
                    help='the plan DWB actually got (not /plan)')
    ap.add_argument('--eval-topic', default='/evaluation')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--fpd', type=float, default=0.2,
                    help='a PathAlign.forward_point_distance to evaluate')
    ap.add_argument('--fpd-alt', type=float, default=0.7,
                    help='a second one, for comparison')
    ap.add_argument('--nav-config',
                    default='~/.scuba/nav/config/nav2_dwb.yaml',
                    help='read the CONFIGURED forward_point_distance from here '
                         'so the report cannot claim the wrong value')
    ap.add_argument('--plan-eps', type=float, default=0.10,
                    help='m; plans differing by more than this count as changed')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    import tf2_ros
    from nav_msgs.msg import Path
    from geometry_msgs.msg import Twist
    from dwb_msgs.msg import LocalPlanEvaluation

    class Diag(Node):
        def __init__(self):
            super().__init__('diag_jitter')
            # H1/H2
            self.plans = 0
            self.changes = 0
            self.prev_sig = None
            self.prev_side = None
            self.side_flips = 0
            self.tort = []            # (turn_density, rev_per_m, stair, used)
            self.cur_plan = None
            # H2 forward-point swing
            self.fp_prev = {a.fpd: None, a.fpd_alt: None}
            self.fp_swing = {a.fpd: [], a.fpd_alt: []}
            # H3
            self.ev_cycles = 0
            self.ev_full = 0
            self.critic_spread = {}   # name -> [spread per cycle]
            self.decider = {}         # name -> cycles where it had most spread
            self.best_hist = []       # (t, vx, wz)
            self.legal_turnonly = 0
            self.legal_walk = 0
            self.legal_total = 0
            self.samples_total = 0
            self.rejected_by = {}
            self.partial_cycles = 0
            # H4
            self.cmds = []            # (t, vx, vy, wz)

            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Path, a.plan_topic, self.on_plan, q)
            self.create_subscription(LocalPlanEvaluation, a.eval_topic,
                                     self.on_eval, q)
            self.create_subscription(Twist, a.cmd_topic, self.on_cmd, 10)
            self.create_timer(2.0, self.tick)
            self.t0 = time.time()
            self.get_logger().info(
                'subscribe-only. plan=%s eval=%s cmd=%s  fpd %.2f vs %.2f'
                % (a.plan_topic, a.eval_topic, a.cmd_topic, a.fpd, a.fpd_alt))

        # ---------------- H1 + H2
        def on_plan(self, m):
            xy = np.array([[p.pose.position.x, p.pose.position.y]
                           for p in m.poses], dtype=np.float64)
            if len(xy) < 4:
                return
            self.plans += 1
            self.cur_plan = xy
            sig = resample(xy, 20)
            if self.prev_sig is not None:
                if float(np.max(np.hypot(*(sig - self.prev_sig).T))) > a.plan_eps:
                    self.changes += 1
                    sd = side_offset(xy)
                    if (sd is not None and self.prev_side is not None
                            and abs(sd) > 0.05 and abs(self.prev_side) > 0.05
                            and sd * self.prev_side < 0):
                        self.side_flips += 1
                    if sd is not None:
                        self.prev_side = sd
            else:
                self.prev_side = side_offset(xy)
            self.prev_sig = sig
            t = tortuosity(xy)
            if t:
                self.tort.append(t)

        def robot_xy_yaw(self):
            try:
                tr = self.buf.lookup_transform(a.map_frame, a.base_frame,
                                               rclpy.time.Time())
            except Exception:
                return None
            q = tr.transform.rotation
            t = tr.transform.translation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return np.array([t.x, t.y]), yaw

        def fp_heading(self, xy, rxy, d):
            """Heading from the robot to the point d metres further along the
            plan than its own projection: what PathAlign is told to face."""
            s = arclen(xy)
            i = int(np.argmin(np.hypot(*(xy - rxy).T)))
            tgt = point_at(xy, min(s[i] + d, s[-1]))
            v = tgt - rxy
            if math.hypot(v[0], v[1]) < 1e-6:
                return None
            return math.atan2(v[1], v[0])

        # ---------------- H3
        def on_eval(self, m):
            self.ev_cycles += 1
            names = []
            for s in m.twists:
                if s.total >= 0 and len(s.scores) > len(names):
                    names = [c.name for c in s.scores]
            if not names:
                return
            full, totals, legal = [], [], 0
            for s in m.twists:
                self.samples_total += 1
                if s.total < 0:
                    who = s.scores[0].name if s.scores else '?'
                    self.rejected_by[who] = self.rejected_by.get(who, 0) + 1
                    continue
                legal += 1
                v = s.traj.velocity
                if abs(v.x) < MIN_SPEED_XY:
                    self.legal_turnonly += 1
                else:
                    self.legal_walk += 1
                if len(s.scores) == len(names):
                    full.append([float(c.raw_score) * float(c.scale)
                                 for c in s.scores])
                    totals.append(float(s.total))
            self.legal_total += legal
            if len(full) < 3:
                self.partial_cycles += 1
                return
            self.ev_full += 1
            A = np.array(full)                     # samples x critics
            T = np.array(totals)
            # DWB minimises the total, so the contenders are the SMALLEST totals.
            # max-min over every sample is outlier-dominated: a handful of awful
            # samples beside the obstacle inflate the range without saying what
            # separates the candidates that could actually win. Spread among the
            # top few is what decides the outcome.
            top = A[np.argsort(T)[:max(3, min(10, len(T)))]]
            spreads = {}
            for j, nm in enumerate(names):
                col = A[:, j]
                mean = float(np.mean(np.abs(col)))
                sp = float(np.max(col) - np.min(col))
                tcol = top[:, j]
                sp_top = float(np.max(tcol) - np.min(tcol))
                spreads[nm] = sp_top
                self.critic_spread.setdefault(nm, []).append(
                    (sp, sp / mean if mean > 1e-9 else 0.0, sp_top))
            win = max(spreads, key=lambda k: spreads[k])
            self.decider[win] = self.decider.get(win, 0) + 1
            if 0 <= m.best_index < len(m.twists):
                v = m.twists[m.best_index].traj.velocity
                self.best_hist.append((time.time(), float(v.x), float(v.theta)))

        # ---------------- H4
        def on_cmd(self, m):
            self.cmds.append((time.time(), m.linear.x, m.linear.y, m.angular.z))

        def tick(self):
            el = time.time() - self.t0
            p = self.robot_xy_yaw()
            if p is not None and self.cur_plan is not None:
                for d in (a.fpd, a.fpd_alt):
                    h = self.fp_heading(self.cur_plan, p[0], d)
                    if h is not None:
                        if self.fp_prev[d] is not None:
                            self.fp_swing[d].append(abs(wrap(h - self.fp_prev[d])))
                        self.fp_prev[d] = h
            self.get_logger().info(
                '%5.1fs  plans %d (changed %d, side-flips %d)  eval %d (usable %d)'
                '  cmd %d' % (el, self.plans, self.changes, self.side_flips,
                              self.ev_cycles, self.ev_full, len(self.cmds)))
            if a.seconds > 0 and el >= a.seconds:
                raise SystemExit(0)

    # ----- a faster sampler for the forward-point swing than the 2 s log tick
    rclpy.init()
    n = Diag()

    def fast():
        p = n.robot_xy_yaw()
        if p is None or n.cur_plan is None:
            return
        for d in (a.fpd, a.fpd_alt):
            h = n.fp_heading(n.cur_plan, p[0], d)
            if h is None:
                continue
            if n.fp_prev[d] is not None:
                n.fp_swing[d].append(abs(wrap(h - n.fp_prev[d])))
            n.fp_prev[d] = h
    n.create_timer(0.1, fast)

    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    el = max(time.time() - n.t0, 1e-6)
    report(a, n, el)
    n.destroy_node()
    rclpy.shutdown()


# --------------------------------------------------------------------- H4 sim
def simulate_bridge(cmds):
    """Replay recorded /cmd_vel through the bridge's clamp + slew limiter."""
    if len(cmds) < 5:
        return None
    t0, t1 = cmds[0][0], cmds[-1][0]
    dt = 1.0 / BR_RATE_HZ
    cur = [0.0, 0.0, 0.0]
    k = 0
    out = []
    t = t0
    last_tick = t
    while t <= t1:
        while k + 1 < len(cmds) and cmds[k + 1][0] <= t:
            k += 1
        last_t, vx, vy, wz = cmds[k]
        stale = (t - last_t) > BR_CMD_TIMEOUT
        if stale:
            tx = ty = tw = 0.0
        else:
            tx = clamp(vx, BR_MAX_VX)
            ty = clamp(vy, BR_MAX_VY)
            tw = clamp(wz, BR_MAX_VYAW)
            if abs(tx) < BR_DEADBAND:
                tx = 0.0
            if abs(ty) < BR_DEADBAND:
                ty = 0.0
            if abs(tw) < BR_DEADBAND:
                tw = 0.0
        step = min(t - last_tick, 0.2)
        last_tick = t
        f = BR_STALE_DECEL if stale else 1.0
        cur[0] = approach(cur[0], tx, f * BR_MAX_LIN_ACC * step)
        cur[1] = approach(cur[1], ty, f * BR_MAX_LIN_ACC * step)
        cur[2] = approach(cur[2], tw, f * BR_MAX_ANG_ACC * step)
        out.append((t, vx, wz, cur[0], cur[2]))
        t += dt
    return out


def flip_periods(ts, vals, dead=0.05):
    """Median seconds between sign changes of a signal (ignoring near-zero)."""
    sig = [(t, v) for t, v in zip(ts, vals) if abs(v) > dead]
    if len(sig) < 3:
        return None, 0
    flips = []
    last_t, last_s = sig[0][0], math.copysign(1, sig[0][1])
    for t, v in sig[1:]:
        s = math.copysign(1, v)
        if s != last_s:
            flips.append(t - last_t)
            last_t, last_s = t, s
    if not flips:
        return None, 0
    return float(np.median(flips)), len(flips)


def configured_fpd(path):
    """PathAlign.forward_point_distance as the controller is actually running
    it. Reported separately so a stale --fpd default cannot mislabel a column."""
    try:
        import yaml
        with open(os.path.expanduser(path)) as fh:      # io is not imported
            d = yaml.safe_load(fh)
        return float(d['controller_server']['ros__parameters']['FollowPath']
                     ['PathAlign.forward_point_distance'])
    except Exception:
        return None


def verdict(real, label_real='REAL', label_no='not seen'):
    return label_real if real else label_no


def report(a, n, el):
    W = 78
    print('\n' + '=' * W)
    print('jitter diagnosis   %.0f s observed' % el)
    print('=' * W)

    # ---------------- H1
    print('\nH1  plan churn -- the global plan keeps being replaced')
    if n.plans < 3:
        print('    NO DATA: only %d plan messages on %s.' % (n.plans, a.plan_topic))
        print('    Is send_goal running, and publish_global_plan true?')
        h1 = None
    else:
        rate = n.changes / el * 60.0
        fl = n.side_flips / el * 60.0
        print('    plans received      %d  (%.1f/min)' % (n.plans, n.plans / el * 60))
        print('    plans CHANGED       %d  (%.1f/min, >%.2f m apart)'
              % (n.changes, rate, a.plan_eps))
        print('    left/right FLIPS    %d  (%.1f/min)' % (n.side_flips, fl))
        h1 = (n.side_flips >= 3 and fl >= 4.0)
        print('    verdict             %s' % verdict(h1))
        if h1:
            print('      the plan alternates sides faster than 4x/min: the blocked-')
            print('      path branch in send_goal.py is bypassing the hysteresis.')
        elif n.changes and not n.side_flips:
            print('      the plan changes but never swaps sides -- churn is not')
            print('      what is steering the robot back and forth.')

    # ---------------- H2
    print('\nH2  tortuosity -- staircase plan, PathAlign chases the quantization')
    if not n.tort:
        print('    NO DATA: no plan long enough to measure.')
        h2 = None
    else:
        T = np.array(n.tort)
        td, rev, stair = float(np.median(T[:, 0])), float(np.median(T[:, 1])), \
            float(np.median(T[:, 2]))
        print('    heading change      %.2f rad/m   (a smooth arc is < 0.5)' % td)
        print('    direction reversals %.2f per m' % rev)
        print('    45-deg step share   %.0f%% of segments' % (stair * 100))
        s_now = n.fp_swing[a.fpd]
        s_alt = n.fp_swing[a.fpd_alt]
        h2 = (td > 1.0 or stair > 0.25)
        print('    verdict             %s' % verdict(h2))
        if len(s_now) > 20 and len(s_alt) > 20:
            m_now = float(np.mean(s_now)) * 180 / math.pi
            m_alt = float(np.mean(s_alt)) * 180 / math.pi
            cfg = configured_fpd(a.nav_config)
            print('    PathAlign target heading swing, per 0.1 s:')
            print('      CONFIGURED forward_point_distance: %s'
                  % ('%.2f m' % cfg if cfg is not None else 'unknown'))
            for d, m in ((a.fpd, m_now), (a.fpd_alt, m_alt)):
                mark = '  <-- in use' if (cfg is not None
                                         and abs(d - cfg) < 1e-6) else ''
                print('        d = %.2f m   %.2f deg%s' % (d, m, mark))
            if m_alt > 1e-6:
                r = m_now / m_alt
                print('      ratio %.2fx  -> raising it %s'
                      % (r, 'WOULD help' if r >= 1.5 else 'would NOT help much'))
        else:
            print('    (not enough TF+plan overlap to compare forward_point_distance)')

    # ---------------- H3
    print('\nH3  obstacle-cost saturation -- BaseObstacle decides nothing')
    if n.ev_full < 3:
        print('    NO DATA: %d evaluation cycles, %d usable.' % (n.ev_cycles, n.ev_full))
        if n.partial_cycles:
            print('    %d cycles had truncated score lists. That is'
                  % n.partial_cycles)
            print('    short_circuit_trajectory_evaluation: set it false to measure:')
            print('      ros2 param set /controller_server '
                  'FollowPath.short_circuit_trajectory_evaluation false')
        h3 = None
    else:
        print('    usable cycles       %d of %d' % (n.ev_full, n.ev_cycles))
        rows = []
        for nm, v in n.critic_spread.items():
            A = np.array(v)
            rows.append((nm, float(np.median(A[:, 0])), float(np.median(A[:, 1])),
                         float(np.median(A[:, 2])), n.decider.get(nm, 0)))
        # rank by the top-candidate spread: that is what picks the winner
        rows.sort(key=lambda r: -r[3])
        print('    critic         spread_all  rel  spread_top10  decided')
        for nm, sp, rel, spt, dec in rows:
            print('      %-13s %9.2f %5.0f%%   %9.2f   %d'
                  % (nm, sp, rel * 100, spt, dec))
        print('      spread_all is max-min over EVERY sample (outlier-prone);')
        print('      spread_top10 is over the 10 best totals - the contenders.')
        bo = [r for r in rows if r[0] == 'BaseObstacle']
        h3 = bool(bo) and bo[0][3] < 1e-6
        print('    verdict             %s' % verdict(h3))
        print('      the critic with the largest SPREAD is the one actually')
        print('      choosing the direction -- absolute size does not matter.')
        if bo and bo[0][3] < 1e-6:
            print('      BaseObstacle separates the contenders by 0.00: it is')
            print('      saturated, so %s is picking the turn.' % rows[0][0])
        elif bo:
            share = 100.0 * bo[0][3] / max(sum(r[3] for r in rows), 1e-9)
            print('      BaseObstacle is %.0f%% of the total top-10 spread'
                  % share)
        tot = max(n.legal_turnonly + n.legal_walk, 1)
        print('    legal samples       %d of %d sampled (%.0f%% rejected)'
              % (n.legal_total, n.samples_total,
                 100.0 * (n.samples_total - n.legal_total) / max(n.samples_total, 1)))
        print('    turn-only (vx<%.2f) %.0f%%   walking %.0f%%'
              % (MIN_SPEED_XY, 100.0 * n.legal_turnonly / tot,
                 100.0 * n.legal_walk / tot))
        if n.rejected_by:
            top = sorted(n.rejected_by.items(), key=lambda kv: -kv[1])[:4]
            print('    rejected by         %s'
                  % ', '.join('%s %d' % kv for kv in top))
        if len(n.best_hist) > 5:
            ts = [h[0] for h in n.best_hist]
            wz = [h[2] for h in n.best_hist]
            per, cnt = flip_periods(ts, wz)
            if per:
                print('    chosen turn direction flips every %.2f s (%d flips)'
                      % (per, cnt))

    # ---------------- H4
    print('\nH4  bridge clamp + slew -- the decision cannot be executed')
    sim = simulate_bridge(n.cmds)
    if sim is None:
        print('    NO DATA: %d /cmd_vel messages.' % len(n.cmds))
        h4 = None
    else:
        S = np.array(sim)
        ts, dwb_w, would_w = S[:, 0], S[:, 2], S[:, 4]
        md, mw = float(np.mean(np.abs(dwb_w))), float(np.mean(np.abs(would_w)))
        ratio = mw / md if md > 1e-9 else 0.0
        per, cnt = flip_periods(ts, dwb_w)
        clipped = float(np.mean(np.abs(dwb_w) > BR_MAX_VYAW + 1e-9)) * 100
        stalled = float(np.mean((np.abs(dwb_w) > 0.30) & (np.abs(would_w) < 0.15))) * 100
        print('    /cmd_vel samples    %d' % len(n.cmds))
        print('    mean |turn| asked   %.3f rad/s' % md)
        print('    mean |turn| sendable %.3f rad/s   (%.0f%% of the ask)'
              % (mw, ratio * 100))
        print('    above max_vyaw %.2f  %.0f%% of ticks' % (BR_MAX_VYAW, clipped))
        print('    asked >0.30 but would send <0.15   %.0f%% of ticks' % stalled)
        ramp = BR_MAX_VYAW / BR_MAX_ANG_ACC
        if per:
            print('    turn direction flips every %.2f s (%d flips)' % (per, cnt))
            print('    slew needs         %.2f s to reach max_vyaw from zero' % ramp)
        # two independent ways the bridge cannot execute a decision:
        #   magnitude - the ask is clipped/ramped away  (ratio)
        #   direction - it reverses before the ramp completes  (period < ramp)
        h4_mag = ratio < 0.5
        h4_dir = per is not None and per < ramp
        h4 = h4_mag or h4_dir
        print('    verdict             %s' % verdict(h4))
        if h4_dir:
            print('      REVERSAL: flips every %.2f s but needs %.2f s to ramp,'
                  % (per, ramp))
            print('      so the turn is abandoned before it is ever completed.')
            print('      The robot twitches instead of turning. Damp the')
            print('      DECISION, not the magnitude.')
        if h4_mag:
            print('      MAGNITUDE: only %.0f%% of the asked turn is sendable.'
                  % (ratio * 100))

    print('\n' + '-' * W)
    print('ACT ON:  %s' % (', '.join(
        k for k, v in (('H1', h1), ('H2', h2), ('H3', h3), ('H4', h4))
        if v) or 'nothing measured as real -- widen the window or re-run'
        ' during an actual jitter episode'))
    print('-' * W)

    if a.csv and n.cmds:
        with open(os.path.expanduser(a.csv), 'w') as f:
            f.write('t,dwb_vx,dwb_wz,would_vx,would_wz\n')
            for r in (sim or []):
                f.write('%.3f,%.4f,%.4f,%.4f,%.4f\n'
                        % (r[0], r[1], r[2], r[3], r[4]))
        print('cmd_vel replay written to %s' % a.csv)


if __name__ == '__main__':
    main()
