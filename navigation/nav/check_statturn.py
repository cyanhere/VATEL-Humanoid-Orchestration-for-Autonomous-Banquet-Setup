#!/usr/bin/env python3
"""Decide, from a nav_log.py CSV, whether the meaningless stationary turn happened.

The signature we are hunting (seen in nav_20261020_135311.csv) is a cycle where
the robot is commanded to TURN while its forward command sits in the dead band
   0.02  <  vx  <  0.20
Below 0.20 m/s the gait ignores forward entirely, so such a cycle spends its
whole duration rotating and translating nothing - DWB thinks it is driving an
arc, the robot just spins.

  /usr/bin/python3 ~/.scuba/nav/check_statturn.py <csv> [more.csv ...]
"""
import csv, sys, math

WALK = 0.20      # measured forward gait floor, m/s
TURN = 0.30      # measured yaw gait floor, rad/s (re-measured 2026-10-22)
DEAD = 0.02      # nav_bridge deadband


def col(rows, name):
    return [r.get(name, '') for r in rows]


def f(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def report(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print('%s: empty' % path)
        return
    hdr = list(rows[0].keys())

    # the columns differ a little between nav_log versions, so find them
    def pick(*cands):
        for c in cands:
            if c in hdr:
                return c
        return None

    cvx = pick('cmd_vx', 'vx_cmd', 'cmd_x', 'vx')
    cvw = pick('cmd_vyaw', 'vyaw_cmd', 'cmd_yaw', 'vyaw', 'cmd_wz', 'wz')
    avx = pick('api_vx', 'sent_vx', 'out_vx')
    avw = pick('api_vyaw', 'sent_vyaw', 'out_vyaw', 'sent_wz', 'api_wz', 'out_wz')
    t = [f(x) for x in col(rows, 't')]
    dur = (t[-1] - t[0]) if len(t) > 1 else 0.0

    print('=' * 72)
    print(path.split('/')[-1])
    print('  %d rows, %.1f s' % (len(rows), dur))
    print('  columns: %s' % ', '.join(hdr))
    if not cvx or not cvw:
        print('  !! no recognisable velocity columns - cannot judge')
        return
    print('  using cmd: %s / %s%s' % (cvx, cvw,
          ('   api: %s / %s' % (avx, avw)) if avx and avw else ''))

    # per-row dt, so "time spent" is honest even if the logger stuttered
    dt = []
    for i in range(len(t)):
        if i + 1 < len(t):
            dt.append(max(0.0, min(0.5, t[i + 1] - t[i])))
        else:
            dt.append(dt[-1] if dt else 0.0)

    def scan(vxn, vwn, label):
        vx = [abs(f(v)) for v in col(rows, vxn)]
        vw = [abs(f(v)) for v in col(rows, vwn)]
        moving = [i for i in range(len(rows)) if vx[i] > DEAD or vw[i] > DEAD]
        if not moving:
            print('  %s: never commanded anything above the deadband' % label)
            return
        t_move = sum(dt[i] for i in moving)
        # THE signature: turning, with forward stuck in the dead band
        bad = [i for i in moving if vw[i] > DEAD and DEAD < vx[i] < WALK]
        t_bad = sum(dt[i] for i in bad)
        # pure in-place rotation is legitimate (that is how it aims)
        pure = [i for i in moving if vw[i] > DEAD and vx[i] <= DEAD]
        t_pure = sum(dt[i] for i in pure)
        # forward in the dead band with no turn either: pure wasted twitch
        twitch = [i for i in moving if vw[i] <= DEAD and DEAD < vx[i] < WALK]
        t_twitch = sum(dt[i] for i in twitch)
        # turn commanded, but BELOW the yaw floor -> the robot does nothing
        dead_turn = [i for i in moving if DEAD < vw[i] < TURN and vx[i] < WALK]
        t_dead = sum(dt[i] for i in dead_turn)
        good = [i for i in moving if vx[i] >= WALK]
        t_good = sum(dt[i] for i in good)

        print('  %s  (%.1f s commanded)' % (label, t_move))
        print('     max vx %.3f   max |vyaw| %.3f' % (max(vx), max(vw)))
        print('     STATIONARY-TURN  turn + vx in deadband : %6.1f s  %5.1f%%  %d cyc'
              % (t_bad, 100 * t_bad / t_move, len(bad)))
        print('     wasted twitch    vx in deadband, no turn: %6.1f s  %5.1f%%  %d cyc'
              % (t_twitch, 100 * t_twitch / t_move, len(twitch)))
        print('     DEAD TURN        0 < |yaw| < %.2f, vx < %.2f : %6.1f s  %5.1f%%  %d cyc'
              % (TURN, WALK, t_dead, 100 * t_dead / t_move, len(dead_turn)))
        print('     legit in-place   turn, vx == 0          : %6.1f s  %5.1f%%  %d cyc'
              % (t_pure, 100 * t_pure / t_move, len(pure)))
        if dead_turn:
            w = sorted(vw[i] for i in dead_turn)
            print('        yaw commanded while dead: min %.3f med %.3f max %.3f'
                  '  (floor is %.2f)' % (w[0], w[len(w)//2], w[-1], TURN))
        print('     real walking     vx >= %.2f             : %6.1f s  %5.1f%%  %d cyc'
              % (WALK, t_good, 100 * t_good / t_move, len(good)))
        if bad:
            v = sorted(vx[i] for i in bad)
            print('     vx while stationary-turning: min %.3f med %.3f max %.3f'
                  % (v[0], v[len(v) // 2], v[-1]))
        return t_bad / t_move if t_move else 0.0

    frac = scan(cvx, cvw, 'DWB COMMANDED (/cmd_vel)')
    if avx and avw:
        scan(avx, avw, 'ACTUALLY SENT (/api/sport/request, post slew+clamp)')

    mf, mw = pick('meas_fwd'), pick('meas_wz')
    if mf and mw:
        vx = [abs(f(v)) for v in col(rows, cvx)]
        vw = [abs(f(v)) for v in col(rows, cvw)]
        gf = [abs(f(v)) for v in col(rows, mf)]
        gw = [abs(f(v)) for v in col(rows, mw)]
        bad = [i for i in range(len(rows))
               if vw[i] > DEAD and DEAD < vx[i] < WALK]
        good = [i for i in range(len(rows)) if vx[i] >= WALK]
        mean = lambda a: (sum(a) / len(a)) if a else 0.0
        print('  MEASURED RESPONSE (odom, what the gait actually did)')
        if bad:
            print('     on stationary-turn cycles: cmd vx %.3f -> meas fwd %.3f'
                  '   cmd yaw %.3f -> meas yaw %.3f'
                  % (mean([vx[i] for i in bad]), mean([gf[i] for i in bad]),
                     mean([vw[i] for i in bad]), mean([gw[i] for i in bad])))
        if good:
            print('     on real-walking cycles   : cmd vx %.3f -> meas fwd %.3f'
                  '   cmd yaw %.3f -> meas yaw %.3f'
                  % (mean([vx[i] for i in good]), mean([gf[i] for i in good]),
                     mean([vw[i] for i in good]), mean([gw[i] for i in good])))
        if bad and good:
            rb = mean([gf[i] for i in bad]) / max(mean([vx[i] for i in bad]), 1e-9)
            rg = mean([gf[i] for i in good]) / max(mean([vx[i] for i in good]), 1e-9)
            print('     forward tracking gain: %.2f when sub-threshold vs %.2f'
                  ' when above it' % (rb, rg))

    # how much ground was covered vs how much heading was burned
    x = [f(v) for v in col(rows, 'x')]
    y = [f(v) for v in col(rows, 'y')]
    h = [f(v) for v in col(rows, 'heading_deg')]
    path_m = sum(math.hypot(x[i + 1] - x[i], y[i + 1] - y[i])
                 for i in range(len(x) - 1))
    turn_d = sum(abs((h[i + 1] - h[i] + 180) % 360 - 180)
                 for i in range(len(h) - 1))
    net = math.hypot(x[-1] - x[0], y[-1] - y[0])
    print('  travelled %.2f m along the path, %.2f m net, burned %.0f deg of heading'
          % (path_m, net, turn_d))
    if path_m > 0.05:
        print('  %.0f deg of turning per metre advanced  (lower is better)'
              % (turn_d / path_m))
    else:
        print('  !! went essentially nowhere (%.2f m) while turning %.0f deg'
              % (path_m, turn_d))

    if frac is not None:
        print('  VERDICT: %s'
              % ('stationary turn PRESENT - %.0f%% of commanded time' % (100 * frac)
                 if frac > 0.05 else
                 'clean - stationary turn is %.1f%% of commanded time' % (100 * frac)))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        report(p)
