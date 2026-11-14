#!/usr/bin/env python3
"""Per-lap drift, relocaliser on vs off - tape at the start mark after every lap.

Left:  position error per lap, grouped bars (one axis, both conditions).
Right: the error VECTOR per lap (estimate minus tape) in the start frame.
       Odometry-only marches off in one direction; relocaliser-on scatters
       around zero. Direction is what separates drift from tape noise.

Laps 1-4: estimator arrival pose vs the user's tape readings (2026-11-14).
Lap 5:    settled end pose (median over the still window) vs .truth.json,
          i.e. the same numbers as the report's 2.2 / 18.9 cm.
"""
import csv, json, math, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
SANS, MONO = 'Liberation Sans', 'DejaVu Sans Mono'
INK, INK2, INK3, GRID = '#14140f', '#55544d', '#86857c', '#dedcd5'
BLUE, ORANGE = '#2a78d6', '#eb6834'
B = os.path.dirname(os.path.abspath(__file__))


def load(csv_name, truth_name):
    rows = [r for r in csv.DictReader(open(os.path.join(B, csv_name)))
            if r.get('map_body_x') not in (None, '') and r.get('phase')]
    st = [r for r in rows if r['phase'] == 'settle-start']
    run = [r for r in rows if r['phase'] == 'run']
    en = [r for r in rows if r['phase'] == 'settle-end'] or rows[-60:]

    def med(s, c):
        v = sorted(float(r[c]) for r in s)
        return v[len(v) // 2]
    sx, sy, sh = med(st, 'map_body_x'), med(st, 'map_body_y'), med(st, 'map_body_yaw')
    h = math.radians(sh)
    cos, sin = math.cos(h), math.sin(h)

    def to_start_frame(x, y):
        dx, dy = x - sx, y - sy
        return dx * cos + dy * sin, -dx * sin + dy * cos   # forward, left

    path = [to_start_frame(float(r['map_body_x']), float(r['map_body_y']))
            for r in run]
    est_end = to_start_frame(med(en, 'map_body_x'), med(en, 'map_body_y'))
    tr = json.load(open(os.path.join(B, truth_name)))
    return path, est_end, (float(tr['dx']), float(tr['dy']))



OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drift_per_lap.png')

# (estimate fwd, left), (tape fwd, left), cm - laps 1-5, all at ARRIVAL
ON_14 = [((49.7, 15.0), (47, 15)), ((12.8, -30.7), (11, -28)),
         ((16.8, -26.3), (15, -28)), ((17.4, -28.6), (15, -26)),
         ((12.7, -25.2), (15, -26))]
OFF_14 = [((46.0, 16.0), (44, 16)), ((-5.3, -50.2), (-3, -46)),
          ((-5.8, -43.8), (0, -37)), ((5.6, -37.0), (12, -30)),
          ((21.8, -39.8), (18, -32))]


def errs(laps, csv_name, truth_name):
    return [(e[0] - t[0], e[1] - t[1]) for e, t in laps]


on = errs(ON_14, 'drift_navlaps-with-reloc_1111_165105.csv',
          'drift_navlaps-with-reloc_1111_165105.truth.json')
off = errs(OFF_14, 'drift_navlaps-seed-only_1111_171609.csv',
           'drift_navlaps-seed-only_1111_171609.truth.json')
mag = lambda v: [math.hypot(*p) for p in v]
on_m, off_m = mag(on), mag(off)
for i in range(5):
    print('lap %d  off %5.1f cm   on %5.1f cm' % (i + 1, off_m[i], on_m[i]))

plt.rcParams['font.family'] = SANS
fig, (ax, bx) = plt.subplots(1, 2, figsize=(12.5, 5.2),
                             gridspec_kw={'width_ratios': [1.25, 1]})
fig.patch.set_facecolor('white')


def style(a):
    a.set_facecolor('white')
    a.grid(color=GRID, lw=0.9, zorder=0)
    a.set_axisbelow(True)
    for s in ('top', 'right'):
        a.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        a.spines[s].set_color(GRID)
    a.tick_params(colors=INK2, labelsize=9.5, length=0)


# ---- left: grouped bars --------------------------------------------------
style(ax)
ax.grid(axis='x', visible=False)
w, gap = 0.36, 0.02
dist = [11.5, 23.0, 34.6, 46.1, 57.6]  # even split of the run total
xs = range(5)
for off_x, vals, col, lab in ((-w / 2 - gap / 2, off_m, ORANGE, 'relocaliser off'),
                              (w / 2 + gap / 2, on_m, BLUE, 'relocaliser on')):
    ax.bar([x + off_x for x in xs], vals, w, color=col, label=lab, zorder=3)
    ax.text(4 + off_x, vals[4] + 0.25, '%.1f' % vals[4], ha='center', va='bottom',
            fontname=MONO, fontsize=10.5, fontweight='bold', color=INK)
ax.set_xticks(list(xs))
ax.set_xticklabels(['lap %d\n≈%.0f m' % (i + 1, d) for i, d in enumerate(dist)])
ax.set_ylabel('position error vs tape (cm)', fontsize=10.5, color=INK2)
ax.set_ylim(0, 12)
ax.legend(loc='upper left', frameon=False, fontsize=10, labelcolor=INK)
ax.set_title('Error at the start mark after each lap', loc='left',
             fontsize=12.5, fontweight='semibold', color=INK, pad=10)

# ---- right: error vectors -------------------------------------------------
style(bx)
bx.axhline(0, color=INK3, lw=0.9, zorder=1)
bx.axvline(0, color=INK3, lw=0.9, zorder=1)
bx.plot([0] + [p[0] for p in off], [0] + [p[1] for p in off], color=ORANGE,
        lw=1.6, alpha=0.55, zorder=2)
for v, col, z in ((off, ORANGE, 3), (on, BLUE, 5)):
    for i, (fx, ly) in enumerate(v):
        bx.plot(fx, ly, 'o', ms=11, color=col, markeredgecolor='white',
                markeredgewidth=1.5, zorder=z)
        bx.text(fx, ly, str(i + 1), ha='center', va='center', fontsize=6.5,
                fontweight='bold', color='white', zorder=z + 1)
bx.text(off[4][0], off[4][1] - 0.8, 'off, lap 5', fontsize=9.5,
        color=INK2, va='top', ha='center')
bx.text(-8.5, 2.2, 'on: all laps\nwithin 3.5 cm', fontsize=9.5, color=INK2)
bx.set_xlim(-9, 7)
bx.set_ylim(-10, 4)
bx.set_aspect('equal')
bx.set_xlabel('forward error (cm)', fontsize=10.5, color=INK2)
bx.set_ylabel('left error (cm)', fontsize=10.5, color=INK2)
bx.set_title('Direction of the error (estimate − tape)', loc='left',
             fontsize=12.5, fontweight='semibold', color=INK, pad=10)

fig.text(0.01, 0.01, 'Numbered dots = lap. Every lap: estimate on arrival vs tape on arrival, '
         'before the final turn in place.', fontsize=8.5, color=INK3)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(OUT, dpi=200)
print('wrote', OUT)
