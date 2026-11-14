#!/usr/bin/env python3
"""Bird's-eye: the estimated path, and where the robot ACTUALLY ended.

HONESTY NOTE BUILT INTO THE FIGURE
Ground truth exists at exactly TWO points - the start mark and the tape
measurement at the end. The drawn path is therefore the ESTIMATE, not reality.
For the relocalizer-on run the estimate is good to 0.022 m so the path is
approximately real; for the odometry-only run the estimate and reality diverge
progressively and we only know the gap at the end. The figure says so, and
marks the two known-real points differently from the line.

Small multiples (2x2) rather than an overlay: the full path is ~5 m across
while the endpoint error is ~0.02-0.19 m, a 250x range, so the detail row
carries its own scale instead of a second axis on one plot.
"""
import csv, json, math, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SANS, MONO = 'Liberation Sans', 'DejaVu Sans Mono'
INK, INK2, INK3, GRID = '#14140f', '#55544d', '#86857c', '#dedcd5'
BLUE, ORANGE = '#2a78d6', '#eb6834'
B = os.path.dirname(os.path.abspath(__file__))

RUNS = [
    ('relocaliser on', BLUE,
     'drift_navlaps-with-reloc_1111_165105.csv',
     'drift_navlaps-with-reloc_1111_165105.truth.json'),
    ('relocaliser off', ORANGE,
     'drift_navlaps-seed-only_1111_171609.csv',
     'drift_navlaps-seed-only_1111_171609.truth.json'),
]


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


data = [(lab, col) + load(c, t) for lab, col, c, t in RUNS]

# 2026-11-14: compare at lap-5 ARRIVAL, before the final finish_yaw turn, so
# both conditions are measured at the same moment (the ON .truth.json was
# taped before settling; the OFF one after). Estimate = logged back-leg
# arrival pose; tape = user's arrival readings. The path is cut at arrival.
ARRIVAL = {'relocaliser on':  ((0.127, -0.252), (0.15, -0.26)),
           'relocaliser off': ((0.218, -0.398), (0.18, -0.32))}


def cut_at(path, est):
    n0 = int(len(path) * 0.85)
    i = min(range(n0, len(path)),
            key=lambda k: math.hypot(path[k][0] - est[0], path[k][1] - est[1]))
    return path[:i + 1]


data = [(lab, col, cut_at(path, ARRIVAL[lab][0])) + ARRIVAL[lab]
        for lab, col, path, _e, _t in data]

fig = plt.figure(figsize=(12.6, 7.8), dpi=200)
fig.patch.set_facecolor('white')
gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1.0], hspace=0.46, wspace=0.17,
                      left=0.068, right=0.975, top=0.78, bottom=0.145)

for col_i, (lab, col, path, est, tape) in enumerate(data):
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]

    # ---- top: the whole route
    ax = fig.add_subplot(gs[0, col_i])
    ax.set_facecolor('white')
    ax.plot(xs, ys, color=col, lw=1.0, alpha=0.75, zorder=3,
            solid_capstyle='round')
    ax.plot([0], [0], marker='o', ms=11, color='white',
            markeredgecolor=INK, markeredgewidth=2.0, zorder=6)
    ax.annotate('start mark (known real)', (0, 0), xytext=(14, -16),
                textcoords='offset points', ha='left', va='top',
                fontname=SANS, fontsize=9.5, color=INK2)
    # SAME limits on both panels - different y-scales would make a
    # side-by-side comparison meaningless
    ax.set_xlim(-0.55, 5.25)
    ax.set_ylim(-1.05, 1.05)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(color=GRID, lw=0.9, zorder=1)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.set_title('%s  —  estimated path, 5 laps' % lab, fontname=SANS,
                 fontsize=12.5, color=col, loc='left', pad=10,
                 fontweight='semibold')
    ax.set_xlabel('forward from start (m)', fontname=SANS, fontsize=10.5,
                  color=INK2)
    if col_i == 0:
        ax.set_ylabel('left of start (m)', fontname=SANS, fontsize=10.5,
                      color=INK2)
    for lb in ax.get_xticklabels() + ax.get_yticklabels():
        lb.set_fontname(MONO); lb.set_fontsize(9.5); lb.set_color(INK2)
    ax.tick_params(length=0, pad=5)

    # ---- bottom: the endpoint, where the only other real measurement is
    ax2 = fig.add_subplot(gs[1, col_i])
    ax2.set_facecolor('white')
    tailn = max(1, int(len(path) * 0.04))
    ax2.plot(xs[-tailn:], ys[-tailn:], color=col, lw=1.2, alpha=0.45, zorder=2)
    ax2.plot([est[0]], [est[1]], marker='o', ms=12, color='white',
             markeredgecolor=col, markeredgewidth=2.4, zorder=5,
             label='robot BELIEVED it was here')
    ax2.plot([tape[0]], [tape[1]], marker='X', ms=14, color=INK, zorder=6,
             label='tape says it ACTUALLY was here')
    err = math.hypot(est[0] - tape[0], est[1] - tape[1])
    ax2.plot([est[0], tape[0]], [est[1], tape[1]], color=INK, lw=1.6,
             ls=':', zorder=4)
    mx, my = 0.5 * (est[0] + tape[0]), 0.5 * (est[1] + tape[1])
    ax2.annotate('%.1f cm' % (err * 100), (mx, my), xytext=(0, -42),
                 textcoords='offset points', ha='center', va='top',
                 fontname=MONO, fontsize=13, fontweight='bold', color=INK)
    cx, cy = 0.5 * (est[0] + tape[0]), 0.5 * (est[1] + tape[1])
    pad = 0.22
    ax2.set_xlim(cx - pad, cx + pad)
    ax2.set_ylim(cy - pad, cy + pad)
    ax2.set_aspect('equal')
    ax2.grid(color=GRID, lw=0.9, zorder=1)
    ax2.set_axisbelow(True)
    for s in ('top', 'right'):
        ax2.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax2.spines[s].set_color(GRID)
    ax2.set_title('end of lap 5, on arrival (0.44 m across)', fontname=SANS, fontsize=11.5,
                  color=INK, loc='left', pad=8, fontweight='semibold')
    ax2.set_xlabel('forward from start (m)', fontname=SANS, fontsize=10.5,
                   color=INK2)
    for lb in ax2.get_xticklabels() + ax2.get_yticklabels():
        lb.set_fontname(MONO); lb.set_fontsize(9.5); lb.set_color(INK2)
    ax2.tick_params(length=0, pad=5)
    if col_i == 0:
        handles, labels = ax2.get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right', frameon=False,
                   bbox_to_anchor=(0.975, 0.985),
                   prop={'family': SANS, 'size': 10.5}, handletextpad=0.6)

fig.text(0.013, 0.962, 'Where the robot thought it was, and where it actually '
         'was', fontname=SANS, fontsize=18, fontweight='bold', color=INK,
         ha='left', va='top')
fig.text(0.013, 0.908, 'Unitree G1, five 5 m out-and-back laps under Nav2, '
         '~58 m per run, plotted in the start pose’s own frame, up to arrival after lap 5.',
         fontname=SANS, fontsize=11.5, color=INK2, ha='left', va='top')
fig.text(0.013, 0.048, 'The drawn path is the ESTIMATE; tape measurements exist only at the start mark after each lap. With the relocaliser the estimate stayed within 3.5 cm of the tape on every lap, so that path is close to real.',
         fontname=SANS, fontsize=9.5, color=INK3, ha='left', va='bottom')
fig.text(0.013, 0.015, 'Without it the estimate and reality drifted apart by up to 9.5 cm, so the right-hand path is not the true route.   Measured 2026-11-11.',
         fontname=SANS, fontsize=9.5, color=INK3, ha='left', va='bottom')

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drift_paths.png')
fig.savefig(out, facecolor='white', edgecolor='none')
print('wrote %s' % out)
for lab, col, path, est, tape in data:
    print('  %-16s est (%+.3f, %+.3f)  tape (%+.3f, %+.3f)  err %.3f m  (%d pts)'
          % (lab, est[0], est[1], tape[0], tape[1],
             math.hypot(est[0]-tape[0], est[1]-tape[1]), len(path)))
