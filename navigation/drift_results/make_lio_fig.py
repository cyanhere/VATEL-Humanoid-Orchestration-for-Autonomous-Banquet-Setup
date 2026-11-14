#!/usr/bin/env python3
"""Figure: backward propagation (de-skewing) and the iterated EKF cycle.

Panel A is GEOMETRICALLY REAL, not a sketch: a flat wall is ray-traced from a
moving, rotating robot across one 100 ms sweep, then reconstructed two ways -
once assuming every point was captured at the sweep's end (the skew), and once
using the true pose at each point's own timestamp (what backward propagation
recovers). The rotation rate is exaggerated so the effect is visible at print
size; the caption says so.
"""
import math
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle

SANS, MONO = 'Liberation Sans', 'DejaVu Sans Mono'
INK, INK2, INK3, GRID = '#14140f', '#55544d', '#86857c', '#dedcd5'
BLUE, ORANGE, GREEN = '#2a78d6', '#eb6834', '#1baf7a'

# ── geometry of one sweep ───────────────────────────────────────────────────
WALL_X = 4.0
T_SWEEP = 0.10            # s, one lidar revolution
V = 0.5                   # m/s forward
OMEGA = math.radians(120) # rad/s  EXAGGERATED for visibility
AZ = [math.radians(a) for a in range(-25, 26, 1)]

true_pts, skewed_pts, poses = [], [], []
for i, alpha in enumerate(AZ):
    t = T_SWEEP * i / (len(AZ) - 1)
    px, py, th = V * t, 0.0, OMEGA * t
    poses.append((px, py, th))
    bearing = th + alpha
    if abs(math.cos(bearing)) < 1e-3:
        continue
    r = (WALL_X - px) / math.cos(bearing)          # range to the wall
    if r <= 0:
        continue
    true_pts.append((WALL_X, py + r * math.sin(bearing)))
    # naive: pretend every point was taken at the END pose
    pex, pey, peth = V * T_SWEEP, 0.0, OMEGA * T_SWEEP
    b2 = peth + alpha
    skewed_pts.append((pex + r * math.cos(b2), pey + r * math.sin(b2)))

fig = plt.figure(figsize=(13.0, 6.4), dpi=200)
fig.patch.set_facecolor('white')
gs = fig.add_gridspec(1, 2, width_ratios=[1.02, 1.0], wspace=0.17,
                      left=0.045, right=0.975, top=0.74, bottom=0.175)

# ── Panel A: backward propagation ───────────────────────────────────────────
ax = fig.add_subplot(gs[0])
ax.set_facecolor('white')
ax.plot([WALL_X, WALL_X], [-2.0, 2.0], color=INK, lw=3.0, zorder=2)
ax.annotate('the real wall', (WALL_X, 1.75), xytext=(-8, 0),
            textcoords='offset points', ha='right', va='center',
            fontname=SANS, fontsize=10.5, color=INK)

sx = [p[0] for p in skewed_pts]; sy = [p[1] for p in skewed_pts]
tx = [p[0] for p in true_pts];   ty = [p[1] for p in true_pts]
ax.plot(sx, sy, 'o', ms=4.2, color=ORANGE, zorder=4,
        label='raw sweep, all points assumed taken at one instant')
ax.plot(tx, ty, 'o', ms=4.2, color=BLUE, zorder=5,
        label='de-skewed: each point placed at its own pose')

# the robot, start and end of the sweep
for (px, py, th), a_, lab in ((poses[0], 0.95, 'sweep starts'),
                              (poses[-1], 0.95, 'sweep ends, 100 ms later')):
    ax.add_patch(Circle((px, py), 0.09, facecolor='white', edgecolor=INK,
                        lw=1.8, zorder=7))
    ax.arrow(px, py, 0.42 * math.cos(th), 0.42 * math.sin(th),
             head_width=0.1, head_length=0.1, fc=INK, ec=INK, lw=1.3, zorder=7)
ax.annotate('sweep starts', poses[0][:2], xytext=(-6, -22),
            textcoords='offset points', ha='center', fontname=SANS,
            fontsize=9.5, color=INK2)
ax.annotate('100 ms later,\nmoved and turned', poses[-1][:2], xytext=(16, 26),
            textcoords='offset points', ha='left', fontname=SANS,
            fontsize=9.5, color=INK2, linespacing=1.35)
# a couple of rays, to show the fan
for idx in (0, len(AZ)//2, len(AZ)-1):
    px, py, th = poses[idx]
    b = th + AZ[idx]
    r = (WALL_X - px) / math.cos(b)
    ax.plot([px, px + r*math.cos(b)], [py, py + r*math.sin(b)],
            color=INK3, lw=0.7, ls=':', zorder=3)

ax.annotate('the same wall, bent', (sx[len(sx)//2], sy[len(sy)//2]),
            xytext=(-30, -40), textcoords='offset points', ha='right',
            fontname=SANS, fontsize=10.5, color=ORANGE, fontweight='bold',
            arrowprops=dict(arrowstyle='->', color=ORANGE, lw=1.4,
                            connectionstyle='arc3,rad=0.25'))
ax.set_xlim(-0.75, 5.15); ax.set_ylim(-2.35, 2.35)
ax.set_aspect('equal', adjustable='box')
for s in ('top', 'right', 'left', 'bottom'):
    ax.spines[s].set_visible(False)
ax.set_xticks([]); ax.set_yticks([])
ax.set_title('A.  Backward propagation', fontname=SANS, fontsize=13.5,
             color=INK, loc='left', pad=22, fontweight='semibold')
ax.annotate('removes the robot\u2019s own motion from inside one sweep',
            (0.0, 1.0), xycoords='axes fraction', xytext=(0, 8),
            textcoords='offset points', ha='left', va='bottom',
            fontname=SANS, fontsize=10.5, color=INK2)
ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.02), frameon=False,
          prop={'family': SANS, 'size': 10}, handletextpad=0.5, ncol=1)

# ── Panel B: the iterated EKF cycle ─────────────────────────────────────────
ax2 = fig.add_subplot(gs[1])
ax2.set_facecolor('white')
ax2.set_xlim(-0.2, 10.2); ax2.set_ylim(0, 10)
ax2.axis('off')
ax2.set_title('B.  The iterated EKF', fontname=SANS, fontsize=13.5,
              color=INK, loc='left', pad=22, fontweight='semibold')
ax2.annotate('runs once per sweep; the LiDAR corrects the IMU\u2019s biases',
             (0.0, 1.0), xycoords='axes fraction', xytext=(0, 8),
             textcoords='offset points', ha='left', va='bottom',
             fontname=SANS, fontsize=10.5, color=INK2)

def box(x, y, w, h, text, fc, ec, fs=10, bold=False):
    ax2.add_patch(FancyBboxPatch((x, y), w, h,
                                 boxstyle='round,pad=0.12,rounding_size=0.18',
                                 facecolor=fc, edgecolor=ec, lw=1.6, zorder=3))
    ax2.text(x + w/2, y + h/2, text, ha='center', va='center',
             fontname=SANS, fontsize=fs, color=INK, zorder=4,
             linespacing=1.4, fontweight='bold' if bold else 'normal')

def arrow(x1, y1, x2, y2, col=INK2, rad=0.0, lw=1.6, ls='-'):
    ax2.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                  arrowstyle='-|>', mutation_scale=14,
                                  color=col, lw=lw, linestyle=ls, zorder=2,
                                  connectionstyle='arc3,rad=%g' % rad))

box(0.2, 8.0, 3.1, 1.25, 'IMU\n182 Hz', '#eaf2fc', BLUE, 10.5, True)
box(6.7, 8.0, 3.1, 1.25, 'LiDAR sweep\n10 Hz', '#fdeee7', ORANGE, 10.5, True)
box(0.2, 5.5, 4.4, 1.5, 'propagate the state forward\nand de-skew the sweep  (A)',
    'white', BLUE)
box(5.6, 5.5, 4.2, 1.5, 'match each point to the map\n(point-to-plane residual)',
    'white', ORANGE)
box(2.0, 3.1, 6.0, 1.4, 'update the state, then RE-LINEARISE\nand repeat  — the "iterated" part',
    '#eafaf3', GREEN, 10.5, True)
box(1.4, 0.7, 7.2, 1.35,
    'corrected pose  +  updated IMU biases\nthis is why the bias never runs away',
    'white', INK3)

arrow(1.75, 8.0, 1.75, 7.05, BLUE)
arrow(8.25, 8.0, 7.9, 7.05, ORANGE)
arrow(4.65, 6.25, 5.55, 6.25, INK2)
arrow(7.7, 5.5, 6.2, 4.55, ORANGE)
arrow(2.4, 5.5, 3.4, 4.55, BLUE)
arrow(5.0, 3.1, 5.0, 2.1, GREEN)
# the iteration loop
ax2.add_patch(FancyArrowPatch((8.1, 3.8), (8.1, 6.0), arrowstyle='-|>',
                              mutation_scale=13, color=GREEN, lw=1.6,
                              connectionstyle='arc3,rad=-0.38', zorder=2))
ax2.text(8.78, 4.9, 'iterate', ha='center', va='center', rotation=90,
         fontname=SANS, fontsize=9.5, color=GREEN, fontweight='bold')
# feedback to propagation
ax2.add_patch(FancyArrowPatch((1.4, 1.38), (0.75, 5.5), arrowstyle='-|>',
                              mutation_scale=13, color=INK3, lw=1.4, ls=':',
                              connectionstyle='arc3,rad=0.42', zorder=2))
ax2.text(0.05, 3.5, 'next sweep', ha='left', va='center', rotation=90,
         fontname=SANS, fontsize=9, color=INK3)

fig.text(0.013, 0.955, 'How FAST-LIO2 keeps odometry from running away',
         fontname=SANS, fontsize=18, fontweight='bold', color=INK,
         ha='left', va='top')
fig.text(0.013, 0.898,
         'The IMU predicts and de-skews; the LiDAR corrects both the pose and '
         'the IMU biases. Neither sensor could do this alone.',
         fontname=SANS, fontsize=11.5, color=INK2, ha='left', va='top')
fig.text(0.013, 0.052, 'Panel A is ray-traced from a flat wall across one '
         '100 ms sweep, robot moving at 0.5 m/s.',
         fontname=SANS, fontsize=9.5, color=INK3, ha='left', va='bottom')
fig.text(0.013, 0.016, 'The turn rate is exaggerated to 120 deg/s so the skew '
         'is visible at this size; at a walking turn rate it is smaller but '
         'the same shape.',
         fontname=SANS, fontsize=9.5, color=INK3, ha='left', va='bottom')

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fig_lio_mechanism.png')
fig.savefig(out, facecolor='white', edgecolor='none')
print('wrote %s' % out)
print('  skew at the sweep edge: %.3f m' % max(
    abs(s[0]-WALL_X) for s in skewed_pts))
