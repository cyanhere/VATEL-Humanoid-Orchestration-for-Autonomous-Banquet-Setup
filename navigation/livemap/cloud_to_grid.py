#!/usr/bin/env python3
"""Turn an accumulated cloud into a ROS occupancy grid (.pgm + .yaml).

    /usr/bin/python3 ~/.scuba/livemap/cloud_to_grid.py \
        --in ~/.scuba/livemap/maps/live/live_cloud.npy \
        --out ~/.scuba/livemap/maps/run_$(date +%H%M)

Offline and read-only on the cloud. Run it after a session to keep the static
map the robot built, or mid-run to look at progress.

HOW FREE SPACE IS DECIDED
The cloud has no ray information, so free cells cannot be raytraced. But a
FLOOR return is itself evidence: the beam reached the ground there, so the
column was clear. So

    occupied  cells with hits in the obstacle band (0.20 .. 1.60 m), the same
              band nav2's obstacle_layer marks from
    free      cells with floor hits (|z| < floor_band) and no obstacle hits
    unknown   everything else

Removed/unseen cells become 205 (unknown), NOT free: a planner running
allow_unknown:false must refuse to cross ground that was never observed.
"""
import argparse
import os

import numpy as np

FREE, OCC, UNKNOWN = 254, 0, 205


def write_pgm(path, img, comment):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(('# %s\n' % comment).encode())
        f.write(('%d %d\n255\n' % (w, h)).encode())
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='src',
                    default='~/.scuba/livemap/maps/live/live_cloud.npy')
    ap.add_argument('--out', default='~/.scuba/livemap/maps/live_grid',
                    help='base path; .pgm and .yaml are appended')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--obstacle-band', type=float, nargs=2, default=[0.20, 1.60],
                    help='matches nav2 min/max_obstacle_height')
    ap.add_argument('--floor-band', type=float, default=0.15,
                    help='|z| under this counts as a floor return')
    ap.add_argument('--min-hits', type=int, default=1,
                    help='points needed in a cell before it counts. The cloud is '
                         'already voxel-downsampled (0.15 m), so at a 0.05 m grid '
                         'most cells hold exactly ONE point - a threshold of 2 '
                         'erases nearly all free space')
    ap.add_argument('--margin', type=float, default=1.0,
                    help='metres of padding around the cloud extent')
    a = ap.parse_args()

    src = os.path.expanduser(a.src)
    out = os.path.expanduser(a.out)
    M = np.load(src).astype(np.float64)
    if len(M) == 0:
        raise SystemExit('!! cloud is empty: %s' % src)

    lo, hi = a.obstacle_band
    obs = M[(M[:, 2] > lo) & (M[:, 2] < hi)]
    flr = M[np.abs(M[:, 2]) < a.floor_band]
    print('cloud %d pts | obstacle band %.2f-%.2f m: %d | floor |z|<%.2f: %d'
          % (len(M), lo, hi, len(obs), a.floor_band, len(flr)))
    if len(obs) == 0 and len(flr) == 0:
        raise SystemExit('!! nothing in either band - is the floor at z=0?')

    xy = M[:, :2]
    x0, y0 = xy.min(axis=0) - a.margin
    x1, y1 = xy.max(axis=0) + a.margin
    w = int(np.ceil((x1 - x0) / a.res))
    h = int(np.ceil((y1 - y0) / a.res))
    print('grid %d x %d cells @ %.2f m  (%.1f x %.1f m)  origin (%.3f, %.3f)'
          % (w, h, a.res, w * a.res, h * a.res, x0, y0))

    def counts(P):
        c = np.zeros((h, w), np.int32)
        if len(P) == 0:
            return c
        ix = ((P[:, 0] - x0) / a.res).astype(np.int64)
        iy = ((P[:, 1] - y0) / a.res).astype(np.int64)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[ok], iy[ok]
        # row 0 of a ROS pgm is the TOP row, i.e. max y
        np.add.at(c, (h - 1 - iy, ix), 1)
        return c

    c_obs, c_flr = counts(obs), counts(flr)
    img = np.full((h, w), UNKNOWN, np.uint8)
    img[c_flr >= a.min_hits] = FREE
    img[c_obs >= a.min_hits] = OCC          # obstacle wins over floor

    n_occ = int((img == OCC).sum())
    n_free = int((img == FREE).sum())
    n_unk = int((img == UNKNOWN).sum())
    print('cells: occupied %d (%.1f%%)  free %d (%.1f%%)  unknown %d (%.1f%%)'
          % (n_occ, 100.0*n_occ/img.size, n_free, 100.0*n_free/img.size,
             n_unk, 100.0*n_unk/img.size))

    write_pgm(out + '.pgm', img, 'G1 Mid360 - built live, %s' % os.path.basename(src))
    with open(out + '.yaml', 'w') as f:
        f.write('image: %s\n' % os.path.basename(out + '.pgm'))
        f.write('resolution: %.6f\n' % a.res)
        f.write('origin:\n- %.9f\n- %.9f\n- 0.0\n' % (x0, y0))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                'mode: trinary\n')
    print('wrote %s.pgm (%d bytes) and %s.yaml'
          % (out, os.path.getsize(out + '.pgm'), out))


if __name__ == '__main__':
    main()
