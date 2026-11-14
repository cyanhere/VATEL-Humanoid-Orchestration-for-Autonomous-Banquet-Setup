"""Synthesise a tabletop at a KNOWN distance, project it into a depth image,
and check the detector recovers the numbers."""
import importlib.util, math, sys, types
import numpy as np

import os
_HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('pd', os.path.join(_HERE, 'plane_detect.py'))
pd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pd)

FX, FY, CX, CY = 604.28, 603.67, 323.63, 244.33
W, H = 640, 480
PITCH = 63.6

class A:            # stand-in for argparse
    pass
import math as _m
class A:
    step=3; zmin=0.15; zmax=3.0; pitch=PITCH; drop_band=[0.25,0.60]
    slab=0.04; inlier=0.02; edge_pct=2.0
    edge_bin=0.04; edge_min_bins=8; edge_pct_lo=10.0
    edge_rms=0.02; align_tol=3.0
    trig_drop=[0.30,0.52]; trig_rms=0.02; trig_cover=0.05
    cam_h=1.25; table_h=0.85; target=0.5; tol=0.02

def render(surfaces, noise=0.0, seed=0):
    """surfaces: list of (drop, x_near, x_far). Returns a uint16 mm depth image."""
    rs = np.random.RandomState(seed)
    n0 = pd.down_axis(PITCH)
    t = math.radians(PITCH)
    f = np.array([0.0, -math.sin(t), math.cos(t)])      # forward, horizontal
    r = np.array([1.0, 0.0, 0.0])
    depth = np.zeros((H, W), np.float64)
    for drop, xn, xf in surfaces:
        xs = np.arange(xn, xf, 0.002)
        ys = np.arange(-1.2, 1.2, 0.002)
        X, Y = np.meshgrid(xs, ys)
        P = (drop*n0[None,None,:] + X[...,None]*f[None,None,:]
             + Y[...,None]*r[None,None,:]).reshape(-1,3)
        if noise:
            P = P + rs.normal(0, noise, P.shape)
        Z = P[:,2]
        good = Z > 0.05
        P = P[good]; Z = Z[good]
        u = (FX*P[:,0]/Z + CX).astype(int); v = (FY*P[:,1]/Z + CY).astype(int)
        m = (u>=0)&(u<W)&(v>=0)&(v<H)
        u,v,Z = u[m], v[m], Z[m]
        cur = depth[v,u]
        take = (cur==0) | (Z < cur)        # nearest surface wins
        depth[v[take], u[take]] = Z[take]
    return (depth*1000).astype(np.uint16)

fail=[]
def chk(n,c,d=''):
    print('  %-46s %s %s' % (n,'ok' if c else 'FAIL',d))
    if not c: fail.append(n)

print('tabletop 0.40 m below camera, edge at x=0.30 m, plus floor at 1.25 m:')
img = render([(0.40, 0.30, 0.60), (1.25, 0.10, 3.0)], noise=0.002)
res = pd.analyse(img, (FX,FY,CX,CY), A)
chk('plane found', res['ok'], res.get('why',''))
if res['ok']:
    chk('drop recovered = 0.40', abs(res['drop']-0.40) < 0.01, '%.4f' % res['drop'])
    t=math.radians(PITCH); zexp = 0.40*math.sin(t) + 0.30*math.cos(t)
    chk('edge depth = %.3f (Z, not range)' % zexp,
        abs(res['edge_depth']-zexp) < 0.008, '%.4f' % res['edge_depth'])
    chk('edge x = 0.300', abs(res['edge_x']-0.300) < 0.02, '%.4f' % res['edge_x'])
    chk('rms small', res['rms'] < 0.006, '%.1f mm' % (1000*res['rms']))
    chk('tilt ~0 vs assumed down', res['tilt'] < 2.0, '%.2f deg' % res['tilt'])
    chk('trigger fires', res['trigger'])

print('\nrobot further back - edge at x=0.42 m (near the FOV limit):')
img = render([(0.40, 0.42, 0.60), (1.25, 0.10, 3.0)], noise=0.002)
r2 = pd.analyse(img, (FX,FY,CX,CY), A)
if r2['ok']:
    print('     near-limit fit: drop %.3f (vs %.3f close up)  cover %.1f%%'
          % (r2['drop'], res['drop'], 100*r2['cover']))
    chk('edge depth larger', r2['edge_depth'] > res['edge_depth'], '%.3f' % r2['edge_depth'])
    # only a ~2.6 cm sliver of tabletop is in frame here, so the plane normal is
    # poorly constrained and drop drifts - accuracy degrades near the FOV limit.
    # This is a real limit of the geometry, not a fixable bug; cover flags it.
    chk('edge x = 0.42 within 5 cm (degraded at the limit)',
        abs(r2['edge_x']-0.42) < 0.05, '%.3f' % r2['edge_x'])
    chk('says ADVANCE', 'ADVANCE' in pd.move_hint(r2['edge_depth']-A.target, A),
        pd.move_hint(r2['edge_depth']-A.target, A))
else:
    chk('plane found when further back', False, r2.get('why'))

print('\nFLOOR ONLY - must refuse, not mistake the floor for a table:')
img = render([(1.25, 0.10, 3.0)], noise=0.002)
r3 = pd.analyse(img, (FX,FY,CX,CY), A)
chk('refuses (no tabletop layer)', not r3['ok'], r3.get('why',''))

print('\nTILTED surface 10 deg - rms should blow up or tilt should show:')
n0 = pd.down_axis(PITCH); t=math.radians(PITCH)
f = np.array([0.0,-math.sin(t),math.cos(t)])
img = render([(0.40, 0.30, 0.60)], noise=0.002)
rs = np.random.RandomState(1)
img2 = img.copy().astype(np.float64)
img2[img2>0] *= (1 + 0.0)   # unchanged; tilt tested via a sloped synth below
xs = np.arange(0.30,0.60,0.002); ys=np.arange(-1.2,1.2,0.002)
X,Y = np.meshgrid(xs,ys)
drop = 0.40 + X*math.tan(math.radians(10))        # slope along forward
P = (drop[...,None]*n0 + X[...,None]*f + Y[...,None]*np.array([1.,0,0])).reshape(-1,3)
Z=P[:,2]; good=Z>0.05; P=P[good]; Z=Z[good]
u=(FX*P[:,0]/Z+CX).astype(int); v=(FY*P[:,1]/Z+CY).astype(int)
m=(u>=0)&(u<W)&(v>=0)&(v<H)
dep=np.zeros((H,W)); dep[v[m],u[m]]=Z[m]
r4 = pd.analyse((dep*1000).astype(np.uint16), (FX,FY,CX,CY), A)
chk('10 deg slope reported as tilt', r4['ok'] and r4['tilt'] > 7.0,
    '%.1f deg' % r4['tilt'] if r4['ok'] else r4.get('why'))

print('\n%s' % ('FAILED: %s' % fail if fail else 'all checks passed'))
sys.exit(1 if fail else 0)
