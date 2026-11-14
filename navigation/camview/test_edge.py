"""Render a tabletop whose near edge is at a KNOWN angle; check recovery."""
import importlib.util, os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('pd', os.path.join(HERE, 'plane_detect.py'))
pd = importlib.util.module_from_spec(spec); spec.loader.exec_module(pd)

FX, FY, CX, CY, W, H, PITCH = 604.28, 603.67, 323.63, 244.33, 640, 480, 63.6

class A:
    step=3; zmin=0.15; zmax=3.0; pitch=PITCH; drop_band=[0.25,0.60]
    slab=0.04; inlier=0.02; edge_pct=2.0
    edge_bin=0.04; edge_min_bins=8; edge_pct_lo=10.0; edge_rms=0.02; align_tol=3.0
    trig_drop=[0.35,0.45]; trig_rms=0.02; trig_cover=0.015
    cam_h=1.25; table_h=0.85
    target=0.40*math.sin(math.radians(PITCH))+0.30*math.cos(math.radians(PITCH))
    tol=0.02

def render_table(drop, b0, depth, phi_deg, half_w=1.2, noise=0.002, seed=0):
    """Tabletop whose NEAR boundary is the line b = b0 + a*tan(phi)."""
    rs = np.random.RandomState(seed)
    n0 = pd.down_axis(PITCH); r, f = pd.horiz_axes(PITCH)
    a = np.arange(-half_w, half_w, 0.004)
    b = np.arange(0.0, depth, 0.004)
    Agr, Bgr = np.meshgrid(a, b)
    Bgr = Bgr + b0 + Agr*math.tan(math.radians(phi_deg))
    P = (drop*n0[None,None,:] + Agr[...,None]*r[None,None,:]
         + Bgr[...,None]*f[None,None,:]).reshape(-1,3)
    P = P + rs.normal(0, noise, P.shape)
    # floor, so the detector has to pick the right layer
    bf = np.arange(0.1, 3.0, 0.01); af = np.arange(-2.0, 2.0, 0.01)
    Af, Bf = np.meshgrid(af, bf)
    F_ = (1.25*n0[None,None,:] + Af[...,None]*r[None,None,:]
          + Bf[...,None]*f[None,None,:]).reshape(-1,3)
    dep = np.zeros((H, W))
    for Q in (F_, P):                      # table last: nearest wins anyway
        Z = Q[:,2]; g = Z > 0.05; Q2, Z = Q[g], Z[g]
        u = (FX*Q2[:,0]/Z + CX).astype(int); v = (FY*Q2[:,1]/Z + CY).astype(int)
        m = (u>=0)&(u<W)&(v>=0)&(v<H); u,v,Z = u[m],v[m],Z[m]
        cur = dep[v,u]; take = (cur==0)|(Z<cur)
        dep[v[take],u[take]] = Z[take]
    return (dep*1000).astype(np.uint16)

fail=[]
def chk(n,c,d=''):
    print('  %-50s %s %s'%(n,'ok' if c else 'FAIL',d))
    if not c: fail.append(n)

print('edge angle recovery (table 0.40 m below, near edge 0.30 m forward):')
print('  %8s %10s %9s %7s %s' % ('true','measured','err deg','rms mm','verdict'))
for phi in (-20.0,-10.0,-5.0,0.0,5.0,10.0,20.0):
    img = render_table(0.40, 0.30, 0.45, phi)
    r = pd.analyse(img, (FX,FY,CX,CY), A)
    if not r['ok']:
        chk('phi=%+.0f plane found'%phi, False, r.get('why')); continue
    ea = r.get('edge_angle')
    if ea is None:
        chk('phi=%+.0f edge found'%phi, False, '%d bins'%r['edge_bins']); continue
    err = ea - phi
    good = abs(err) < 2.5
    print('  %+8.1f %10.1f %9.2f %7.1f %s'
          % (phi, ea, err, 1000*r['edge_rms'], 'ok' if good else 'FAIL'))
    if not good: fail.append('phi %+.0f'%phi)

print()
img = render_table(0.40, 0.30, 0.45, 0.0)
r = pd.analyse(img,(FX,FY,CX,CY),A)
chk('square table reports aligned', r.get('aligned') is True,
    'angle %+.2f'%r['edge_angle'])
img = render_table(0.40, 0.30, 0.45, 12.0)
r = pd.analyse(img,(FX,FY,CX,CY),A)
chk('12 deg table reports NOT aligned', r.get('aligned') is False,
    'angle %+.2f'%r['edge_angle'])
chk('sign: +phi means right-hand end further', r['edge_angle'] > 0,
    '%+.1f'%r['edge_angle'])

# a curved / ragged boundary must not be reported as a straight edge
img = render_table(0.40, 0.30, 0.45, 0.0, noise=0.05, seed=7)
r = pd.analyse(img,(FX,FY,CX,CY),A)
if r['ok'] and r.get('edge_angle') is not None:
    chk('ragged boundary -> high edge rms', r['edge_rms'] > 0.01,
        '%.0f mm'%(1000*r['edge_rms']))
else:
    chk('ragged boundary handled', True, 'no edge reported')

print('\n%s'%('FAILED: %s'%fail if fail else 'all checks passed'))
sys.exit(1 if fail else 0)
