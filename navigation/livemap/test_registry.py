"""The property the design rests on: entries survive re-segmentation."""
import importlib.util, os, sys, math
import numpy as np
HERE=os.path.dirname(os.path.abspath(__file__))
spec=importlib.util.spec_from_file_location('tr', os.path.join(HERE,'table_registry.py'))
tr=importlib.util.module_from_spec(spec); spec.loader.exec_module(tr)
fail=[]
def chk(n,c,d=''):
    print('  %-52s %s %s'%(n,'ok' if c else 'FAIL',d))
    if not c: fail.append(n)

KEEP=['TABLE corner','TABLE face','partial']
R=tr.Registry(0.75, KEEP)
# pass 1: two tables
c1=[dict(cx=2.0,cy=0.0,L=1.35,W=0.7,yaw=0.0,n=20,cls='TABLE face'),
    dict(cx=6.0,cy=1.0,L=1.40,W=1.4,yaw=0.3,n=50,cls='TABLE corner')]
R.update(c1, 100.0)
chk('two clusters -> two entries', len(R.e)==2, str(len(R.e)))
ids1=sorted(R.e.keys())

# pass 2: SAME tables, geometry改善 and centroids shifted 0.2 m (map grew)
c2=[dict(cx=2.15,cy=0.10,L=1.40,W=1.38,yaw=0.02,n=55,cls='TABLE corner'),
    dict(cx=6.1,cy=1.05,L=1.41,W=1.40,yaw=0.31,n=70,cls='TABLE corner')]
R.update(c2, 110.0)
chk('same two entries, no duplicates', len(R.e)==2, str(len(R.e)))
chk('ids unchanged', sorted(R.e.keys())==ids1)
e=R.e[ids1[0]]
chk('geometry updated as the map improved', abs(e['L']-1.40)<0.01, '%.2f'%e['L'])
chk('n_obs incremented', e['n_obs']==2, str(e['n_obs']))

# mark one VISITED, then see it again - state must NOT regress
R.event(dict(id=ids1[0], state='VISITED'))
chk('event marks VISITED', R.e[ids1[0]]['state']=='VISITED')
R.update(c2, 120.0)
chk('VISITED survives re-detection', R.e[ids1[0]]['state']=='VISITED',
    R.e[ids1[0]]['state'])
chk('visited entry drops out of the mission queue',
    ids1[0] not in [x['id'] for x in R.ranked(0,0)])

# a genuinely new table far away must NOT absorb into an existing entry
R.update(c2+[dict(cx=12.0,cy=0.0,L=1.4,W=1.4,yaw=0,n=40,cls='TABLE corner')],130.0)
chk('a new distant cluster makes a NEW entry', len(R.e)==3, str(len(R.e)))
# a cluster just inside the gate must NOT
R.update(c2+[dict(cx=2.15+0.5,cy=0.10,L=1.4,W=1.4,yaw=0,n=40,cls='TABLE corner')],140.0)
chk('a cluster within the gate does not duplicate', len(R.e)==3, str(len(R.e)))

# ranking is nearest-first from the robot
R2=tr.Registry(0.75,KEEP)
R2.update([dict(cx=5.0,cy=0,L=1.4,W=1.4,yaw=0,n=30,cls='TABLE corner'),
           dict(cx=1.0,cy=0,L=1.4,W=1.4,yaw=0,n=30,cls='TABLE corner')],100.0)
rk=R2.ranked(0.0,0.0)
chk('ranked nearest first', abs(rk[0]['range']-1.0)<1e-6, '%.2f'%rk[0]['range'])

# standoff geometry
en=dict(cx=0.0,cy=0.0,yaw=0.0)
ps=tr.standoff_poses(en,0.30,0.28)
d=[math.hypot(p['x'],p['y']) for p in ps]
chk('4 sides', len(ps)==4)
chk('all at SIDE/2+standoff+radius', all(abs(x-(0.70+0.30+0.28))<1e-9 for x in d),
    '%.3f'%d[0])
for p in ps:
    hx,hy=math.cos(p['yaw']),math.sin(p['yaw'])
    dot=(hx*(-p['x'])+hy*(-p['y']))/math.hypot(p['x'],p['y'])
    if abs(dot-1.0)>1e-9: fail.append('pose faces table'); break
else:
    chk('every pose faces the table centre', True)
print('\n%s'%('FAILED: %s'%fail if fail else 'all checks passed'))
sys.exit(1 if fail else 0)
