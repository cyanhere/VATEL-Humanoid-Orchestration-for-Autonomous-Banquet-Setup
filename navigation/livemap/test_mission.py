"""Test the parts that decide WHICH WAY the robot moves."""
import importlib.util, os, sys, math, json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
HERE=os.path.dirname(os.path.abspath(__file__))
spec=importlib.util.spec_from_file_location('tm', os.path.join(HERE,'table_mission.py'))
tm=importlib.util.module_from_spec(spec); spec.loader.exec_module(tm)
fail=[]
def chk(n,c,d=''):
    print('  %-54s %s %s'%(n,'ok' if c else 'FAIL',d))
    if not c: fail.append(n)

PITCH=63.6; DROP=1.262-0.85
TARGET=DROP*math.sin(math.radians(PITCH))+0.30*math.cos(math.radians(PITCH))
print('target edge depth for a 0.30 m standoff: %.3f m\n' % TARGET)

print('approach direction:')
far = tm.approach_move(TARGET+0.05, TARGET, PITCH)
near= tm.approach_move(TARGET-0.05, TARGET, PITCH)
chk('reads LONGER than target -> move FORWARD', far > 0, '%+.3f m'%far)
chk('reads SHORTER than target -> move BACK',   near < 0, '%+.3f m'%near)
chk('at target -> no move', abs(tm.approach_move(TARGET,TARGET,PITCH))<1e-9)
chk('depth error scales up by 1/cos(pitch)',
    abs(far - 0.05/math.cos(math.radians(PITCH))) < 1e-9, '%.3f'%far)
# a 0.05 m depth error is 0.11 m of ground - the gait cannot creep that
chk('0.05 m depth error = 11 cm of ground', abs(far-0.112)<0.002, '%.3f m'%far)

print('\nalign direction:')
chk('right end further (+angle) -> turn LEFT (+wz)', tm.align_move(12.0) > 0)
chk('left end further (-angle) -> turn RIGHT (-wz)', tm.align_move(-12.0) < 0)

print('\nside poses (1.40 m square, radius 0.28):')
P=tm.side_poses(0.0,0.0,0.0,1.40,0.28,{'stage':0.90,'stand':0.30})
chk('4 sides x 2 distances', len(P)==8, str(len(P)))
for sd in 'ABCD':
    xs,ys,_=P[(sd,'stand')]
    chk('side %s standoff at 0.70+0.30+0.28'%sd,
        abs(math.hypot(xs,ys)-1.28)<1e-9, '%.3f'%math.hypot(xs,ys))
for sd in 'ABCD':
    xg,yg,_=P[(sd,'stage')]
    chk('side %s stage at 0.70+0.90+0.28'%sd,
        abs(math.hypot(xg,yg)-1.88)<1e-9, '%.3f'%math.hypot(xg,yg))
for sd in 'ABCD':
    x,y,yaw=P[(sd,'stand')]
    d=math.hypot(x,y)
    dot=(math.cos(yaw)*(-x)+math.sin(yaw)*(-y))/d
    chk('side %s faces the table'%sd, abs(dot-1.0)<1e-9, '%.6f'%dot)
# the stage pose must clear the costmap's 0.50 m no-go band around the table
clear = 1.88 - 0.70 - 0.28
chk('stage pose clears the 0.50 m no-go band', clear > 0.50, '%.2f m clear'%clear)
stand_clear = 1.28 - 0.70 - 0.28
chk('standoff pose is INSIDE it (hence the back-off)', stand_clear < 0.50,
    '%.2f m'%stand_clear)
# adjacent sides must be far enough apart to be distinct goals
d_adj = math.hypot(P[('A','stage')][0]-P[('B','stage')][0],
                   P[('A','stage')][1]-P[('B','stage')][1])
chk('adjacent stage poses are distinct', d_adj > 1.0, '%.2f m apart'%d_adj)

print('\nplane client (fake server):')
PAY={'ok':True,'trigger':True,'edge_depth':TARGET,'edge_x':0.30,
     'edge_angle':1.2,'edge_rms':0.006,'rms':0.004}
class H(BaseHTTPRequestHandler):
    def log_message(self,*_): pass
    def do_GET(self):
        b=json.dumps(PAY).encode()
        self.send_response(200); self.send_header('Content-Length',str(len(b)))
        self.end_headers(); self.wfile.write(b)
srv=ThreadingHTTPServer(('127.0.0.1',8891),H); srv.daemon_threads=True
threading.Thread(target=srv.serve_forever,daemon=True).start()
p=tm.Plane('http://127.0.0.1:8891/stat', 2.0)
chk('reads a good payload', p.fresh_ok() is not None)
PAY['trigger']=False
chk('rejects when trigger is false', p.fresh_ok() is None)
PAY['trigger']=True
p2=tm.Plane('http://127.0.0.1:9999/stat', 2.0)       # nothing listening
chk('unreachable detector -> None, no crash', p2.fresh_ok() is None)
# staleness only matters when the FETCH FAILS - read() refreshes otherwise.
# So: get a good reading, kill the server, and check the cached value expires
# instead of being served forever.
# max_age must exceed ThreadingHTTPServer.shutdown()'s poll interval (0.5 s),
# or the shutdown itself eats the freshness window and the test lies.
p3=tm.Plane('http://127.0.0.1:8891/stat', 3.0)
chk('cached a good reading', p3.read() is not None)
srv.shutdown(); srv.server_close()
chk('server gone, cache still fresh -> still served', p3.read() is not None)
p3.last_t = time.time() - 3.5          # now age it past max_age, fetch will fail
chk('server gone and cache expired -> None', p3.read() is None)
chk('and fresh_ok refuses too', p3.fresh_ok() is None)
print('\n%s'%('FAILED: %s'%fail if fail else 'all checks passed'))
sys.exit(1 if fail else 0)
