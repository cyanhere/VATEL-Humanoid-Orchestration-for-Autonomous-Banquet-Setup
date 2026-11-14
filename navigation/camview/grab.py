#!/usr/bin/env python3
"""One-shot colour + aligned depth capture, with geometry stats."""
import sys, time, json
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

S = {}
br = CvBridge()
rospy.init_node('grab', anonymous=True, disable_signals=True)
rospy.Subscriber('/camera/color/image_raw', Image,
                 lambda m: S.setdefault('c', br.imgmsg_to_cv2(m, 'bgr8')), queue_size=1)
rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image,
                 lambda m: S.setdefault('d', br.imgmsg_to_cv2(m, 'passthrough')), queue_size=1)
rospy.Subscriber('/camera/color/camera_info', CameraInfo,
                 lambda m: S.setdefault('K', (m.K[0], m.K[4], m.K[2], m.K[5])), queue_size=1)
t0 = time.time()
while time.time() - t0 < 15 and not all(k in S for k in ('c', 'd', 'K')):
    time.sleep(0.1)
if not all(k in S for k in ('c', 'd', 'K')):
    print('missing:', [k for k in ('c','d','K') if k not in S]); sys.exit(1)

c, d, (fx, fy, cx, cy) = S['c'], S['d'], S['K']
cv2.imwrite('/tmp/grab_color.jpg', c, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
dm = d.astype(np.float32) / 1000.0
valid = dm > 0
print('colour %s  depth %s  valid %.1f%%' % (c.shape, d.shape, 100*valid.mean()))
print('intrinsics fx %.1f fy %.1f cx %.1f cy %.1f' % (fx, fy, cx, cy))
z = dm[valid]
print('depth range %.2f .. %.2f m   median %.2f' % (z.min(), z.max(), np.median(z)))
h, e = np.histogram(z, bins=np.arange(0.0, 3.0, 0.1))
print('depth histogram:')
for cc, lo in zip(h, e[:-1]):
    if cc: print('  %4.1f m %s %d' % (lo, '#'*int(40*cc/h.max()), cc))

# deproject to camera-optical XYZ
H, W = dm.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
X = (u - cx) / fx * dm; Y = (v - cy) / fy * dm; Z = dm
P = np.stack([X[valid], Y[valid], Z[valid]], 1)
print('\npoints %d' % len(P))

# dominant plane by RANSAC - is there a flat surface, and how flat?
rs = np.random.RandomState(0); best = None
for _ in range(400):
    i = rs.choice(len(P), 3, replace=False)
    a, b, cc_ = P[i]
    n = np.cross(b - a, cc_ - a); nn = np.linalg.norm(n)
    if nn < 1e-9: continue
    n = n / nn
    dist = np.abs((P - a) @ n)
    inl = int((dist < 0.01).sum())
    if best is None or inl > best[0]: best = (inl, n, a)
inl, n, a = best
dist = np.abs((P - a) @ n)
m = dist < 0.01
print('dominant plane: %d inliers (%.0f%% of points), rms %.4f m'
      % (inl, 100.0*inl/len(P), float(np.sqrt((dist[m]**2).mean()))))
print('  plane normal in optical frame:', np.round(n, 3))
ang = np.degrees(np.arccos(abs(n @ np.array([0,0,1.0]))))
print('  angle between plane normal and optical axis: %.1f deg' % ang)
# how much of the FRAME is that plane?
mask = np.zeros(dm.shape, bool); mask[valid] = m
print('  that plane covers %.0f%% of the image' % (100.0*mask.mean()))
cv2.imwrite('/tmp/grab_plane.png', (mask*255).astype(np.uint8))

dn = np.clip((dm - 0.2)/(1.2-0.2), 0, 1); dn[~valid] = 1.0
cv2.imwrite('/tmp/grab_depth.png',
            cv2.applyColorMap((255*(1-dn)).astype(np.uint8), cv2.COLORMAP_TURBO))
json.dump({'fx':fx,'fy':fy,'cx':cx,'cy':cy,'plane_normal':n.tolist(),
           'plane_cover':float(mask.mean())}, open('/tmp/grab.json','w'))
print('\nwrote /tmp/grab_color.jpg /tmp/grab_depth.png /tmp/grab_plane.png')
