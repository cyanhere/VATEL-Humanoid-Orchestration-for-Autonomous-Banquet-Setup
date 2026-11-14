"""Build livemap/relocalizer_live.py from growmap/relocalizer_grow.py.

Both originals are read-only here.

The difference: there is NO prior map and NO surveyed frame to line up with, so
the start pose DEFINES the origin. x, y and yaw are therefore 0 by definition -
nothing to specify and nothing to get wrong. The only quantity that still
matters is the sensor's height above the floor, because the cloud must have the
floor at z=0, and that is MEASURED from the scan rather than typed in.
"""
import io, os

SRC = '/home/unitree/.scuba/growmap/relocalizer_grow.py'
DST = '/home/unitree/.scuba/livemap/relocalizer_live.py'
s = io.open(SRC, encoding='utf-8').read()
n0 = len(s)


def rep(old, new):
    global s
    assert s.count(old) == 1, 'anchor x%d: %r' % (s.count(old), old[:70])
    s = s.replace(old, new)


rep('"""GROW-MAP VARIANT - see ~/.scuba/growmap/README.md. The original at\n'
    '~/.scuba/reloc/relocalizer.py is untouched.',
    '"""LIVE-MAP VARIANT - see ~/.scuba/livemap/README.md. Built from\n'
    'growmap/relocalizer_grow.py; that file and ~/.scuba/reloc/relocalizer.py\n'
    'are untouched.\n'
    '\n'
    'NO PRIOR MAP AND NO SURVEYED FRAME. The robot starts wherever it starts\n'
    'and that pose IS the map origin, so seed_x, seed_y and seed_yaw are 0 by\n'
    'definition - there is nothing to measure and nothing to get wrong.\n'
    '\n'
    'What still matters is the sensor height above the floor, because the\n'
    'stored cloud must have the floor at z=0 (z_band, and every consumer,\n'
    'assume it). That is MEASURED from the first scans, not typed in:\n'
    'gravity levelling already tells us which way is up, so the floor is the\n'
    'dominant horizontal layer below the sensor. A hand-typed seed_z of 1.2 m\n'
    'when the truth was ~1.4 m put the first run\'s whole map 0.20 m under the\n'
    'floor - the measured floor peak sat at -0.20 m instead of 0.')

# ---- parameters: origin is the start pose; height is measured
rep("        p('map_npy', os.path.expanduser("
    "'~/.scuba/growmap/maps/live/live_cloud.npy'))",
    "        p('map_npy', os.path.expanduser("
    "'~/.scuba/livemap/maps/live/live_cloud.npy'))")
rep("        p('seed_x', 0.0); p('seed_y', 0.0); p('seed_z', 1.2); p('seed_yaw', 0.0)",
    "        # the start pose IS the origin: do not set these\n"
    "        p('seed_x', 0.0); p('seed_y', 0.0); p('seed_z', 0.0); p('seed_yaw', 0.0)\n"
    "        # measure the sensor height from the floor in the first scans\n"
    "        # instead of trusting a typed seed_z\n"
    "        p('auto_z', True)\n"
    "        p('auto_z_scans', 3)        # scans to gather before seeding\n"
    "        p('auto_z_wait', 3.0)       # ...or seed anyway after this long")

rep("        self.seed_yaw, self.seed_z = float(g('seed_yaw')), float(g('seed_z'))",
    "        self.seed_yaw, self.seed_z = float(g('seed_yaw')), float(g('seed_z'))\n"
    "        self.auto_z = bool(g('auto_z'))\n"
    "        self.auto_z_scans = int(g('auto_z_scans'))\n"
    "        self.auto_z_wait = float(g('auto_z_wait'))")

# ---- wait for a few scans before seeding, so the floor estimate has data
rep("""            if not self.level or len(self.imu_buf) >= 50:
                self._apply_seed(T)""",
"""            ready = (not self.level) or len(self.imu_buf) >= 50
            if ready and self.auto_z and len(self.accum) < self.auto_z_scans \\
                    and time.time() - self.odom_first < self.auto_z_wait:
                return          # a few more scans first, for the floor estimate
            if ready:
                self._apply_seed(T)""")

# ---- the floor measurement, and its use
rep("""    def _apply_seed(self, T_odom_body):""",
"""    def _floor_z(self, R, T_odom_body):
        \"\"\"Sensor height above the floor, measured from the scan.

        CALLER HOLDS THE LOCK - same contract as _seed_R. on_odom() runs this
        path inside `with self.lock` and threading.Lock is not reentrant, so
        taking it again here deadlocks the first odometry callback.

        Gravity levelling already fixes which way is up, so the floor is the
        dominant horizontal layer below the sensor. Returns the z offset that
        puts that layer at 0, or None if there is not enough to go on.
        \"\"\"
        if not self.accum:
            return None
        P = np.vstack(self.accum)
        if len(P) < 500:
            return None
        T = np.eye(4)
        T[:3, :3] = R
        T = T @ np.linalg.inv(T_odom_body)      # map <- odom, with z offset 0
        z = P @ T[2, :3] + T[2, 3]
        z = z[np.isfinite(z)]
        if len(z) < 500:
            return None
        # the floor is the densest layer in the lowest ~2 m of the scan
        lo = float(np.percentile(z, 1))
        sel = z[(z > lo - 0.10) & (z < lo + 2.0)]
        if len(sel) < 200:
            return None
        hist, edges = np.histogram(sel, bins=42)
        k = int(np.argmax(hist))
        floor = 0.5 * float(edges[k] + edges[k + 1])
        share = float(hist[k]) / max(len(sel), 1)
        if share < 0.04:
            return None             # no dominant layer: refuse to guess
        self._floor_share = share
        return -floor

    def _apply_seed(self, T_odom_body):""")

rep("""        R, up = self._seed_R(self.seed_yaw)
        self.T_seed[:3, :3] = R
        self.T = self.T_seed @ np.linalg.inv(T_odom_body)""",
"""        R, up = self._seed_R(self.seed_yaw)
        self.T_seed[:3, :3] = R
        if self.auto_z:
            z0 = self._floor_z(R, T_odom_body)
            if z0 is None:
                self.get_logger().warn(
                    'auto_z: no dominant floor layer in %d scans - falling back '
                    'to seed_z=%.2f. If the map ends up offset in z, this is '
                    'why.' % (len(self.accum), self.seed_z))
            else:
                self.T_seed[2, 3] = z0
                self.seed_z = z0
                self.get_logger().info(
                    'auto_z: floor measured at %.2f m below the sensor '
                    '(%.0f%% of the lowest layer) -> seed_z %.2f'
                    % (z0, 100.0 * getattr(self, '_floor_share', 0.0), z0))
        self.T = self.T_seed @ np.linalg.inv(T_odom_body)""")

compile(s, DST, 'exec')
os.makedirs(os.path.dirname(DST), exist_ok=True)
io.open(DST, 'w', encoding='utf-8').write(s)
os.chmod(DST, 0o755)
print('built %s  (%d -> %d bytes)' % (DST, n0, len(s)))
