#!/usr/bin/env python3
"""Start and stop plane_detect.py on demand, from a Foxy process.

    # standalone: run it for 20 s and report what it costs
    /usr/bin/python3 ~/.scuba/camview/plane_proc.py --secs 20

    # from table_mission.py (Foxy):
    from plane_proc import PlaneDetect
    with PlaneDetect(table_h=0.76, cam_h=1.262, pitch=63.6) as pd:
        ...   # search() / align() / approach() - the camera phase
    # stopped on the way out, including on an exception

WHY THIS EXISTS

plane_detect.py was measured at **251% CPU** - seven numpy/OpenBLAS worker
threads at ~31% each plus 12.7% main. On an 8-core box already at 717%/800%
that starved FAST-LIO into waiting 1127 ms on the runqueue for every 1034 ms it
ran, which put map->body 1.86 s behind real time. Nothing needs the plane
detector during edge-to-edge navigation, so it should not be running then.

TWO TRAPS THIS AVOIDS

1. THE ENV CROSSING. plane_detect is ROS 1 (Noetic); table_mission is ROS 2
   (Foxy). `camview/env.sh` deliberately exits if ROS_DISTRO=foxy, so a Foxy
   process cannot simply source it - and inheriting Foxy's AMENT_PREFIX_PATH,
   PYTHONPATH and LD_LIBRARY_PATH into a Noetic process is the documented way
   to break it. The child is therefore given a SCRUBBED environment built from
   scratch, not the parent's with edits.

2. KILLING IT. `pkill -f <path>` has, on this robot, matched the command line
   of the very shell issuing it and killed that instead. This kills by PID,
   and by process GROUP. The `exec` in the launch line means bash replaces
   itself with python, so the group is usually a single pid - but it is the
   group that is signalled anyway, to cover the two cases where it is not:
   env.sh backgrounding something, and `exec` failing so bash survives.

THE BLAS CAP

OPENBLAS/OMP/MKL thread counts are pinned to 1. A per-frame plane fit over a
few thousand points gains nothing from threaded BLAS, and the idle workers
spin. `--no-blas-cap` leaves them alone so the difference can be measured.
"""
import argparse
import os
import signal
import subprocess
import time

HOME = os.path.expanduser('~')
ENV_SH = HOME + '/.scuba/camview/env.sh'
SCRIPT = HOME + '/.scuba/camview/plane_detect.py'

# Pinned to 1: the 251% was 7 BLAS workers, not useful work.
BLAS_VARS = ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
             'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS')


def child_env(blas_cap=True):
    """A Noetic-safe environment built from scratch.

    Deliberately NOT os.environ.copy() with deletions: a Foxy shell exports a
    dozen interlocking variables (AMENT_*, COLCON_*, CMAKE_PREFIX_PATH,
    ROS_VERSION, ROS_PYTHON_VERSION, RMW_IMPLEMENTATION ...) and missing one
    leaks the wrong ROS into the child. Allow-list, never deny-list.
    """
    keep = {}
    for k in ('HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL', 'TERM',
              'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR'):
        if os.environ.get(k):
            keep[k] = os.environ[k]
    keep.setdefault('HOME', HOME)
    keep['PATH'] = '/usr/local/bin:/usr/bin:/bin:/usr/local/sbin:/usr/sbin:/sbin'
    keep['SHELL'] = '/bin/bash'
    if blas_cap:
        for v in BLAS_VARS:
            keep[v] = '1'
    return keep


class PlaneDetect(object):
    def __init__(self, table_h=0.76, cam_h=1.262, pitch=63.6, extra=None,
                 warmup=4.0, ready=None, ready_timeout=25.0, blas_cap=True,
                 log=print, env_sh=ENV_SH, script=SCRIPT):
        self.args = ['--table-h', '%g' % table_h, '--cam-h', '%g' % cam_h,
                     '--pitch', '%g' % pitch] + list(extra or [])
        self.warmup = warmup
        self.ready = ready
        self.ready_timeout = ready_timeout
        self.blas_cap = blas_cap
        self.log = log
        self.env_sh = env_sh
        self.script = script
        self.p = None
        self._t0 = None

    # ---- lifecycle
    def start(self):
        if self.p is not None and self.p.poll() is None:
            self.log('    plane_detect already running (pid %d)' % self.p.pid)
            return True
        for f in (self.env_sh, self.script):
            if not os.path.exists(f):
                self.log('    !! missing %s - not starting' % f)
                return False

        inner = ('source %s >/dev/null 2>&1 || exit 97; exec /usr/bin/python3 '
                 '%s %s' % (self.env_sh, self.script,
                            ' '.join(self.args)))
        self.log('    starting plane_detect%s'
                 % ('' if self.blas_cap else ' (BLAS cap OFF)'))
        self.p = subprocess.Popen(
            ['/bin/bash', '-lc', inner],
            env=child_env(self.blas_cap),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            # its own session => its own process group, so the whole thing can
            # be signalled at once and it does not die with our terminal
            start_new_session=True)
        self._t0 = time.time()
        self.log('      pid %d (group %d)' % (self.p.pid, self.p.pid))

        # exit 97 is env.sh refusing - the Foxy-contamination guard firing.
        time.sleep(0.6)
        rc = self.p.poll()
        if rc is not None:
            self.log('    !! plane_detect exited immediately, rc %d%s'
                     % (rc, ' - camview/env.sh refused (ROS_DISTRO leaked in)'
                        if rc == 97 else ''))
            self.p = None
            return False
        if self.wait_ready():
            return True
        # start() must be all-or-nothing. A readiness TIMEOUT leaves a live
        # process that nobody owns: the caller got False and will not call
        # stop(), so a 251% detector would run for the rest of the mission.
        # (The `with` form is safe either way; this covers start() used bare.)
        self.log('    rolling back the start')
        self.stop()
        return False

    def wait_ready(self):
        """Do not walk forward until the detector can actually see.

        Starting the forward search against a detector that is still importing
        means `search()` reads "no plane" for its first seconds and can
        conclude the table is not there.
        """
        t0 = time.time()
        while time.time() - t0 < self.warmup:
            if self.p.poll() is not None:
                self.log('    !! plane_detect died during warmup (rc %s)'
                         % self.p.poll())
                return False
            time.sleep(0.2)
        if self.ready is None:
            self.log('      warmed up %.1fs (no readiness check supplied - '
                     'this is a TIMER, not proof it is publishing)'
                     % self.warmup)
            return True
        while time.time() - t0 < self.ready_timeout:
            if self.p.poll() is not None:
                self.log('    !! plane_detect died waiting to be ready')
                return False
            try:
                if self.ready():
                    self.log('      ready after %.1fs' % (time.time() - t0))
                    return True
            except Exception as e:
                self.log('      readiness check raised: %s' % e)
            time.sleep(0.3)
        self.log('    !! plane_detect not ready in %.0fs' % self.ready_timeout)
        return False

    def alive(self):
        return self.p is not None and self.p.poll() is None

    def cpu_percent(self):
        """Average CPU over this child's life, from /proc. Covers the whole
        process group, since the BLAS workers are threads of the python."""
        if not self.alive():
            return None
        total = 0.0
        try:
            tick = os.sysconf('SC_CLK_TCK')
            for pid in self.group_pids():
                with open('/proc/%d/stat' % pid) as f:
                    st = f.read()
                tail = st[st.rindex(')') + 2:].split()
                total += (int(tail[11]) + int(tail[12])) / tick
        except Exception:
            return None
        el = max(time.time() - self._t0, 1e-3)
        return total / el * 100.0

    def cpu_window(self):
        """CPU% since the previous call to this method.

        cpu_percent() is a LIFETIME average, which is the wrong statistic for
        "did the BLAS cap take effect": startup imports are expensive and drag
        the average up for the first several seconds, so a capped run and an
        uncapped one look similar early on. This is the rate right now.
        """
        tick = os.sysconf('SC_CLK_TCK')
        total = 0.0
        try:
            for pid in self.group_pids():
                with open('/proc/%d/stat' % pid) as f:
                    st = f.read()
                tail = st[st.rindex(')') + 2:].split()
                total += (int(tail[11]) + int(tail[12])) / tick
        except Exception:
            return None
        now = time.time()
        prev = getattr(self, '_w', None)
        self._w = (now, total)
        if prev is None:
            return None
        dt = now - prev[0]
        if dt < 1e-3:
            return None
        return (total - prev[1]) / dt * 100.0

    def group_pids(self):
        """Every pid in the child's process group (bash + python)."""
        out = []
        if self.p is None:
            return out
        try:
            pgid = os.getpgid(self.p.pid)
        except Exception:
            return out
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                if os.getpgid(int(d)) == pgid:
                    out.append(int(d))
            except Exception:
                continue
        return out

    def stop(self, grace=3.0):
        if self.p is None:
            return
        pid = self.p.pid
        try:
            pgid = os.getpgid(pid)
        except Exception:
            pgid = None
        cpu = self.cpu_percent()
        if cpu is not None:
            self.log('    stopping plane_detect (pid %d, averaged %.0f%% CPU)'
                     % (pid, cpu))
        else:
            self.log('    stopping plane_detect (pid %d)' % pid)
        # the GROUP, not just the shell: bash -lc is the parent of the python
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass
        t0 = time.time()
        while time.time() - t0 < grace:
            if self.p.poll() is not None:
                break
            time.sleep(0.1)
        left = [q for q in self.group_pids()] if pgid is not None else []
        if self.p.poll() is None or left:
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    pass
            self.log('      needed SIGKILL')
        try:
            self.p.wait(timeout=2)
        except Exception:
            pass
        self.log('      stopped')
        self.p = None

    # ---- context manager: stopped even if the camera phase raises
    def __enter__(self):
        self.ok = self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


class RealSense(object):
    """The ROS 1 RealSense launch, started and stopped on demand.

    Measured 2026-10-30: the nodelet costs **65-97% CPU** and is only needed
    for the camera phase, making it the largest remaining item once
    plane_detect is capped.

    THREE THINGS THAT MAKE THIS DIFFERENT FROM plane_detect

    1. `roslaunch` puts each node it starts in its OWN process group - the
       nodelet manager and the nodelet loader had pgids of their own, not
       roslaunch's. So killpg on roslaunch does NOT reap them. SIGINT to
       roslaunch does: it runs its own orderly node shutdown. Stragglers are
       then killed by pid, found via ppid, never by cmdline match.
    2. `roscore` is NOT a child of roslaunch here (verified: pid 8910, parent
       -bash), so stopping the camera leaves the master up. We never touch it.
    3. Readiness is asked of the **ROS 1 master over plain XML-RPC**, which
       needs no ROS libraries at all and so works from a Foxy process.

    It will NOT stop a camera it did not start. If the launch is already up -
    because you have it in a terminal - this adopts it for the window and
    leaves it running on exit.
    """

    LAUNCH = ['realsense2_camera', 'rs_aligned_depth.launch',
              'enable_infra1:=false', 'enable_infra2:=false',
              'enable_gyro:=false', 'enable_accel:=false',
              'enable_fisheye:=false', 'enable_pointcloud:=false']

    def __init__(self, depth_topic='/camera/aligned_depth_to_color/image_raw',
                 master='http://localhost:11311', launch=None, warmup=2.0,
                 ready_timeout=40.0, tries=2, log=print):
        self.depth_topic = depth_topic
        self.master = master
        self.launch = list(launch or self.LAUNCH)
        self.warmup = warmup
        self.ready_timeout = ready_timeout
        self.tries = tries
        self.log = log
        self.p = None
        self.adopted = False
        self._kids = []

    # ---- readiness, via the ROS 1 master. No rospy, no env crossing.
    #
    # TRI-STATE, deliberately. An earlier version returned False on any
    # exception, which conflates "the camera is down" with "I could not ask".
    # That false negative is dangerous in start(): it would launch a SECOND
    # camera on top of a running one.
    #
    # It is not hypothetical. Measured 2026-10-30: killing plane_detect leaves
    # single-threaded rosmaster retrying contact with the dead node, and a
    # getPublishedTopics issued in that window times out while the camera is
    # perfectly healthy - 49 /camera/* topics still registered.
    def _proxy(self):
        import xmlrpc.client
        import http.client

        class T(xmlrpc.client.Transport):
            def make_connection(self, host):
                # the default ServerProxy has NO timeout and can hang forever
                return http.client.HTTPConnection(host, timeout=2.0)

        return xmlrpc.client.ServerProxy(self.master, transport=T())

    def query_topics(self):
        """List of published topic names, or None if the master could not be
        asked. None is NOT an empty list."""
        try:
            code, _, topics = self._proxy().getPublishedTopics('/plane_proc', '')
        except Exception:
            return None
        if code != 1:
            return None
        return [x[0] for x in topics]

    def topic_present(self, tries=3):
        """True / False / None(unknown). Retries before admitting unknown."""
        for i in range(tries):
            names = self.query_topics()
            if names is not None:
                return self.depth_topic in names
            if i + 1 < tries:
                time.sleep(0.6)
        return None

    def master_up(self):
        try:
            self._proxy().getPid('/plane_proc')
            return True
        except Exception:
            return False

    def children(self):
        """pids whose parent is our roslaunch - the nodelets it spawned."""
        if self.p is None:
            return []
        out = []
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                with open('/proc/%s/stat' % d) as f:
                    st = f.read()
                if int(st[st.rindex(')') + 2:].split()[1]) == self.p.pid:
                    out.append(int(d))
            except Exception:
                continue
        return out

    def start(self):
        if not self.master_up():
            self.log('    !! no ROS 1 master at %s - start roscore first; the '
                     'camera launch registers against it' % self.master)
            return False
        present = self.topic_present()
        if present is None:
            # Refuse rather than guess. Launching a second camera over a live
            # one is far worse than declining to start.
            self.log('    !! could not ask the ROS 1 master whether the camera '
                     'is up (%s timed out).' % self.master)
            self.log('       NOT launching: a second RealSense over a running '
                     'one is worse than not starting. Retry in a moment.')
            return False
        if present:
            self.adopted = True
            self.log('    RealSense is ALREADY running - adopting it, and it '
                     'will be left up on exit (we did not start it).')
            self.log('      while that terminal is open, --camera-manage '
                     'cannot free its 65-97% CPU.')
            return True

        for attempt in range(1, self.tries + 1):
            self.log('    starting RealSense%s'
                     % ('' if attempt == 1 else ' (attempt %d)' % attempt))
            inner = ('source %s >/dev/null 2>&1 || exit 97; exec roslaunch %s'
                     % (ENV_SH, ' '.join(self.launch)))
            self.p = subprocess.Popen(
                ['/bin/bash', '-lc', inner], env=child_env(True),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.log('      pid %d' % self.p.pid)
            time.sleep(self.warmup)
            t0 = time.time()
            while time.time() - t0 < self.ready_timeout:
                rc = self.p.poll()
                if rc is not None:
                    self.log('      !! roslaunch exited rc %d%s' % (rc,
                             ' - camview/env.sh refused' if rc == 97 else ''))
                    break
                if self.topic_present(tries=1) is True:
                    self.log('      %s up after %.1fs'
                             % (self.depth_topic, time.time() - t0))
                    self._kids = self.children()
                    return True
                time.sleep(0.5)
            else:
                self.log('      !! %s never appeared in %.0fs'
                         % (self.depth_topic, self.ready_timeout))
            # A RealSense that fails to enumerate often succeeds on a retry
            # after being released, which is why this loops rather than
            # giving up. USB enumeration is the flaky part, not the launch.
            self.stop()
        return False

    def stop(self):
        if self.adopted:
            self.log('    leaving the adopted RealSense running')
            self.adopted = False
            self.p = None
            return
        if self.p is None:
            return
        pid = self.p.pid
        kids = self.children() or self._kids
        self.log('    stopping RealSense (pid %d, %d node(s))' % (pid, len(kids)))
        # SIGINT, not killpg: roslaunch gives each node its own process group,
        # so killpg would leave the nodelets orphaned and still eating CPU.
        try:
            self.p.send_signal(signal.SIGINT)
        except Exception:
            pass
        t0 = time.time()
        while time.time() - t0 < 10.0:
            if self.p.poll() is not None:
                break
            time.sleep(0.2)
        if self.p.poll() is None:
            self.log('      roslaunch ignored SIGINT; terminating')
            try:
                self.p.terminate()
            except Exception:
                pass
            time.sleep(2.0)
        left = []
        for k in kids:
            try:
                os.kill(k, 0)
                left.append(k)
            except OSError:
                pass
        for k in left:
            self.log('      straggler pid %d - killing by pid' % k)
            for sg in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.kill(k, sg)
                except OSError:
                    break
                time.sleep(0.5)
        try:
            self.p.wait(timeout=2)
        except Exception:
            pass
        self.p = None
        self._kids = []
        self.log('      stopped')


class CameraStack(object):
    """RealSense + plane_detect as one window, in dependency order.

    Up: camera, then detector (the detector subscribes to the camera, and
    starting it first means it reads "no plane" while it waits).
    Down: detector, then camera.
    """

    def __init__(self, manage_camera=True, camera=None, log=print, **plane_kw):
        self.log = log
        self.manage_camera = manage_camera
        plane_kw.setdefault('log', log)
        self.plane = PlaneDetect(**plane_kw)
        self.rs = camera if camera is not None else RealSense(log=log)
        self.ok = False

    def start(self):
        if self.manage_camera and not self.rs.start():
            self.log('    camera did not come up - not starting the detector')
            self.ok = False
            return False
        if not self.plane.start():
            if self.manage_camera:
                self.rs.stop()      # all-or-nothing, as PlaneDetect.start is
            self.ok = False
            return False
        self.ok = True
        return True

    def stop(self):
        self.plane.stop()
        if self.manage_camera:
            self.rs.stop()
        self.ok = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


def main():
    ap = argparse.ArgumentParser(
        description='Start plane_detect, hold it, stop it, and report what it '
                    'cost. Use it to verify the BLAS cap.')
    ap.add_argument('--secs', type=float, default=20.0)
    ap.add_argument('--table-h', type=float, default=0.76)
    ap.add_argument('--cam-h', type=float, default=1.262)
    ap.add_argument('--pitch', type=float, default=63.6)
    ap.add_argument('--warmup', type=float, default=4.0)
    ap.add_argument('--no-blas-cap', action='store_true',
                    help='leave OPENBLAS/OMP threads alone, to measure the '
                         'difference the cap makes')
    a = ap.parse_args()

    pd = PlaneDetect(table_h=a.table_h, cam_h=a.cam_h, pitch=a.pitch,
                     warmup=a.warmup, blas_cap=not a.no_blas_cap)
    if not pd.start():
        return 1
    print('  holding %.0f s; sampling CPU' % a.secs)
    t0 = time.time()
    samples = []
    while time.time() - t0 < a.secs:
        time.sleep(2.0)
        c = pd.cpu_window()
        if c is not None:
            samples.append(c)
            print('    %5.1fs  %6.1f%% CPU now  (lifetime avg %5.1f%%, '
                  '%d pids)' % (time.time() - t0, c, pd.cpu_percent() or 0.0,
                                len(pd.group_pids())))
        else:
            pd.cpu_window()   # prime the window on the first pass
    pd.stop()
    if samples:
        s = sorted(samples)
        print('\n  CPU (windowed, startup excluded): median %.0f%%  '
              'min %.0f%%  max %.0f%%'
              % (s[len(s) // 2], s[0], s[-1]))
        print('  reference: 251%% measured 2026-10-29 with BLAS threads '
              'uncapped')
        print('  BLAS cap: %s' % ('OFF' if a.no_blas_cap else 'ON'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
