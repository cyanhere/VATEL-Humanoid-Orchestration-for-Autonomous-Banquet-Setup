#!/usr/bin/env python3
"""Who is burning the CPU, what are they publishing, and who is being starved.

    /usr/bin/python3 ~/.scuba/nav/loadprobe.py
    /usr/bin/python3 ~/.scuba/nav/loadprobe.py --secs 30
    /usr/bin/python3 ~/.scuba/nav/loadprobe.py --skip /livox/lidar

READ-ONLY. It subscribes and reads /proc. It publishes nothing, commands
nothing, and kills nothing.

WHY NOT `ros2 topic hz`
Measured 2026-10-29 on this robot at load 16: `ros2 topic hz` reported
"NO MESSAGES in 12s" for /tf, /Odometry and /cloud_registered while all three
were in fact publishing at 26, 9 and 9 Hz. Under load the CLI's own python
process does not get scheduled often enough to see its own subscription. Every
rate here is counted inside ONE node instead, and the per-process numbers come
from /proc, which cannot be starved.

WHY NOT `ps` %CPU
`ps` pcpu is the average over the process's whole LIFETIME. A process that ate
4 cores for the first minute and has been idle for an hour still reads high.
This samples utime+stime at both ends of the window and divides by the window,
so the number is what that process is doing NOW.

THE THREE NUMBERS THAT MATTER
  cpu%     work done. 100% = one core saturated.
  waited   ms spent on the runqueue, ready but with no CPU to run on
           (/proc/<pid>/schedstat field 2). THIS is the lag number: a process
           whose `waited` approaches its run time is being starved, and its
           output is late no matter how fast the machine's clock is.
  cpu/Hz   cost per message published. The trimming metric: a process with a
           high cpu% and no topics is pure compute, and a process with a high
           cpu% per Hz is doing a lot of work per message it hands anyone.
"""
import argparse
import os
import time
import collections

TICK = os.sysconf('SC_CLK_TCK')

# node name -> a substring of the cmdline of the process that hosts it.
# Needed where the two do not share a name. Nav2 hosts the costmap nodes
# INSIDE the planner/controller processes, which is why global_costmap has no
# process of its own.
NODE_HINTS = {
    '/laser_mapping': 'fastlio_mapping',
    '/relocalizer': 'relocalizer_live.py',
    '/map_accumulator': 'accumulator.py',
    '/costmap_decay': 'costmap_decay.py',
    '/path_viewer': 'path_viewer.py',
    '/g1_nav_bridge': 'nav_bridge',
    '/livox_driver_node': 'livox_ros_driver2_node',
    '/global_costmap/global_costmap': 'nav2_planner/planner_server',
    '/local_costmap/local_costmap': 'nav2_controller/controller_server',
    '/planner_server': 'nav2_planner/planner_server',
    '/controller_server': 'nav2_controller/controller_server',
    '/lifecycle_manager': 'nav2_lifecycle_manager',
    '/table_mission': 'table_mission.py',
    '/table_registry': 'table_registry.py',
    '/mount_tf': 'mount_tf',
}

# processes that are ROS 1 (Noetic) or not ROS at all. They will never appear
# in the ROS 2 graph, so "no topics" is expected and not a finding.
ROS1_MARKS = ('ros/noetic', 'nodelet', 'roscore', 'rosmaster', 'roslaunch',
              'plane_detect.py', 'camview')


def proc_snapshot(pids=None):
    """utime+stime ticks, runqueue wait ns, thread count, cmdline per pid."""
    out = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        pid = int(d)
        if pids is not None and pid not in pids:
            continue
        try:
            with open('/proc/%d/stat' % pid) as f:
                st = f.read()
            # comm can contain spaces and parens; split after the last ')'
            tail = st[st.rindex(')') + 2:].split()
            utime, stime = int(tail[11]), int(tail[12])
            threads = int(tail[17])
            state = tail[0]
            run_ns = wait_ns = 0
            try:
                with open('/proc/%d/schedstat' % pid) as f:
                    a = f.read().split()
                run_ns, wait_ns = int(a[0]), int(a[1])
            except Exception:
                pass
            with open('/proc/%d/cmdline' % pid) as f:
                cmd = f.read().replace('\0', ' ').strip()
            if not cmd:
                continue
            out[pid] = dict(ticks=utime + stime, run_ns=run_ns, wait_ns=wait_ns,
                            threads=threads, state=state, cmd=cmd)
        except Exception:
            continue
    return out


def short(cmd, n=58):
    """A readable label: the script or binary name, plus telling args."""
    parts = cmd.split()
    if not parts:
        return '?'
    exe = os.path.basename(parts[0])
    rest = parts[1:]
    if exe.startswith('python') and rest:
        # the first non-flag argument is the script
        for i, r in enumerate(rest):
            if not r.startswith('-'):
                exe = os.path.basename(r)
                rest = rest[i + 1:]
                break
    lab = exe
    for r in rest:
        if r.startswith('--ros-args'):
            break
        if len(lab) + len(r) + 1 > n:
            break
        lab += ' ' + r
    return lab[:n]


def main():
    ap = argparse.ArgumentParser(
        description='Per-process CPU and runqueue starvation, per-topic rate '
                    'and bandwidth, and who publishes what. Read-only.')
    ap.add_argument('--secs', type=float, default=15.0,
                    help='measurement window (default 15)')
    ap.add_argument('--skip', nargs='*', default=[],
                    help='topics not to subscribe to. Subscribing to a big '
                         'cloud costs real bandwidth, so exclude it if the '
                         'probe itself perturbs what you are measuring')
    ap.add_argument('--all-topics', action='store_true',
                    help='also /rosout and /parameter_events')
    ap.add_argument('--light', action='store_true',
                    help='skip the high-rate sensor topics. MEASURED: '
                         'subscribing to everything cost this probe 68.6%% of '
                         'a core on an already oversubscribed box, which '
                         'perturbs the thing being measured. Rates for the '
                         'skipped topics are still reported from the publisher '
                         'side by --secs on a later run.')
    ap.add_argument('--top', type=int, default=18,
                    help='how many processes to list')
    ap.add_argument('--min-cpu', type=float, default=0.5,
                    help='%% CPU below which a process is not listed')
    ap.add_argument('--lag-frames', nargs=2, default=['map', 'body'],
                    metavar=('PARENT', 'CHILD'),
                    help='the TF whose stamp lag to report. map->body is '
                         'Nav2 robot_base_frame and every servo reads it')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, \
        DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    import tf2_ros

    rclpy.init()
    n = Node('loadprobe')

    # BEST_EFFORT + VOLATILE is the maximally COMPATIBLE subscriber: it can
    # read a RELIABLE publisher and a TRANSIENT_LOCAL one, while the reverse
    # pairing is incompatible and would silently receive nothing.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.VOLATILE)

    # let discovery settle, or half the graph is invisible
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    topics = sorted(n.get_topic_names_and_types())
    boring = () if a.all_topics else ('/rosout', '/parameter_events')
    # the four that cost the most to receive, measured 2026-10-30
    HEAVY = ('/livox/lidar', '/livox/imu', '/Laser_map',
             '/cloud_registered_body', '/cloud_registered',
             '/state_estimator/odom_pelvis', '/state_estimator/odom_torso')
    if a.light:
        a.skip = list(a.skip) + [h for h in HEAVY if h not in a.skip]
    count = collections.Counter()
    bytes_ = collections.Counter()
    subs = []
    unsubbed = []

    for name, types in topics:
        if name in a.skip or name in boring:
            unsubbed.append((name, 'skipped'))
            continue
        try:
            cls = get_message(types[0])
        except Exception as e:
            unsubbed.append((name, 'no type: %s' % e))
            continue

        def cb(msg, name=name):
            count[name] += 1
            bytes_[name] += len(msg)

        try:
            # raw=True hands the callback the serialized bytes: it gives the
            # true wire size AND skips deserialising big clouds, so the probe
            # stays cheap enough not to distort its own measurement.
            subs.append(n.create_subscription(cls, name, cb, qos, raw=True))
        except TypeError:
            try:
                def cb2(msg, name=name):
                    count[name] += 1
                subs.append(n.create_subscription(cls, name, cb2, qos))
                unsubbed.append((name, 'no raw= on this rclpy: rate only'))
            except Exception as e:
                unsubbed.append((name, str(e)[:60]))
        except Exception as e:
            unsubbed.append((name, str(e)[:60]))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    # publisher map BEFORE the window, so the lookups do not steal from it
    pubs = {}
    for name, _ in topics:
        try:
            pubs[name] = [p.node_name for p in
                          n.get_publishers_info_by_topic(name)]
        except Exception:
            pubs[name] = []

    print('measuring for %.0f s over %d topics (%d subscribed)...'
          % (a.secs, len(topics), len(subs)))

    before = proc_snapshot()
    lag = []
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(n, timeout_sec=0.02)
        try:
            tr = buf.lookup_transform(a.lag_frames[0], a.lag_frames[1],
                                      rclpy.time.Time())
            s = tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9
            lag.append(n.get_clock().now().nanoseconds / 1e9 - s)
        except Exception:
            pass
    el = time.time() - t0
    after = proc_snapshot(set(before))

    # ---- node -> pid
    def pid_for_node(node_name):
        # Normalise BOTH sides: the graph API returns bare names, while a
        # fully-qualified name carries a leading slash and may be namespaced.
        key = '/' + node_name.lstrip('/')
        hint = NODE_HINTS.get(key) or NODE_HINTS.get(node_name)
        base = key.lstrip('/').split('/')[0]

        def wrapper(c):
            # `ros2 run <pkg> <exe>` and roslaunch start a PYTHON launcher
            # whose command line also contains the executable's name, so a
            # plain substring test matches the launcher as well as the real
            # process - and the launcher does no work and publishes nothing.
            # Rank it below the real one instead of letting dict order decide.
            return (('/bin/ros2' in c and ' run ' in c)
                    or 'roslaunch' in c or '/bin/ros2 launch' in c)

        scored = []
        for pid, v in after.items():
            c = v['cmd']
            s = 0
            if hint and hint in c:
                s = 3
            elif base and (base in c or
                           base.replace('_', '') in c.replace('_', '')):
                s = 1
            if not s:
                continue
            if wrapper(c):
                s -= 2          # still a candidate, but the last resort
            scored.append((s, pid))
        if not scored:
            return None
        scored.sort(reverse=True)
        best = scored[0][0]
        top = [pid for s, pid in scored if s == best]
        # ambiguous only if two NON-wrapper processes tie
        return top[0] if len(top) == 1 else None

    node2pid = {}
    for name, nodes in pubs.items():
        for nd in nodes:
            if nd not in node2pid:
                node2pid[nd] = pid_for_node(nd)

    # ---- per process rollup
    rows = []
    for pid, aft in after.items():
        bef = before.get(pid)
        if not bef:
            continue
        cpu = (aft['ticks'] - bef['ticks']) / TICK / el * 100.0
        waited = (aft['wait_ns'] - bef['wait_ns']) / 1e6
        ran = (aft['run_ns'] - bef['run_ns']) / 1e6
        mine = [t for t, nds in pubs.items()
                if any(node2pid.get(x) == pid for x in nds)]
        hz = sum(count[t] for t in mine) / el
        mbs = sum(bytes_[t] for t in mine) / el / 1e6
        rows.append(dict(pid=pid, cpu=cpu, waited=waited, ran=ran,
                         threads=aft['threads'], label=short(aft['cmd']),
                         cmd=aft['cmd'], topics=mine, hz=hz, mbs=mbs))
    rows.sort(key=lambda r: -r['cpu'])

    # A process that STARTED during the window has no "before" sample, so a
    # delta cannot be taken and it would silently vanish from the table - the
    # worst kind of omission in a tool whose whole job is finding the heavy
    # one. Report those separately, using lifetime average, which is all that
    # is available for them.
    newborn = []
    for pid, aft in after.items():
        if pid in before:
            continue
        try:
            with open('/proc/%d/stat' % pid) as f:
                st = f.read()
            tail = st[st.rindex(')') + 2:].split()
            start_ticks = int(tail[19])
            with open('/proc/uptime') as f:
                up = float(f.read().split()[0])
            age = max(up - start_ticks / TICK, 1e-3)
            newborn.append(dict(pid=pid, age=age,
                                cpu=aft['ticks'] / TICK / age * 100.0,
                                label=short(aft['cmd'])))
        except Exception:
            continue
    newborn.sort(key=lambda r: -r['cpu'])

    ncpu = os.cpu_count() or 1
    tot = sum(r['cpu'] for r in rows)
    print('\n' + '=' * 78)
    print('PROCESSES  (window %.1f s, %d cores = %d%% available)'
          % (el, ncpu, ncpu * 100))
    print('=' * 78)
    print('%7s %-40s %6s %5s %8s %7s' %
          ('pid', 'what', 'cpu%', 'thr', 'waited', 'pub Hz'))
    shown = 0
    for r in rows:
        if r['cpu'] < a.min_cpu or shown >= a.top:
            continue
        shown += 1
        print('%7d %-40s %6.1f %5d %7.0fms %7.1f'
              % (r['pid'], r['label'][:40], r['cpu'], r['threads'],
                 r['waited'], r['hz']))
    print('-' * 78)
    me = next((r['cpu'] for r in rows if r['pid'] == os.getpid()), 0.0)
    print('%49.1f%% of %d%% used' % (tot, ncpu * 100))
    if me > 1.0:
        print('%49.1f%% of that is THIS PROBE (subtract it: %.1f%%)'
              % (me, tot - me))
        print('%49s' % '--light cuts it')
    if tot > ncpu * 100 * 0.85:
        print('!! OVERSUBSCRIBED. Above ~85%% of capacity, scheduling delay')
        print('   grows faster than load: everything is late, not just the')
        print('   heaviest thing.')

    fresh = [r for r in newborn if r['cpu'] >= a.min_cpu]
    if fresh:
        print('\nSTARTED DURING THE WINDOW (lifetime average, not windowed):')
        for r in fresh[:6]:
            print('%7d %-40s %6.1f  (age %.1fs)'
                  % (r['pid'], r['label'][:40], r['cpu'], r['age']))

    print('\n' + '=' * 78)
    print('STARVED  (waited on the runqueue vs actually ran)')
    print('=' * 78)
    # Require real runtime before trusting a ratio: over a few ms it is noise,
    # and the noise outranks the signal.
    MIN_RAN_MS = 200.0
    st = sorted((r for r in rows if r['ran'] >= MIN_RAN_MS),
                key=lambda r: -(r['waited'] / max(r['ran'], 1e-9)))
    tiny = [r for r in rows if 1.0 <= r['ran'] < MIN_RAN_MS
            and r['waited'] > r['ran']]
    print('%-40s %9s %9s %7s' % ('what', 'ran', 'waited', 'ratio'))
    for r in st[:10]:
        ratio = r['waited'] / max(r['ran'], 1e-9)
        flag = '  <-- STARVED' if ratio > 0.5 else ''
        print('%-40s %8.0fms %8.0fms %6.2f%s'
              % (r['label'][:40], r['ran'], r['waited'], ratio, flag))
    if tiny:
        print('\n   (%d process(es) ran < %.0f ms and are not ranked: a ratio '
              'over\n    that little runtime is noise)' % (len(tiny), MIN_RAN_MS))
    print('\n   ratio > 0.5 means it spent half again as long waiting for a')
    print('   CPU as using one. Its output is LATE. A ratio near 1.0 on the')
    print('   odometry node is the signature behind "the map is drifting".')

    print('\n' + '=' * 78)
    print('TRIM CANDIDATES  (cost vs what they deliver)')
    print('=' * 78)
    for r in rows:
        if r['cpu'] < 5.0:
            continue
        ros1 = any(m in r['cmd'] for m in ROS1_MARKS)
        if not r['topics']:
            why = ('ROS 1 / non-ROS: invisible to the ROS 2 graph, so "no '
                   'topics" is expected' if ros1
                   else 'publishes NOTHING on ROS 2 - pure compute or a '
                        'subscriber only')
            print('  %6.1f%%  %-34s %s' % (r['cpu'], r['label'][:34], why))
        else:
            per = r['cpu'] / r['hz'] if r['hz'] > 0.05 else float('inf')
            print('  %6.1f%%  %-34s %.1f Hz, %.2f MB/s -> %s cpu%% per Hz'
                  % (r['cpu'], r['label'][:34], r['hz'], r['mbs'],
                     ('%.2f' % per) if per != float('inf') else 'inf'))

    print('\n' + '=' * 78)
    print('TOPICS by bandwidth')
    print('=' * 78)
    print('%-34s %8s %10s  %s' % ('topic', 'Hz', 'MB/s', 'publisher'))
    for t in sorted(count, key=lambda t: -bytes_[t])[:16]:
        print('%-34s %8.2f %10.3f  %s'
              % (t[:34], count[t] / el, bytes_[t] / el / 1e6,
                 ','.join(pubs.get(t, []))[:26]))
    silent = [t for t, _ in topics
              if t not in boring and t not in a.skip and count[t] == 0]
    if silent:
        print('\n   silent (0 msgs in %.0f s): %s' % (el, ' '.join(silent[:12])))
        print('   A topic can also read silent because no publisher matched')
        print('   our QoS - but BEST_EFFORT/VOLATILE is compatible with')
        print('   everything, so silence here is real.')

    if lag:
        lag.sort()
        print('\n' + '=' * 78)
        print('POSE LAG  %s -> %s' % (a.lag_frames[0], a.lag_frames[1]))
        print('=' * 78)
        med = lag[len(lag) // 2]
        print('   median %+.3f s   min %+.3f   max %+.3f  (n %d)'
              % (med, lag[0], lag[-1], len(lag)))
        print('   at 0.25 m/s that is %.2f m of position error' % (med * 0.25))
        print('   at 10.2 deg/s that is %.1f deg of heading error' % (med * 10.2))
        if med > 0.6:
            print('   !! EXCEEDS --yaw-settle 0.6 s. Any servo closing on this')
            print('      TF reads a pose from BEFORE its own last step, so it')
            print('      cannot converge however its steps are sized.')
    else:
        print('\n   no %s->%s TF seen' % tuple(a.lag_frames))

    for t, why in unsubbed[:8]:
        print('   note: %s - %s' % (t, why))

    n.destroy_node()
    rclpy.shutdown()


main()
