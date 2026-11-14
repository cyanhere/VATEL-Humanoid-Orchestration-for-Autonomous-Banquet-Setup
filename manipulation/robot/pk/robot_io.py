"""Robot backends behind one interface: DDS on the robot, a simulated arm (MockBackend) for offline tests."""
import time

import numpy as np

from . import config


class State(object):
    """One snapshot of the robot. Arrays are copies; times are time.time()."""
    __slots__ = ("t", "fresh_t", "tick", "q", "dq", "tau", "temp", "motor_err", "head", "waist_roll", "rpy", "legs",
                 "hand_q", "hand_cur", "hand_t", "touch", "touch_t", "mode_machine")

    def __init__(self):
        self.t = self.fresh_t = 0.0
        self.tick = -1
        self.q, self.dq, self.tau = np.zeros(config.N_ARM), np.zeros(config.N_ARM), np.zeros(config.N_ARM)
        self.temp = np.zeros(config.N_ARM)
        self.motor_err = False
        self.head = np.zeros(2)
        self.waist_roll = 0.0
        self.rpy = np.zeros(3)
        self.legs = np.zeros(12)                 # lowstate slots 0..11 (hip pitch/roll/yaw, knee, ankle pitch/roll; left, right)
        self.hand_q, self.hand_cur = np.zeros(12), np.zeros(12)
        self.hand_t = [0.0, 0.0]                 # last new hand state, left / right
        self.touch = np.zeros((2, 5, 4))         # side x [Thumb, Index, Middle, Ring, Pinky] x [normal, tangential, dir, proximity]
        self.touch_t = [0.0, 0.0]
        self.mode_machine = 0

    def age(self, now=None):
        return (now or time.time()) - self.fresh_t

    def hands_ok(self, now=None, max_age=0.3):
        now = now or time.time()
        return all(now - t < max_age for t in self.hand_t)

    def touch_ok(self, now=None, max_age=0.3):
        now = now or time.time()
        return all(now - t < max_age for t in self.touch_t)


# ======================================================================================== the real robot
class DDSBackend(object):
    def __init__(self, iface="eth10", domain=0):
        from cyclonedds.qos import Policy, Qos
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from unitree_sdk2py.core.channel import ChannelFactory, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        ChannelFactoryInitialize(domain, iface)          # raises if DDS cannot start on that interface
        part = ChannelFactory()._ChannelFactory__participant
        qos = Qos(Policy.History.KeepLast(1), Policy.Reliability.BestEffort)
        self._low = DataReader(part, Topic(part, "rt/lowstate", LowState_), qos)
        self._hand = [DataReader(part, Topic(part, "rt/brainco/%s/state" % s, MotorStates_), qos) for s in ("left", "right")]
        self._touch = [DataReader(part, Topic(part, "rt/brainco/%s/touch" % s, MotorStates_), qos) for s in ("left", "right")]
        self.s = State()
        self._pub_arm = self._pub_hand = None
        self._cmd = self._hcmd = None
        self._crc = None
        self.iface = iface
        self.low_msg = None                              # the last rt/lowstate and hand state messages, whole (demo/record.py)
        self.hand_msg = [None, None]

    def read(self):
        s, now = self.s, time.time()
        s.t = now
        got = self._low.take(N=1)
        if got:
            m = got[0]
            self.low_msg = m
            if m.tick != s.tick:
                s.tick, s.fresh_t = m.tick, now
            ms = m.motor_state
            err = False
            for i, slot in enumerate(config.ARM_SLOT):
                x = ms[slot]
                s.q[i], s.dq[i], s.tau[i] = x.q, x.dq, x.tau_est
                s.temp[i] = x.temperature[0] if hasattr(x.temperature, "__len__") else x.temperature
                err = err or (x.motorstate != 0)
            s.motor_err = err
            s.head[0], s.head[1] = ms[config.HEAD_SLOT[0]].q, ms[config.HEAD_SLOT[1]].q
            s.waist_roll = ms[config.WAIST_ROLL_SLOT].q
            for i in range(12):
                s.legs[i] = ms[i].q
            s.rpy[:] = m.imu_state.rpy
            s.mode_machine = m.mode_machine
        for k in range(2):
            h = self._hand[k].take(N=1)
            if h:
                self.hand_msg[k] = h[0]
                st = h[0].states
                for i in range(min(6, len(st))):
                    s.hand_q[6 * k + i], s.hand_cur[6 * k + i] = st[i].q, st[i].tau_est
                s.hand_t[k] = now
            tc = self._touch[k].take(N=1)
            if tc:
                st = tc[0].states
                for i in range(min(5, len(st))):
                    s.touch[k, i] = (st[i].q, st[i].dq, st[i].ddq, st[i].reserve[0])
                s.touch_t[k] = now
        return s

    def open_arm(self):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        self._pub_arm = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub_arm.Init()
        self.open_hands()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()

    def open_hands(self):
        """Only the hand publishers (hands.py, when no arm server runs)."""
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
        self._pub_hand = []
        for s in ("left", "right"):
            p = ChannelPublisher("rt/brainco/%s/cmd" % s, MotorCmds_)
            p.Init()
            self._pub_hand.append(p)
        self._hcmd = [MotorCmds_(cmds=[unitree_go_msg_dds__MotorCmd_() for _ in range(6)]) for _ in range(2)]

    def send_arm(self, q11, head2, weight, kp11, kd11, dq11=None):
        """dq11: the planned joint speed as the motors' speed target (their damping then pulls toward the plan's speed
        instead of toward standing still); None = 0, as before."""
        c = self._cmd
        c.mode_pr = int(round(100 * min(max(weight, 0.0), 1.0)))      # R1: arm_sdk blend weight, percent
        c.mode_machine = int(self.s.mode_machine)
        for i, slot in enumerate(config.ARM_SLOT):
            m = c.motor_cmd[slot]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, float(q11[i]), 0.0 if dq11 is None else float(dq11[i]), 0.0, float(kp11[i]), float(kd11[i])
        for i, slot in enumerate(config.HEAD_SLOT):
            m = c.motor_cmd[slot]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, float(head2[i]), 0.0, 0.0, config.HEAD_KP, config.HEAD_KD
        c.crc = self._crc.Crc(c)
        self._pub_arm.Write(c)

    def send_hands(self, left6, right6):
        for k, v in enumerate((left6, right6)):
            self.send_hand(k, v)

    def send_hand(self, k, v6):
        """One hand: k 0 = the robot's left, 1 = its right."""
        for i in range(6):
            m = self._hcmd[k].cmds[i]
            m.q = float(min(max(v6[i], 0.0), 1.0))
            m.dq = 1.0                                       # speed, vendor setting
        self._pub_hand[k].Write(self._hcmd[k])

    def close(self):
        pass


# ======================================================================================== offline
C_TABLE = 60.0                                          # N s/m, the mock table's contact damping


class MockBackend(object):
    """Simulated R1 arms for offline tests (no DDS).

    Each arm joint: PD at the commanded gains blended with the robot's own controller, URDF gravity times
    `grav_scale`, Coulomb friction and first-order motor dynamics. Fingers follow their command at a limited speed
    and can be stopped by an obstacle (finger_stop), then report touch counts. `contact`: optional function
    (pinky end point, velocity) -> force on the left pinky end, for push tests.
    """

    def __init__(self, grav_scale=1.2, friction=0.4, rest=None, seed=0, robot=None):
        from .model import Robot, PINKY_END
        self.rob = robot or Robot(offsets=None)       # the simulated arm itself: its real joints, no calibration
        self.pinky_end = PINKY_END
        self.s = State()
        self.rng = np.random.RandomState(seed)
        self.grav_scale, self.friction = grav_scale, friction
        rest = np.array([-0.156, 0.103, 0.527, 1.557, 0.201, 0.025, -0.040, -0.146, 1.259, -0.116, 0.0102]) if rest is None else rest
        # the robot's own pose for the arms (hanging, FINDINGS); the waist as it read on not 0
        self.rest = np.asarray(rest, float)
        self.s.q[:] = self.rest
        self.s.head[:] = (0.0, 0.0)
        self.s.legs[:] = [-0.28, 0.058, -0.144, 0.401, -0.177, -0.069, -0.329, -0.032, 0.139, 0.425, -0.156, 0.031]
        self.v = np.zeros(config.N_ARM)
        self.cmd = None
        self.hcmd = np.zeros(12)
        self.finger_stop = np.ones(12) * 2.0            # > 1: no obstacle
        self.contact = None                             # fn(p, v) -> force (3,) on the left pinky end, pelvis frame
        # or several left-hand points (push10 tests): [(name, link, p_local, pad finger or None)]; then contact(p, v, name),
        # and a point with a pad finger ("Index") makes that pad read 100 counts per newton
        self.contact_pts = None
        self.pad_counts_per_N, self.pad_dead_N = 100.0, 0.0   # below the dead band a pad reads 0 (a load on the tip, which has no sensor)
        self.prox_contact = None                        # fn(N) -> raw counts: a pad finger's proximity under that load
        self.t_sim = time.time()
        self.opened = False
        self.sent = []                                  # (t, weight) of every arm command, for tests
        self.hands_alive = True
        self.touch_alive = True                         # False: no rt/brainco/*/touch
        self.last_force = None
        self.payload = None                             # (side, mass, link, point in link) carried by a hand
        self.open_delay = 0.6
        # arm extended: ~3.7 kg at ~0.25 m plus reflected rotor inertia; motor damping (viscous) as the real arm's smooth logs
        self.inertia = np.array([0.40, 0.30, 0.15, 0.15, 0.05] * 2 + [1.0])
        self.damping = 1.0
        self.limp = False                               # the robot's mode ignores arm_sdk: no torque, the arms rest
        self.stiction = 1.0                             # breakaway torque / sliding friction (> 1: stick-slip, as the R1)
        self.proximity = None                           # fn(point, pelvis frame) -> raw counts: the left pads' proximity
        # a flat table under both hands (table_check tests): dict(z, pts={side: [(link, N x 3 local points)]}, k, prox=fn(gap
        # mm) -> counts); a hand's lowest listed point is pushed up out of it
        self.table = None
        self.table_force = {}
        # encoder zero errors (rad, N_ARM): the motors and what is reported work from (true angle - enc_off), so the arm
        # really sits enc_off away from where its joints say (table_check grid / arm_offsets_fit tests)
        self.enc_off = np.zeros(config.N_ARM)
        self._out = None

    def read(self):
        now = time.time()
        self._advance(now)
        s = self.s
        s.t = s.fresh_t = now
        s.tick += 1
        if self.hands_alive:
            s.hand_t = [now, now]
            if self.touch_alive:
                s.touch_t = [now, now]
        if np.any(self.enc_off):
            o = self._out = self._out or State()
            for k in State.__slots__:
                v = getattr(s, k)
                setattr(o, k, v.copy() if isinstance(v, np.ndarray) else v)
            o.q = s.q - self.enc_off
            return o
        return s

    def _advance(self, now):
        dt_total = min(now - self.t_sim, 0.1)
        self.t_sim = now
        # gravity and contact change slowly: once per read, then 2 ms dynamics sub-steps
        T = self.rob.fk(self.s.q, self.s.hand_q)
        g = np.zeros(config.N_ARM)
        g[config.LEFT] = self.rob.gravity(T, "left")
        g[config.RIGHT] = self.rob.gravity(T, "right")
        g *= self.grav_scale                            # torque the motors must supply to hold still
        if self.payload is not None:                    # e.g. the plate in the right hand (tests set it)
            side, mass, link, p_loc = self.payload
            c = T[link][:3, :3] @ np.asarray(p_loc, float) + T[link][:3, 3]
            sl = config.LEFT if side == "left" else config.RIGHT
            g[sl] += self.rob.gravity(T, side, extra=[(mass, c)]) - self.rob.gravity(T, side)
        # contact: evaluated every 2 ms sub-step at the fingertip, linearised from this read's Jacobian
        p0 = J = None
        pts = []
        if self.contact is not None and self.contact_pts is not None:
            for name, link, p_loc, finger in self.contact_pts:
                pp = self.rob.point(T, link, p_loc)
                pts.append((name, pp, self.rob.jacobian(T, "left", pp), finger))
        elif self.contact is not None:
            p0 = self.rob.point(T, "left_pinky_distal_link", self.pinky_end["left"])
            J = self.rob.jacobian(T, "left", p0)
        pad_F = {}
        q0 = self.s.q[config.LEFT].copy()
        self.last_force = None
        tab = {}
        if self.table is not None:
            for side in ("left", "right"):
                low = None
                for ln, v in self.table["pts"][side]:
                    w = v @ T[ln][:3, :3].T + T[ln][:3, 3]
                    k = int(np.argmin(w[:, 2]))
                    if low is None or w[k, 2] < low[2]:
                        low = w[k]
                s_ = config.LEFT if side == "left" else config.RIGHT
                tab[side] = (s_, low, self.rob.jacobian(T, side, low), self.s.q[s_].copy())
            self.table_force = {}
        n = max(1, int(dt_total / 0.002))
        dt = dt_total / n
        for _ in range(n):
            ext = np.zeros(config.N_ARM)
            F = None
            if J is not None:
                p = p0 + J @ (self.s.q[config.LEFT] - q0)
                v = J @ self.v[config.LEFT]
                F = self.contact(p, v)
                if F is not None and np.any(F):
                    ext[config.LEFT] = J.T @ F
                    self.last_force = F
            for name, pp0, Jc, finger in pts:
                Fc = self.contact(pp0 + Jc @ (self.s.q[config.LEFT] - q0), Jc @ self.v[config.LEFT], name)
                if Fc is not None and np.any(Fc):
                    ext[config.LEFT] += Jc.T @ Fc
                    self.last_force = Fc if self.last_force is None else self.last_force + Fc
                if finger:
                    pad_F[finger] = Fc
            for side, (s_, pt0, Jt, qt0) in tab.items():
                p = pt0 + Jt @ (self.s.q[s_] - qt0)
                gh = self.table.get("ghost")             # a stop in free air, no proximity (a joint stuck by friction)
                if gh and gh["side"] == side and p[2] > gh["z"] + 0.01 and time.time() > gh.get("arm_after", 0.0):
                    gh["armed"] = True                   # only from above, and only once the test is under way
                if gh and gh["side"] == side and gh.get("armed") and not gh.get("broken") and p[2] < gh["z"]:
                    Fg = self.table.get("k", 4000.0) * (gh["z"] - p[2])
                    if Fg > gh["break_N"]:
                        gh["broken"] = True
                    else:
                        ext[s_] += Jt.T @ np.array([0.0, 0.0, Fg])
                if p[2] < self.table["z"]:
                    vz = float((Jt @ self.v[s_])[2])
                    Ft = np.array([0.0, 0.0, max(0.0, self.table.get("k", 4000.0) * (self.table["z"] - p[2]) - C_TABLE * vz)])
                    ext[s_] += Jt.T @ Ft
                    self.table_force[side] = Ft
            self._step(dt, g, ext, F)
        for finger, Fc in pad_F.items():
            self.s.touch[0, config.TOUCH_FINGERS.index(finger), 0] = min(2500.0, self.pad_counts_per_N * max(0.0, float(np.linalg.norm(Fc)) - self.pad_dead_N))
            if self.prox_contact is not None:
                self.s.touch[0, config.TOUCH_FINGERS.index(finger), 3] = self.prox_contact(float(np.linalg.norm(Fc)))
        for side, (s_, pt0, Jt, qt0) in tab.items():
            k = 0 if side == "left" else 1
            Ft = self.table_force.get(side)
            if Ft is not None:
                self.s.touch[k, 2, 0] = min(2500.0, 150.0 * float(np.linalg.norm(Ft)))     # the middle finger's pad
            if self.table.get("prox"):
                p = pt0 + Jt @ (self.s.q[s_] - qt0)
                self.s.touch[k, 4, 3] = self.table["prox"](1000 * max(0.0, float(p[2]) - self.table["z"]))
        if self.proximity is not None:
            # the little finger: its lowest point (as the R1's curve was measured); the index: its pad's centre
            from .model import PINKY_LOW, pad
            T = self.rob.fk(self.s.q, self.s.hand_q)
            M = T["left_pinky_distal_link"]
            low = PINKY_LOW["left"] @ M[:3, :3].T + M[:3, 3]
            self.s.touch[0, 4, 3] = self.proximity(low[np.argmin(low[:, 2])])
            self.s.touch[0, 1, 3] = self.proximity(pad(T, "left", "index")[0])

    def _step(self, dt, g, ext, F):
        s = self.s
        q, v = s.q, self.v
        if self.limp:                                   # as the R1 on still, every motor ~0 N m
            v[:] = 0.0
            s.dq[:], s.tau[:] = 0.0, 0.0
        else:
            own = 20.0 * (self.rest - q) - 2.0 * v + g  # the robot's own arm controller (with gravity compensation)
            if self.cmd is not None:
                qc, w, kp, kd, _ = self.cmd
                pd = kp * (qc - (q - self.enc_off)) + kd * (getattr(self, "cmd_dq", 0.0) - v)
                tau_m = (1 - w) * own + w * pd
            else:
                tau_m = own
            tau_net = tau_m - g + ext
            fr = self.friction * np.tanh(v / 0.01) + self.damping * v
            a = (tau_net - fr) / self.inertia
            stuck = (np.abs(v) < 1e-3) & (np.abs(tau_net) < self.friction * self.stiction)
            a[stuck] = 0.0
            v[:] = np.where(stuck, 0.0, v + a * dt)
            q += v * dt
            q[:] = np.minimum(np.maximum(q, config.ARM_LO), config.ARM_HI)
            s.dq[:], s.tau[:] = v, tau_m
            if self.cmd is not None:
                hd = self.cmd[4]
                s.head += np.clip(hd - s.head, -1.0 * dt, 1.0 * dt)
        # hands: 1.2 / s towards the command, stopped by obstacles
        tgt = np.minimum(self.hcmd, self.finger_stop)
        s.hand_q += np.clip(tgt - s.hand_q, -1.2 * dt, 1.2 * dt)
        s.touch[:] = 0.0
        press = np.maximum(self.hcmd - self.finger_stop, 0.0)
        for side in range(2):
            for i, f in enumerate(config.TOUCH_FINGERS):
                k = 6 * side + config.FINGERS.index(f)
                s.touch[side, i, 0] = min(2500.0, 8000.0 * press[k])
        if F is not None and np.any(F):
            s.touch[0, 4, 0] = min(2500.0, 150.0 * float(np.linalg.norm(F)))     # left pinky pad
        s.hand_cur[:] = 0.0

    def open_arm(self):
        time.sleep(self.open_delay)                     # the real SDK pauses ~0.2 s per publisher it creates
        self.opened = True

    def send_arm(self, q11, head2, weight, kp11, kd11, dq11=None):
        assert self.opened, "send_arm before open_arm"
        self.cmd = (np.array(q11, float), float(weight), np.array(kp11, float), np.array(kd11, float), np.array(head2, float))
        self.cmd_dq = np.zeros(config.N_ARM) if dq11 is None else np.array(dq11, float)
        self.sent.append((time.time(), float(weight)))

    def open_hands(self):
        pass

    def send_hands(self, left6, right6):
        self.hcmd = np.clip(np.r_[left6, right6], 0.0, 1.0)

    def send_hand(self, k, v6):
        self.hcmd[6 * k:6 * k + 6] = np.clip(v6, 0.0, 1.0)

    def close(self):
        pass
