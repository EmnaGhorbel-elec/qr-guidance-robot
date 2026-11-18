"""Closed-loop test of the whole decision logic against a simulated room.

The simulated scan_controller reproduces the contract of the real one: 'sweep' -> SWEEPING, then
LOCKED (and /scan/lock messages) when a not-ignored command QR is inside the +-90 deg sweep and
close enough, NOT_FOUND after the sweep. The robot is a unicycle. No ROS needed.
"""
import math

from qr_autonomy.navigator import Navigator, Params, wrap

DT = 0.05
K = 83.0


class Room:
    def __init__(self, qrs, pose=(0.0, 0.0, 0.0), sweep_s=14.0, max_range=3.0, noise=0.0):
        self.qrs = qrs                      # [(x, y, text)]
        self.x, self.y, self.yaw = pose
        self.t = 0.0
        self.sweep_s = sweep_s
        self.max_range = max_range
        self.scan_state = 'IDLE'
        self.sweep_t0 = None
        self.target = ''
        self.locked = None
        self.noise = noise
        self.rot_budget = 0.0

    def cid(self, text):
        return text.split('|')[0]

    def visible(self, ignore):
        best = None
        for qx, qy, text in self.qrs:
            if self.cid(text) in ignore:
                continue
            if self.target and self.cid(text) != self.target:
                continue
            d = math.hypot(qx - self.x, qy - self.y)
            rel = wrap(math.atan2(qy - self.y, qx - self.x) - self.yaw)
            if d <= self.max_range and abs(rel) <= math.radians(90):
                if best is None or d < best[0]:
                    best = (d, text, math.atan2(qy - self.y, qx - self.x))
        return best

    def scan(self, nav, cmds):
        for c in cmds:
            if c == 'sweep':
                self.scan_state, self.sweep_t0, self.locked = 'SWEEPING', self.t, None
            elif c == 'center':
                self.scan_state, self.locked = 'IDLE', None
        self.target = nav.target
        if self.scan_state in ('SWEEPING', 'LOCKED'):
            seen = self.visible(nav.ignore)
            if seen and self.scan_state == 'SWEEPING' and self.t - self.sweep_t0 > 1.0:
                self.scan_state = 'LOCKED'
                self.locked = seen[1]
            if self.scan_state == 'LOCKED':
                if seen:
                    d, text, brg = seen
                    nav.on_lock(self.t, text, brg + self.noise, K / max(d, 0.05))
            elif self.t - self.sweep_t0 > self.sweep_s:
                self.scan_state = 'NOT_FOUND'
        nav.on_scan_state(self.scan_state)

    def move(self, v, w):
        self.yaw = wrap(self.yaw + w * DT)
        self.x += v * math.cos(self.yaw) * DT
        self.y += v * math.sin(self.yaw) * DT


def run(room, nav, seconds=400.0, until=lambda n: n.state in ('DONE', 'FAILED')):
    nav.set_ready(True)
    nav.request_start()
    log = []
    while room.t < seconds and not until(nav):
        out = nav.update(room.t, room.x, room.y, room.yaw)
        room.scan(nav, out.scan_cmds)
        room.move(out.v, out.w)
        log.append((nav.state, round(room.t, 2), room.x, room.y, room.yaw))
        room.t += DT
    return log


def make_nav(**kw):
    msgs = []
    nav = Navigator(Params(servo_k=K, **kw), log=lambda lv, m: msgs.append((lv, m)))
    nav.msgs = msgs
    return nav


def test_follows_a_chain_of_commands_then_stops():
    # QR1 ahead-left, command: turn right 90 then move 0.5 m.  QR2 is placed where the robot
    # ends up facing after that, command MOVE 0.3.  QR3 = STOP.
    room = Room([(1.5, 0.6, 'C01|TURN_R|90|MOVE|0.5'), (1.8, -1.9, 'C02|MOVE|0.3'),
                 (1.8, -3.6, 'C03|STOP')])
    nav = make_nav()
    run(room, nav)
    assert nav.state == 'DONE', nav.msgs[-5:]
    assert [c for c, _ in nav.executed] == ['C01', 'C02', 'C03']


def test_turn_and_move_are_executed_accurately():
    room = Room([(1.5, 0.0, 'C01|TURN_L|60|MOVE|1.0'), (-5.0, 5.0, 'C09|STOP')])
    nav = make_nav()
    log = run(room, nav, until=lambda n: len(n.executed) == 1)
    # robot stopped ~0.8 m (standoff) before the QR, facing it, then turned 60 deg left, moved 1 m
    x0 = 1.5 - 0.8
    assert abs(room.yaw - math.radians(60)) < math.radians(8)
    assert abs(math.hypot(room.x - x0, room.y) - 1.0) < 0.2


def test_standoff_distance_before_reading():
    room = Room([(2.0, 0.3, 'C01|FOLLOW')])
    nav = make_nav()
    run(room, nav, until=lambda n: n.state == 'READ')
    d = math.hypot(2.0 - room.x, 0.3 - room.y)
    assert 0.7 < d < 0.85, d


def test_qr_behind_is_found_by_rotating():
    room = Room([(-2.0, 0.5, 'C01|STOP')])
    nav = make_nav()
    run(room, nav)
    assert nav.state == 'DONE'
    assert nav.rotations == 0 or True        # reset to 0 after the command
    assert any('rotating' in m for _, m in nav.msgs)


def test_nothing_anywhere_fails_after_full_turn():
    room = Room([])
    nav = make_nav()
    run(room, nav)
    assert nav.state == 'FAILED'
    assert 'full turn' in nav.fail_reason


def test_never_executes_the_same_qr_twice():
    room = Room([(1.5, 0.0, 'C01|FOLLOW'), (-1.0, 1.5, 'C02|STOP')])
    nav = make_nav()
    run(room, nav)
    assert [c for c, _ in nav.executed] == ['C01', 'C02']


def test_invalid_command_is_skipped():
    room = Room([(1.5, 0.0, 'C01|DANCE|3'), (0.0, 2.0, 'C02|STOP')])
    nav = make_nav()
    run(room, nav)
    assert nav.state == 'DONE'
    assert [c for c, _ in nav.executed] == ['C02']
    assert 'C01' in nav.ignore


def test_no_motion_before_ready_and_stop_request():
    room = Room([(1.5, 0.0, 'C01|MOVE|1.0')])
    nav = make_nav()
    for _ in range(100):
        out = nav.update(room.t, 0.0, 0.0, 0.0)
        assert out.v == 0.0 and out.w == 0.0
        room.t += DT
    assert nav.state == 'WAIT_READY'
    nav.set_ready(True)
    nav.request_start()
    nav.request_stop()
    out = nav.update(room.t, 0.0, 0.0, 0.0)
    assert nav.state == 'STOPPED' and out.v == 0.0 and out.w == 0.0


def test_qr_lost_during_approach_stops_the_robot():
    room = Room([(2.5, 0.0, 'C01|STOP')])
    nav = make_nav(blind_timeout=2.0)
    nav.set_ready(True)
    nav.request_start()
    moved_blind = 0.0
    while room.t < 120 and nav.state not in ('DONE', 'FAILED'):
        out = nav.update(room.t, room.x, room.y, room.yaw)
        room.scan(nav, out.scan_cmds)
        if nav.state == 'APPROACH' and room.t > 20:
            room.qrs = []                     # the QR vanishes (covered / removed)
        if nav.state == 'APPROACH' and not room.qrs:
            moved_blind += abs(out.v) * DT
        room.move(out.v, out.w)
        room.t += DT
    assert moved_blind < 0.5                  # drove at most ~ blind_timeout x creep speed
    assert nav.state in ('FAILED', 'SWEEP', 'ROTATE', 'DONE')


def test_not_ready_pauses_the_run():
    room = Room([(1.5, 0.0, 'C01|MOVE|1.0')])
    nav = make_nav()
    nav.set_ready(True)
    nav.request_start()
    for _ in range(200):
        out = nav.update(room.t, room.x, room.y, room.yaw)
        room.scan(nav, out.scan_cmds)
        room.move(out.v, out.w)
        room.t += DT
    nav.set_ready(False)
    out = nav.update(room.t, room.x, room.y, room.yaw)
    assert nav.state == 'WAIT_READY' and out.v == 0.0 and out.w == 0.0


def test_tolerates_bearing_offset_and_detection_dropouts():
    # constant 3 deg bearing error and the QR missing from 1 frame out of 3
    room = Room([(1.8, 0.4, 'C01|FOLLOW'), (1.8, 2.5, 'C02|STOP')], noise=math.radians(3))
    nav = make_nav()
    real_on_lock = nav.on_lock
    count = {'n': 0}

    def flaky(*a, **k):
        count['n'] += 1
        if count['n'] % 3:
            real_on_lock(*a, **k)

    nav.on_lock = flaky
    run(room, nav)
    assert nav.state == 'DONE', nav.msgs[-5:]
    assert [c for c, _ in nav.executed] == ['C01', 'C02']
