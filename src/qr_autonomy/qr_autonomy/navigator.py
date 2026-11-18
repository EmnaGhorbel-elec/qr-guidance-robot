"""Autonomous QR-command navigator. Pure Python (no ROS): the node is a thin shell around it,
so the whole logic can be tested in closed loop on a PC.

No table, no map, no known QR position. The robot only knows the commands it reads.

  WAIT_READY  gyro calibrated + start delay
  INIT        camera servo to the centre (90 deg = straight ahead)
  SWEEP       camera sweeps 90 deg right, then 90 deg left, then back to the centre (scan_controller).
              Stops at the first valid command QR: the servo angle there IS the bearing of the QR.
  ALIGN       turn on the spot (gyro heading) until the robot faces that bearing
  APPROACH    drive toward the QR, the camera keeps tracking it. Stop at `standoff` metres.
  READ        stand still, read the command text
  EXECUTE     TURN_L / TURN_R / MOVE / WAIT / FOLLOW / STOP, one after the other
  ROTATE      nothing seen in a sweep: turn `rotate_deg` on the spot and sweep again
  --> back to SWEEP, until a STOP command (DONE) or an error (FAILED)

The robot never drives toward something it does not see: if the QR is lost during the approach
for `blind_timeout` s it stops and sweeps again. Heading comes from the gyro, distances from
the QR size (k / w_px) and, between detections, from odometry.
"""
import math
from dataclasses import dataclass, field

from qr_autonomy.command_parser import command_id, parse_command

IDLE_STATES = ('DONE', 'FAILED', 'STOPPED')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class Params:
    servo_k: float = 83.0          # distance (camera -> QR) = servo_k / w_px.  MEASURE IT (README)
    standoff: float = 0.8          # m: stop this far from the QR (camera -> QR)
    slow_zone: float = 0.4         # m before the standoff where the robot slows down
    v_cruise: float = 0.15         # m/s while approaching
    v_min: float = 0.08            # m/s: below this the wheels stall (min_pwm)
    move_speed: float = 0.15       # m/s for MOVE commands
    w_max: float = 0.8             # rad/s
    w_min: float = 0.35            # rad/s: a skid-steer robot does not turn slower than this
    k_turn: float = 1.5            # turn on the spot: w = k * heading error
    k_head: float = 1.5            # heading hold while driving
    turn_tol: float = 0.07         # rad (4 deg)
    turn_hold: float = 0.25        # s inside the tolerance before a turn counts as finished
    turn_timeout: float = 25.0     # s: the robot does not rotate -> FAILED
    approach_heading_limit: float = 0.6   # rad: above this error, turn first and do not drive
    det_max_age: float = 1.0       # s: a detection older than this is not used to steer
    blind_timeout: float = 3.0     # s without the QR during ALIGN / APPROACH -> stop and sweep again
    approach_timeout: float = 90.0
    read_settle: float = 0.5       # s standing still before the text is read
    init_wait: float = 1.5         # s for the servo to reach the centre
    ack_timeout: float = 2.0       # s to wait for scan_controller to confirm a command
    sweep_timeout: float = 60.0
    rotate_deg: float = 90.0
    max_rotations: int = 4         # 4 x 90 deg = a full turn without any QR -> FAILED
    max_lost: int = 4              # QR lost this many times in a row -> FAILED
    autostart: bool = True
    start_delay: float = 5.0       # s after ready
    require_ready: bool = True


@dataclass
class Out:
    v: float = 0.0
    w: float = 0.0
    scan_cmds: list = field(default_factory=list)   # to publish on /scan/command, in order


class Navigator:
    def __init__(self, params=None, log=None):
        self.p = params or Params()
        self.log = log or (lambda level, msg: None)
        self.state = 'WAIT_READY'
        self.t_state = None
        self.ready = False
        self.t_ready = None
        self.start_requested = False
        self.stop_requested = False
        self.scan_state = ''
        self.lock = None              # dict: t (rx time), text, cid, bearing, w_px
        self.ignore = set()           # command ids already executed (or invalid)
        self.target = ''              # command id the camera must keep tracking ('' = any)
        self.executed = []            # [(cid, text)] for the log
        self.rotations = 0
        self.lost = 0
        self._cmds = []
        self._reset_motion()
        self.cid = ''
        self.text = ''
        self.actions = []
        self.idx = 0
        self.fail_reason = ''

    # ---------------------------------------------------------------- inputs
    def set_ready(self, ready):
        self.ready = bool(ready)

    def request_start(self):
        self.start_requested = True

    def request_stop(self):
        self.stop_requested = True

    def on_scan_state(self, state):
        self.scan_state = state

    def on_lock(self, t, text, bearing, w_px):
        cid = command_id(text)
        if not cid:
            return
        self.lock = {'t': t, 'text': text, 'cid': cid, 'bearing': bearing, 'w_px': w_px}

    # ---------------------------------------------------------------- helpers
    def _reset_motion(self):
        self.hold_t = None
        self.target_yaw = 0.0
        self.bearing = 0.0
        self.t_lock = None
        self.d_ref = None
        self.pos_ref = None
        self.last_lock_t = None
        self.sweep_acked = False
        self.t_cmd = None
        self.start_pos = None
        self.hold_yaw = 0.0
        self.t_end = None

    def _enter(self, state, t, keep=False):
        """Change state. keep=True carries the QR bearing / distance over (ALIGN -> APPROACH -> READ)."""
        self.log('info', f'{self.state} -> {state}')
        saved = (self.bearing, self.t_lock, self.d_ref, self.pos_ref, self.last_lock_t)
        self.state = state
        self.t_state = t
        self._reset_motion()
        if keep:
            self.bearing, self.t_lock, self.d_ref, self.pos_ref, self.last_lock_t = saved

    def _cmd(self, text):
        self._cmds.append(text)

    def _fail(self, t, reason):
        self.fail_reason = reason
        self.log('error', f'FAILED: {reason}')
        self.target = ''
        self._cmd('center')
        self._enter('FAILED', t)

    def _fresh_lock(self, t, max_age):
        lk = self.lock
        if lk is None or t - lk['t'] > max_age:
            return None
        if lk['cid'] in self.ignore:
            return None
        if self.target and lk['cid'] != self.target:
            return None
        return lk

    def _turn(self, t, err):
        """Turn on the spot toward err (rad). Returns (w, finished)."""
        p = self.p
        if abs(err) <= p.turn_tol:
            if self.hold_t is None:
                self.hold_t = t
            return 0.0, t - self.hold_t >= p.turn_hold
        self.hold_t = None
        w = clip(p.k_turn * err, -p.w_max, p.w_max)
        if abs(w) < p.w_min:
            w = math.copysign(p.w_min, err)
        return w, False

    def _start_sweep(self, t):
        self.target = ''
        self.lock = None
        self._enter('SWEEP', t)
        self.t_cmd = t
        self._cmd('sweep')

    def _qr_lost(self, t, where):
        self.lost += 1
        self.log('warn', f'QR lost during {where} ({self.lost}/{self.p.max_lost}): sweeping again')
        if self.lost > self.p.max_lost:
            self._fail(t, 'the QR keeps getting lost')
            return
        self._start_sweep(t)

    # ---------------------------------------------------------------- main step
    def update(self, t, x, y, yaw):
        """One control step. Returns Out(v, w, scan_cmds)."""
        self._cmds = []
        v = w = 0.0
        if self.t_state is None:
            self.t_state = t
        if self.stop_requested and self.state != 'STOPPED':
            self.target = ''
            self._cmd('center')
            self._enter('STOPPED', t)
        elif (self.p.require_ready and not self.ready
              and self.state not in ('WAIT_READY',) + IDLE_STATES):
            self.log('warn', 'robot not ready (Mega / gyro lost): waiting, the run restarts from a sweep')
            self.target = ''
            self._enter('WAIT_READY', t)
        handler = getattr(self, '_st_' + self.state.lower())
        res = handler(t, x, y, yaw)
        if res is not None:
            v, w = res
        return Out(v, w, self._cmds)

    # ---------------------------------------------------------------- states
    def _st_wait_ready(self, t, x, y, yaw):
        p = self.p
        ok = self.ready or not p.require_ready
        if not ok:
            self.t_ready = None
            return
        if self.t_ready is None:
            self.t_ready = t
        if self.start_requested or (p.autostart and t - self.t_ready >= p.start_delay):
            self.start_requested = False
            self._cmd('center')
            self._enter('INIT', t)

    def _st_init(self, t, x, y, yaw):
        if t - self.t_state >= self.p.init_wait:
            self._start_sweep(t)

    def _st_sweep(self, t, x, y, yaw):
        p = self.p
        if not self.sweep_acked:
            if self.scan_state == 'SWEEPING':
                self.sweep_acked = True
            elif t - self.t_cmd > p.ack_timeout:
                self.t_cmd = t
                self._cmd('sweep')
            return
        lk = self._fresh_lock(t, p.det_max_age)
        if self.scan_state == 'LOCKED' and lk is not None:
            try:
                cmd = parse_command(lk['text'])
            except ValueError as exc:
                self.log('warn', f'invalid command QR ignored: {exc}')
                self.ignore.add(lk['cid'])
                self._start_sweep(t)
                return
            self.cid, self.text = cmd.cid, lk['text']
            self.target = cmd.cid
            self.log('info', f'QR {cmd.cid} seen: heading {math.degrees(lk["bearing"]):.0f} deg (odom)')
            self._enter('ALIGN', t)
            self.bearing, self.t_lock, self.last_lock_t = lk['bearing'], lk['t'], lk['t']
            return
        if self.scan_state == 'NOT_FOUND' or t - self.t_state > p.sweep_timeout:
            if self.rotations >= p.max_rotations:
                self._fail(t, f'no command QR found after {self.rotations} rotations (full turn)')
                return
            self.rotations += 1
            self.log('info', f'nothing in view: rotating {p.rotate_deg:.0f} deg '
                             f'({self.rotations}/{p.max_rotations})')
            self._enter('ROTATE', t)
            self.target_yaw = wrap(yaw + math.radians(p.rotate_deg))

    def _st_rotate(self, t, x, y, yaw):
        w, done = self._turn(t, wrap(self.target_yaw - yaw))
        if done:
            self._start_sweep(t)
            return 0.0, 0.0
        if t - self.t_state > self.p.turn_timeout:
            self._fail(t, 'the robot does not rotate (raise min_pwm, check TURN DIRECTION ERROR)')
            return 0.0, 0.0
        return 0.0, w

    def _use_lock(self, t, x, y, yaw):
        """Refresh bearing and distance from the newest detection of the target QR."""
        lk = self._fresh_lock(t, self.p.det_max_age)
        if lk is None or lk['t'] == self.last_lock_t:
            return
        self.last_lock_t = lk['t']
        self.bearing, self.t_lock = lk['bearing'], lk['t']
        self.text = lk['text']
        if lk['w_px'] > 0.0:
            d_cam = self.p.servo_k / lk['w_px']
            if self.d_ref is not None and self.pos_ref is not None:
                pred = self.d_ref - math.hypot(x - self.pos_ref[0], y - self.pos_ref[1])
                d_cam = 0.5 * pred + 0.5 * d_cam              # smooth the w_px jitter
            self.d_ref, self.pos_ref = d_cam, (x, y)

    def _st_align(self, t, x, y, yaw):
        self._use_lock(t, x, y, yaw)
        if t - self.t_lock > self.p.blind_timeout:
            self._qr_lost(t, 'ALIGN')
            return 0.0, 0.0
        w, done = self._turn(t, wrap(self.bearing - yaw))
        if done:
            self._enter('APPROACH', t, keep=True)
            self.t_lock = t                                # fresh start for the blind timer
            return 0.0, 0.0
        if t - self.t_state > self.p.turn_timeout:
            self._fail(t, 'the robot does not rotate (raise min_pwm, check TURN DIRECTION ERROR)')
            return 0.0, 0.0
        return 0.0, w

    def _st_approach(self, t, x, y, yaw):
        p = self.p
        if self.t_lock is None:
            self.t_lock = t
        self._use_lock(t, x, y, yaw)
        if t - self.t_lock > p.blind_timeout:
            self._qr_lost(t, 'APPROACH')
            return 0.0, 0.0
        if t - self.t_state > p.approach_timeout:
            self._qr_lost(t, 'APPROACH (timeout)')
            return 0.0, 0.0
        if self.d_ref is None:
            return 0.0, 0.0          # no size yet (w_px unknown): wait for a detection
        d = self.d_ref - math.hypot(x - self.pos_ref[0], y - self.pos_ref[1])
        if d <= p.standoff:
            self.log('info', f'{self.cid}: at {d:.2f} m, reading')
            self._enter('READ', t, keep=True)
            return 0.0, 0.0
        err = wrap(self.bearing - yaw)
        w = clip(p.k_head * err, -p.w_max, p.w_max)
        if abs(err) > p.approach_heading_limit:
            if abs(w) < p.w_min:
                w = math.copysign(p.w_min, err)
            return 0.0, w                          # too far off: turn first
        gap = d - p.standoff
        v = p.v_cruise if gap >= p.slow_zone else p.v_min + (p.v_cruise - p.v_min) * gap / p.slow_zone
        if t - self.t_lock > p.det_max_age:
            v = min(v, p.v_min)                    # not seen right now: creep, do not rush
        v *= max(0.0, math.cos(err))
        return v, w

    def _st_read(self, t, x, y, yaw):
        self._use_lock(t, x, y, yaw)
        if t - self.t_state < self.p.read_settle:
            return 0.0, 0.0
        try:
            cmd = parse_command(self.text)
        except ValueError as exc:
            self.log('warn', f'invalid command QR ignored: {exc}')
            self.ignore.add(self.cid)
            self._start_sweep(t)
            return 0.0, 0.0
        self.actions, self.idx = cmd.actions, 0
        self.log('info', f'{cmd.cid}: {cmd.actions}')
        self._cmd('center')            # the camera stops tracking, back to straight ahead
        self.target = ''
        self._enter('EXECUTE', t)
        self._begin_action(t, x, y, yaw)
        return 0.0, 0.0

    # ---------------------------------------------------------------- EXECUTE
    def _begin_action(self, t, x, y, yaw):
        self.hold_t = None
        name, val = self.actions[self.idx]
        self.start_pos, self.hold_yaw, self.t_end = (x, y), yaw, None
        if name == 'TURN_L':
            self.target_yaw = wrap(yaw + math.radians(val))
        elif name == 'TURN_R':
            self.target_yaw = wrap(yaw - math.radians(val))
        elif name == 'WAIT':
            self.t_end = t + val
        self.t_action = t

    def _finish_command(self, t, stop):
        self.executed.append((self.cid, self.text))
        self.ignore.add(self.cid)
        self.rotations = 0
        self.lost = 0
        self.log('info', f'{self.cid} executed')
        if stop:
            self.log('info', 'STOP command: run finished')
            self._enter('DONE', t)
        else:
            self._start_sweep(t)

    def _st_execute(self, t, x, y, yaw):
        p = self.p
        name, val = self.actions[self.idx]
        v = w = 0.0
        done = False
        if name in ('TURN_L', 'TURN_R'):
            w, done = self._turn(t, wrap(self.target_yaw - yaw))
            if not done and t - self.t_action > p.turn_timeout:
                self._fail(t, f'{self.cid}: the robot does not rotate')
                return 0.0, 0.0
        elif name == 'MOVE':
            travelled = math.hypot(x - self.start_pos[0], y - self.start_pos[1])
            if travelled >= val:
                done = True
            else:
                v = p.move_speed
                w = clip(p.k_head * wrap(self.hold_yaw - yaw), -0.5 * p.w_max, 0.5 * p.w_max)
                if t - self.t_action > 2.0 * val / p.move_speed + 5.0:
                    self._fail(t, f'{self.cid}: MOVE {val:g} m takes too long')
                    return 0.0, 0.0
        elif name == 'WAIT':
            done = t >= self.t_end
        elif name == 'FOLLOW':
            done = True
        elif name == 'STOP':
            self._finish_command(t, stop=True)
            return 0.0, 0.0
        if done:
            self.idx += 1
            if self.idx >= len(self.actions):
                self._finish_command(t, stop=False)
            else:
                self._begin_action(t, x, y, yaw)
            return 0.0, 0.0
        return v, w

    # ---------------------------------------------------------------- idle states
    def _st_done(self, t, x, y, yaw):
        return None

    _st_failed = _st_done
    _st_stopped = _st_done

    def status(self):
        extra = ''
        if self.state in ('ALIGN', 'APPROACH', 'READ', 'EXECUTE'):
            extra = f' cid={self.cid}'
        if self.state == 'EXECUTE' and self.actions:
            extra += f' action={self.actions[self.idx][0]}'
        if self.state == 'FAILED':
            extra = f' reason={self.fail_reason}'
        return f'{self.state}{extra} done={len(self.executed)}'
