"""Pure logic of sim_base_node (no ROS): gyro heading and the Mega's servo protocol."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GyroHeading:
    """Heading from a z gyro, like arduino_bridge_node: bias measured while the robot is still.

    feed(t, rate) with t in seconds (sensor stamps, i.e. simulation time).
    Calibration: ignore the first `settle` s (the robot drops onto the floor at spawn), then
    average `calib` s of samples. If they spread more than `max_spread` rad/s the robot was
    moving: start again.
    """

    def __init__(self, settle=1.0, calib=1.5, max_spread=0.05, deadband=0.003):
        self.settle, self.calib, self.max_spread, self.deadband = settle, calib, max_spread, deadband
        self.t0 = None
        self.samples = []
        self.bias = 0.0
        self.ready = False
        self.yaw = 0.0
        self.rate = 0.0
        self.t_last = None

    def feed(self, t, rate):
        if self.t0 is None:
            self.t0 = t
        if not self.ready:
            if t - self.t0 >= self.settle:
                self.samples.append((t, rate))
                span = self.samples[-1][0] - self.samples[0][0]
                if span >= self.calib:
                    rates = [r for _, r in self.samples]
                    if max(rates) - min(rates) <= self.max_spread:
                        self.bias = sum(rates) / len(rates)
                        self.ready = True
                    self.samples = []
            self.t_last = t
            return
        dt = 0.0 if self.t_last is None else t - self.t_last
        self.t_last = t
        if dt <= 0.0 or dt > 0.5:
            return
        r = rate - self.bias
        self.rate = 0.0 if abs(r) < self.deadband else r
        self.yaw = wrap(self.yaw + self.rate * dt)


class ServoSim:
    """The Mega's camera servo: S = sweep, H = halt, G<angle> = go to angle (20..160 deg).

    update(t) steps the sweep (5 deg every 0.7 s, like the firmware). Every method returns the
    new angle when it changed (the firmware then sends 'A:<angle>'), else None.
    """

    MIN, MAX, STEP, DWELL = 20, 160, 5, 0.7

    def __init__(self, angle=90):
        self.angle = angle
        self.scanning = False
        self.dir = 1
        self.t_move = None

    def command(self, cmd, t):
        cmd = (cmd or '').strip()
        if not cmd:
            return None
        if cmd[0] == 'S':
            self.scanning = True
            self.t_move = t
        elif cmd[0] == 'H':
            self.scanning = False
        elif cmd[0] == 'G':
            try:
                a = int(float(cmd[1:]))
            except ValueError:
                return None
            self.scanning = False
            self.angle = max(self.MIN, min(self.MAX, a))
            return self.angle                       # the firmware always echoes G
        return None

    def update(self, t):
        if not self.scanning:
            return None
        if self.t_move is not None and t - self.t_move < self.DWELL:
            return None
        self.t_move = t
        self.angle += self.dir * self.STEP
        if self.angle >= self.MAX:
            self.angle, self.dir = self.MAX, -1
        elif self.angle <= self.MIN:
            self.angle, self.dir = self.MIN, 1
        return self.angle
