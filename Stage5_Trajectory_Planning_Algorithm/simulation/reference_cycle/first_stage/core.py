"""Time-stamped ENU positions, bounded prediction, PID and landing supervisor.

Units: metres, seconds, metres/second. No ROS or scientific dependencies.
"""
from dataclasses import dataclass
from collections import deque
from bisect import bisect_left
import math


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def norm(a):
    return math.sqrt(sum(x*x for x in a))


def clip(x, limit):
    return max(-limit, min(limit, x))


@dataclass(frozen=True)
class Sample:
    t: float
    p: tuple
    frame: str = "world"


class History:
    """Reject duplicate/out-of-order samples; interpolate only inside history.

    Predict forward using a least-squares velocity over a short trailing window.
    Prediction beyond max_prediction fails, rather than silently extrapolating.
    """
    def __init__(self, max_prediction=0.35, window=0.20, max_speed=3.0):
        self.samples = deque(maxlen=200)
        self.max_prediction = max_prediction
        self.window = window
        self.max_speed = max_speed
        self.rejected = 0

    def push(self, sample):
        if (sample.frame != "world" or len(sample.p) != 3 or
                not all(math.isfinite(x) for x in (sample.t, *sample.p)) or
                (self.samples and sample.t <= self.samples[-1].t)):
            self.rejected += 1
            return False
        self.samples.append(sample)
        return True

    def velocity(self):
        if len(self.samples) < 2:
            return (0., 0., 0.)
        end = self.samples[-1].t
        points = [s for s in self.samples if s.t >= end-self.window]
        if len(points) < 2:
            points = list(self.samples)[-2:]
        tm = sum(s.t for s in points)/len(points)
        denominator = sum((s.t-tm)**2 for s in points)
        if denominator < 1e-12:
            return (0., 0., 0.)
        v = tuple(sum((s.t-tm)*s.p[k] for s in points)/denominator for k in range(3))
        speed = norm(v)
        return scale(v, min(1., self.max_speed/max(speed, 1e-12)))

    def at(self, t, predict=True):
        if not self.samples or t < self.samples[0].t-1e-9:
            return None
        last = self.samples[-1]
        if t >= last.t:
            age = t-last.t
            if predict and age > self.max_prediction+1e-9:
                return None
            return add(last.p, scale(self.velocity(), age)) if predict else last.p
        points = list(self.samples)
        i = bisect_left([s.t for s in points], t)
        if i == 0:
            return points[0].p
        a, b = points[i-1:i+1]
        return add(a.p, scale(sub(b.p, a.p), (t-a.t)/(b.t-a.t)))


class PID:
    def __init__(self, kp, ki, kd, limit, integral_limit=3., derivative_tau=0.10):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit, self.integral_limit = limit, integral_limit
        self.tau = derivative_tau
        self.reset()

    def reset(self):
        self.integral = 0.
        self.previous = None
        self.derivative = 0.

    def step(self, error, dt, feedforward=0.):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        raw_d = 0. if self.previous is None else (error-self.previous)/dt
        self.derivative += dt/(self.tau+dt)*(raw_d-self.derivative)
        candidate = clip(self.integral+error*dt, self.integral_limit)
        raw = feedforward+self.kp*error+self.ki*candidate+self.kd*self.derivative
        # Conditional integration prevents winding up in the saturated direction.
        if abs(raw) <= self.limit or raw*error < 0:
            self.integral = candidate
        raw = feedforward+self.kp*error+self.ki*self.integral+self.kd*self.derivative
        self.previous = error
        return clip(raw, self.limit)


@dataclass
class Config:
    kp: float = 1.2
    ki: float = 0.15
    kd: float = 0.08
    max_xy_speed: float = 2.0
    max_z_speed: float = 0.6
    hover_height: float = 2.0
    descent_speed: float = 0.25
    align_error: float = 0.18
    abort_error: float = 0.35
    align_speed: float = 0.15
    stable_time: float = 1.5
    touchdown_height: float = 0.06
    touchdown_speed: float = 0.12
    touchdown_time: float = 0.5
    data_timeout: float = 0.4
    max_prediction: float = 0.35
    velocity_window: float = 0.4
    prediction: bool = True
    feedforward: bool = False
    landing_enabled: bool = True
    require_contact: bool = False

    def __post_init__(self):
        values = [v for v in vars(self).values() if type(v) is not bool]
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError("configuration must be finite and nonnegative")
        if min(self.max_xy_speed, self.max_z_speed, self.data_timeout,
               self.max_prediction, self.velocity_window, self.descent_speed,
               self.stable_time, self.touchdown_time) <= 0:
            raise ValueError("speed, duration and timeout limits must be positive")
        if self.abort_error <= self.align_error or self.hover_height <= self.touchdown_height:
            raise ValueError("invalid landing hysteresis or height")


class Controller:
    def __init__(self, config=None):
        self.cfg = config or Config()
        c = self.cfg
        self.vehicle = History(c.max_prediction, c.velocity_window)
        self.uav = History(c.max_prediction, c.velocity_window)
        self.pids = [PID(c.kp, c.ki, c.kd, c.max_xy_speed) for _ in range(2)]
        self.state = "TRACK"
        self.stable_since = None
        self.contact_since = None
        self.z_ref = None
        self.last_time = None
        self.transitions = []

    def transition(self, state, now):
        if state != self.state:
            self.transitions.append((now, self.state, state))
            self.state = state
            self.stable_since = self.contact_since = None

    def step(self, now, dt, contact=False):
        c = self.cfg
        if not math.isfinite(now) or not math.isfinite(dt) or dt <= 0:
            raise ValueError("now and dt must be finite; dt must be positive")
        if self.last_time is not None and now < self.last_time:
            # A simulation-clock reset invalidates all history and controller memory.
            self.__init__(c)
        self.last_time = now
        # Landing completion is terminal until an explicit reset/new controller.
        if self.state == "LANDED":
            return (0., 0., 0.), {"state": "LANDED", "valid": True}
        vp = self.vehicle.at(now, c.prediction)
        up = self.uav.at(now, True)
        fresh = (vp is not None and up is not None and
                 all(-1e-6 <= now-h.samples[-1].t <= c.data_timeout
                     for h in (self.vehicle, self.uav)))
        if not fresh:
            self.transition("HOLD", now)
            self.z_ref = None
            for pid in self.pids:
                pid.reset()
            return (0., 0., 0.), {"state": self.state, "valid": False}
        if self.state == "HOLD":
            self.transition("TRACK", now)
        vp_velocity, up_velocity = self.vehicle.velocity(), self.uav.velocity()
        error = sub(vp, up)
        xy_error = norm(error[:2])
        relative_speed = norm(sub(vp_velocity, up_velocity)[:2])
        height = up[2]-vp[2]
        aligned = xy_error <= c.align_error and relative_speed <= c.align_speed
        if self.state == "TRACK":
            self.z_ref = vp[2]+c.hover_height
            ready = aligned and abs(height-c.hover_height) <= 0.15
            if ready and c.landing_enabled:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= c.stable_time:
                    self.transition("DESCEND", now)
                    self.z_ref = up[2]
            else:
                self.stable_since = None
        elif self.state == "DESCEND":
            if xy_error > c.abort_error or relative_speed > 2*c.align_speed:
                self.transition("TRACK", now)
                self.z_ref = vp[2]+c.hover_height
            else:
                # Small penetration reference produces contact instead of an
                # asymptotic hover just above the platform; plant resolves contact.
                self.z_ref = max(vp[2]-0.04, self.z_ref-c.descent_speed*dt)
                geometry = (height <= c.touchdown_height and height >= -0.03
                            and aligned and abs(up_velocity[2]-vp_velocity[2]) <= c.touchdown_speed)
                touched = geometry and (contact or not c.require_contact)
                if touched:
                    if self.contact_since is None:
                        self.contact_since = now
                    if now-self.contact_since >= c.touchdown_time:
                        self.transition("LANDED", now)
                else:
                    self.contact_since = None
        if self.state == "LANDED":
            command = (0., 0., 0.)
        else:
            xy = tuple(self.pids[k].step(error[k], dt,
                       vp_velocity[k] if c.feedforward else 0.) for k in range(2))
            speed = norm(xy)
            xy = scale(xy, min(1., c.max_xy_speed/max(speed, 1e-12)))
            vz = clip(1.5*(self.z_ref-up[2]), c.max_z_speed)
            command = (*xy, vz)
        return command, {"state": self.state, "valid": True, "xy_error_est": xy_error,
                         "relative_speed_est": relative_speed, "height_est": height,
                         "vehicle_age": now-self.vehicle.samples[-1].t}
