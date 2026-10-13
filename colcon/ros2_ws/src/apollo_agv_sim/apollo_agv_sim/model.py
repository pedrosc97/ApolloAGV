"""2D model of the AGV and its surroundings. Plain Python, no ROS.

Everything lives in one flat world frame (the same as ROS "odom" here): x, y in metres, angles in radians.

What is modelled, and how roughly:
    base    a unicycle: forward speed v and turn rate w, each limited in size and in how fast it can change.
            No wheel slip, no motor dynamics.
    ring    one angle relative to the base, chasing a target angle with limited speed and acceleration.
            That is how a stepper behaves as long as it does not lose steps.
    subject a point walking a closed loop of waypoints at constant speed, optionally pausing at some of them.
    sonar   four cones on the base; each reports the nearest disc inside its cone.
    camera  the RealSense on the ring plus the person detector: sees the subject only inside its field of
            view, at its frame rate, late by its latency, with noise and dropped frames. The default is
            perfect (sees all round, every step, exactly); scenarios.py has realistic settings.
    imu     yaw rate and body accelerations, exact.
"""
import math
import random
from collections import deque
from dataclasses import dataclass, field


def wrap(angle):
    """Angle folded into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def approach(current, target, max_step):
    """Move current towards target by at most max_step."""
    return current + max(-max_step, min(max_step, target - current))


@dataclass
class Limits:
    v_max: float = 1.5            # m/s
    w_max: float = 2.5            # rad/s
    a_max: float = 1.0            # m/s^2
    alpha_max: float = 4.0        # rad/s^2
    ring_vel_max: float = 3.0     # rad/s      PROVISIONAL until the ring drive is built and measured
    ring_acc_max: float = 12.0    # rad/s^2    PROVISIONAL


@dataclass
class Agv:
    limits: Limits = field(default_factory=Limits)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    w: float = 0.0
    a: float = 0.0                # m/s^2, forward acceleration over the last step
    ring: float = 0.0             # relative to the base; 0 = facing forward. Unbounded (it can keep turning).
    ring_vel: float = 0.0

    def step(self, dt, v_cmd, w_cmd, ring_target):
        lim = self.limits
        v_cmd = max(-lim.v_max, min(lim.v_max, v_cmd))
        w_cmd = max(-lim.w_max, min(lim.w_max, w_cmd))
        v_before = self.v
        self.v = approach(self.v, v_cmd, lim.a_max * dt)
        self.a = (self.v - v_before) / dt
        self.w = approach(self.w, w_cmd, lim.alpha_max * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + self.w * dt)

        # Ring: take the short way round. The fastest speed from which it can still stop on the target
        # is sqrt(2 * a * distance); never ask for more than that, so it arrives without overshooting.
        error = wrap(ring_target - self.ring)
        stop_speed = math.sqrt(2.0 * lim.ring_acc_max * abs(error))
        wanted = math.copysign(min(lim.ring_vel_max, stop_speed, abs(error) / dt), error)
        self.ring_vel = approach(self.ring_vel, wanted, lim.ring_acc_max * dt)
        self.ring += self.ring_vel * dt

    def ring_world(self):
        """Direction the camera faces, in the world frame."""
        return wrap(self.yaw + self.ring)

    def to_robot_frame(self, wx, wy):
        dx, dy = wx - self.x, wy - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy


# A walk with long straights, a sharp sideways cut and a turn back towards the robot's start.
DEFAULT_WAYPOINTS = [(3.0, 0.0), (9.0, 0.0), (9.0, 4.0), (5.0, 6.0), (1.0, 4.0), (1.0, -3.0), (6.0, -3.0)]


@dataclass
class Subject:
    waypoints: list = field(default_factory=lambda: list(DEFAULT_WAYPOINTS))
    speed: float = 1.2            # m/s, walking pace
    radius: float = 0.25          # m, as seen by the ultrasonics
    dwell: dict = field(default_factory=dict)   # waypoint index -> seconds to stand still on reaching it
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    _next: int = 1
    _wait: float = 0.0

    def __post_init__(self):
        self.x, self.y = self.waypoints[0]

    def step(self, dt):
        time_left = dt
        x0, y0 = self.x, self.y
        while time_left > 1e-12:
            if self._wait > 0.0:
                waited = min(self._wait, time_left)
                self._wait -= waited
                time_left -= waited
                continue
            if self.speed <= 0.0:
                break
            remaining = self.speed * time_left
            tx, ty = self.waypoints[self._next]
            gap = math.hypot(tx - self.x, ty - self.y)
            if gap <= remaining:
                self.x, self.y = tx, ty
                time_left -= gap / self.speed
                self._wait = self.dwell.get(self._next, 0.0)
                self._next = (self._next + 1) % len(self.waypoints)
            else:
                self.x += (tx - self.x) / gap * remaining
                self.y += (ty - self.y) / gap * remaining
                time_left = 0.0
        self.vx, self.vy = (self.x - x0) / dt, (self.y - y0) / dt


@dataclass
class Sonar:
    """One ultrasonic sensor, placed in the robot frame."""
    x: float
    y: float
    yaw: float
    fov: float = 0.52
    min_range: float = 0.02
    max_range: float = 4.0

    def read(self, agv, discs):
        """Range to the nearest disc (x, y, radius) whose centre is inside the cone, else max_range."""
        c, s = math.cos(agv.yaw), math.sin(agv.yaw)
        sx = agv.x + c * self.x - s * self.y
        sy = agv.y + s * self.x + c * self.y
        heading = agv.yaw + self.yaw
        best = self.max_range
        for dx, dy, radius in discs:
            if abs(wrap(math.atan2(dy - sy, dx - sx) - heading)) <= self.fov / 2:
                best = min(best, math.hypot(dx - sx, dy - sy) - radius)
        return max(self.min_range, best)


@dataclass
class Camera:
    """What perception will deliver: subject positions in the world frame, captured at some time, released later.

    Sees the subject only when it is within fov/2 of the direction the ring faces and within max_range.
    """
    fov: float = 2 * math.pi      # rad, full horizontal field of view; 2*pi = sees all round
    max_range: float = 20.0       # m
    rate: float = 0.0             # Hz; 0 = a frame every simulation step
    latency: float = 0.0          # s, from capture to the measurement being available
    noise: float = 0.0            # m, standard deviation on each axis
    dropout: float = 0.0          # probability that a frame in view still yields no detection
    seed: int = 0

    def __post_init__(self):
        self._rng = random.Random(self.seed)
        self._pending = deque()   # (release time, capture time, x, y)
        self._next_frame = 0.0

    def sees(self, agv, subject):
        dx, dy = subject.x - agv.x, subject.y - agv.y
        in_fov = abs(wrap(math.atan2(dy, dx) - agv.ring_world())) <= self.fov / 2
        return in_fov and math.hypot(dx, dy) <= self.max_range

    def step(self, time, agv, subject):
        """Capture a frame if one is due; return the measurements [(capture time, x, y)] released by now."""
        if time + 1e-9 >= self._next_frame:
            self._next_frame = time + (1.0 / self.rate if self.rate > 0 else 0.0)
            if self.sees(agv, subject) and self._rng.random() >= self.dropout:
                x = subject.x + self._rng.gauss(0.0, self.noise) if self.noise else subject.x
                y = subject.y + self._rng.gauss(0.0, self.noise) if self.noise else subject.y
                self._pending.append((time + self.latency, time, x, y))
        released = []
        while self._pending and self._pending[0][0] <= time + 1e-9:
            released.append(self._pending.popleft()[1:])
        return released


def default_sonars(base_length=0.40, base_width=0.30, **kwargs):
    return {
        'front': Sonar(base_length / 2, 0.0, 0.0, **kwargs),
        'rear': Sonar(-base_length / 2, 0.0, math.pi, **kwargs),
        'left': Sonar(0.0, base_width / 2, math.pi / 2, **kwargs),
        'right': Sonar(0.0, -base_width / 2, -math.pi / 2, **kwargs),
    }


@dataclass
class World:
    agv: Agv = field(default_factory=Agv)
    subject: Subject = field(default_factory=Subject)
    sonars: dict = field(default_factory=default_sonars)
    camera: Camera = field(default_factory=Camera)
    obstacles: list = field(default_factory=list)      # static discs: (x, y, radius)
    time: float = 0.0

    def step(self, dt, v_cmd, w_cmd, ring_target):
        self.subject.step(dt)
        self.agv.step(dt, v_cmd, w_cmd, ring_target)
        self.time += dt

    def detections(self):
        """Subject measurements [(capture time, x, y)] that perception has delivered by now. Call once per step."""
        return self.camera.step(self.time, self.agv, self.subject)

    def imu(self):
        """(yaw rate, forward acceleration, sideways acceleration) in the robot frame."""
        return self.agv.w, self.agv.a, self.agv.v * self.agv.w

    def ranges(self):
        discs = self.obstacles + [(self.subject.x, self.subject.y, self.subject.radius)]
        return {name: sonar.read(self.agv, discs) for name, sonar in self.sonars.items()}

    def subject_in_robot_frame(self):
        return self.agv.to_robot_frame(self.subject.x, self.subject.y)

    def subject_distance(self):
        return math.hypot(self.subject.x - self.agv.x, self.subject.y - self.agv.y)

    def ring_pointing_error(self):
        """Angle between where the camera faces and where the subject is. 0 = subject centred."""
        bearing = math.atan2(self.subject.y - self.agv.y, self.subject.x - self.agv.x)
        return wrap(bearing - self.agv.ring_world())
