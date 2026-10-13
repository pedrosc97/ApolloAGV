"""The guidance contract: everything an algorithm is given each step, and everything it must hand back.

Plain Python, no ROS. The same GuidanceInput is built by the ROS node (from topics) and by the 2D runner
(from the model), so an algorithm written against it runs unchanged in run2d, in sim2d and on the robot.

Frames and units
    odom   flat world frame: x, y in metres, yaw in radians (counter-clockwise from x)
    robot  x forward, y left, origin on the drive axle (base_footprint)
    ring   angle relative to the robot's forward direction, rad, positive = left. Unbounded: it can wind up.

Inputs each step (GuidanceInput)
    t, dt     time now and since the previous step, s
    robot     measured pose and speed of the base (odometry)
    ring      measured ring angle and speed (joint_states)
    subject   tracked subject: position and velocity in odom, how stale the last measurement is
    ranges    the four ultrasonics, m; math.inf when a sensor has not reported
    imu       yaw rate and body accelerations
    goal      what the operator asked for: distance to hold, and where around the subject to sit

Outputs (GuidanceOutput)
    v      forward speed, m/s
    w      turn rate, rad/s, positive = left
    ring   ring angle to hold, rad, relative to the robot
    debug  any named numbers worth looking at: plotted by run2d, published on guidance/debug/<name> by the node
"""
import math
from dataclasses import dataclass, field, fields, replace


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0              # m/s, measured forward speed
    w: float = 0.0              # rad/s, measured turn rate


@dataclass
class RingState:
    angle: float = 0.0          # rad, relative to the robot
    velocity: float = 0.0       # rad/s


@dataclass
class SubjectState:
    visible: bool = False       # at least one measurement has ever arrived
    age: float = math.inf       # s since the last measurement; grows while perception has lost the subject
    x: float = 0.0              # odom frame, m (predicted forward to now when measurements are late)
    y: float = 0.0
    vx: float = 0.0             # odom frame, m/s, estimated
    vy: float = 0.0


@dataclass
class Ranges:
    front: float = math.inf
    rear: float = math.inf
    left: float = math.inf
    right: float = math.inf


@dataclass
class ImuState:
    yaw_rate: float = 0.0       # rad/s
    accel_x: float = 0.0        # m/s^2, robot frame, gravity removed
    accel_y: float = 0.0


@dataclass
class FollowGoal:
    distance: float = 2.5       # m, distance to hold from the subject
    angle: float = math.pi      # rad, where to sit relative to the subject's heading: pi = behind, pi/2 = on its left


@dataclass
class GuidanceInput:
    t: float = 0.0
    dt: float = 0.0
    robot: RobotState = field(default_factory=RobotState)
    ring: RingState = field(default_factory=RingState)
    subject: SubjectState = field(default_factory=SubjectState)
    ranges: Ranges = field(default_factory=Ranges)
    imu: ImuState = field(default_factory=ImuState)
    goal: FollowGoal = field(default_factory=FollowGoal)

    def to_robot_frame(self, x, y):
        """A point in odom, seen from the robot (x forward, y left)."""
        dx, dy = x - self.robot.x, y - self.robot.y
        c, s = math.cos(self.robot.yaw), math.sin(self.robot.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def subject_in_robot_frame(self):
        return self.to_robot_frame(self.subject.x, self.subject.y)

    def subject_velocity_in_robot_frame(self):
        """Subject velocity in odom, rotated into the robot's axes (not relative to the robot's own motion)."""
        c, s = math.cos(self.robot.yaw), math.sin(self.robot.yaw)
        return c * self.subject.vx + s * self.subject.vy, -s * self.subject.vx + c * self.subject.vy


@dataclass
class GuidanceOutput:
    v: float = 0.0
    w: float = 0.0
    ring: float = 0.0
    debug: dict = field(default_factory=dict)


class Guidance:
    """Base class for a guidance algorithm.

    Subclass it, set Params to a dataclass of tunables (each becomes a ROS parameter `<name>.<field>` and can
    be set from run2d with --set field=value), and implement step(). Keep any memory (filters, integrators,
    modes) on self and clear it in reset().
    """
    name = 'base'
    Params = None

    def __init__(self, params=None):
        self.params = params if params is not None else (self.Params() if self.Params else None)
        self.reset()

    def reset(self):
        """Forget all internal state. Called once at start and whenever the run restarts."""

    def step(self, inp: GuidanceInput) -> GuidanceOutput:
        raise NotImplementedError


def with_overrides(params, overrides):
    """Copy of a Params dataclass with some fields replaced, each value cast to the field's current type."""
    if not overrides:
        return params
    current = {f.name: getattr(params, f.name) for f in fields(params)}
    unknown = set(overrides) - set(current)
    if unknown:
        raise KeyError(f"unknown parameter(s) {sorted(unknown)}; known: {sorted(current)}")
    return replace(params, **{k: type(current[k])(v) for k, v in overrides.items()})


def sanitize(out, v_max, w_max, ring_fallback=0.0):
    """Last line of defence between any algorithm and the motors: no NaNs, nothing past the hard limits.
    A non-finite ring angle becomes ring_fallback (callers pass the measured angle, so the ring holds still)."""
    def finite(value, fallback=0.0):
        return value if math.isfinite(value) else fallback
    return GuidanceOutput(
        v=max(-v_max, min(v_max, finite(out.v))),
        w=max(-w_max, min(w_max, finite(out.w))),
        ring=finite(out.ring, ring_fallback),
        debug=out.debug,
    )
