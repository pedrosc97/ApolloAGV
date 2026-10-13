"""Baseline follow law. Plain Python, no ROS, so it runs and is tested anywhere.

This is a deliberately simple placeholder that closes the loop in simulation. It is the thing to replace
once the real follow behaviour is designed; the inputs and outputs are the contract worth keeping.

Inputs each step
    subject position in the robot frame (x forward, y left), metres
    front and rear ultrasonic ranges, metres

Outputs
    v     forward speed, m/s
    w     turn rate, rad/s (positive = left)
    ring  ring angle to hold, rad, relative to the robot's forward direction

The idea, in three independent parts:
    ring:  always point straight at the subject. The ring is light and fast, so it does the tracking.
    turn:  rotate the base towards the subject, which lets the ring unwind back towards zero.
    drive: close the distance error, but only as far as the base is already facing the subject
           (the cos(bearing) factor), so it does not drive off sideways while still turning.
"""
import math
from dataclasses import dataclass


@dataclass
class FollowParams:
    distance: float = 2.5        # m, distance to hold
    deadband: float = 0.15       # m, distance error ignored (stops hunting back and forth)
    k_distance: float = 0.8      # (m/s) per m of distance error
    k_bearing: float = 1.5       # (rad/s) per rad of bearing
    v_max: float = 1.2           # m/s forward
    v_reverse_max: float = 0.3   # m/s backward, when the subject walks towards the robot
    w_max: float = 2.0           # rad/s
    stop_range: float = 0.40     # m, an obstacle this close in the direction of travel stops the drive
    slow_range: float = 1.00     # m, speed scales down linearly between slow_range and stop_range


@dataclass
class FollowCommand:
    v: float
    w: float
    ring: float


def clamp(value, low, high):
    return max(low, min(high, value))


def obstacle_scale(obstacle_range, params):
    """1.0 when the way is clear, falling to 0.0 at stop_range."""
    span = params.slow_range - params.stop_range
    return clamp((obstacle_range - params.stop_range) / span, 0.0, 1.0)


def follow_step(subject_x, subject_y, front_range, rear_range, params):
    distance = math.hypot(subject_x, subject_y)
    bearing = math.atan2(subject_y, subject_x)

    w = clamp(params.k_bearing * bearing, -params.w_max, params.w_max)

    error = distance - params.distance
    if abs(error) < params.deadband:
        v = 0.0
    else:
        v = clamp(params.k_distance * error, -params.v_reverse_max, params.v_max)
        v *= max(0.0, math.cos(bearing))
    v *= obstacle_scale(front_range if v > 0.0 else rear_range, params)

    return FollowCommand(v=v, w=w, ring=bearing)


def idle_command(ring=0.0):
    """What to send when there is no subject: stand still, hold the ring where it is."""
    return FollowCommand(v=0.0, w=0.0, ring=ring)
