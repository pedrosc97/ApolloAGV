import math

import pytest

from apollo_agv_sim.model import Agv, Limits, Sonar, Subject, World, wrap

DT = 0.01


def test_drives_straight_and_respects_acceleration():
    agv = Agv()
    agv.step(DT, 1.0, 0.0, 0.0)
    assert agv.v == pytest.approx(agv.limits.a_max * DT)       # cannot jump to full speed
    for _ in range(500):
        agv.step(DT, 1.0, 0.0, 0.0)
    assert agv.v == pytest.approx(1.0)
    assert agv.y == pytest.approx(0.0) and agv.x > 3.0


def test_turning_in_place_changes_only_heading():
    agv = Agv()
    for _ in range(200):
        agv.step(DT, 0.0, 1.0, 0.0)
    assert (agv.x, agv.y) == (0.0, 0.0) and agv.yaw > 1.0


def test_ring_reaches_target_without_overshoot_and_within_limits():
    agv = Agv(limits=Limits(ring_vel_max=2.0, ring_acc_max=8.0))
    target, peak, top_speed = 2.0, 0.0, 0.0
    for _ in range(400):
        agv.step(DT, 0.0, 0.0, target)
        peak, top_speed = max(peak, agv.ring), max(top_speed, abs(agv.ring_vel))
    assert agv.ring == pytest.approx(target, abs=0.01)
    assert peak <= target + 0.01
    assert top_speed <= 2.0 + 1e-9


def test_ring_takes_the_short_way_round():
    agv = Agv()
    for _ in range(300):
        agv.step(DT, 0.0, 0.0, math.radians(350))
    assert agv.ring == pytest.approx(math.radians(-10), abs=0.01)


def test_subject_walks_at_constant_speed_and_loops():
    subject = Subject(waypoints=[(0, 0), (1, 0), (1, 1), (0, 1)], speed=1.0)
    for _ in range(100):                                       # 1 s -> first corner
        subject.step(DT)
    assert (subject.x, subject.y) == pytest.approx((1.0, 0.0), abs=1e-6)
    for _ in range(300):                                       # 3 more sides -> back at the start
        subject.step(DT)
    assert (subject.x, subject.y) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert math.hypot(subject.vx, subject.vy) == pytest.approx(1.0)


def test_sonar_sees_only_what_is_inside_its_cone():
    front = Sonar(0.2, 0.0, 0.0, fov=0.5, max_range=4.0)
    agv = Agv()
    assert front.read(agv, [(2.2, 0.0, 0.25)]) == pytest.approx(1.75)   # 2.0 m from the sensor, minus the radius
    assert front.read(agv, [(0.2, 2.0, 0.25)]) == 4.0                   # off to the side
    assert front.read(agv, [(-2.0, 0.0, 0.25)]) == 4.0                  # behind
    agv.yaw = math.pi / 2                                               # turn left: the side disc is now ahead
    assert front.read(agv, [(0.0, 2.2, 0.25)]) == pytest.approx(1.75)


def test_world_frames_agree():
    world = World()
    world.agv.x, world.agv.y, world.agv.yaw = 1.0, 2.0, math.pi / 2
    world.subject.x, world.subject.y = 1.0, 5.0                         # 3 m straight ahead of the robot
    assert world.subject_in_robot_frame() == pytest.approx((3.0, 0.0))
    assert world.ring_pointing_error() == pytest.approx(0.0)
    world.agv.ring = 0.3
    assert world.ring_pointing_error() == pytest.approx(-0.3)
    assert wrap(3 * math.pi) == pytest.approx(math.pi)
