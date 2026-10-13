import math

import pytest

from apollo_agv_sim.model import Agv, Camera, Limits, Sonar, Subject, World, wrap

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


def test_subject_pauses_at_dwell_waypoints():
    subject = Subject(waypoints=[(0, 0), (1, 0), (2, 0)], speed=1.0, dwell={1: 2.0})
    for _ in range(200):                                       # 1 s to reach (1, 0), then 1 s of a 2 s pause
        subject.step(DT)
    assert (subject.x, subject.y) == pytest.approx((1.0, 0.0), abs=1e-6)
    assert (subject.vx, subject.vy) == pytest.approx((0.0, 0.0))
    for _ in range(150):                                       # 1 s more pause, then 0.5 s of walking
        subject.step(DT)
    assert subject.x == pytest.approx(1.5, abs=1e-6)


def test_camera_sees_only_inside_its_view_from_the_ring():
    world = World(subject=Subject(waypoints=[(0.0, 3.0), (0.0, 3.0001)], speed=0.0),
                  camera=Camera(fov=math.radians(60)))
    assert world.detections() == []                            # subject on the left, ring facing forward
    world.agv.ring = math.pi / 2
    [(t, x, y)] = world.detections()
    assert (t, x, y) == (0.0, 0.0, 3.0)
    world.camera.max_range = 2.0
    assert world.detections() == []                            # in view but too far


def test_camera_rate_and_latency():
    world = World(camera=Camera(rate=10.0, latency=0.25))
    got = []
    for _ in range(100):                                       # 1 s
        got += world.detections()
        world.step(DT, 0.0, 0.0, 0.0)
    captures = [t for t, _, _ in got]
    assert len(captures) == 8                                  # frames at 0.0..0.7 s released by 0.95..
    assert captures[1] - captures[0] == pytest.approx(0.1, abs=DT)


def test_camera_noise_and_dropout_are_repeatable():
    def frames(seed):
        world = World(camera=Camera(noise=0.05, dropout=0.3, seed=seed))
        out = []
        for _ in range(200):
            out.append(world.detections())
            world.step(DT, 0.0, 0.0, 0.0)
        return out
    a = frames(1)
    assert a == frames(1) and a != frames(2)
    assert 0.5 < sum(1 for f in a if f) / len(a) < 0.9


def test_imu_reports_turn_rate_and_acceleration():
    agv = Agv()
    agv.step(DT, 1.0, 0.5, 0.0)
    world = World(agv=agv)
    yaw_rate, ax, ay = world.imu()
    assert yaw_rate == pytest.approx(agv.w)
    assert ax == pytest.approx(agv.limits.a_max)
    assert ay == pytest.approx(agv.v * agv.w)
