"""The controller and the model together: does the AGV actually follow?"""
import math

from apollo_agv_follow.controller import FollowParams
from apollo_agv_sim.model import Subject, World
from apollo_agv_sim.run2d import run, summarise


def test_follows_a_straight_walk():
    params = FollowParams()
    world = World(subject=Subject(waypoints=[(3.0, 0.0), (200.0, 0.0)]))
    _, log = run(seconds=40.0, params=params, world=world)
    stats = summarise(log, params, settle=10.0)
    # A proportional law trails a moving subject by speed / gain: 1.2 / 0.8 = 1.5 m. That lag is expected.
    expected_lag = world.subject.speed / params.k_distance
    assert abs(stats['distance_error_mean'] - expected_lag) < 0.2
    assert stats['ring_error_max_deg'] < 2.0


def test_follows_the_default_walk_without_losing_or_hitting_the_subject():
    params = FollowParams()
    _, log = run(seconds=90.0, params=params)
    stats = summarise(log, params)
    assert stats['closest_approach'] > 0.6, 'ran into the subject'
    assert stats['distance_error_max'] < 3.0, 'fell too far behind'
    assert stats['ring_error_max_deg'] < 30.0, 'camera lost the subject'
    assert stats['ring_error_mean_deg'] < 5.0


def test_stops_when_the_subject_stops():
    params = FollowParams()
    world = World(subject=Subject(waypoints=[(6.0, 2.0), (6.0, 2.0001)], speed=0.0))
    world, log = run(seconds=30.0, params=params, world=world)
    assert abs(log[-1]['distance'] - params.distance) <= params.deadband + 0.05
    assert abs(world.agv.v) < 1e-3 and abs(world.agv.w) < 0.05
    assert abs(math.degrees(log[-1]['ring_error'])) < 1.0
