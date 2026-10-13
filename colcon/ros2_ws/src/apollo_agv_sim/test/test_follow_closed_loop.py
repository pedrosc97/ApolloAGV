"""The guidance and the model together: does the AGV actually follow?"""
import math

import pytest

from apollo_agv_follow import guidance
from apollo_agv_follow.guidance import FollowGoal
from apollo_agv_sim import scenarios
from apollo_agv_sim.model import Subject, World
from apollo_agv_sim.run2d import SAFETY_V_MAX, SAFETY_W_MAX, run, summarise

GOAL = FollowGoal()


def baseline():
    return guidance.make('baseline')


def test_follows_a_straight_walk():
    algorithm = baseline()
    world = World(subject=Subject(waypoints=[(3.0, 0.0), (200.0, 0.0)]))
    _, log = run(seconds=40.0, algorithm=algorithm, world=world, goal=GOAL)
    stats = summarise(log, GOAL.distance, settle=10.0)
    # A proportional law trails a moving subject by speed / gain: 1.2 / 0.8 = 1.5 m. That lag is expected.
    expected_lag = world.subject.speed / algorithm.params.k_distance
    assert abs(stats['distance_error_mean'] - expected_lag) < 0.2
    assert stats['ring_error_max_deg'] < 2.0


def test_follows_the_default_walk_without_losing_or_hitting_the_subject():
    _, log = run(seconds=90.0, algorithm=baseline(), goal=GOAL)
    stats = summarise(log, GOAL.distance)
    assert stats['closest_approach'] > 0.6, 'ran into the subject'
    assert stats['distance_error_max'] < 3.0, 'fell too far behind'
    assert stats['ring_error_max_deg'] < 30.0, 'camera lost the subject'
    assert stats['ring_error_mean_deg'] < 5.0


def test_stops_when_the_subject_stops():
    world = World(subject=Subject(waypoints=[(6.0, 2.0), (6.0, 2.0001)], speed=0.0))
    world, log = run(seconds=30.0, algorithm=baseline(), world=world, goal=GOAL)
    assert abs(log[-1]['distance'] - GOAL.distance) <= baseline().params.deadband + 0.05
    assert abs(world.agv.v) < 1e-3 and abs(world.agv.w) < 0.05
    assert abs(math.degrees(log[-1]['ring_error'])) < 1.0


def test_goal_distance_is_what_the_baseline_holds():
    world = World(subject=Subject(waypoints=[(6.0, 2.0), (6.0, 2.0001)], speed=0.0))
    _, log = run(seconds=30.0, algorithm=baseline(), world=world, goal=FollowGoal(distance=4.0))
    assert log[-1]['distance'] == pytest.approx(4.0, abs=0.2)


def test_baseline_keeps_the_subject_in_view_through_a_realistic_camera():
    world, seconds = scenarios.make('noisy')
    _, log = run(seconds=seconds, algorithm=baseline(), world=world, goal=GOAL)
    stats = summarise(log, GOAL.distance)
    assert stats['out_of_view_pct'] < 5.0
    assert stats['closest_approach'] > 0.6


@pytest.mark.parametrize('name', sorted(guidance.ALGORITHMS))
@pytest.mark.parametrize('scenario', sorted(scenarios.SCENARIOS))
def test_every_algorithm_runs_every_scenario(name, scenario):
    """The plumbing smoke test: no exceptions, commands always finite and inside the safety limits."""
    world, _ = scenarios.make(scenario)
    _, log = run(seconds=10.0, algorithm=guidance.make(name), world=world, goal=GOAL)
    assert log
    for r in log:
        assert math.isfinite(r['v_cmd']) and math.isfinite(r['w_cmd'])
        assert abs(r['v_cmd']) <= SAFETY_V_MAX and abs(r['w_cmd']) <= SAFETY_W_MAX
        assert all(math.isfinite(value) for value in r['debug'].values())

