import math

import pytest

from apollo_agv_follow import guidance
from apollo_agv_follow.guidance import (FollowGoal, GuidanceInput, GuidanceOutput, RobotState, SubjectState,
                                        sanitize, with_overrides)
from apollo_agv_follow.guidance.apollo import unwrap_near
from apollo_agv_follow.tracking import SubjectTracker, TrackerParams


def seen(x, y, vx=0.0, vy=0.0):
    return SubjectState(visible=True, age=0.0, x=x, y=y, vx=vx, vy=vy)


def test_registry_builds_every_algorithm_with_defaults_and_overrides():
    for name, cls in guidance.ALGORITHMS.items():
        algo = guidance.make(name)
        assert isinstance(algo, cls) and algo.name == name
    assert guidance.make('baseline', {'k_bearing': 2}).params.k_bearing == 2.0
    with pytest.raises(KeyError):
        guidance.make('nope')
    with pytest.raises(KeyError):
        guidance.make('baseline', {'no_such_gain': 1.0})


def test_with_overrides_keeps_types():
    params = with_overrides(guidance.ALGORITHMS['apollo'].Params(), {'k_ring': '0.5'})
    assert params.k_ring == 0.5 and isinstance(params.k_ring, float)


def test_subject_in_robot_frame():
    inp = GuidanceInput(robot=RobotState(x=1.0, y=2.0, yaw=math.pi / 2), subject=seen(1.0, 5.0, vx=0.0, vy=1.0))
    assert inp.subject_in_robot_frame() == pytest.approx((3.0, 0.0))
    assert inp.subject_velocity_in_robot_frame() == pytest.approx((1.0, 0.0))


def test_sanitize_clamps_and_replaces_nans():
    out = sanitize(GuidanceOutput(v=5.0, w=math.nan, ring=math.inf), v_max=1.0, w_max=2.0, ring_fallback=0.3)
    assert (out.v, out.w, out.ring) == (1.0, 0.0, 0.3)
    assert sanitize(GuidanceOutput(v=-5.0, w=-5.0), 1.0, 2.0).w == -2.0


@pytest.mark.parametrize('name', sorted(guidance.ALGORITHMS))
def test_every_algorithm_stands_still_without_a_subject(name):
    out = guidance.make(name).step(GuidanceInput(subject=SubjectState()))
    assert (out.v, out.w) == (0.0, 0.0)
    out = guidance.make(name).step(GuidanceInput(subject=SubjectState(visible=True, age=10.0, x=3.0)))
    assert (out.v, out.w) == (0.0, 0.0)


@pytest.mark.parametrize('name', sorted(guidance.ALGORITHMS))
def test_every_algorithm_points_the_ring_at_a_visible_subject(name):
    out = guidance.make(name).step(GuidanceInput(subject=seen(0.0, 3.0), goal=FollowGoal()))
    assert math.atan2(math.sin(out.ring), math.cos(out.ring)) == pytest.approx(math.pi / 2)


def test_unwrap_near_takes_the_short_way():
    assert unwrap_near(math.radians(-170), math.radians(170)) == pytest.approx(math.radians(190))
    assert unwrap_near(0.1, 4 * math.pi) == pytest.approx(4 * math.pi + 0.1)


def test_tracker_passes_through_and_estimates_velocity():
    tracker = SubjectTracker()
    assert not tracker.state(0.0).visible
    for i in range(60):                                        # 2 s of 30 Hz measurements, walking at 1 m/s in x
        t = i / 30
        tracker.update(t, t, 2.0)
    state = tracker.state(2.0)
    assert state.visible and state.age == pytest.approx(2.0 - 59 / 30)
    assert (state.vx, state.vy) == pytest.approx((1.0, 0.0), abs=0.02)
    assert state.x == pytest.approx(2.0, abs=0.03)             # predicted forward to the asking time


def test_tracker_restarts_after_a_long_gap_and_caps_prediction():
    tracker = SubjectTracker(TrackerParams(max_gap=1.0, max_predict=0.5))
    tracker.update(0.0, 0.0, 0.0)
    tracker.update(0.1, 0.1, 0.0)
    assert tracker.state(10.0).x < 0.1 + tracker.vx * 0.5 + 1e-9
    tracker.update(5.0, 7.0, 7.0)                              # long gap: jump, no velocity spike
    assert (tracker.x, tracker.y, tracker.vx, tracker.vy) == (7.0, 7.0, 0.0, 0.0)
