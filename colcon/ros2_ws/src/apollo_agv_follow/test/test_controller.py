import math

import pytest

from apollo_agv_follow.controller import FollowParams, follow_step, obstacle_scale

P = FollowParams()
CLEAR = math.inf


def test_holds_still_at_the_set_distance():
    cmd = follow_step(P.distance, 0.0, CLEAR, CLEAR, P)
    assert (cmd.v, cmd.w, cmd.ring) == (0.0, 0.0, 0.0)


def test_drives_forward_when_subject_is_far():
    cmd = follow_step(P.distance + 2.0, 0.0, CLEAR, CLEAR, P)
    assert 0.0 < cmd.v <= P.v_max


def test_backs_up_slowly_when_subject_is_close():
    cmd = follow_step(0.5, 0.0, CLEAR, CLEAR, P)
    assert -P.v_reverse_max <= cmd.v < 0.0


def test_ring_points_at_subject_and_base_turns_towards_it():
    cmd = follow_step(0.0, 3.0, CLEAR, CLEAR, P)          # subject straight to the left
    assert cmd.ring == pytest.approx(math.pi / 2)
    assert cmd.w > 0.0
    assert cmd.v == pytest.approx(0.0, abs=1e-9)          # does not drive while facing 90 degrees away


def test_speed_limits_hold():
    cmd = follow_step(-50.0, 0.1, CLEAR, CLEAR, P)
    assert abs(cmd.w) <= P.w_max and abs(cmd.v) <= P.v_max


def test_front_obstacle_stops_forward_drive_only():
    far = P.distance + 2.0
    assert follow_step(far, 0.0, P.stop_range, CLEAR, P).v == 0.0
    assert follow_step(far, 0.0, CLEAR, P.stop_range, P).v > 0.0      # rear obstacle is irrelevant going forward
    assert follow_step(0.5, 0.0, CLEAR, P.stop_range, P).v == 0.0     # ...and blocks reversing


def test_obstacle_scale_is_linear_between_slow_and_stop():
    assert obstacle_scale(P.slow_range, P) == 1.0
    assert obstacle_scale((P.slow_range + P.stop_range) / 2, P) == pytest.approx(0.5)
    assert obstacle_scale(0.0, P) == 0.0
