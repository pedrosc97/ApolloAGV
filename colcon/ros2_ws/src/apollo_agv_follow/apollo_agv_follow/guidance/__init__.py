"""Guidance algorithms, selectable by name (ROS parameter `algorithm`, run2d --algorithm).

To add one: write a Guidance subclass in this folder and add it to ALGORITHMS below.
"""
from apollo_agv_follow.guidance.apollo import ApolloGuidance
from apollo_agv_follow.guidance.baseline import BaselineGuidance
from apollo_agv_follow.guidance.interface import (FollowGoal, Guidance, GuidanceInput, GuidanceOutput, ImuState,
                                                  Ranges, RingState, RobotState, SubjectState, sanitize,
                                                  with_overrides)

ALGORITHMS = {cls.name: cls for cls in (ApolloGuidance, BaselineGuidance)}
DEFAULT = 'apollo'


def make(name, overrides=None):
    """A fresh algorithm by name, with optional {param: value} overrides."""
    try:
        cls = ALGORITHMS[name]
    except KeyError:
        raise KeyError(f"unknown guidance algorithm '{name}'; known: {sorted(ALGORITHMS)}") from None
    return cls(with_overrides(cls.Params(), overrides) if cls.Params else None)


__all__ = ['ALGORITHMS', 'DEFAULT', 'make', 'FollowGoal', 'Guidance', 'GuidanceInput', 'GuidanceOutput',
           'ImuState', 'Ranges', 'RingState', 'RobotState', 'SubjectState', 'sanitize', 'with_overrides']
