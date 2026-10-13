"""The original proportional follow law (controller.py) behind the Guidance interface.

Kept as the yardstick: run2d --algorithm all puts any new algorithm side by side with it.
"""
from dataclasses import replace

from apollo_agv_follow.controller import FollowParams, follow_step, idle_command
from apollo_agv_follow.guidance.interface import Guidance, GuidanceOutput


class BaselineGuidance(Guidance):
    name = 'baseline'
    Params = FollowParams
    subject_timeout = 0.5       # s; past this the subject counts as lost and the robot stands still

    def reset(self):
        self.last_ring = 0.0

    def step(self, inp):
        if not inp.subject.visible or inp.subject.age > self.subject_timeout:
            cmd = idle_command(self.last_ring)
        else:
            x, y = inp.subject_in_robot_frame()
            params = replace(self.params, distance=inp.goal.distance)
            cmd = follow_step(x, y, inp.ranges.front, inp.ranges.rear, params)
        self.last_ring = cmd.ring
        return GuidanceOutput(v=cmd.v, w=cmd.w, ring=cmd.ring)
