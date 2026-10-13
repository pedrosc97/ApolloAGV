"""Apollo guidance: the algorithm to design. Everything around it (inputs, outputs, ROS, simulation) is wired.

Iterate with:
    bash scripts/dev.sh run2d --plot run.png                         default scenario, this algorithm
    bash scripts/dev.sh run2d --scenario all                         every scenario, one table
    bash scripts/dev.sh run2d --algorithm all --scenario noisy       side by side with the baseline
    bash scripts/dev.sh run2d --set k_ring=2.0 --plot run.png        try a parameter without editing
    bash scripts/dev.sh sim algorithm:=apollo                        the same code in ROS, viewed in Foxglove

What is here now is a placeholder that only points the ring at the subject and keeps the base still,
so the plumbing can be seen working end to end. Replace step() with the real thing.
See interface.py for exactly what `inp` contains and what to return.
"""
import math
from dataclasses import dataclass

from apollo_agv_follow.guidance.interface import Guidance, GuidanceOutput


@dataclass
class ApolloParams:
    # Every field here becomes a ROS parameter `apollo.<field>` (config/follow.yaml in apollo_agv_bringup)
    # and can be overridden from run2d with --set <field>=<value>. Add what the algorithm needs.
    subject_timeout: float = 0.5    # s, measurement age past which the subject counts as lost
    k_ring: float = 0.0             # rad of lead per rad/s of the subject's bearing rate (0 = point where it is)


def unwrap_near(angle, reference):
    """angle + 2*pi*k closest to reference. Lets the ring take the short way without unwinding a full turn."""
    return reference + math.atan2(math.sin(angle - reference), math.cos(angle - reference))


class ApolloGuidance(Guidance):
    name = 'apollo'
    Params = ApolloParams

    def reset(self):
        self.ring_hold = 0.0

    def step(self, inp):
        p = self.params
        lost = not inp.subject.visible or inp.subject.age > p.subject_timeout
        if lost:
            # TODO: lost-subject behaviour (stop? search with the ring? coast on the last velocity?).
            return GuidanceOutput(v=0.0, w=0.0, ring=self.ring_hold, debug={'lost': 1.0})

        sx, sy = inp.subject_in_robot_frame()
        distance = math.hypot(sx, sy)
        bearing = math.atan2(sy, sx)

        # Bearing rate of the subject as seen from the robot, from the relative velocity. Useful for leading
        # the ring (k_ring) and for anticipating turns.
        svx, svy = inp.subject_velocity_in_robot_frame()
        rel_vx, rel_vy = svx - inp.robot.v + inp.robot.w * sy, svy - inp.robot.w * sx
        bearing_rate = (sx * rel_vy - sy * rel_vx) / max(distance * distance, 1e-6) if distance > 1e-3 else 0.0

        # Ring: point at the subject (plus optional lead), choosing the turn nearest the current ring angle.
        ring = unwrap_near(bearing + p.k_ring * bearing_rate, inp.ring.angle)
        self.ring_hold = ring

        # TODO: base guidance. Everything needed is in `inp`:
        #   distance / bearing to the subject, its velocity (inp.subject.vx, vy), the goal (inp.goal.distance,
        #   inp.goal.angle), the measured base motion (inp.robot.v, w), ultrasonics (inp.ranges) and IMU.
        v, w = 0.0, 0.0

        return GuidanceOutput(v=v, w=w, ring=ring, debug={
            'distance_error': distance - inp.goal.distance,
            'bearing': bearing,
            'bearing_rate': bearing_rate,
            'lost': 0.0,
        })
