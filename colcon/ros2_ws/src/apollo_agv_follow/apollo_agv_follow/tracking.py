"""Subject tracker: turns sparse, noisy, late position measurements into a position and velocity at any time.

Plain Python, no ROS. Used by the follower node and by run2d, so the guidance sees the same thing in both.

A constant-velocity alpha-beta filter in the odom frame. Each measurement corrects the prediction made for
its own capture time; between measurements the estimate is carried forward at the estimated velocity.
alpha = 1, beta = 0 passes measurements straight through (velocity stays zero).
"""
from dataclasses import dataclass

from apollo_agv_follow.guidance.interface import SubjectState


@dataclass
class TrackerParams:
    alpha: float = 0.85         # position correction gain, 0..1
    beta: float = 0.3           # velocity correction gain, 0..2
    max_gap: float = 1.0        # s; after a gap this long the next measurement restarts the track
    max_predict: float = 0.5    # s; never extrapolate further than this past the last measurement


class SubjectTracker:
    def __init__(self, params=None):
        self.params = params or TrackerParams()
        self.reset()

    def reset(self):
        self.x = self.y = self.vx = self.vy = 0.0
        self.stamp = None           # capture time of the last measurement, s

    def update(self, t, x, y):
        """A measurement of the subject at (x, y) in odom, captured at time t."""
        p = self.params
        if self.stamp is None or t - self.stamp > p.max_gap:
            self.x, self.y, self.vx, self.vy, self.stamp = x, y, 0.0, 0.0, t
            return
        dt = t - self.stamp
        if dt <= 0.0:               # out of order or duplicate: ignore
            return
        px, py = self.x + self.vx * dt, self.y + self.vy * dt
        rx, ry = x - px, y - py
        self.x, self.y = px + p.alpha * rx, py + p.alpha * ry
        self.vx += p.beta * rx / dt
        self.vy += p.beta * ry / dt
        self.stamp = t

    def state(self, t):
        """Estimate at time t (usually now, which is later than the last measurement)."""
        if self.stamp is None:
            return SubjectState()
        age = max(0.0, t - self.stamp)
        ahead = min(age, self.params.max_predict)
        return SubjectState(visible=True, age=age, x=self.x + self.vx * ahead, y=self.y + self.vy * ahead,
                            vx=self.vx, vy=self.vy)
