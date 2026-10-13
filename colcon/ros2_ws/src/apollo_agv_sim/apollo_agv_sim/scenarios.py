"""Named test situations for the guidance. Each builds a fresh World; run2d and the sim2d node both use them.

    loop         the default walk: straights, a sharp sideways cut, a turn back towards the robot
    straight     a long straight walk away from the robot
    stop_and_go  walks, stands still for a few seconds, walks on
    towards      walks straight at the robot and past it
    obstacles    the default walk with posts beside and on the robot's likely path
    noisy        the default walk seen through a realistic camera: narrow view on the ring, 30 Hz, 100 ms late,
                 5 cm noise, 5 % dropped frames. The ring has to keep the subject in view or lose it.

Add a scenario by writing a function that returns (World, seconds) and listing it in SCENARIOS.
"""
import math

from apollo_agv_sim.model import DEFAULT_WAYPOINTS, Camera, Subject, World


# A RealSense D435 colour stream behind a person detector, roughly. PROVISIONAL until measured on the Pi.
def realistic_camera(seed=0):
    return Camera(fov=math.radians(69), max_range=8.0, rate=30.0, latency=0.10, noise=0.05, dropout=0.05,
                  seed=seed)


def loop():
    return World(), 90.0


def straight():
    return World(subject=Subject(waypoints=[(3.0, 0.0), (200.0, 0.0)])), 40.0


def stop_and_go():
    walk = [(3.0, 0.0), (10.0, 0.0), (10.0, 6.0), (3.0, 6.0)]
    return World(subject=Subject(waypoints=walk, dwell={1: 4.0, 2: 6.0, 3: 3.0})), 60.0


def towards():
    return World(subject=Subject(waypoints=[(8.0, 0.5), (-8.0, 0.5)], dwell={0: 3.0, 1: 3.0})), 40.0


def obstacles():
    posts = [(6.0, -0.6, 0.15), (9.0, 2.0, 0.2), (3.0, 4.5, 0.2), (2.0, 0.0, 0.2)]
    return World(obstacles=posts), 90.0


def noisy():
    return World(subject=Subject(waypoints=list(DEFAULT_WAYPOINTS)), camera=realistic_camera()), 90.0


SCENARIOS = {f.__name__: f for f in (loop, straight, stop_and_go, towards, obstacles, noisy)}
DEFAULT = 'loop'


def make(name):
    """(World, seconds) for a scenario name."""
    try:
        return SCENARIOS[name]()
    except KeyError:
        raise KeyError(f"unknown scenario '{name}'; known: {sorted(SCENARIOS)}") from None
