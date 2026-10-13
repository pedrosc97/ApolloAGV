"""Closed-loop run of the follow controller against the 2D model, with no ROS involved.

    python3 -m apollo_agv_sim.run2d                      print how well it followed
    python3 -m apollo_agv_sim.run2d --plot follow.png    also save a picture (needs matplotlib)

On the Mac, run it from the repo root with:  bash scripts/dev.sh run2d
This is the fastest loop for tuning: change a gain, rerun, read the numbers.
"""
import argparse
import math

from apollo_agv_follow.controller import FollowParams, follow_step
from apollo_agv_sim.model import World


def run(seconds=60.0, dt=1.0 / 50.0, params=None, world=None):
    """Simulate and return (world, log). log is a list of dicts, one per step."""
    params = params or FollowParams()
    world = world or World()
    log = []
    for _ in range(int(round(seconds / dt))):
        sx, sy = world.subject_in_robot_frame()
        ranges = world.ranges()
        cmd = follow_step(sx, sy, ranges['front'], ranges['rear'], params)
        world.step(dt, cmd.v, cmd.w, cmd.ring)
        log.append({
            't': world.time, 'x': world.agv.x, 'y': world.agv.y, 'yaw': world.agv.yaw,
            'sx': world.subject.x, 'sy': world.subject.y,
            'distance': world.subject_distance(), 'ring_error': world.ring_pointing_error(),
            'ring': world.agv.ring, 'v': world.agv.v, 'w': world.agv.w,
        })
    return world, log


def summarise(log, params, settle=5.0):
    rows = [r for r in log if r['t'] >= settle]
    dist_err = [abs(r['distance'] - params.distance) for r in rows]
    ring_err = [abs(r['ring_error']) for r in rows]
    return {
        'distance_error_mean': sum(dist_err) / len(dist_err),
        'distance_error_max': max(dist_err),
        'ring_error_mean_deg': math.degrees(sum(ring_err) / len(ring_err)),
        'ring_error_max_deg': math.degrees(max(ring_err)),
        'closest_approach': min(r['distance'] for r in log),
    }


def plot(log, params, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t = [r['t'] for r in log]
    fig, (top, mid, low) = plt.subplots(3, 1, figsize=(8, 11), gridspec_kw={'height_ratios': [3, 1, 1]})
    top.plot([r['sx'] for r in log], [r['sy'] for r in log], label='subject')
    top.plot([r['x'] for r in log], [r['y'] for r in log], label='AGV')
    top.set_aspect('equal'), top.set_xlabel('x (m)'), top.set_ylabel('y (m)'), top.legend(), top.set_title('Paths')
    mid.plot(t, [r['distance'] for r in log])
    mid.axhline(params.distance, linestyle='--', linewidth=1)
    mid.set_ylabel('distance to subject (m)')
    low.plot(t, [math.degrees(r['ring_error']) for r in log])
    low.set_ylabel('camera pointing error (deg)'), low.set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--distance', type=float, default=None, help='follow distance in metres')
    parser.add_argument('--plot', metavar='FILE', help='save a picture of the run')
    args = parser.parse_args()

    params = FollowParams()
    if args.distance is not None:
        params.distance = args.distance
    _, log = run(seconds=args.seconds, params=params)
    stats = summarise(log, params)
    print(f'follow distance {params.distance:.2f} m, {args.seconds:.0f} s simulated (first 5 s ignored)')
    print(f"  distance error   mean {stats['distance_error_mean']:.2f} m   max {stats['distance_error_max']:.2f} m")
    print(f"  camera pointing  mean {stats['ring_error_mean_deg']:.1f} deg  max {stats['ring_error_max_deg']:.1f} deg")
    print(f"  closest approach {stats['closest_approach']:.2f} m")
    if args.plot:
        try:
            plot(log, params, args.plot)
            print(f'  saved {args.plot}')
        except ImportError:
            print('  no picture: matplotlib is not installed (python3 -m pip install matplotlib)')


if __name__ == '__main__':
    main()
