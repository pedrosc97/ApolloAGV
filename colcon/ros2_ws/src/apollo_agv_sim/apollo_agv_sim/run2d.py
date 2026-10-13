"""Closed-loop runs of a guidance algorithm against the 2D model, with no ROS involved.

    python3 -m apollo_agv_sim.run2d                                  default algorithm, default scenario
    python3 -m apollo_agv_sim.run2d --scenario all                   every scenario, one table
    python3 -m apollo_agv_sim.run2d --algorithm all --plot run.png   every algorithm, overlaid in one picture
    python3 -m apollo_agv_sim.run2d --set k_ring=0.3 --distance 3    override parameters without editing code

On the Mac, run it from the repo root with:  bash scripts/dev.sh run2d [args]
This is the fastest loop for designing guidance: change the code, rerun, read the numbers, look at the picture.

The loop mirrors the ROS stack: the camera delivers subject measurements, the same SubjectTracker the follower
node uses turns them into an estimate, and the algorithm runs at the node's rate on a GuidanceInput built the
same way. The physics runs faster, and holds the last command between guidance steps.
"""
import argparse
import math
import os

from apollo_agv_follow import guidance
from apollo_agv_follow.guidance import (FollowGoal, GuidanceInput, ImuState, Ranges, RingState, RobotState,
                                        sanitize)
from apollo_agv_follow.tracking import SubjectTracker
from apollo_agv_sim import scenarios
from apollo_agv_sim.model import World

PHYSICS_DT = 0.01               # s
CONTROL_RATE = 30.0             # Hz, the follower node's default
SAFETY_V_MAX = 1.5              # m/s, the node's hard limits on any algorithm's output
SAFETY_W_MAX = 2.5              # rad/s


def observe(world, tracker, goal, dt):
    """The GuidanceInput the follower node would build from the topics the simulator publishes."""
    agv = world.agv
    yaw_rate, accel_x, accel_y = world.imu()
    return GuidanceInput(
        t=world.time, dt=dt,
        robot=RobotState(x=agv.x, y=agv.y, yaw=agv.yaw, v=agv.v, w=agv.w),
        ring=RingState(angle=agv.ring, velocity=agv.ring_vel),
        subject=tracker.state(world.time),
        ranges=Ranges(**world.ranges()),
        imu=ImuState(yaw_rate=yaw_rate, accel_x=accel_x, accel_y=accel_y),
        goal=goal,
    )


def run(seconds=60.0, algorithm=None, world=None, goal=None, control_rate=CONTROL_RATE, physics_dt=PHYSICS_DT):
    """Simulate and return (world, log). log is a list of dicts, one per guidance step."""
    algorithm = algorithm or guidance.make(guidance.DEFAULT)
    world = world or World()
    goal = goal or FollowGoal()
    tracker = SubjectTracker()
    algorithm.reset()
    control_dt = 1.0 / control_rate
    next_control, last_control = 0.0, 0.0
    cmd = None
    log = []
    for _ in range(int(round(seconds / physics_dt))):
        for t, x, y in world.detections():
            tracker.update(t, x, y)
        if world.time + 1e-9 >= next_control:
            inp = observe(world, tracker, goal, world.time - last_control)
            cmd = sanitize(algorithm.step(inp), SAFETY_V_MAX, SAFETY_W_MAX, inp.ring.angle)
            last_control, next_control = world.time, next_control + control_dt
            log.append({
                't': world.time, 'x': world.agv.x, 'y': world.agv.y, 'yaw': world.agv.yaw,
                'sx': world.subject.x, 'sy': world.subject.y, 'ex': inp.subject.x, 'ey': inp.subject.y,
                'seen': world.camera.sees(world.agv, world.subject),
                'distance': world.subject_distance(), 'ring_error': world.ring_pointing_error(),
                'ring': world.agv.ring, 'v': world.agv.v, 'w': world.agv.w, 'v_cmd': cmd.v, 'w_cmd': cmd.w,
                'obstacle': min(world.ranges().values()), 'debug': dict(cmd.debug),
            })
        world.step(physics_dt, cmd.v, cmd.w, cmd.ring)
    return world, log


def summarise(log, goal_distance, settle=5.0):
    rows = [r for r in log if r['t'] >= settle]
    dist_err = [abs(r['distance'] - goal_distance) for r in rows]
    ring_err = [abs(r['ring_error']) for r in rows]
    return {
        'distance_error_mean': sum(dist_err) / len(dist_err),
        'distance_error_max': max(dist_err),
        'ring_error_mean_deg': math.degrees(sum(ring_err) / len(ring_err)),
        'ring_error_max_deg': math.degrees(max(ring_err)),
        'closest_approach': min(r['distance'] for r in log),
        'closest_obstacle': min(r['obstacle'] for r in log),
        'out_of_view_pct': 100.0 * sum(not r['seen'] for r in rows) / len(rows),
    }


COLUMNS = [('distance_error_mean', 'dist err mean', 'm'), ('distance_error_max', 'max', 'm'),
           ('ring_error_mean_deg', 'aim err mean', 'deg'), ('ring_error_max_deg', 'max', 'deg'),
           ('closest_approach', 'closest subj', 'm'), ('closest_obstacle', 'closest sonar', 'm'),
           ('out_of_view_pct', 'out of view', '%')]


def table(results):
    """results: list of (scenario, algorithm, stats). Returns printable text."""
    head = f"{'scenario':<12} {'algorithm':<10}" + ''.join(f' {label:>13}' for _, label, _ in COLUMNS)
    units = ' ' * 23 + ''.join(f' {unit:>13}' for _, _, unit in COLUMNS)
    lines = [head, units]
    for scenario, algorithm, stats in results:
        lines.append(f'{scenario:<12} {algorithm:<10}' + ''.join(f' {stats[key]:>13.2f}' for key, _, _ in COLUMNS))
    return '\n'.join(lines)


def plot(runs, goal_distance, path, title=''):
    """runs: list of (label, world, log), overlaid in one figure."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    debug_keys = sorted({k for _, _, log in runs for r in log for k in r['debug']})
    panels = 4 + (1 if debug_keys else 0)
    fig, axes = plt.subplots(panels, 1, figsize=(9, 4 + 2.2 * panels),
                             gridspec_kw={'height_ratios': [3] + [1] * (panels - 1)})
    top, dist, aim, cmd = axes[:4]

    _, world, log = runs[0]
    top.plot([r['sx'] for r in log], [r['sy'] for r in log], 'k-', linewidth=1, label='subject')
    for x, y, radius in world.obstacles:
        top.add_patch(plt.Circle((x, y), radius, color='0.5'))
    styles = ['-', '--', ':', '-.']
    for i, (label, _, log) in enumerate(runs):
        color = f'C{i}'                                  # one colour per algorithm, in every panel
        top.plot([r['x'] for r in log], [r['y'] for r in log], color=color, label=f'AGV {label}')
        t = [r['t'] for r in log]
        dist.plot(t, [r['distance'] for r in log], color=color, label=label)
        aim.plot(t, [math.degrees(r['ring_error']) for r in log], color=color, label=label)
        cmd.plot(t, [r['v_cmd'] for r in log], color=color, label=f'v {label}')
        cmd.plot(t, [r['w_cmd'] for r in log], color=color, linestyle='--', alpha=0.6, label=f'w {label}')
        keys = sorted({k for r in log for k in r['debug']})
        for j, key in enumerate(keys):                   # one line style per debug channel
            axes[4].plot(t, [r['debug'].get(key, math.nan) for r in log], color=color,
                         linestyle=styles[j % len(styles)], linewidth=1, label=f'{key} ({label})')
    top.set_aspect('equal'), top.set_xlabel('x (m)'), top.set_ylabel('y (m)'), top.legend(fontsize=8)
    top.set_title(title or 'Paths')
    dist.axhline(goal_distance, linestyle=':', linewidth=1, color='k')
    dist.set_ylabel('distance (m)'), dist.legend(fontsize=7)
    aim.set_ylabel('aim error (deg)')
    cmd.set_ylabel('command (m/s, rad/s)'), cmd.legend(fontsize=7, ncol=2)
    if debug_keys:
        axes[4].set_ylabel('debug'), axes[4].legend(fontsize=7, ncol=2)
    axes[-1].set_xlabel('time (s)')
    for ax in axes[1:]:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def parse_overrides(pairs):
    out = {}
    for pair in pairs:
        name, sep, value = pair.partition('=')
        if not sep:
            raise SystemExit(f'--set expects name=value, got {pair!r}')
        out[name.strip()] = float(value)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--algorithm', default=guidance.DEFAULT,
                        help=f"one of {sorted(guidance.ALGORITHMS)}, or 'all' (default {guidance.DEFAULT})")
    parser.add_argument('--scenario', default=scenarios.DEFAULT,
                        help=f"one of {sorted(scenarios.SCENARIOS)}, or 'all' (default {scenarios.DEFAULT})")
    parser.add_argument('--seconds', type=float, default=None, help="override the scenario's length")
    parser.add_argument('--distance', type=float, default=FollowGoal.distance, help='follow distance in metres')
    parser.add_argument('--angle', type=float, default=None, help='where to sit around the subject, degrees (180 = behind)')
    parser.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                        help='override an algorithm parameter (repeatable); applied to every algorithm that has it')
    parser.add_argument('--rate', type=float, default=CONTROL_RATE, help='guidance rate, Hz')
    parser.add_argument('--plot', metavar='FILE', help='save a picture; with several scenarios, one file each')
    args = parser.parse_args()

    algorithms = sorted(guidance.ALGORITHMS) if args.algorithm == 'all' else [args.algorithm]
    names = list(scenarios.SCENARIOS) if args.scenario == 'all' else [args.scenario]
    overrides = parse_overrides(args.set)
    goal = FollowGoal(distance=args.distance)
    if args.angle is not None:
        goal.angle = math.radians(args.angle)

    results = []
    for name in names:
        runs = []
        for algo in algorithms:
            cls = guidance.ALGORITHMS[algo]
            known = {k: v for k, v in overrides.items() if hasattr(cls.Params(), k)}
            world, seconds = scenarios.make(name)
            world, log = run(seconds=args.seconds or seconds, algorithm=guidance.make(algo, known), world=world,
                             goal=goal, control_rate=args.rate)
            results.append((name, algo, summarise(log, goal.distance)))
            runs.append((algo, world, log))
        if args.plot:
            stem, ext = os.path.splitext(args.plot)
            path = args.plot if len(names) == 1 else f'{stem}_{name}{ext}'
            try:
                plot(runs, goal.distance, path, title=f'{name}, follow {goal.distance:.1f} m')
                print(f'saved {path}')
            except ImportError:
                print('no picture: matplotlib is not installed (python3 -m pip install matplotlib)')
    unused = [k for k in overrides if not any(hasattr(guidance.ALGORITHMS[a].Params(), k) for a in algorithms)]
    if unused:
        print(f'warning: no selected algorithm has parameter(s) {sorted(unused)}')
    print(f'follow distance {goal.distance:.2f} m, guidance at {args.rate:.0f} Hz, first 5 s ignored')
    print(table(results))


if __name__ == '__main__':
    main()
