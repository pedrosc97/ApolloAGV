# ApolloAGV

ROS 2 stack for Project Apollo's follow-cam AGV: a differential-drive base with a pedestal, a rotating ring
carrying an Intel RealSense that tracks the subject, and a DJI Osmo gimbal with an iPhone on the ring's static
centre. The recording camera is deliberately outside this stack.

Target: ROS 2 Jazzy, in Docker, on a Raspberry Pi 5 (8 GB). The Mac (Apple silicon) and the Pi are both arm64,
so both run the same image, defined in `colcon/docker/Dockerfile`.

## Layout

One repo, two workspaces, one protocol schema shared between them.

```
ApolloAGV/
├── comms/
│   ├── protocol.yaml       # Pi <-> Arduino serial protocol: the source of truth
│   └── gen_protocol.py     # writes both generated files below; run by hand, not by either build
├── arduino/                # Arduino workspace (arduino-cli, no ROS). See arduino/README.md
│   ├── ring_stepper/       # ring stepper sketch; ring_protocol.h is generated
│   └── test/               # host-side tests (C++ header vs Python, byte for byte)
├── colcon/                 # everything ROS; mounted as /apollo in the container
│   ├── ros2_ws/            # colcon workspace: build, test and launch from here
│   │   └── src/
│   │       ├── apollo_agv_description/
│   │       ├── apollo_agv_follow/
│   │       ├── apollo_agv_sim/
│   │       ├── apollo_agv_bringup/
│   │       └── apollo_ring/    # ring link: ring_protocol.py (generated); ring_bridge node and bench CLI go here
│   ├── docker/             # one image for the Mac (dev) and the Pi (robot)
│   └── .devcontainer/
└── scripts/                # dev.sh (Mac dev loop), bench/ (standalone hardware smoke tests)
```

To change the protocol: edit `comms/protocol.yaml`, run `bash scripts/dev.sh gen`, commit the schema and both
generated files together. `test-native` fails if the generated files are out of date.

## Packages

| Package | What it is | State |
| --- | --- | --- |
| `apollo_agv_description` | URDF of the AGV. All dimensions come from `config/geometry.yaml`. | Working, tested. Dimensions are placeholders. |
| `apollo_agv_follow` | Follow controller: holds a distance, points the ring at the subject. | Simple baseline, tested. To be replaced by the real design. |
| `apollo_agv_sim` | 2D simulator that stands in for motors, ring, ultrasonics and perception. | Working, tested |
| `apollo_agv_bringup` | Launch files and parameters. | `sim2d` works; `robot` has no hardware drivers yet |
| `apollo_ring` | Pi side of the ring stepper link. | Protocol generated and tested; bridge node not in yet |

Not here yet: the DDSM115 and IMU drivers, the ring bridge node and the firmware sketch, RealSense perception,
`ros2_control`, and a Webots world. The image already contains `ros2_control` and, on the Mac, `webots_ros2`.

## Three ways to run it, fastest first

All commands are run on the Mac from the repo folder. `bash scripts/dev.sh` with no arguments lists them.

**1. Plain Python, no Docker, no ROS.** The controller and the 2D model are ordinary Python.

```bash
bash scripts/dev.sh test-native            # unit tests + closed-loop tests
bash scripts/dev.sh run2d --plot follow.png  # one closed-loop run, with numbers and a picture
```

Use this for tuning: change a gain in `colcon/ros2_ws/src/apollo_agv_follow/apollo_agv_follow/controller.py`, rerun, read the numbers.

**2. ROS 2 in Docker, 2D simulator.** The real nodes, topics and TF, with the simulator in place of hardware.

```bash
bash scripts/dev.sh image    # once, and after changing colcon/docker/Dockerfile
bash scripts/dev.sh test     # colcon build + colcon test
bash scripts/dev.sh sim      # then in Foxglove on the Mac: Open connection -> ws://localhost:8765
```

In Foxglove add a 3D panel (fixed frame `odom`) to see the robot model, the ring turning and the subject.
`bash scripts/dev.sh shell` gives a shell in the container for `ros2 topic echo` and the like.

**3. ROS 2 in Docker, Webots on the Mac.** 3D physics and rendered sensors. Webots runs natively on the Mac;
the container starts it and drives it over the network. Needs Webots R2025a or newer in `/Applications`.

```bash
bash scripts/dev.sh webots-server   # terminal 1, leave running
bash scripts/dev.sh webots-check    # terminal 2: stock TurtleBot demo, proves the link works
```

There is no Apollo world for Webots yet; that comes once the ring's mechanical design is settled.

## Topics

| Topic | Type | From | Meaning |
| --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | follower | base command |
| `ring/command` | `std_msgs/Float64` | follower | ring angle, rad, relative to the robot's forward direction |
| `odom` (+ TF `odom -> base_footprint`) | `nav_msgs/Odometry` | simulator / base driver | robot pose |
| `joint_states` | `sensor_msgs/JointState` | simulator / drivers | `left_wheel_joint`, `right_wheel_joint`, `ring_joint` |
| `ultrasonic/{front,rear,left,right}` | `sensor_msgs/Range` | simulator / driver | obstacle ranges |
| `subject/pose` | `geometry_msgs/PoseStamped` | simulator / perception | subject position in `odom` |

The follower only knows these topics, so it runs unchanged against the 2D simulator, Webots or the robot.

## On the Raspberry Pi

The repo is pushed from the Mac over SSH (the Pi does not pull from GitHub) into `~/ApolloAGV`, then built
inside the `robot` image; colcon builds in place under `~/ApolloAGV/colcon/ros2_ws`. The scripts for that live in
the Apollo workspace, not in this repo:

```bash
bash ~/Projects/Apollo/scripts/setup_pi_ssh.sh     # once: key login as `ssh apollo-pi`, records what the Pi runs
bash ~/Projects/Apollo/scripts/ship_agv.sh         # commit, optionally push to GitHub, push to the Pi, build, test
```

To run by hand on the Pi:

```bash
cd ~/ApolloAGV
docker compose -f colcon/docker/compose.yaml run --rm robot ros2 launch apollo_agv_bringup robot.launch.py
```
