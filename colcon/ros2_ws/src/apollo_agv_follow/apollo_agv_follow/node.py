"""ROS 2 wrapper around a guidance algorithm: topics in, GuidanceInput built, algorithm stepped, commands out.

Subscribes   odom              nav_msgs/Odometry           robot pose and speed in the odom frame
             joint_states      sensor_msgs/JointState      ring_joint position and velocity
             subject/pose      geometry_msgs/PoseStamped   subject position in odom; header.stamp = capture time
             ultrasonic/front, rear, left, right   sensor_msgs/Range
             imu/data          sensor_msgs/Imu             yaw rate and accelerations, robot frame
Publishes    cmd_vel           geometry_msgs/Twist
             ring/command      std_msgs/Float64            ring angle, rad, relative to the robot
             subject/estimate  geometry_msgs/PoseStamped   what the tracker believes, in odom
             guidance/debug/<name>  std_msgs/Float64       every number the algorithm puts in its debug dict

Parameters   algorithm         which guidance to run (see apollo_agv_follow/guidance/__init__.py)
             <algorithm>.<p>   that algorithm's tunables; can be changed while running (ros2 param set, Foxglove)
             goal_distance, goal_angle, rate, odom_timeout, safety_v_max, safety_w_max, tracker.<p>

In simulation subject/pose comes from the simulator's camera model. On the robot it will come from perception
(RealSense on the ring), which is why the guidance only ever sees that one topic.
"""
import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import Float64

from apollo_agv_follow import guidance
from apollo_agv_follow.guidance import (FollowGoal, GuidanceInput, GuidanceOutput, ImuState, Ranges, RingState,
                                        RobotState, sanitize, with_overrides)
from apollo_agv_follow.tracking import SubjectTracker, TrackerParams


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def declare_dataclass(node, prefix, params):
    """Declare one ROS parameter per dataclass field, defaulting to its current value; return the filled copy."""
    values = {f.name: node.declare_parameter(f'{prefix}.{f.name}', getattr(params, f.name)).value
              for f in fields(params)}
    return with_overrides(params, values)


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        p = self.declare_parameter
        name = p('algorithm', guidance.DEFAULT).value
        self.algorithm = guidance.make(name)
        if self.algorithm.params is not None:
            self.algorithm.params = declare_dataclass(self, name, self.algorithm.params)
        self.rate = p('rate', 30.0).value
        self.odom_timeout = p('odom_timeout', 0.5).value
        self.v_max, self.w_max = p('safety_v_max', 1.5).value, p('safety_w_max', 2.5).value
        self.goal = FollowGoal(distance=p('goal_distance', FollowGoal.distance).value,
                               angle=p('goal_angle', FollowGoal.angle).value)
        self.tracker = SubjectTracker(declare_dataclass(self, 'tracker', TrackerParams()))
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.robot, self.odom_stamp = RobotState(), None
        self.ring = RingState()
        self.ranges = Ranges()
        self.imu = ImuState()
        self.last_step = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(PoseStamped, 'subject/pose', self.on_subject, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        for side in ('front', 'rear', 'left', 'right'):
            self.create_subscription(Range, f'ultrasonic/{side}', lambda m, s=side: setattr(self.ranges, s, m.range), 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ring_pub = self.create_publisher(Float64, 'ring/command', 10)
        self.estimate_pub = self.create_publisher(PoseStamped, 'subject/estimate', 10)
        self.debug_pubs = {}
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(f"follower up: algorithm '{name}', holding {self.goal.distance:.1f} m")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_set_parameters(self, params):
        """Live tuning: `<algorithm>.<field>`, goal_distance and goal_angle take effect on the next step."""
        try:
            prefix = f'{self.algorithm.name}.'
            mine = {q.name[len(prefix):]: q.value for q in params if q.name.startswith(prefix)}
            if mine and self.algorithm.params is not None:
                self.algorithm.params = with_overrides(self.algorithm.params, mine)
            for q in params:
                if q.name == 'goal_distance':
                    self.goal.distance = float(q.value)
                elif q.name == 'goal_angle':
                    self.goal.angle = float(q.value)
        except (KeyError, TypeError, ValueError) as err:
            return SetParametersResult(successful=False, reason=str(err))
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        pose, twist = msg.pose.pose, msg.twist.twist
        self.robot = RobotState(x=pose.position.x, y=pose.position.y, yaw=yaw_of(pose.orientation),
                                v=twist.linear.x, w=twist.angular.z)
        self.odom_stamp = self.now()

    def on_joints(self, msg):
        if 'ring_joint' in msg.name:
            i = msg.name.index('ring_joint')
            self.ring = RingState(angle=msg.position[i], velocity=msg.velocity[i] if len(msg.velocity) > i else 0.0)

    def on_subject(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tracker.update(stamp if stamp > 0.0 else self.now(), msg.pose.position.x, msg.pose.position.y)

    def on_imu(self, msg):
        self.imu = ImuState(yaw_rate=msg.angular_velocity.z,
                            accel_x=msg.linear_acceleration.x, accel_y=msg.linear_acceleration.y)

    def step(self):
        t = self.now()
        dt = t - self.last_step if self.last_step is not None else 1.0 / self.rate
        self.last_step = t
        if self.odom_stamp is None or t - self.odom_stamp > self.odom_timeout:
            cmd = GuidanceOutput(ring=self.ring.angle)          # no idea where we are: stop, hold the ring
        else:
            inp = GuidanceInput(t=t, dt=dt, robot=self.robot, ring=self.ring, subject=self.tracker.state(t),
                                ranges=self.ranges, imu=self.imu, goal=self.goal)
            try:
                cmd = self.algorithm.step(inp)
            except Exception as err:                            # a bug in the algorithm must not leave the base moving
                self.get_logger().error(f'guidance failed, stopping: {err!r}', throttle_duration_sec=1.0)
                cmd = GuidanceOutput(ring=self.ring.angle)
            self.publish_estimate(inp)
        cmd = sanitize(cmd, self.v_max, self.w_max, self.ring.angle)

        twist = Twist()
        twist.linear.x, twist.angular.z = float(cmd.v), float(cmd.w)
        self.cmd_pub.publish(twist)
        self.ring_pub.publish(Float64(data=float(cmd.ring)))
        for key, value in cmd.debug.items():
            if key not in self.debug_pubs:
                self.debug_pubs[key] = self.create_publisher(Float64, f'guidance/debug/{key}', 10)
            self.debug_pubs[key].publish(Float64(data=float(value)))

    def publish_estimate(self, inp):
        if not inp.subject.visible:
            return
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'odom'
        msg.pose.position.x, msg.pose.position.y = inp.subject.x, inp.subject.y
        heading = math.atan2(inp.subject.vy, inp.subject.vx)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        self.estimate_pub.publish(msg)


def main():
    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
