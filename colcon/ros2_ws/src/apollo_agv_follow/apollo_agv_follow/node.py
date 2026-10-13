"""ROS 2 wrapper around controller.follow_step.

Subscribes   subject/pose      geometry_msgs/PoseStamped   subject position in the odom frame
             odom              nav_msgs/Odometry           robot pose in the odom frame
             ultrasonic/front  sensor_msgs/Range
             ultrasonic/rear   sensor_msgs/Range
Publishes    cmd_vel           geometry_msgs/Twist
             ring/command      std_msgs/Float64            ring angle, rad, relative to the robot

In simulation subject/pose is ground truth from the simulator. On the robot it will come from perception
(RealSense on the ring), which is why the controller only ever sees that one topic.
"""
import math
from dataclasses import fields

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64

from apollo_agv_follow.controller import FollowParams, follow_step, idle_command


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        defaults = FollowParams()
        self.params = FollowParams(**{f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                                      for f in fields(FollowParams)})
        self.rate = self.declare_parameter('rate', 30.0).value
        self.subject_timeout = self.declare_parameter('subject_timeout', 0.5).value

        self.odom = None
        self.subject = None
        self.subject_stamp = None
        self.ranges = {'front': math.inf, 'rear': math.inf}
        self.last_ring = 0.0

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, 'subject/pose', self.on_subject, 10)
        for side in self.ranges:
            self.create_subscription(Range, f'ultrasonic/{side}', lambda m, s=side: self.ranges.__setitem__(s, m.range), 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ring_pub = self.create_publisher(Float64, 'ring/command', 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(f'follower up: holding {self.params.distance:.1f} m')

    def on_odom(self, msg):
        self.odom = msg.pose.pose

    def on_subject(self, msg):
        self.subject = msg.pose.position
        self.subject_stamp = self.get_clock().now()

    def step(self):
        fresh = (self.subject_stamp is not None and
                 (self.get_clock().now() - self.subject_stamp).nanoseconds * 1e-9 < self.subject_timeout)
        if self.odom is None or not fresh:
            cmd = idle_command(self.last_ring)
        else:
            # Subject from the odom frame into the robot frame.
            dx = self.subject.x - self.odom.position.x
            dy = self.subject.y - self.odom.position.y
            yaw = yaw_of(self.odom.orientation)
            x = math.cos(yaw) * dx + math.sin(yaw) * dy
            y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            cmd = follow_step(x, y, self.ranges['front'], self.ranges['rear'], self.params)
        self.last_ring = cmd.ring

        twist = Twist()
        twist.linear.x = cmd.v
        twist.angular.z = cmd.w
        self.cmd_pub.publish(twist)
        self.ring_pub.publish(Float64(data=cmd.ring))


def main():
    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
