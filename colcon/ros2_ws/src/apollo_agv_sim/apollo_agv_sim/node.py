"""ROS 2 face of the 2D model: stands in for the motors, the ring drive, the ultrasonics and perception.

Subscribes   cmd_vel             geometry_msgs/Twist        base command
             ring/command        std_msgs/Float64           ring target angle, rad, relative to the robot
Publishes    odom                nav_msgs/Odometry          + TF odom -> base_footprint
             joint_states        sensor_msgs/JointState     wheels and ring, so the URDF model moves
             ultrasonic/<side>   sensor_msgs/Range          front, rear, left, right
             subject/pose        geometry_msgs/PoseStamped  ground truth, in the odom frame
             subject/marker      visualization_msgs/Marker  something to look at in Foxglove / RViz

The robot-side nodes only see these topics, so they cannot tell this from the real hardware.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from apollo_agv_sim.model import Agv, Limits, Subject, World, default_sonars


class Sim2D(Node):
    def __init__(self):
        super().__init__('sim2d')
        p = self.declare_parameter
        self.dt = 1.0 / p('rate', 50.0).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.wheel_radius = p('wheel_radius', 0.05).value
        self.wheel_separation = p('wheel_separation', 0.32).value
        limits = Limits(v_max=p('v_max', 1.5).value, w_max=p('w_max', 2.5).value,
                        a_max=p('a_max', 1.0).value, alpha_max=p('alpha_max', 4.0).value,
                        ring_vel_max=p('ring_vel_max', 3.0).value, ring_acc_max=p('ring_acc_max', 12.0).value)
        sonars = default_sonars(p('base_length', 0.40).value, p('base_width', 0.30).value,
                                fov=p('ultrasonic_fov', 0.52).value, min_range=p('ultrasonic_min_range', 0.02).value,
                                max_range=p('ultrasonic_max_range', 4.0).value)
        subject = Subject(speed=p('subject_speed', 1.2).value)
        flat = p('subject_waypoints', [0.0]).value          # x0, y0, x1, y1, ...; fewer than 2 points = built-in walk
        if len(flat) >= 4:
            subject = Subject(waypoints=list(zip(flat[0::2], flat[1::2])), speed=subject.speed)
        self.world = World(agv=Agv(limits=limits), subject=subject, sonars=sonars)

        self.cmd = (0.0, 0.0)
        self.cmd_stamp = None
        self.ring_target = 0.0
        self.wheel_angle = [0.0, 0.0]

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Float64, 'ring/command', lambda m: setattr(self, 'ring_target', m.data), 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.subject_pub = self.create_publisher(PoseStamped, 'subject/pose', 10)
        self.marker_pub = self.create_publisher(Marker, 'subject/marker', 10)
        self.range_pubs = {side: self.create_publisher(Range, f'ultrasonic/{side}', 10) for side in self.world.sonars}
        self.tf = TransformBroadcaster(self)
        self.create_timer(self.dt, self.step)
        self.get_logger().info('2D simulator up')

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_stamp = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        # Like a real motor driver: stop if commands stop arriving.
        alive = self.cmd_stamp is not None and (now - self.cmd_stamp).nanoseconds * 1e-9 < self.cmd_timeout
        v_cmd, w_cmd = self.cmd if alive else (0.0, 0.0)
        self.world.step(self.dt, v_cmd, w_cmd, self.ring_target)
        agv, subject, stamp = self.world.agv, self.world.subject, now.to_msg()
        qz, qw = math.sin(agv.yaw / 2), math.cos(agv.yaw / 2)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = agv.x, agv.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = agv.v, agv.w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = agv.x, agv.y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)

        half = self.wheel_separation / 2
        self.wheel_angle[0] += (agv.v - agv.w * half) / self.wheel_radius * self.dt
        self.wheel_angle[1] += (agv.v + agv.w * half) / self.wheel_radius * self.dt
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint', 'ring_joint']
        joints.position = [self.wheel_angle[0], self.wheel_angle[1], agv.ring]
        joints.velocity = [(agv.v - agv.w * half) / self.wheel_radius, (agv.v + agv.w * half) / self.wheel_radius, agv.ring_vel]
        self.joint_pub.publish(joints)

        for side, value in self.world.ranges().items():
            sonar = self.world.sonars[side]
            msg = Range()
            msg.header.stamp, msg.header.frame_id = stamp, f'ultrasonic_{side}_link'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view, msg.min_range, msg.max_range, msg.range = sonar.fov, sonar.min_range, sonar.max_range, value
            self.range_pubs[side].publish(msg)

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'odom'
        pose.pose.position.x, pose.pose.position.y = subject.x, subject.y
        pose.pose.orientation.w = 1.0
        self.subject_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns, marker.type, marker.action = 'subject', Marker.CYLINDER, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = subject.x, subject.y, 0.85
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 2 * subject.radius
        marker.scale.z = 1.7
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.6, 1.0, 0.9
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = Sim2D()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
