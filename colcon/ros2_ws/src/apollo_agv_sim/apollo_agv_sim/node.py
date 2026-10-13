"""ROS 2 face of the 2D model: stands in for the motors, the ring drive, the ultrasonics and perception.

Subscribes   cmd_vel             geometry_msgs/Twist        base command
             ring/command        std_msgs/Float64           ring target angle, rad, relative to the robot
Publishes    odom                nav_msgs/Odometry          + TF odom -> base_footprint
             joint_states        sensor_msgs/JointState     wheels and ring, so the URDF model moves
             ultrasonic/<side>   sensor_msgs/Range          front, rear, left, right
             imu/data            sensor_msgs/Imu            yaw rate and accelerations, frame imu_link
             subject/pose        geometry_msgs/PoseStamped  what the camera model delivers, in odom: only when the
                                                            subject is in view, stamped with the capture time
             subject/ground_truth geometry_msgs/PoseStamped where the subject really is, every step
             subject/marker      visualization_msgs/Marker  something to look at in Foxglove / RViz
             obstacles/markers   visualization_msgs/MarkerArray  the scenario's posts

Parameters   scenario            one of apollo_agv_sim/scenarios.py (loop, straight, noisy, ...)
             camera              'scenario' (whatever it defines), 'perfect' or 'realistic'

The robot-side nodes only see these topics (never subject/ground_truth), so they cannot tell this from the
real hardware.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from apollo_agv_sim import scenarios
from apollo_agv_sim.model import Agv, Camera, Limits, Subject, default_sonars


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
        self.scenario = p('scenario', scenarios.DEFAULT).value
        self.world, _ = scenarios.make(self.scenario)
        self.world.agv, self.world.sonars = Agv(limits=limits), sonars
        camera = p('camera', 'scenario').value
        if camera == 'perfect':
            self.world.camera = Camera()
        elif camera == 'realistic':
            self.world.camera = scenarios.realistic_camera()
        elif camera != 'scenario':
            raise ValueError(f"camera must be 'scenario', 'perfect' or 'realistic', not {camera!r}")
        speed = p('subject_speed', -1.0).value               # negative = the scenario's own pace
        flat = p('subject_waypoints', [0.0]).value          # x0, y0, x1, y1, ...; fewer than 2 points = the scenario's walk
        if len(flat) >= 4:
            self.world.subject = Subject(waypoints=list(zip(flat[0::2], flat[1::2])))
        if speed >= 0.0:
            self.world.subject.speed = speed
        self.clock_offset = None                            # ROS time - model time, s

        self.cmd = (0.0, 0.0)
        self.cmd_stamp = None
        self.ring_target = 0.0
        self.wheel_angle = [0.0, 0.0]

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Float64, 'ring/command', lambda m: setattr(self, 'ring_target', m.data), 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.subject_pub = self.create_publisher(PoseStamped, 'subject/pose', 10)
        self.truth_pub = self.create_publisher(PoseStamped, 'subject/ground_truth', 10)
        self.marker_pub = self.create_publisher(Marker, 'subject/marker', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, 'obstacles/markers', 10)
        self.range_pubs = {side: self.create_publisher(Range, f'ultrasonic/{side}', 10) for side in self.world.sonars}
        self.tf = TransformBroadcaster(self)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f"2D simulator up: scenario '{self.scenario}', camera '{camera}'")

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
        if self.clock_offset is None:
            self.clock_offset = now.nanoseconds * 1e-9 - self.world.time
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

        yaw_rate, accel_x, accel_y = self.world.imu()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.z = yaw_rate
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel_x, accel_y, 9.81
        self.imu_pub.publish(imu)

        # Perception: what the camera model releases now, stamped with when each frame was captured.
        for t, x, y in self.world.detections():
            seen = PoseStamped()
            seen.header.stamp, seen.header.frame_id = self.ros_time(t), 'odom'
            seen.pose.position.x, seen.pose.position.y = x, y
            seen.pose.orientation.w = 1.0
            self.subject_pub.publish(seen)

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'odom'
        pose.pose.position.x, pose.pose.position.y = subject.x, subject.y
        pose.pose.orientation.w = 1.0
        self.truth_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns, marker.type, marker.action = 'subject', Marker.CYLINDER, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = subject.x, subject.y, 0.85
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 2 * subject.radius
        marker.scale.z = 1.7
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.6, 1.0, 0.9
        self.marker_pub.publish(marker)

        posts = MarkerArray()
        for i, (x, y, radius) in enumerate(self.world.obstacles):
            post = Marker()
            post.header = pose.header
            post.ns, post.id, post.type, post.action = 'obstacles', i, Marker.CYLINDER, Marker.ADD
            post.pose.position.x, post.pose.position.y, post.pose.position.z = x, y, 0.5
            post.pose.orientation.w = 1.0
            post.scale.x = post.scale.y = 2 * radius
            post.scale.z = 1.0
            post.color.r, post.color.g, post.color.b, post.color.a = 0.5, 0.5, 0.5, 1.0
            posts.markers.append(post)
        if posts.markers:
            self.obstacle_pub.publish(posts)

    def ros_time(self, model_time):
        """A model time (s since the simulation started) as a ROS stamp."""
        total = self.clock_offset + model_time
        return Time(sec=int(total), nanosec=int((total - int(total)) * 1e9))


def main():
    rclpy.init()
    node = Sim2D()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
