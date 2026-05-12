#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


class PIDTrackerNode(Node):
    def __init__(self):
        super().__init__('pid_tracker_node')

        self.path = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.linear_speed = 0.45

        self.kp_cte = 1.2
        self.kp_heading = 2.0
        self.ki = 0.0
        self.kd = 0.25

        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        self.max_angular_speed = 1.5

        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def path_callback(self, msg):
        self.path = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if self.path is None or len(self.path.poses) < 2:
            return

        closest_index = self.find_closest_point_index()

        if closest_index >= len(self.path.poses) - 1:
            closest_index = len(self.path.poses) - 2

        p1 = self.path.poses[closest_index].pose.position
        p2 = self.path.poses[closest_index + 1].pose.position

        path_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
        heading_error = self.normalize_angle(path_yaw - self.yaw)

        dx = self.x - p1.x
        dy = self.y - p1.y

        # signed cross-track error
        cross_track_error = math.sin(path_yaw) * dx - math.cos(path_yaw) * dy

        total_error = self.kp_cte * cross_track_error + self.kp_heading * heading_error

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        self.integral_error += total_error * dt
        derivative_error = (total_error - self.last_error) / dt

        angular_z = total_error + self.ki * self.integral_error + self.kd * derivative_error

        angular_z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_z)
        )

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        self.last_error = total_error
        self.last_time = now

    def find_closest_point_index(self):
        min_dist = float('inf')
        closest_index = 0

        for i, pose in enumerate(self.path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - self.x, py - self.y)

            if dist < min_dist:
                min_dist = dist
                closest_index = i

        return closest_index


def main(args=None):
    rclpy.init(args=args)
    node = PIDTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()