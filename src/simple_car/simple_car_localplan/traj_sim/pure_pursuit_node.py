#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node_AStar')

        self.path = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.lookahead_distance = 0.8
        self.linear_speed = 0.5
        self.max_angular_speed = 1.5

        self.path_sub = self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

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

    def control_loop(self):
        if self.path is None or len(self.path.poses) == 0:
            return
        
        goal_x = self.path.poses[-1].pose.position.x
        goal_y = self.path.poses[-1].pose.position.y

        dist_to_goal = math.sqrt(
        (goal_x - self.x) ** 2 +
        (goal_y - self.y) ** 2
    )

        if dist_to_goal < 0.25:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        target = self.find_lookahead_point()
        if target is None:
            return

        tx, ty = target

        dx = tx - self.x
        dy = ty - self.y

        # transform target point into vehicle coordinate frame
        local_x = math.cos(-self.yaw) * dx - math.sin(-self.yaw) * dy
        local_y = math.sin(-self.yaw) * dx + math.cos(-self.yaw) * dy

        if local_x < 0.0:
            return

        curvature = 2.0 * local_y / (self.lookahead_distance ** 2)
        angular_z = self.linear_speed * curvature

        angular_z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_z)
        )

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def find_lookahead_point(self):
        points = self.path.poses

        min_dist = float('inf')
        closest_index = 0

        for i, pose in enumerate(points):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - self.x, py - self.y)

            if dist < min_dist:
                min_dist = dist
                closest_index = i

        for i in range(closest_index, len(points)):
            px = points[i].pose.position.x
            py = points[i].pose.position.y
            dist = math.hypot(px - self.x, py - self.y)

            if dist >= self.lookahead_distance:
                return px, py

        return points[-1].pose.position.x, points[-1].pose.position.y


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()