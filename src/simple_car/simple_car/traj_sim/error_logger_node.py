#!/usr/bin/env python3
import math
import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path


class ErrorLoggerNode(Node):
    def __init__(self):
        super().__init__('error_logger_node')

        self.path = None
        self.start_time = self.get_clock().now()

        home = os.path.expanduser('~')
        self.file_path = os.path.join(home, 'trajectory_error_log.csv')

        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y', 'cross_track_error'])

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

        self.get_logger().info(f'Logging error to {self.file_path}')

    def path_callback(self, msg):
        self.path = msg

    def odom_callback(self, msg):
        if self.path is None or len(self.path.poses) == 0:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        min_dist = float('inf')

        for pose in self.path.poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - x, py - y)

            if dist < min_dist:
                min_dist = dist

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9

        self.writer.writerow([t, x, y, min_dist])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ErrorLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()