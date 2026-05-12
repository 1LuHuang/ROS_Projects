#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Figure8PathNode(Node):
    def __init__(self):
        super().__init__('figure8_path_node')
        self.pub = self.create_publisher(Path, '/reference_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        A = 3.0
        B = 2.0
        N = 300

        for i in range(N):
            t = 2.0 * math.pi * i / N
            x = A * math.sin(t)
            y = B * math.sin(t) * math.cos(t)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = Figure8PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# _init__.py is nessary for Python to recognize the directory as a package.