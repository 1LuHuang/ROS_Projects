#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class ActualPathNode(Node):
    def __init__(self):
        super().__init__('actual_path_node')
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(Path, '/actual_path', 10)

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)

        if len(self.path.poses) > 3000:
            self.path.poses.pop(0)

        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = ActualPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()