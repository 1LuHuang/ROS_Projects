import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.path_points = []

        self.blocked_pub = self.create_publisher(
            Bool,
            '/path_blocked',
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/dynamic_obstacle/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info("Obstacle detector started")

    def path_callback(self, msg):

        self.path_points = []

        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_points.append((x, y))

        # self.get_logger().info(f"Received path with {len(self.path_points)} points")
        

           

    def pose_callback(self, msg):

        self.get_logger().info("pose_callback triggered")

        if len(self.path_points) == 0:
            self.get_logger().warn("No path received yet")
            return

        obs_x = msg.pose.position.x
        obs_y = msg.pose.position.y

        min_dist = 9999.0

        for px, py in self.path_points:
            dist = math.sqrt((obs_x - px)**2 + (obs_y - py)**2)

            if dist < min_dist:
                min_dist = dist

        blocked = min_dist < 0.8

        msg_out = Bool()
        msg_out.data = blocked
        self.blocked_pub.publish(msg_out)

        # self.get_logger().info(
        #     f"Obstacle: x={obs_x:.2f}, y={obs_y:.2f}, min_dist={min_dist:.2f}, blocked={blocked}"
        # )

def main(args=None):

    rclpy.init(args=args)

    node = ObstacleDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()