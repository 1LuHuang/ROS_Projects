
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from simple_car_localplan.traj_sim.AStar import AStarPlanner


class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_path_node')
        self.pub = self.create_publisher(Path, '/reference_path', 10)
        self.planner = AStarPlanner(resolution=0.05, safety_margin=0.2)

        self.origin_path = [(x * 0.05 , 0.0) for x in range(81)
                            ]
        self.obstacle  = [(-2.0,0.0),(2.0,0.0),(-1.0,2.0)]
        self.path_points, self.start_idx, self.goal_idx = self.planner.replan_local_segment(
            self.origin_path,
            self.obstacle,
            buffer_distance=1.2
            )
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':    main()