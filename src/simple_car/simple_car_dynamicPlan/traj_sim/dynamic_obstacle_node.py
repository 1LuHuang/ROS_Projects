import rclpy
from rclpy.node import Node
import subprocess
import math
from geometry_msgs.msg import PoseStamped

# used to update the position of the dynamic obstacle in Gazebo by calling the set_pose service at a fixed frequency. 
# meanwhile, it also publishes the current pose of the dynamic obstacle to a ROS topic, which can be used by other nodes.
class DynamicObstacle(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle')

        self.world_name = 'world_dyn_Obs'
        self.model_name = 'dynamic_obstacle'

        self.t = 0.0
        self.dt = 0.02

        self.timer = self.create_timer(self.dt, self.update_obstacle)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/dynamic_obstacle/pose',
            10
)

    def update_obstacle(self):
        self.t += self.dt

        x = 3.0
        y = 3.0 * math.sin(10 * self.t)
        z = 0.25

        req = f"""
        name: "{self.model_name}"
        position {{
          x: {x}
          y: {y}
          z: {z}
        }}
        orientation {{
          w: 1
          x: 0
          y: 0
          z: 0
        }}
        """

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

        cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", req
        ]

        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()