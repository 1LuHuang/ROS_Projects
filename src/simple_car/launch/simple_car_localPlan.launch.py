from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    world = '/home/yiluhuang/workspace/Desktop/ros2_ws/src/simple_car/worlds/world_withObs.sdf'
    model = '/home/yiluhuang/workspace/Desktop/ros2_ws/src/simple_car/models/simple_model.sdf'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                 
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-file', model,
                        '-name', 'simple_car',
                        '-x', '0',
                        '-y', '0',
                        '-z', '0.2'
                    ],
                    output='screen'
                ),
                ]
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen'
        )
    ])

