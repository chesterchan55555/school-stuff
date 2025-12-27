from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Absolute path to your world file
    world_path = os.path.join(
        os.environ['HOME'],
        'robot_ws/src/my_robot_description/worlds/simple_world.world'
    )

    return LaunchDescription([
        # Launch Gazebo with ROS2 plugin
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '--ros-args',
                '-s', 'libgazebo_ros_factory.so',  # ROS2 spawn plugin
                world_path
            ],
            output='screen'
        ),
    ])
