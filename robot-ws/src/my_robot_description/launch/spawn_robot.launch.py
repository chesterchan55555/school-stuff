from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',  # Name of your robot in Gazebo
                '-file', '/home/chester/robot_ws/src/my_robot_description/urdf/my_robot.urdf'
            ],
            output='screen'
        ),
    ])
