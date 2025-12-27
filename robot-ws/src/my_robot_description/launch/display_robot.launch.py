from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.environ['HOME'],
        'robot_ws/src/my_robot_description/urdf/my_robot.urdf'
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/robot_ws/src/my_robot_description/urdf/my_robot.urdf').read()}]
        ),

        # Joint State Publisher GUI (optional, for moving joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

