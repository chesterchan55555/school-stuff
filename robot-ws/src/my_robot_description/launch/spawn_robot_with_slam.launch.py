from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
import xacro

def generate_launch_description():
    pkg_share = os.path.join(os.environ['HOME'], 'robot_ws', 'src', 'my_robot_description')

    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'random_image_obstacles.world')
    priority_mux_yaml = os.path.join(pkg_share, 'config', 'priority_mux.yaml')

    # Convert XACRO to URDF in Python
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([

        # Launch Gazebo with world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, **robot_description}],
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Spawn robot in Gazebo after 2 seconds
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
                    output='screen'
                )
            ]
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Teleop Twist Keyboard in xterm
        ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e',
                'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop'
            ],
            output='screen'
        ),

        # Coverage Driver
        Node(
            package='my_robot_driver',
            executable='coverage_driver_node',
            name='coverage_driver_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel', '/cmd_vel_coverage')]
        ),

        # Twist Mux
        #Node(
         #   package='twist_mux',
          #  executable='twist_mux',
           # name='twist_mux',
            #output='screen',
            #parameters=[priority_mux_yaml],
        #),


        # Emergency Stop / Resume Node in xterm
        ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e',
                'ros2 run my_robot_driver emergency_control_node'
            ],
            output='screen'
        ),

        # Image Detector
        Node(
            package='image_recognition',
            executable='image_detector_node',
            name='image_detector_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[],
        ),

        # Rosbridge Websocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            arguments=['--port', '9091'],
        ),
    ])
