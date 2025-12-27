from setuptools import find_packages, setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
       ('share/ament_index/resource_index/packages',
         ['resource/my_robot_description']),
       ('share/my_robot_description', ['package.xml']),
       ('share/' + package_name + '/launch', ['launch/spawn_robot.launch.py','launch/display_robot.launch.py','launch/gazebo.launch.py','launch/spawn_robot_with_slam.launch.py',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/my_robot.urdf',  # optional: add URDF so ROS2 can find it
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chester',
    maintainer_email='chester@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
