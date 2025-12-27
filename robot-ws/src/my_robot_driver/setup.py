from setuptools import find_packages, setup

package_name = 'my_robot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'coverage_driver_node = my_robot_driver.coverage_driver:main',
        'emergency_control_node = my_robot_driver.emergency_control_node:main',
    ],
    },
)
