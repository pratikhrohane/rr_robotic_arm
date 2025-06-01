from setuptools import find_packages, setup

package_name = 'rr_robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/rr_robotic_arm/urdf', ['urdf/robot.urdf.xacro']),       #  URDF files
        ('share/rr_robotic_arm/urdf', ['urdf/robot_gazebo.xacro']),       #  URDF files
        ('share/rr_robotic_arm/urdf', ['urdf/robot_ros2_control.xacro']),       #  URDF files
        ('share/rr_robotic_arm/config', ['config/ros2_controllers.yaml']),       #  config files
        ('share/rr_robotic_arm/config', ['config/rr_robotic_arm.rviz']),       #  rviz files
        ('share/rr_robotic_arm/launch', ['launch/gazebo_launch.py']),  #  launch files
        ('share/rr_robotic_arm/launch', ['launch/spawn_robot_launch.py']),  #  launch files
        ('share/rr_robotic_arm/launch', ['launch/launch_controllers_and_rviz.launch.py']),  #  launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratik',
    maintainer_email='pratik@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_arm_to_target = rr_robotic_arm.move_arm_to_target:main',
            'gripper_command = rr_robotic_arm.gripper_command:main',
        ],
    },
)
