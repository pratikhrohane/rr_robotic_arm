from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("rr_robotic_arm"), "urdf", "robot.urdf.xacro"),
        description="Absolute path to robot URDF"
    )

    robot_description = Command(["xacro ", LaunchConfiguration("model")])

    return LaunchDescription([
        model_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "manipulator", "-topic", "robot_description"],
            output="screen"
        ),
    ])
