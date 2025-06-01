from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml

def load_file(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(path, 'r') as f:
        return f.read()

def load_yaml(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Robot description from xacro
    xacro_file = os.path.join(get_package_share_directory('rr_robotic_arm'), 'urdf', 'robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Controller config
    controller_config = load_yaml('rr_robotic_arm', 'config/ros2_controllers.yaml')

    # Controller spawners
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # RViz
    rviz_config_path = os.path.join(get_package_share_directory('rr_robotic_arm'), 'config', 'rr_robotic_arm.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[robot_description]
    )

    return LaunchDescription([
        joint_state_broadcaster,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_controller]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller,
                on_exit=[gripper_controller]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gripper_controller,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[rviz_node]
                    )
                ]
            )
        )
    ])
