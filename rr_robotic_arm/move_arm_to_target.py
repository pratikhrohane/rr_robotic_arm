import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import sys

class MoveArmToTarget(Node):
    def __init__(self, joint1_deg, joint3_deg):
        super().__init__('move_arm_to_target')

        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_names = ['joint1', 'joint3']

        self.target_positions = [
            math.radians(joint1_deg),
            math.radians(joint3_deg)
        ]

        self.get_logger().info(
            f"Sending target angles (deg): joint1={joint1_deg}, joint3={joint3_deg}"
        )

        self.send_trajectory()

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start.sec = 3  # 3 seconds to complete motion

        traj.points = [point]

        self.publisher.publish(traj)
        self.get_logger().info("Trajectory published. Exiting...")

def main(args=None):
    rclpy.init(args=args)

    # Expecting 2 arguments: joint1 and joint3 in degrees
    if len(sys.argv) != 3:
        print("Usage: ros2 run rr_robotic_arm move_arm_to_target <joint1_deg> <joint3_deg>")
        sys.exit(1)

    try:
        j1 = float(sys.argv[1])
        j3 = float(sys.argv[2])
    except ValueError:
        print("Error: Joint angles must be numbers.")
        sys.exit(1)

    node = MoveArmToTarget(j1, j3)

    # Give time for message to be sent before exiting
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
