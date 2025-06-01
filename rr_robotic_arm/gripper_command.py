import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys

class GripperCommandNode(Node):
    def __init__(self, target_positions):
        super().__init__('gripper_command_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        msg = Float64MultiArray()
        msg.data = target_positions

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent gripper command: {target_positions}")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1 and sys.argv[1] == 'close':
        positions = [0.03, -0.03]
    else:
        positions = [0.0, 0.0]

    node = GripperCommandNode(positions)

    # Spin once with a small timeout to allow publishing
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
