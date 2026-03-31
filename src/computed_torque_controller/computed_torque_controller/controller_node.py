import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ComputedTorqueController(Node):

    def __init__(self):
        super().__init__('computed_torque_controller')

        # --- States ---
        self.theta = np.zeros(3)
        self.theta_dot = np.zeros(3)

        self.theta_d = np.zeros(3)
        self.theta_dot_d = np.zeros(3)
        self.theta_ddot_d = np.zeros(3)

        # --- Subscribers ---
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/desired_joint_states',
            self.desired_callback,
            10
        )

        self.get_logger().info("Controller node started and listening...")

    # --- Callbacks ---

    def joint_state_callback(self, msg):
        # Safety: ensure at least 3 joints
        if len(msg.position) < 3 or len(msg.velocity) < 3:
            self.get_logger().warn("JointState message too small")
            return

        self.theta = np.array(msg.position[:3])
        self.theta_dot = np.array(msg.velocity[:3])

        self.get_logger().info(f"θ: {self.theta}, θ_dot: {self.theta_dot}")

    def desired_callback(self, msg):
        if len(msg.data) < 9:
            self.get_logger().warn("Desired state message too small")
            return

        data = msg.data

        self.theta_d = np.array(data[0:3])
        self.theta_dot_d = np.array(data[3:6])
        self.theta_ddot_d = np.array(data[6:9])

        self.get_logger().info(f"θ_d: {self.theta_d}")


def main(args=None):
    rclpy.init(args=args)
    node = ComputedTorqueController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
