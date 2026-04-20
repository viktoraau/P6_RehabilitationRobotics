import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class ComputedTorqueController(Node):

    def __init__(self):
        super().__init__('computed_torque_controller')

        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter(
            'desired_trajectory_topic',
            '/joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3'])

        self.joint_state_topic = (
            self.get_parameter('joint_state_topic').get_parameter_value().string_value
        )
        self.desired_trajectory_topic = (
            self.get_parameter('desired_trajectory_topic').get_parameter_value().string_value
        )
        self.joint_names = list(
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self.dof = len(self.joint_names)

        # --- States ---
        self.q = np.zeros(self.dof)
        self.q_dot = np.zeros(self.dof)
        self.q_effort = None

        self.q_des = np.zeros(self.dof)
        self.q_dot_des = np.zeros(self.dof)
        self.q_ddot_des = np.zeros(self.dof)

        self.have_actual_state = False
        self.have_desired_state = False
        self.ready_logged = False

        # --- Subscribers ---
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )

        self.create_subscription(
            JointTrajectory,
            self.desired_trajectory_topic,
            self.desired_trajectory_callback,
            10
        )

        self.get_logger().info(
            "Computed torque listener started. "
            f"actual='{self.joint_state_topic}', "
            f"desired='{self.desired_trajectory_topic}', "
            f"joints={self.joint_names}"
        )

    # --- Callbacks ---

    def joint_state_callback(self, msg):
        q = self._ordered_values(msg.name, msg.position, 'JointState.position')
        q_dot = self._ordered_values(msg.name, msg.velocity, 'JointState.velocity')

        if q is None or q_dot is None:
            return

        self.q = q
        self.q_dot = q_dot
        self.q_effort = self._ordered_values(
            msg.name,
            msg.effort,
            'JointState.effort',
            required=False
        )
        self.have_actual_state = True
        self._log_ready_once()

    def desired_trajectory_callback(self, msg):
        if not msg.points:
            self.get_logger().warn("Desired trajectory message has no points")
            return

        point = msg.points[0]

        q_des = self._ordered_values(
            msg.joint_names,
            point.positions,
            'JointTrajectoryPoint.positions'
        )
        q_dot_des = self._ordered_values(
            msg.joint_names,
            point.velocities,
            'JointTrajectoryPoint.velocities'
        )
        q_ddot_des = self._ordered_values(
            msg.joint_names,
            point.accelerations,
            'JointTrajectoryPoint.accelerations'
        )

        if q_des is None or q_dot_des is None or q_ddot_des is None:
            return

        self.q_des = q_des
        self.q_dot_des = q_dot_des
        self.q_ddot_des = q_ddot_des
        self.have_desired_state = True
        self._log_ready_once()

    # --- Helpers ---

    def _ordered_values(self, names, values, field_name, required=True):
        if not names:
            if required:
                self.get_logger().warn(f"{field_name} cannot be mapped: joint names are empty")
            return None

        if not values:
            if required:
                self.get_logger().warn(f"{field_name} is empty")
            return None

        name_to_index = {name: index for index, name in enumerate(names)}
        missing_joints = [name for name in self.joint_names if name not in name_to_index]
        if missing_joints:
            if required:
                self.get_logger().warn(
                    f"{field_name} missing expected joints: {missing_joints}"
                )
            return None

        ordered_values = []
        for name in self.joint_names:
            index = name_to_index[name]
            if index >= len(values):
                if required:
                    self.get_logger().warn(
                        f"{field_name} has no value for joint '{name}' at index {index}"
                    )
                return None
            ordered_values.append(values[index])

        return np.array(ordered_values, dtype=float)

    def _log_ready_once(self):
        if self.ready_logged:
            return

        if not self.have_actual_state or not self.have_desired_state:
            return

        self.ready_logged = True
        self.get_logger().info(
            "Controller listener ready: received actual joint state and desired IK trajectory."
        )


def main(args=None):
    rclpy.init(args=args)
    node = ComputedTorqueController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
