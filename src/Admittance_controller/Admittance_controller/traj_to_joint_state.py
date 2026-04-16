#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState


class TrajToJointState(Node):
    def __init__(self):
        super().__init__('traj_to_joint_state')

        self.sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

    def callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        point = msg.points[-1]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = msg.joint_names
        js.position = list(point.positions)
        js.velocity = list(point.velocities)
        js.effort = []

        self.pub.publish(js)


def main():
    rclpy.init()
    node = TrajToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
