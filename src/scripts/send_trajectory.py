#!/usr/bin/env python3
"""Send a CSV excitation trajectory via a follow_joint_trajectory action server.

Steps:
  1. Read current joint positions.
  2. Home to (0, 0, 0) via action and wait for completion.
  3. Send the full excitation trajectory and wait for completion.
"""

import argparse
import csv
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ["joint_1", "joint_2", "joint_3"]
DEFAULT_CONTROLLER = "joint_trajectory_controller"
HOME_DURATION_S = 10.0


def make_duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    return Duration(sec=sec, nanosec=nanosec)


def load_csv(path: str, time_offset: float = 0.3):
    points = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = None
        for row in reader:
            if fieldnames is None:
                fieldnames = set(row.keys())
            t = float(row["t"]) + time_offset
            has_velocities = {"dq1", "dq2", "dq3"}.issubset(fieldnames)
            points.append(
                JointTrajectoryPoint(
                    positions=[float(row["q1"]), float(row["q2"]), float(row["q3"])],
                    velocities=(
                        [float(row["dq1"]), float(row["dq2"]), float(row["dq3"])]
                        if has_velocities else []
                    ),
                    time_from_start=make_duration(t),
                )
            )
    return points


class TrajectorySender(Node):
    def __init__(self, controller_name: str):
        super().__init__("trajectory_sender")
        self._controller_name = controller_name.strip().strip("/")
        action_name = f"/{self._controller_name}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        self._current_pos = None
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)

    def _joint_cb(self, msg: JointState):
        if self._current_pos is None:
            self._current_pos = list(msg.position)

    def get_current_positions(self, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline and self._current_pos is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._current_pos

    def send_trajectory(self, points, label="trajectory"):
        self.get_logger().info(
            f"Waiting for action server /{self._controller_name}/follow_joint_trajectory..."
        )
        self._client.wait_for_server()

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points = points

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # Set header stamp so time_from_start offsets are relative to now
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        # Allow extra time for the controller to settle within goal tolerance
        goal.goal_time_tolerance = make_duration(5.0)

        self.get_logger().info(f"Sending {label} ({len(points)} points)...")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{label} REJECTED by controller!")
            return False

        self.get_logger().info(f"{label} accepted — executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"{label} finished. error_code={result.error_code}")
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="/home/rehabo/Downloads/opt_trajectory.csv",
    )
    parser.add_argument(
        "--controller",
        default=DEFAULT_CONTROLLER,
        help=(
            "Trajectory controller name, for example "
            "'joint_trajectory_controller' or 'joint_trajectory_velocity_controller'."
        ),
    )
    args = parser.parse_args()

    csv_path = args.csv_path
    rclpy.init()
    node = TrajectorySender(args.controller)

    # Step 1: read current position
    node.get_logger().info("Reading current joint positions...")
    current = node.get_current_positions()
    if current is None:
        node.get_logger().error("Could not read /joint_states!")
        rclpy.shutdown()
        sys.exit(1)
    node.get_logger().info(f"Current: {[round(p, 4) for p in current]}")

    # Step 2: home to (0,0,0)
    home_points = [
        JointTrajectoryPoint(
            positions=[0.0, 0.0, 0.0],
            velocities=[0.0, 0.0, 0.0],
            time_from_start=make_duration(HOME_DURATION_S),
        )
    ]
    ok = node.send_trajectory(home_points, label="homing")
    if not ok:
        node.get_logger().error("Homing failed — aborting.")
        rclpy.shutdown()
        sys.exit(1)

    # Step 3: send excitation trajectory
    node.get_logger().info(f"Loading {csv_path}...")
    excit_points = load_csv(csv_path)
    node.get_logger().info(f"Loaded {len(excit_points)} points")
    node.send_trajectory(excit_points, label="excitation trajectory")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
