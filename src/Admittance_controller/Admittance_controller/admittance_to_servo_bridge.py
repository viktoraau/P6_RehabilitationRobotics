"""Bridge: admittance angular velocity/orientation/acceleration → MoveIt Servo + CTC trajectory.

Improvements over the original fixed-map version:
  - Uses the configuration-dependent angular Jacobian J(q) instead of a
    constant approximation evaluated at q=[0,0,0].  Subscribes to /joint_states
    to track the current arm configuration.
  - Subscribes to /desired_angular_acceleration and computes joint accelerations
    analytically: q̈ = J(q)⁻¹ · (α − J̇(q,q̇)·q̇).
  - Subscribes to /desired_orientation and computes q_des via closed-form IK so
    the CTC gets a proper position setpoint (not just velocity).
  - Publishes a JointTrajectory with positions + velocities + accelerations to a
    dedicated topic (trajectory_output_topic) for the CTC feed-forward.
  - Continues to publish JointJog / TwistStamped to MoveIt Servo for
    visualisation and singularity monitoring.
"""

from typing import Optional, Sequence
import math

import numpy as np
import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import QuaternionStamped, TwistStamped, Vector3Stamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ── Kinematics helpers (matches orientation_ik_3r_node.cpp) ──────────────────

def _angular_jacobian(q: np.ndarray) -> np.ndarray:
    """Angular velocity Jacobian: ω = J(q) · q̇.

    Columns are joint axes in base_link frame for
    R = Ry(−q1) · Rz(−q2) · Rx(q3).
    """
    s1, c1 = math.sin(q[0]), math.cos(q[0])
    s2, c2 = math.sin(q[1]), math.cos(q[1])
    return np.array([
        [0.0,  s1,         c1 * c2],
        [-1.0, 0.0,       -s2     ],
        [0.0, -c1,         s1 * c2],
    ])


def _angular_jacobian_dot(q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
    """Time derivative of angular Jacobian: J̇(q, q̇)."""
    s1, c1 = math.sin(q[0]), math.cos(q[0])
    s2, c2 = math.sin(q[1]), math.cos(q[1])
    q1d, q2d = qdot[0], qdot[1]
    Jd = np.zeros((3, 3))
    Jd[:, 1] = [c1 * q1d, 0.0, s1 * q1d]
    Jd[:, 2] = [
        -s1 * c2 * q1d - c1 * s2 * q2d,
        -c2 * q2d,
         c1 * c2 * q1d - s1 * s2 * q2d,
    ]
    return Jd


def _damped_solve(J: np.ndarray, rhs: np.ndarray, lam: float) -> np.ndarray:
    """Damped least-squares: x = Jᵀ (J Jᵀ + λ²I)⁻¹ · rhs."""
    return J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), rhs)


def _orientation_ik(R: np.ndarray) -> np.ndarray:
    """Closed-form IK for R = Ry(−q1)·Rz(−q2)·Rx(q3)."""
    q2 = -math.asin(float(np.clip(R[1, 0], -1.0, 1.0)))
    q1 = math.atan2(float(R[2, 0]), float(R[0, 0]))
    q3 = math.atan2(float(-R[1, 2]), float(R[1, 1]))
    return np.array([q1, q2, q3])


def _choose_nearest(q_raw: np.ndarray, q_last: Optional[np.ndarray]) -> np.ndarray:
    """Pick the 2π-equivalent joint vector closest to q_last."""
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    if q_last is None:
        return np.array([_wrap(a) for a in q_raw])

    best = q_raw.copy()
    best_cost = float(np.sum((best - q_last) ** 2))
    for k1 in (-1, 0, 1):
        for k2 in (-1, 0, 1):
            for k3 in (-1, 0, 1):
                cand = q_raw + 2.0 * math.pi * np.array([float(k1), float(k2), float(k3)])
                cost = float(np.sum((cand - q_last) ** 2))
                if cost < best_cost:
                    best = cand
                    best_cost = cost
    return best


def _quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# ── Node ─────────────────────────────────────────────────────────────────────

class AdmittanceToServoBridge(Node):
    """Bridge admittance outputs into MoveIt Servo commands and a CTC trajectory.

    Supports either:
      - TWIST mode (publishes TwistStamped to /servo_node/delta_twist_cmds)
      - JOINT_JOG mode (publishes JointJog to /servo_node/delta_joint_cmds)

    In both modes also publishes a full JointTrajectory (positions + velocities
    + accelerations) to trajectory_output_topic for the computed-torque controller.
    """

    def __init__(self) -> None:
        super().__init__("admittance_to_servo_bridge")

        self.declare_parameter("input_angular_velocity_topic", "/desired_angular_velocity")
        self.declare_parameter("input_orientation_topic", "/desired_orientation")
        self.declare_parameter("input_angular_acceleration_topic", "/desired_angular_acceleration")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_mode", "twist")
        self.declare_parameter("output_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("output_joint_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter(
            "trajectory_output_topic", "/bridge_desired_trajectory"
        )
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("angular_scale", [1.0, 1.0, 1.0])
        self.declare_parameter("angular_deadband", [0.0, 0.0, 0.0])
        self.declare_parameter("max_angular_speed", [5.0, 5.0, 5.0])
        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3"])
        self.declare_parameter("damping_lambda", 0.01)
        self.declare_parameter("max_direct_input_age_s", 0.1)
        self.declare_parameter("auto_switch_command_type", True)
        self.declare_parameter("switch_service_name", "/servo_node/switch_command_type")
        self.declare_parameter("switch_retry_period_s", 1.0)
        self.declare_parameter("switch_max_attempts", 30)

        self.input_omega_topic = str(self.get_parameter("input_angular_velocity_topic").value)
        self.input_orient_topic = str(self.get_parameter("input_orientation_topic").value)
        self.input_alpha_topic = str(self.get_parameter("input_angular_acceleration_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.output_mode = str(self.get_parameter("output_mode").value).strip().lower()
        if self.output_mode not in {"twist", "joint_jog"}:
            raise ValueError('Parameter "output_mode" must be either "twist" or "joint_jog"')
        self.output_twist_topic = str(self.get_parameter("output_twist_topic").value)
        self.output_joint_topic = str(self.get_parameter("output_joint_topic").value)
        self.traj_out_topic = str(self.get_parameter("trajectory_output_topic").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.auto_switch = bool(self.get_parameter("auto_switch_command_type").value)
        self.switch_service_name = str(self.get_parameter("switch_service_name").value)
        self.switch_retry_period_s = float(self.get_parameter("switch_retry_period_s").value)
        self.switch_max_attempts = int(self.get_parameter("switch_max_attempts").value)

        self.angular_scale = self._read_vec("angular_scale")
        self.angular_deadband = self._read_vec("angular_deadband")
        self.max_angular_speed = self._read_vec("max_angular_speed", positive=True)
        self.joint_names = [str(n) for n in self.get_parameter("joint_names").value]
        if len(self.joint_names) != 3:
            raise ValueError('Parameter "joint_names" must contain exactly 3 joint names')
        self.damping_lambda = float(self.get_parameter("damping_lambda").value)
        self.max_input_age = float(self.get_parameter("max_direct_input_age_s").value)

        # ── State ────────────────────────────────────────────────────────── #
        self._latest_omega: Optional[np.ndarray] = None
        self._latest_omega_sec: float = -1.0
        self._latest_alpha: Optional[np.ndarray] = None
        self._latest_alpha_sec: float = -1.0
        self._current_q: Optional[np.ndarray] = None   # from /joint_states
        self._last_q_des: Optional[np.ndarray] = None  # for branch continuity
        self._last_processed_sec: float = -1.0

        # ── Publishers ───────────────────────────────────────────────────── #
        self.twist_pub: Optional[rclpy.publisher.Publisher] = None
        self.joint_pub: Optional[rclpy.publisher.Publisher] = None
        if self.output_mode == "twist":
            self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 10)
        else:
            self.joint_pub = self.create_publisher(JointJog, self.output_joint_topic, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, self.traj_out_topic, 10)

        # ── Subscriptions ────────────────────────────────────────────────── #
        self.create_subscription(Vector3Stamped, self.input_omega_topic, self._omega_cb, 10)
        self.create_subscription(Vector3Stamped, self.input_alpha_topic, self._alpha_cb, 10)
        self.create_subscription(
            QuaternionStamped, self.input_orient_topic, self._orientation_cb, 10
        )
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)

        # ── Servo command-type auto-switch ───────────────────────────────── #
        self._switch_attempts = 0
        self._switch_future: Optional[rclpy.task.Future] = None
        self._switch_client = self.create_client(ServoCommandType, self.switch_service_name)
        self._switch_timer = None
        if self.auto_switch:
            self._switch_timer = self.create_timer(
                self.switch_retry_period_s, self._try_switch_command_type
            )

        servo_out = self.output_twist_topic if self.output_mode == "twist" else self.output_joint_topic
        self.get_logger().info(
            f"Admittance→Servo bridge started (dynamic Jacobian). "
            f"mode={self.output_mode}, servo_out={servo_out}, "
            f"traj_out={self.traj_out_topic}"
        )
        if self.auto_switch:
            mode_name = "TWIST" if self.output_mode == "twist" else "JOINT_JOG"
            self.get_logger().info(
                f"Will auto-switch Servo to {mode_name} via {self.switch_service_name}"
            )

    # ── Callbacks ─────────────────────────────────────────────────────────── #

    def _omega_cb(self, msg: Vector3Stamped) -> None:
        self._latest_omega = np.array(
            [msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64
        )
        self._latest_omega_sec = _stamp_to_sec(msg.header.stamp)

    def _alpha_cb(self, msg: Vector3Stamped) -> None:
        self._latest_alpha = np.array(
            [msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64
        )
        self._latest_alpha_sec = _stamp_to_sec(msg.header.stamp)

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in name_to_idx for n in self.joint_names):
            return
        if len(msg.position) == 0:
            return
        try:
            self._current_q = np.array(
                [msg.position[name_to_idx[n]] for n in self.joint_names],
                dtype=np.float64,
            )
        except IndexError:
            pass

    def _orientation_cb(self, msg: QuaternionStamped) -> None:
        if self._latest_omega is None or self._latest_alpha is None:
            return

        stamp_sec = _stamp_to_sec(msg.header.stamp)

        if stamp_sec <= self._last_processed_sec:
            return

        age_omega = abs(stamp_sec - self._latest_omega_sec)
        age_alpha = abs(stamp_sec - self._latest_alpha_sec)
        if age_omega > self.max_input_age or age_alpha > self.max_input_age:
            self.get_logger().warn(
                f"Inputs not time-aligned: age_omega={age_omega:.3f}s "
                f"age_alpha={age_alpha:.3f}s",
                throttle_duration_sec=1.0,
            )
            return

        q_msg = msg.quaternion
        R = _quat_to_rotation(q_msg.x, q_msg.y, q_msg.z, q_msg.w)

        omega = self._latest_omega * self.angular_scale
        omega = np.where(np.abs(omega) < self.angular_deadband, 0.0, omega)
        omega = np.clip(omega, -self.max_angular_speed, self.max_angular_speed)
        alpha = self._latest_alpha.copy()

        # Position IK from desired orientation.
        # Branch disambiguation uses the ACTUAL arm position so the IK always
        # picks the solution closest to where the arm is, not where the desired
        # trajectory has been accumulating — the two can diverge under load.
        q_raw = _orientation_ik(R)
        q_ref = self._current_q if self._current_q is not None else self._last_q_des
        q_des = _choose_nearest(q_raw, q_ref)

        # Use actual robot configuration for Jacobian if available,
        # else fall back to the IK solution (correct at startup)
        q_jac = self._current_q if self._current_q is not None else q_des

        J = _angular_jacobian(q_jac)
        qdot = _damped_solve(J, omega, self.damping_lambda)
        qdot = np.clip(qdot, -self.max_angular_speed, self.max_angular_speed)

        Jdot = _angular_jacobian_dot(q_jac, qdot)
        qddot = _damped_solve(J, alpha - Jdot @ qdot, self.damping_lambda)

        self._publish_servo(omega, qdot, msg.header.stamp)
        self._publish_trajectory(q_des, qdot, qddot, msg.header.stamp)

        self._last_q_des = q_des
        self._last_processed_sec = stamp_sec

    # ── Publishers ────────────────────────────────────────────────────────── #

    def _publish_servo(self, omega: np.ndarray, qdot: np.ndarray, stamp_msg) -> None:
        if self.output_mode == "twist":
            out = TwistStamped()
            out.header.stamp = stamp_msg
            out.header.frame_id = self.command_frame
            out.twist.angular.x = float(omega[0])
            out.twist.angular.y = float(omega[1])
            out.twist.angular.z = float(omega[2])
            self.twist_pub.publish(out)
        else:
            jog = JointJog()
            jog.header.stamp = stamp_msg
            jog.header.frame_id = self.command_frame
            jog.joint_names = self.joint_names
            jog.velocities = [float(qdot[0]), float(qdot[1]), float(qdot[2])]
            self.joint_pub.publish(jog)

    def _publish_trajectory(
        self,
        q: np.ndarray,
        qdot: np.ndarray,
        qddot: np.ndarray,
        stamp_msg,
    ) -> None:
        traj = JointTrajectory()
        traj.header.stamp = stamp_msg
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.velocities = qdot.tolist()
        pt.accelerations = qddot.tolist()
        pt.time_from_start.nanosec = 10_000_000  # 10 ms
        traj.points.append(pt)
        self.traj_pub.publish(traj)

    # ── Helpers ───────────────────────────────────────────────────────────── #

    def _read_vec(self, name: str, positive: bool = False) -> np.ndarray:
        raw = self.get_parameter(name).value
        if not isinstance(raw, Sequence) or len(raw) != 3:
            raise ValueError(f'Parameter "{name}" must contain exactly 3 values')
        vec = np.asarray(raw, dtype=np.float64)
        if positive and np.any(vec <= 0.0):
            raise ValueError(f'Parameter "{name}" must contain strictly positive values')
        return vec

    # ── Servo command-type switcher ───────────────────────────────────────── #

    def _try_switch_command_type(self) -> None:
        if self._switch_future is not None and not self._switch_future.done():
            return
        if self.switch_max_attempts > 0 and self._switch_attempts >= self.switch_max_attempts:
            if self._switch_timer is not None:
                self._switch_timer.cancel()
            mode_name = "TWIST" if self.output_mode == "twist" else "JOINT_JOG"
            self.get_logger().warn(
                f"Failed to switch Servo to {mode_name} after {self._switch_attempts} attempts."
            )
            return
        if not self._switch_client.service_is_ready():
            self._switch_attempts += 1
            if self._switch_attempts == 1 or self._switch_attempts % 5 == 0:
                self.get_logger().info(
                    f"Waiting for {self.switch_service_name} "
                    f"(attempt {self._switch_attempts})"
                )
            return
        req = ServoCommandType.Request()
        req.command_type = 1 if self.output_mode == "twist" else 0
        self._switch_attempts += 1
        self._switch_future = self._switch_client.call_async(req)
        self._switch_future.add_done_callback(self._on_switch_response)

    def _on_switch_response(self, future: rclpy.task.Future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"switch_command_type call failed: {exc}")
            return
        mode_name = "TWIST" if self.output_mode == "twist" else "JOINT_JOG"
        if getattr(response, "success", True):
            if self._switch_timer is not None:
                self._switch_timer.cancel()
            self.get_logger().info(f"Servo command type switched to {mode_name}")
        else:
            message = getattr(response, "message", "no details")
            self.get_logger().warn(f"Servo rejected {mode_name} switch: {message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdmittanceToServoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
