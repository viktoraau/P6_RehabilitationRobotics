import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, WrenchStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class OrientationAdmittanceNode(Node):
    """Rotational admittance controller.

    Input:  WrenchStamped (force and torque are both used)
    Output: desired orientation, angular velocity, angular acceleration in base frame

    The wrench is transformed into the base frame, including the force-induced moment from the
    transform translation between the wrench frame and the control frame.

    Dynamics are modeled on a small-angle orientation state theta about a reference orientation:
        I * theta_ddot + D * theta_dot + K * theta = tau

    Desired absolute orientation is then:
        R_des = R_ref * Exp(theta)
    """

    def __init__(self) -> None:
        super().__init__('orientation_admittance_controller')

        for name, default in (
            ('input_topic', '/wrench_input'),
            ('base_frame', 'base_link'),
            ('default_wrench_frame', 'tool0'),
            ('control_rate_hz', 100.0),
            ('wrench_timeout_s', 0.1),
            ('max_dt_s', 0.05),
            ('min_dt_s', 1.0e-4),
            ('inertia', [0.03, 0.03, 0.03]),
            ('damping', [0.08, 0.08, 0.08]),
            ('stiffness', [0.250, 0.250, 0.250]),
            ('torque_deadband_nm', [0.01, 0.01, 0.01]),
            ('torque_lowpass_cutoff_hz', 20.0),
            ('force_to_torque_gain_nm_per_n', [0.01, 0.01, 0.01]),
            ('max_angular_velocity', [1.0, 1.0, 1.0]),
            ('max_angular_acceleration', [8.0, 8.0, 8.0]),
            ('max_orientation_error_rad', [0.5, 1.2, 1.0]),
            ('reference_orientation_xyzw', [0.0, 0.0, 0.0, 1.0]),
            ('initialize_reference_from_tf', False),
            ('reference_tip_frame', 'Body2__3__1'),
            ('orientation_topic', '/desired_orientation'),
            ('angular_velocity_topic', '/desired_angular_velocity'),
            ('angular_acceleration_topic', '/desired_angular_acceleration'),
        ):
            self.declare_parameter(name, default)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.default_wrench_frame = str(self.get_parameter('default_wrench_frame').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.wrench_timeout_s = float(self.get_parameter('wrench_timeout_s').value)
        self.max_dt_s = float(self.get_parameter('max_dt_s').value)
        self.min_dt_s = float(self.get_parameter('min_dt_s').value)

        self.inertia = self._read_vec_param('inertia', positive=True)
        self.damping = self._read_vec_param('damping')
        self.stiffness = self._read_vec_param('stiffness')
        self.torque_deadband_nm = self._read_vec_param('torque_deadband_nm')
        self.max_angular_velocity = self._read_vec_param('max_angular_velocity', positive=True)
        self.max_angular_acceleration = self._read_vec_param('max_angular_acceleration', positive=True)
        self.max_orientation_error_rad = self._read_vec_param('max_orientation_error_rad', positive=True)
        self.torque_lowpass_cutoff_hz = float(self.get_parameter('torque_lowpass_cutoff_hz').value)
        self.force_to_torque_gain_nm_per_n = self._read_vec_param('force_to_torque_gain_nm_per_n')

        q_xyzw = self.get_parameter('reference_orientation_xyzw').value
        if len(q_xyzw) != 4:
            raise ValueError('Parameter "reference_orientation_xyzw" must contain exactly 4 values [x,y,z,w]')
        self.reference_rotation = self._quat_xyzw_to_matrix(np.asarray(q_xyzw, dtype=np.float64))

        self.initialize_reference_from_tf = bool(self.get_parameter('initialize_reference_from_tf').value)
        self.reference_tip_frame = str(self.get_parameter('reference_tip_frame').value)

        self.orientation_topic = str(self.get_parameter('orientation_topic').value)
        self.angular_velocity_topic = str(self.get_parameter('angular_velocity_topic').value)
        self.angular_acceleration_topic = str(self.get_parameter('angular_acceleration_topic').value)

        self.theta = np.zeros(3, dtype=np.float64)
        self.omega = np.zeros(3, dtype=np.float64)
        self.alpha = np.zeros(3, dtype=np.float64)
        self.filtered_torque_base = np.zeros(3, dtype=np.float64)

        self.latest_force_sensor = np.zeros(3, dtype=np.float64)
        self.latest_torque_sensor = np.zeros(3, dtype=np.float64)
        self.latest_wrench_frame = self.default_wrench_frame
        self.last_wrench_rx_time: Optional[rclpy.time.Time] = None
        self.last_control_time: Optional[rclpy.time.Time] = None
        self.reference_initialized = not self.initialize_reference_from_tf
        self._last_warn_time = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            self.input_topic,
            self._wrench_callback,
            10,
        )
        self.orientation_pub = self.create_publisher(QuaternionStamped, self.orientation_topic, 10)
        self.omega_pub = self.create_publisher(Vector3Stamped, self.angular_velocity_topic, 10)
        self.alpha_pub = self.create_publisher(Vector3Stamped, self.angular_acceleration_topic, 10)

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.control_timer = self.create_timer(period, self._control_tick)

        self.get_logger().info(
            'Orientation admittance controller started. '
            f'Input={self.input_topic}, base_frame={self.base_frame}, '
            f'orientation_topic={self.orientation_topic}'
        )

    def _read_vec_param(self, name: str, positive: bool = False) -> np.ndarray:
        raw = self.get_parameter(name).value
        if len(raw) != 3:
            raise ValueError(f'Parameter "{name}" must contain exactly 3 values')
        vec = np.asarray(raw, dtype=np.float64)
        if positive and np.any(vec <= 0.0):
            raise ValueError(f'Parameter "{name}" must contain strictly positive values')
        return vec

    def _wrench_callback(self, msg: WrenchStamped) -> None:
        self.latest_force_sensor = np.asarray(
            (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z),
            dtype=np.float64,
        )
        self.latest_torque_sensor = np.asarray(
            (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z),
            dtype=np.float64,
        )
        self.latest_wrench_frame = msg.header.frame_id or self.default_wrench_frame
        self.last_wrench_rx_time = self.get_clock().now()

    def _control_tick(self) -> None:
        now = self.get_clock().now()

        if self.last_control_time is None:
            self.last_control_time = now
            return

        if not self.reference_initialized:
            self._try_initialize_reference_from_tf()
            self.last_control_time = now
            return

        dt = (now - self.last_control_time).nanoseconds * 1.0e-9
        self.last_control_time = now
        if not math.isfinite(dt):
            return
        dt = float(np.clip(dt, self.min_dt_s, self.max_dt_s))

        torque_base = self._get_torque_in_base(now)
        torque_base = self._apply_deadband(torque_base)
        self.filtered_torque_base = self._lowpass_filter(self.filtered_torque_base, torque_base, dt)

        self.alpha = np.clip(
            (
                self.filtered_torque_base
                - self.damping * self.omega
                - self.stiffness * self.theta
            ) / self.inertia,
            -self.max_angular_acceleration,
            self.max_angular_acceleration,
        )

        self.omega = np.clip(
            self.omega + dt * self.alpha,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )
        self.theta = np.clip(
            self.theta + dt * self.omega,
            -self.max_orientation_error_rad,
            self.max_orientation_error_rad,
        )

        desired_rotation = self.reference_rotation @ self._exp_map(self.theta)
        stamp_msg = now.to_msg()

        self._publish_orientation(desired_rotation, stamp_msg)
        self._publish_vector(self.omega_pub, self.omega, stamp_msg)
        self._publish_vector(self.alpha_pub, self.alpha, stamp_msg)

    def _try_initialize_reference_from_tf(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.reference_tip_frame,
                rclpy.time.Time(),
            )
            rot = tf.transform.rotation
            self.reference_rotation = self._quat_xyzw_to_matrix(
                np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float64)
            )
            self.reference_initialized = True
            self.get_logger().info(
                f"Initialized reference orientation from TF {self.base_frame} -> {self.reference_tip_frame}"
            )
        except TransformException as exc:
            self._warn_throttled('reference_tf', f'Waiting for reference TF: {str(exc)}')

    def _get_torque_in_base(self, now) -> np.ndarray:
        if self.last_wrench_rx_time is None:
            return self._decay_torque()

        age_s = (now - self.last_wrench_rx_time).nanoseconds * 1.0e-9
        if age_s > self.wrench_timeout_s:
            self._warn_throttled(
                'wrench_timeout',
                f'No wrench received for {age_s:.3f}s (timeout={self.wrench_timeout_s:.3f}s).',
            )
            return self._decay_torque()

        if self.latest_wrench_frame == self.base_frame:
            return self.latest_torque_sensor.copy() + self.force_to_torque_gain_nm_per_n * self.latest_force_sensor

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.latest_wrench_frame,
                rclpy.time.Time(),
            )
            rot = tf.transform.rotation
            translation = tf.transform.translation
            rotation_matrix = self._quat_xyzw_to_matrix(
                np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float64)
            )
            force_base = rotation_matrix @ self.latest_force_sensor
            torque_base = rotation_matrix @ self.latest_torque_sensor
            offset_base = np.array((translation.x, translation.y, translation.z), dtype=np.float64)
            return torque_base + np.cross(offset_base, force_base) + self.force_to_torque_gain_nm_per_n * force_base
        except TransformException as exc:
            self._warn_throttled('tf_lookup', f'TF lookup failed: {str(exc)}')
            return self._decay_torque()

    def _decay_torque(self) -> np.ndarray:
        return 0.9 * self.filtered_torque_base

    def _apply_deadband(self, torque: np.ndarray) -> np.ndarray:
        return np.where(np.abs(torque) < self.torque_deadband_nm, 0.0, torque).astype(np.float64)

    def _lowpass_filter(self, prev: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
        if self.torque_lowpass_cutoff_hz <= 0.0:
            return current
        tau = 1.0 / (2.0 * math.pi * self.torque_lowpass_cutoff_hz)
        alpha = dt / (tau + dt)
        return alpha * current + (1.0 - alpha) * prev

    def _publish_orientation(self, rotation: np.ndarray, stamp_msg) -> None:
        q = self._matrix_to_quat_xyzw(rotation)
        msg = QuaternionStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.base_frame
        msg.quaternion.x = float(q[0])
        msg.quaternion.y = float(q[1])
        msg.quaternion.z = float(q[2])
        msg.quaternion.w = float(q[3])
        self.orientation_pub.publish(msg)

    def _publish_vector(self, publisher, vec: np.ndarray, stamp_msg) -> None:
        msg = Vector3Stamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.base_frame
        msg.vector.x = float(vec[0])
        msg.vector.y = float(vec[1])
        msg.vector.z = float(vec[2])
        publisher.publish(msg)

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        return np.array(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0],
            ],
            dtype=np.float64,
        )

    @classmethod
    def _exp_map(cls, rotvec: np.ndarray) -> np.ndarray:
        theta = float(np.linalg.norm(rotvec))
        if theta < 1.0e-12:
            K = cls._skew(rotvec)
            return np.eye(3, dtype=np.float64) + K

        axis = rotvec / theta
        K = cls._skew(axis)
        return np.eye(3, dtype=np.float64) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)

    @staticmethod
    def _quat_xyzw_to_matrix(q_xyzw: np.ndarray) -> np.ndarray:
        x, y, z, w = q_xyzw.astype(np.float64)
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n <= 1.0e-12:
            raise ValueError('Quaternion norm is zero')
        x /= n
        y /= n
        z /= n
        w /= n

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _matrix_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
        trace = float(np.trace(R))
        if trace > 0.0:
            s = 2.0 * math.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        q = np.array([x, y, z, w], dtype=np.float64)
        q /= np.linalg.norm(q)
        return q

    def _warn_throttled(self, key: str, message: str, period_s: float = 1.0) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        previous = self._last_warn_time.get(key)
        if previous is None or (now_s - previous) > period_s:
            self.get_logger().warn(message)
            self._last_warn_time[key] = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrientationAdmittanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
