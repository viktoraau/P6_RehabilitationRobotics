import math

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


class AdmittanceControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('admittance_controller')

        self.declare_parameter('input_topic', '/wrench_input')
        self.declare_parameter('output_topic', '/admittance_controller/cartesian_state')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('default_wrench_frame', 'tool0')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('wrench_timeout_s', 0.1)
        self.declare_parameter('max_dt_s', 0.05)
        self.declare_parameter('min_dt_s', 1.0e-4)

        self.declare_parameter('mass', [1.5, 1.5, 1.5])
        self.declare_parameter('damping', [45.0, 45.0, 45.0])
        self.declare_parameter('stiffness', [120.0, 120.0, 120.0])
        self.declare_parameter('reference_position', [0.0, 0.0, 0.0])
        self.declare_parameter('deadband_n', [0.35, 0.35, 0.35])
        self.declare_parameter('force_lowpass_cutoff_hz', 20.0)
        self.declare_parameter('max_velocity', [0.30, 0.30, 0.30])
        self.declare_parameter('max_acceleration', [3.0, 3.0, 3.0])

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.default_wrench_frame = str(self.get_parameter('default_wrench_frame').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.wrench_timeout_s = float(self.get_parameter('wrench_timeout_s').value)
        self.max_dt_s = float(self.get_parameter('max_dt_s').value)
        self.min_dt_s = float(self.get_parameter('min_dt_s').value)

        self.mass = self._read_vec_param('mass')
        self.damping = self._read_vec_param('damping')
        self.stiffness = self._read_vec_param('stiffness')
        self.reference_position = self._read_vec_param('reference_position')
        self.deadband_n = self._read_vec_param('deadband_n')
        self.max_velocity = self._read_vec_param('max_velocity')
        self.max_acceleration = self._read_vec_param('max_acceleration')
        self.force_lowpass_cutoff_hz = float(self.get_parameter('force_lowpass_cutoff_hz').value)

        self.position = np.zeros(3, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)
        self.filtered_force_base = np.zeros(3, dtype=np.float64)

        self.latest_force_ee = np.zeros(3, dtype=np.float64)
        self.latest_wrench_frame = self.default_wrench_frame
        self.last_wrench_rx_time = None
        self.last_control_time = None
        self._last_warn_time = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            self.input_topic,
            self._wrench_callback,
            10,
        )
        self.state_pub = self.create_publisher(Float64MultiArray, self.output_topic, 10)

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.control_timer = self.create_timer(period, self._control_tick)

        self.get_logger().info(
            f'Admittance controller started at {self.control_rate_hz:.1f} Hz. '
            f'Input={self.input_topic}, output={self.output_topic}, base_frame={self.base_frame}'
        )

    def _read_vec_param(self, name: str) -> np.ndarray:
        raw = self.get_parameter(name).value
        if len(raw) != 3:
            raise ValueError(f'Parameter "{name}" must contain exactly 3 values')
        vec = np.array(raw, dtype=np.float64)
        return vec

    def _wrench_callback(self, msg: WrenchStamped) -> None:
        self.latest_force_ee = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            dtype=np.float64,
        )
        self.latest_wrench_frame = msg.header.frame_id if msg.header.frame_id else self.default_wrench_frame
        self.last_wrench_rx_time = self.get_clock().now()

    def _control_tick(self) -> None:
        now = self.get_clock().now()
        if self.last_control_time is None:
            self.last_control_time = now
            return

        dt = (now - self.last_control_time).nanoseconds * 1.0e-9
        self.last_control_time = now
        if not math.isfinite(dt):
            return
        dt = float(np.clip(dt, self.min_dt_s, self.max_dt_s))

        force_base = self._get_force_in_base(now)
        force_base = self._apply_deadband(force_base)
        self.filtered_force_base = self._lowpass_filter(self.filtered_force_base, force_base, dt)

        self.acceleration = (
            self.filtered_force_base
            - self.damping * self.velocity
            - self.stiffness * (self.position - self.reference_position)
        ) / self.mass
        self.acceleration = np.clip(self.acceleration, -self.max_acceleration, self.max_acceleration)

        self.velocity = self.velocity + dt * self.acceleration
        self.velocity = np.clip(self.velocity, -self.max_velocity, self.max_velocity)

        self.position = self.position + dt * self.velocity

        msg = Float64MultiArray()
        msg.data = np.concatenate((self.position, self.velocity, self.acceleration)).tolist()
        self.state_pub.publish(msg)

    def _get_force_in_base(self, now) -> np.ndarray:
        if self.last_wrench_rx_time is None:
            return self._decay_force()

        age_s = (now - self.last_wrench_rx_time).nanoseconds * 1.0e-9
        if age_s > self.wrench_timeout_s:
            self._warn_throttled(
                'wrench_timeout',
                f'No wrench received for {age_s:.3f}s (timeout={self.wrench_timeout_s:.3f}s).',
            )
            return self._decay_force()

        if self.latest_wrench_frame == self.base_frame:
            return self.latest_force_ee.copy()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.latest_wrench_frame,
                rclpy.time.Time(),
            )
            rot = tf.transform.rotation
            rotation = self._quat_to_matrix(rot.x, rot.y, rot.z, rot.w)
            return rotation.dot(self.latest_force_ee)
        except TransformException as exc:
            self._warn_throttled('tf_lookup', f'TF lookup failed: {str(exc)}')
            return self._decay_force()

    def _decay_force(self) -> np.ndarray:
        return 0.9 * self.filtered_force_base

    def _apply_deadband(self, force: np.ndarray) -> np.ndarray:
        masked = np.where(np.abs(force) < self.deadband_n, 0.0, force)
        return masked.astype(np.float64)

    def _lowpass_filter(self, prev: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
        if self.force_lowpass_cutoff_hz <= 0.0:
            return current
        tau = 1.0 / (2.0 * math.pi * self.force_lowpass_cutoff_hz)
        alpha = dt / (tau + dt)
        return alpha * current + (1.0 - alpha) * prev

    def _quat_to_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
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

    def _warn_throttled(self, key: str, message: str, period_s: float = 1.0) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        previous = self._last_warn_time.get(key)
        if previous is None or (now_s - previous) > period_s:
            self.get_logger().warn(message)
            self._last_warn_time[key] = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdmittanceControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
