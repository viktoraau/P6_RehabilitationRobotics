import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, WrenchStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from wrist_games_interfaces.srv import SetAdmittanceEnabled, SetReferenceJoint, SetStiffness, SetStiffnessScale


class OrientationAdmittanceNode(Node):
    """Rotational admittance controller.

    Input:  WrenchStamped (force and torque are both used)
    Output: desired orientation, angular velocity, angular acceleration in base frame

    Dynamics in base frame:
        I * omega_dot + D * omega + K * theta_err = tau

    theta_err is the live rotation-vector error from the current TF pose to the desired pose.
    The desired orientation q_des is integrated each tick via quaternion kinematics.
    """

    def __init__(self) -> None:
        super().__init__('orientation_admittance_controller')

        inertia = [0.05, 0.04, 0.4]
        stiffness = [0.25, 0.25, 0.50]

        for name, default in (
            ('input_topic', '/ft300/wrench'),
            ('base_frame', 'base_link'),
            ('default_wrench_frame', 'RU_1'),
            ('control_rate_hz', 100.0),
            ('wrench_timeout_s', 0.1),
            ('max_dt_s', 0.05),
            ('min_dt_s', 1.0e-4),
            ('inertia', inertia),
            ('stiffness', stiffness),
            ('damping', [
                2 * (stiffness[0] * inertia[0]) ** 0.5,
                2 * (stiffness[1] * inertia[1]) ** 0.5,
                2 * (stiffness[2] * inertia[2]) ** 0.5,
            ]),
            ('torque_deadband_nm', [0.05, 0.05, 0.05]),
            ('wrench_torque_scale', [0.0, 0.0, 1.0]), # 1 -1 1
            ('torque_lowpass_cutoff_hz', 20.0),
            ('moment_arm', [0.0, 0.0, 0.0]),
            ('force_to_torque_gain', 1.0),
            ('force_deadband_n', [0.0, 0.0, 0.0]),
            ('max_angular_velocity', [5.5, 5.5, 5.0]),
            ('max_angular_acceleration', [5.5, 5.5, 5.5]),
            ('max_orientation_error_rad', [1.2, 5.0, 1.5]),
            ('reference_tip_frame', 'RU_1'),
            ('orientation_topic', '/desired_orientation'),
            ('angular_velocity_topic', '/desired_angular_velocity'),
            ('angular_acceleration_topic', '/desired_angular_acceleration'),
            ('transparent_mode', False),
            ('transparent_inertia', [0.005, 0.005, 0.005]),
        ):
            self.declare_parameter(name, default)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.default_wrench_frame = str(self.get_parameter('default_wrench_frame').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.wrench_timeout_s = float(self.get_parameter('wrench_timeout_s').value)
        self.max_dt_s = float(self.get_parameter('max_dt_s').value)
        self.min_dt_s = float(self.get_parameter('min_dt_s').value)

        self.transparent_mode = bool(self.get_parameter('transparent_mode').value)
        self.transparent_inertia = self._read_vec_param('transparent_inertia', positive=True)

        self.inertia = self._read_vec_param('inertia', positive=True)
        self.stiffness = self._read_vec_param('stiffness')
        self._base_stiffness = self.stiffness.copy()  # snapshot before transparent mode can zero it
        # Critical damping: D = 2 * sqrt(J * K) * zeta, zeta=1
        self.damping = 2.0 * np.sqrt(self.inertia * self.stiffness) * 1.3
        self._base_damping = self.damping.copy()

        if self.transparent_mode:
            self.inertia = self.transparent_inertia
            self.damping = np.zeros(3, dtype=np.float64)
            self.stiffness = np.zeros(3, dtype=np.float64)
        self.torque_deadband_nm = self._read_vec_param('torque_deadband_nm')
        self.wrench_torque_scale = self._read_vec_param('wrench_torque_scale')
        self.max_angular_velocity = self._read_vec_param('max_angular_velocity', positive=True)
        self.max_angular_acceleration = self._read_vec_param('max_angular_acceleration', positive=True)
        self.max_orientation_error_rad = self._read_vec_param('max_orientation_error_rad', positive=True)
        self.torque_lowpass_cutoff_hz = float(self.get_parameter('torque_lowpass_cutoff_hz').value)
        self.moment_arm = self._read_vec_param('moment_arm')
        self.force_to_torque_gain = float(self.get_parameter('force_to_torque_gain').value)
        self.force_deadband_n = self._read_vec_param('force_deadband_n')

        self.reference_tip_frame = str(self.get_parameter('reference_tip_frame').value)

        self.orientation_topic = str(self.get_parameter('orientation_topic').value)
        self.angular_velocity_topic = str(self.get_parameter('angular_velocity_topic').value)
        self.angular_acceleration_topic = str(self.get_parameter('angular_acceleration_topic').value)

        self.q_ref = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)  # fixed reference, set once from TF
        self.q_des = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)  # integrated desired orientation
        self.q_des_initialized = False
        self.omega = np.zeros(3, dtype=np.float64)
        self.alpha = np.zeros(3, dtype=np.float64)
        self.filtered_torque_base = np.zeros(3, dtype=np.float64)

        self.latest_force_sensor = np.zeros(3, dtype=np.float64)
        self.latest_torque_sensor = np.zeros(3, dtype=np.float64)
        self.latest_wrench_frame = self.default_wrench_frame
        self.last_wrench_rx_time: Optional[rclpy.time.Time] = None
        self.last_good_wrench_time: Optional[rclpy.time.Time] = None
        self.last_control_time: Optional[rclpy.time.Time] = None
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

        # Service state
        self._admittance_enabled = True
        self._stiffness_scale = 1.0

        self.create_service(
            SetAdmittanceEnabled,
            'admittance/set_enabled',
            self._srv_set_enabled,
        )
        self.create_service(
            SetStiffness,
            'admittance/set_stiffness',
            self._srv_set_stiffness,
        )
        self.create_service(
            SetStiffnessScale,
            'admittance/set_stiffness_scale',
            self._srv_set_stiffness_scale,
        )
        self.create_service(
            SetReferenceJoint,
            'admittance/set_reference_joint',
            self._srv_set_reference_joint,
        )

        mode_str = 'TRANSPARENT (K=0, D=0)' if self.transparent_mode else 'normal'
        self.get_logger().info(
            f'Orientation admittance controller started [{mode_str}]. '
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
            (0.0, 0.0, msg.wrench.torque.z),
            dtype=np.float64,
        ) #* self.wrench_torque_scale    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        self.latest_wrench_frame = msg.header.frame_id or self.default_wrench_frame
        self.last_wrench_rx_time = self.get_clock().now()

    def _control_tick(self) -> None:
        now = self.get_clock().now()

        # Wait until TF is available to seed both q_ref and q_des
        if not self.q_des_initialized:
            q_init = self._get_tf_orientation_xyzw()
            if q_init is None:
                return
            self.q_ref = q_init.copy()
            self.q_des = q_init.copy()
            self.q_des_initialized = True
            self.last_control_time = now
            self.get_logger().info(
                f'Initialized reference from TF {self.base_frame} -> {self.reference_tip_frame}'
            )
            return

        if self.last_control_time is None:
            self.last_control_time = now

        dt = (now - self.last_control_time).nanoseconds * 1.0e-9
        self.last_control_time = now
        if not math.isfinite(dt) or dt <= 0.0:
            return
        dt = float(np.clip(dt, self.min_dt_s, self.max_dt_s))

        # When disabled, freeze the output and skip dynamics integration.
        if not self._admittance_enabled:
            self.omega = np.zeros(3, dtype=np.float64)
            self.alpha = np.zeros(3, dtype=np.float64)
            stamp_msg = now.to_msg()
            self._publish_orientation(self.q_des, stamp_msg)
            self._publish_vector(self.omega_pub, self.omega, stamp_msg)
            self._publish_vector(self.alpha_pub, self.alpha, stamp_msg)
            return

        # theta_err: rotation vector from fixed q_ref to current q_des
        # Stiffness spring always pulls q_des back toward the startup pose
        theta_err = self._quat_error_vec(self.q_ref, self.q_des)

        torque_base = self._get_torque_in_base(now)
        torque_base = self._apply_deadband(torque_base)
        self.filtered_torque_base = self._lowpass_filter(self.filtered_torque_base, torque_base, dt)

        # tuning
        #mask = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        #self.filtered_torque_base = self.filtered_torque_base * mask
        #tuning ends

        # Tustin / trapezoidal integration of  I*omega_dot + D*omega + K*theta = tau.
        # alpha_now is recomputed from the current state at every tick — caching
        # alpha across iterations was stale (it had been evaluated with the
        # previous theta_err) and broke the trapezoidal invariant.
        alpha_now = (
            self.filtered_torque_base - self.damping * self.omega - self.stiffness * theta_err
        ) / self.inertia

        h = 0.5 * dt
        denom = self.inertia + h * self.damping + h * h * self.stiffness
        omega_new = np.clip(
            (
                self.inertia * (self.omega + h * alpha_now)
                + h * (self.filtered_torque_base - self.stiffness * theta_err)
                - h * h * self.stiffness * self.omega
            ) / denom,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

        # At-limit: stop integrating q_des further away from q_ref
        at_pos_limit = theta_err >= self.max_orientation_error_rad
        at_neg_limit = theta_err <= -self.max_orientation_error_rad
        at_limit = at_pos_limit | at_neg_limit
        omega_new = np.where(at_limit & (theta_err * omega_new > 0.0), 0.0, omega_new)

        # Integrate q_des via quaternion kinematics, then publish alpha consistent
        # with the integrated state (theta_err_new), not the old theta_err.
        # Also apply the at-limit mask to avg_omega so that on the very first
        # zeroing step q_des does not bleed further past the orientation limit
        # due to the non-zero self.omega carried from the previous tick.
        avg_omega = 0.5 * (self.omega + omega_new)
        avg_omega = np.where(at_limit & (theta_err * avg_omega > 0.0), 0.0, avg_omega)
        self.q_des = self._integrate_quaternion(self.q_des, avg_omega, dt)
        theta_err_new = self._quat_error_vec(self.q_ref, self.q_des)

        alpha_new = np.clip(
            (self.filtered_torque_base - self.damping * omega_new - self.stiffness * theta_err_new) / self.inertia,
            -self.max_angular_acceleration,
            self.max_angular_acceleration,
        )
        alpha_new = np.where(at_limit, 0.0, alpha_new)

        self.omega = omega_new
        self.alpha = alpha_new

        stamp_msg = now.to_msg()
        self._publish_orientation(self.q_des, stamp_msg)
        self._publish_vector(self.omega_pub, self.omega, stamp_msg)
        self._publish_vector(self.alpha_pub, self.alpha, stamp_msg)

    def _get_tf_orientation_xyzw(self) -> Optional[np.ndarray]:
        """Return live orientation of reference_tip_frame in base_frame as [x,y,z,w], or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.reference_tip_frame,
                rclpy.time.Time(),
            )
            rot = tf.transform.rotation
            return np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float64)
        except TransformException as exc:
            self._warn_throttled('reference_tf', f'TF lookup failed for reference: {str(exc)}')
            return None

    def _get_torque_in_base(self, now) -> np.ndarray:
        # No wrench has ever arrived — nothing to integrate against.
        if self.last_wrench_rx_time is None:
            return self._decay_torque()

        # Latest message is stale: the publisher has stopped or stalled.
        age_rx = (now - self.last_wrench_rx_time).nanoseconds * 1.0e-9
        if age_rx > self.wrench_timeout_s:
            self._warn_throttled(
                'wrench_timeout',
                f'No wrench received for {age_rx:.3f}s (timeout={self.wrench_timeout_s:.3f}s).',
            )
            return self._decay_torque()

        # Frame matches base — use the torque directly.
        if self.latest_wrench_frame == self.base_frame:
            self.last_good_wrench_time = self.last_wrench_rx_time
            force = np.where(np.abs(self.latest_force_sensor) < self.force_deadband_n, 0.0, self.latest_force_sensor)
            tau_extra = self.force_to_torque_gain * np.cross(self.moment_arm, force)
            return self.latest_torque_sensor.copy() + tau_extra

        # Otherwise rotate into base via TF.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.latest_wrench_frame,
                rclpy.time.Time(),
            )
            rot = tf.transform.rotation
            rotation_matrix = self._quat_xyzw_to_matrix(
                np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float64)
            )
            force_sensor = np.where(np.abs(self.latest_force_sensor) < self.force_deadband_n, 0.0, self.latest_force_sensor)
            force_base = rotation_matrix @ force_sensor
            torque_base = rotation_matrix @ self.latest_torque_sensor
            self.last_good_wrench_time = self.last_wrench_rx_time
            r_base = rotation_matrix @ self.moment_arm
            tau_extra = self.force_to_torque_gain * np.cross(r_base, force_base)
            return torque_base + tau_extra
        except TransformException as exc:
            self._warn_throttled('tf_lookup', f'TF lookup failed: {str(exc)}')
            # If we previously had a usable wrench within the timeout window,
            # hold the filtered value rather than letting an untransformable
            # burst reset the integrated state. Otherwise decay so a steady
            # stream of bad-frame messages doesn't lock the controller open.
            if self.last_good_wrench_time is None:
                self._warn_throttled(
                    'no_good_wrench',
                    f'Wrench frame {self.latest_wrench_frame!r} cannot be transformed '
                    f'to {self.base_frame!r}; no usable wrench yet.',
                )
                return self._decay_torque()
            good_age = (now - self.last_good_wrench_time).nanoseconds * 1.0e-9
            if good_age > self.wrench_timeout_s:
                return self._decay_torque()
            return self.filtered_torque_base.copy()

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

    def _publish_orientation(self, q_xyzw: np.ndarray, stamp_msg) -> None:
        msg = QuaternionStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.base_frame
        msg.quaternion.x = float(q_xyzw[0])
        msg.quaternion.y = float(q_xyzw[1])
        msg.quaternion.z = float(q_xyzw[2])
        msg.quaternion.w = float(q_xyzw[3])
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
    def _quat_error_vec(q_ref_xyzw: np.ndarray, q_des_xyzw: np.ndarray) -> np.ndarray:
        """Rotation vector (axis*angle) from q_ref to q_des in base frame.

        Uses the exact log map theta = 2*atan2(|v|, w), not the small-angle 2*v
        approximation: at the 1.2 rad max-orientation-error limit the latter
        underestimates the spring torque by ~6%, shifting the steady-state pose.
        """
        rx, ry, rz, rw = q_ref_xyzw
        dx, dy, dz, dw = q_des_xyzw
        # Hamilton product q_des ⊗ conj(q_ref), where conj flips the vector part
        cx = dw * (-rx) + dx * rw + dy * (-rz) - dz * (-ry)
        cy = dw * (-ry) - dx * (-rz) + dy * rw + dz * (-rx)
        cz = dw * (-rz) + dx * (-ry) - dy * (-rx) + dz * rw
        cw = dw * rw + dx * rx + dy * ry + dz * rz
        # Pick the shorter arc (q and -q represent the same rotation)
        if cw < 0.0:
            cx, cy, cz, cw = -cx, -cy, -cz, -cw
        v_norm = math.sqrt(cx * cx + cy * cy + cz * cz)
        if v_norm < 1.0e-12:
            return np.zeros(3, dtype=np.float64)
        theta = 2.0 * math.atan2(v_norm, cw)
        scale = theta / v_norm
        return np.array([cx * scale, cy * scale, cz * scale], dtype=np.float64)

    @staticmethod
    def _integrate_quaternion(q_xyzw: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
        """Integrate unit quaternion given angular velocity omega in base frame."""
        ox, oy, oz = omega
        qx, qy, qz, qw = q_xyzw
        # q_dot = 0.5 * [ox,oy,oz,0] ⊗ q  (world-frame omega convention)
        dqx = 0.5 * ( ox * qw + oy * qz - oz * qy)
        dqy = 0.5 * (-ox * qz + oy * qw + oz * qx)
        dqz = 0.5 * ( ox * qy - oy * qx + oz * qw)
        dqw = 0.5 * (-ox * qx - oy * qy - oz * qz)
        q_new = q_xyzw + dt * np.array([dqx, dqy, dqz, dqw], dtype=np.float64)
        norm = np.linalg.norm(q_new)
        if norm < 1.0e-12:
            return q_xyzw.copy()
        return q_new / norm

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

    # ── Service callbacks ──────────────────────────────────────────────────────

    def _srv_set_enabled(
        self,
        request: SetAdmittanceEnabled.Request,
        response: SetAdmittanceEnabled.Response,
    ) -> SetAdmittanceEnabled.Response:
        self._admittance_enabled = bool(request.enable)
        state = 'enabled' if self._admittance_enabled else 'disabled'
        self.get_logger().info(f'Admittance controller {state} via service.')
        response.success = True
        response.message = f'Admittance {state}.'
        return response

    def _srv_set_stiffness(
        self,
        request: SetStiffness.Request,
        response: SetStiffness.Response,
    ) -> SetStiffness.Response:
        new_k = np.asarray(request.stiffness, dtype=np.float64)
        if np.any(new_k < 0.0):
            response.success = False
            response.message = 'Stiffness values must be non-negative.'
            return response
        self.stiffness = new_k
        self._base_stiffness = new_k.copy()
        # Where K=0, keep the existing damping so the system remains damped.
        new_d = np.where(new_k > 0.0, 2.0 * np.sqrt(self.inertia * new_k) * 1.3, self._base_damping)
        self.damping = new_d
        self._base_damping = np.where(new_k > 0.0, new_d, self._base_damping)
        self.get_logger().info(f'Stiffness updated to {new_k.tolist()} via service.')
        response.success = True
        response.message = f'Stiffness set to {new_k.tolist()}.'
        return response

    def _srv_set_stiffness_scale(
        self,
        request: SetStiffnessScale.Request,
        response: SetStiffnessScale.Response,
    ) -> SetStiffnessScale.Response:
        alpha = float(request.alpha)
        if alpha < 0.0:
            response.success = False
            response.message = 'alpha must be non-negative.'
            return response
        self._stiffness_scale = alpha
        self.stiffness = self._base_stiffness * alpha
        # Where scaled K=0 (alpha=0 or base K=0), fall back to base damping.
        self.damping = np.where(
            self.stiffness > 0.0,
            2.0 * np.sqrt(self.inertia * self.stiffness) * 1.3,
            self._base_damping,
        )
        self.get_logger().info(
            f'Stiffness scale set to {alpha:.4f} → effective stiffness {self.stiffness.tolist()}, '
            f'damping {self.damping.tolist()}.'
        )
        response.success = True
        response.message = f'Stiffness scale set to {alpha:.4f}.'
        return response

    def _srv_set_reference_joint(
        self,
        request: SetReferenceJoint.Request,
        response: SetReferenceJoint.Response,
    ) -> SetReferenceJoint.Response:
        # Snap reference to the current live TF orientation of reference_tip_frame.
        # The joint_positions field is ignored here — the reference is a quaternion
        # derived from the kinematic chain, not individual joint angles.
        q_new = self._get_tf_orientation_xyzw()
        if q_new is None:
            response.success = False
            response.message = (
                f'TF lookup from {self.base_frame!r} to {self.reference_tip_frame!r} failed; '
                'reference not updated.'
            )
            return response
        self.q_ref = q_new.copy()
        self.q_des = q_new.copy()
        self.omega = np.zeros(3, dtype=np.float64)
        self.alpha = np.zeros(3, dtype=np.float64)
        self.filtered_torque_base = np.zeros(3, dtype=np.float64)
        self.get_logger().info(
            f'Reference orientation reset to current TF pose '
            f'({self.base_frame} → {self.reference_tip_frame}) via service.'
        )
        response.success = True
        response.message = f'Reference orientation reset to current pose of {self.reference_tip_frame!r}.'
        return response

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
