"""Joint-space admittance controller.

Replaces the base-frame admittance + IK + Jacobian-inversion + bridge chain.

Pipeline
--------
1.  Rotate wrench into base_link via TF.
2.  Apply wrench_force_scale / wrench_torque_scale *in base frame* — fixes
    frame-mismatch sign errors without relying on sensor-frame pre-scaling.
    where scaling was done in sensor frame before rotation, which caused any
    sensor-frame Z torque to contaminate base-frame X/Y as soon as q1 != 0.
3.  tau_joint = J(q_current)^T · tau_base   (no matrix inversion).
4.  Lowpass-filter tau_joint, apply per-joint deadband.
5.  Tustin-integrate joint-space admittance:
        M · q_ddot + D · q_dot + K · (q_des - q_ref) = tau_joint
6.  Publish JointTrajectory (positions + velocities + accelerations) → CTC.

Kinematic model: R = Ry(-q1) · Rz(-q2) · Rx(q3)
  joint_1 axis: -Y,  joint_2 axis: -Z,  joint_3 axis: +X  (all intersecting)
"""

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from wrist_games_interfaces.srv import (
    SetAdmittanceEnabled,
    SetDampingGain,
    SetInertia,
    SetReferenceJoint,
    SetStiffness,
    SetStiffnessScale,
)


# ── Kinematic helpers ─────────────────────────────────────────────────────────

def _angular_jacobian(q: np.ndarray) -> np.ndarray:
    """Angular-velocity Jacobian:  omega_base = J(q) · q_dot.

    Columns are the joint rotation axes expressed in base_link for
    R = Ry(-q1) · Rz(-q2) · Rx(q3).

    J^T maps base-frame torque to joint torque:
        tau_joint = J(q)^T · tau_base
    """
    s1, c1 = math.sin(q[0]), math.cos(q[0])
    s2, c2 = math.sin(q[1]), math.cos(q[1])
    return np.array([
        [0.0,  s1,       c1 * c2],
        [-1.0, 0.0,     -s2     ],
        [0.0, -c1,       s1 * c2],
    ])


def _quat_xyzw_to_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = q_xyzw.astype(np.float64)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1.0e-12:
        raise ValueError('Quaternion norm is zero')
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


# ── Node ─────────────────────────────────────────────────────────────────────

class JointSpaceAdmittanceNode(Node):
    """Rotational admittance controller operating directly in joint space.

    Input:  WrenchStamped (FT sensor) + JointState
    Output: JointTrajectory (q_des, q_dot, q_ddot) for the CTC

    Dynamics per joint:
        M_i * q_ddot_i + D_i * q_dot_i + K_i * (q_des_i - q_ref_i) = tau_joint_i
    where
        tau_joint = J(q_current)^T · (wrench_torque_scale * tau_base)
    """

    def __init__(self) -> None:
        super().__init__('joint_space_admittance_controller')

        inertia_d = [0.05, 0.04, 0.4]
        stiffness_d = [0.25, 0.25, 0.50]

        for name, default in (
            ('input_topic',              '/ft300/wrench'),
            ('joint_state_topic',        '/joint_states'),
            ('output_topic',             '/bridge_desired_trajectory'),
            ('base_frame',               'base_link'),
            ('default_wrench_frame',     'RU_1'),
            ('control_rate_hz',          100.0),
            ('wrench_timeout_s',         0.1),
            ('max_dt_s',                 0.05),
            ('min_dt_s',                 1.0e-4),
            ('joint_names',              ['joint_1', 'joint_2', 'joint_3']),
            ('inertia',                  inertia_d),
            ('stiffness',                stiffness_d),
            ('damping_gain',             1.3),
            ('damping',                  [
                2.0 * math.sqrt(stiffness_d[i] * inertia_d[i]) * 1.8
                for i in range(3)
            ]),
            ('torque_deadband_nm',       [0.05, 0.05, 0.05]),
            # Applied in base frame AFTER TF rotation — avoids the sensor-frame
            # mismatch that contaminates base-frame axes when arm is deflected.
            ('wrench_force_scale',       [1.0, 1.0, 1.0]),
            ('wrench_torque_scale',      [1.0, -1.0, 1.0]),
            ('torque_lowpass_cutoff_hz', 20.0),
            ('moment_arm',               [0.0, 0.0, 0.0]),
            ('force_to_torque_gain',     0.0),
            ('force_deadband_n',         [0.3, 0.3, 0.3]),
            ('max_joint_velocity',       [5.5, 5.5, 5.0]),
            ('max_joint_acceleration',   [5.5, 5.5, 5.5]),
            # Upper bounds on |q_des - q_ref|; spring clamps here.
            ('max_joint_error_rad',      [1.2, 1.047198, 0.523599]),
            ('transparent_mode',         False),
            ('transparent_inertia',      [0.005, 0.005, 0.005]),
            ('admittance_enabled_on_startup', True),
        ):
            self.declare_parameter(name, default)

        self.input_topic         = str(self.get_parameter('input_topic').value)
        self.joint_state_topic   = str(self.get_parameter('joint_state_topic').value)
        self.output_topic        = str(self.get_parameter('output_topic').value)
        self.base_frame          = str(self.get_parameter('base_frame').value)
        self.default_wrench_frame = str(self.get_parameter('default_wrench_frame').value)
        self.control_rate_hz     = float(self.get_parameter('control_rate_hz').value)
        self.wrench_timeout_s    = float(self.get_parameter('wrench_timeout_s').value)
        self.max_dt_s            = float(self.get_parameter('max_dt_s').value)
        self.min_dt_s            = float(self.get_parameter('min_dt_s').value)
        self.joint_names         = list(self.get_parameter('joint_names').value)

        self.transparent_mode    = bool(self.get_parameter('transparent_mode').value)
        self.transparent_inertia = self._read_vec('transparent_inertia', positive=True)
        self.admittance_enabled_on_startup = bool(
            self.get_parameter('admittance_enabled_on_startup').value
        )

        self.inertia   = self._read_vec('inertia', positive=True)
        self.stiffness = self._read_vec('stiffness')
        self.damping_gain = float(self.get_parameter('damping_gain').value)
        self._base_stiffness = self.stiffness.copy()
        self.damping         = self._compute_damping(self.stiffness)
        self._base_damping   = self.damping.copy()

        if self.transparent_mode:
            self.inertia   = self.transparent_inertia
            self.damping   = np.zeros(3, dtype=np.float64)
            self.stiffness = np.zeros(3, dtype=np.float64)

        self.torque_deadband_nm       = self._read_vec('torque_deadband_nm')
        self.wrench_force_scale       = self._read_vec('wrench_force_scale')
        self.wrench_torque_scale      = self._read_vec('wrench_torque_scale')
        self.torque_lowpass_cutoff_hz = float(self.get_parameter('torque_lowpass_cutoff_hz').value)
        self.moment_arm               = self._read_vec('moment_arm')
        self.force_to_torque_gain     = float(self.get_parameter('force_to_torque_gain').value)
        self.force_deadband_n         = self._read_vec('force_deadband_n')
        self.max_joint_velocity       = self._read_vec('max_joint_velocity', positive=True)
        self.max_joint_acceleration   = self._read_vec('max_joint_acceleration', positive=True)
        self.max_joint_error_rad      = self._read_vec('max_joint_error_rad', positive=True)

        # ── Admittance state ─────────────────────────────────────────────── #
        self.q_ref:   Optional[np.ndarray] = None   # set once from first joint state
        self.q_des    = np.zeros(3, dtype=np.float64)
        self.q_dot    = np.zeros(3, dtype=np.float64)
        self.q_ddot   = np.zeros(3, dtype=np.float64)
        self.filtered_tau_joint = np.zeros(3, dtype=np.float64)

        # ── Sensor state ─────────────────────────────────────────────────── #
        self.latest_force_sensor  = np.zeros(3, dtype=np.float64)
        self.latest_torque_sensor = np.zeros(3, dtype=np.float64)   # raw, unscaled
        self.latest_wrench_frame  = self.default_wrench_frame
        self.last_wrench_rx_time: Optional[rclpy.time.Time] = None
        self.last_good_wrench_time: Optional[rclpy.time.Time] = None
        self.last_control_time:   Optional[rclpy.time.Time] = None

        # ── Joint-state cache ────────────────────────────────────────────── #
        # Used for J(q_current)^T and for initialising q_ref / q_des.
        self._current_q:    Optional[np.ndarray] = None
        self._current_qdot: Optional[np.ndarray] = None
        self._name_to_joint_idx: dict = {}

        self._last_warn_time: dict = {}

        # ── Service state ─────────────────────────────────────────────────── #
        self._admittance_enabled = self.admittance_enabled_on_startup
        self._stiffness_scale    = 1.0

        # ── TF ────────────────────────────────────────────────────────────── #
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────── #
        self.create_subscription(WrenchStamped, self.input_topic, self._wrench_cb, 10)
        self.create_subscription(JointState,    self.joint_state_topic, self._joint_state_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────── #
        self.traj_pub = self.create_publisher(JointTrajectory, self.output_topic, 10)

        # ── Control timer ─────────────────────────────────────────────────── #
        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.control_timer = self.create_timer(period, self._control_tick)

        # ── Services ──────────────────────────────────────────────────────── #
        self.create_service(SetAdmittanceEnabled, 'admittance/set_enabled',        self._srv_set_enabled)
        self.create_service(SetStiffness,         'admittance/set_stiffness',       self._srv_set_stiffness)
        self.create_service(SetStiffnessScale,    'admittance/set_stiffness_scale', self._srv_set_stiffness_scale)
        self.create_service(SetDampingGain,       'admittance/set_damping_gain',    self._srv_set_damping_gain)
        self.create_service(SetInertia,           'admittance/set_inertia',         self._srv_set_inertia)
        self.create_service(SetReferenceJoint,    'admittance/set_reference_joint', self._srv_set_reference_joint)

        mode_str = 'TRANSPARENT (K=0, D=0)' if self.transparent_mode else 'normal'
        enabled_str = 'enabled' if self._admittance_enabled else 'disabled'
        self.get_logger().info(
            f'Joint-space admittance controller started [{mode_str}]. '
            f'Input={self.input_topic}, output={self.output_topic}, '
            f'admittance={enabled_str}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        # Store raw sensor readings — scale is applied after TF rotation.
        self.latest_force_sensor  = np.asarray(
            (msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z),
            dtype=np.float64,
        )
        self.latest_torque_sensor = np.asarray(
            (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z),
            dtype=np.float64,
        )
        self.latest_wrench_frame  = msg.header.frame_id or self.default_wrench_frame
        self.last_wrench_rx_time  = self.get_clock().now()

    def _joint_state_cb(self, msg: JointState) -> None:
        if not self._name_to_joint_idx:
            self._name_to_joint_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            idx = [self._name_to_joint_idx[n] for n in self.joint_names]
        except KeyError:
            return
        if len(msg.position) == 0:
            return
        try:
            self._current_q = np.array(
                [msg.position[i] for i in idx], dtype=np.float64
            )
            if len(msg.velocity) >= len(idx):
                self._current_qdot = np.array(
                    [msg.velocity[i] for i in idx], dtype=np.float64
                )
        except IndexError:
            pass

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_tick(self) -> None:
        now = self.get_clock().now()

        # Initialise q_ref and q_des from the first joint state received.
        if self.q_ref is None:
            if self._current_q is None:
                return
            self.q_ref  = self._current_q.copy()
            self.q_des  = self._current_q.copy()
            self.last_control_time = now
            self.get_logger().info(
                f'Initialized reference from joint states: {self.q_ref.tolist()}'
            )
            return

        if self.last_control_time is None:
            self.last_control_time = now

        dt = (now - self.last_control_time).nanoseconds * 1.0e-9
        self.last_control_time = now
        if not math.isfinite(dt) or dt <= 0.0:
            return
        dt = float(np.clip(dt, self.min_dt_s, self.max_dt_s))

        if not self._admittance_enabled:
            self.q_dot  = np.zeros(3, dtype=np.float64)
            self.q_ddot = np.zeros(3, dtype=np.float64)
            self._publish(now)
            return

        # ── 1. Wrench → base-frame torque ────────────────────────────────── #
        tau_base = self._get_torque_in_base(now)

        # ── 2. Apply scale in base frame (not sensor frame) ──────────────── #
        tau_base = tau_base * self.wrench_torque_scale

        # ── 3. Map to joint space via J(q_current)^T ─────────────────────── #
        q_jac = self._current_q if self._current_q is not None else self.q_des
        J = _angular_jacobian(q_jac)
        tau_joint_raw = J.T @ tau_base

        # ── 4. Deadband + lowpass ─────────────────────────────────────────── #
        tau_joint_raw = np.where(
            np.abs(tau_joint_raw) < self.torque_deadband_nm, 0.0, tau_joint_raw
        ).astype(np.float64)
        self.filtered_tau_joint = self._lowpass(
            self.filtered_tau_joint, tau_joint_raw, dt
        )
        tau = self.filtered_tau_joint

        # ── 5. Tustin-integrate joint-space admittance ────────────────────── #
        q_err = self.q_des - self.q_ref

        alpha_now = (tau - self.damping * self.q_dot - self.stiffness * q_err) / self.inertia

        h     = 0.5 * dt
        denom = self.inertia + h * self.damping + h * h * self.stiffness
        q_dot_new = np.clip(
            (
                self.inertia * (self.q_dot + h * alpha_now)
                + h * (tau - self.stiffness * q_err)
                - h * h * self.stiffness * self.q_dot
            ) / denom,
            -self.max_joint_velocity,
            self.max_joint_velocity,
        )

        # At-limit: stop integrating q_des further away from q_ref
        at_pos   = q_err >=  self.max_joint_error_rad
        at_neg   = q_err <= -self.max_joint_error_rad
        at_limit = at_pos | at_neg
        q_dot_new = np.where(at_limit & (q_err * q_dot_new > 0.0), 0.0, q_dot_new)

        avg_qdot = 0.5 * (self.q_dot + q_dot_new)
        avg_qdot = np.where(at_limit & (q_err * avg_qdot > 0.0), 0.0, avg_qdot)
        self.q_des = self.q_des + dt * avg_qdot
        q_err_new  = self.q_des - self.q_ref

        q_ddot_new = np.clip(
            (tau - self.damping * q_dot_new - self.stiffness * q_err_new) / self.inertia,
            -self.max_joint_acceleration,
            self.max_joint_acceleration,
        )
        q_ddot_new = np.where(at_limit, 0.0, q_ddot_new)

        self.q_dot  = q_dot_new
        self.q_ddot = q_ddot_new

        self._publish(now)

    # ── Wrench helpers ────────────────────────────────────────────────────────

    def _get_torque_in_base(self, now) -> np.ndarray:
        """Return torque in base_link frame (raw, before wrench_torque_scale)."""
        if self.last_wrench_rx_time is None:
            return self._decay_torque()

        age = (now - self.last_wrench_rx_time).nanoseconds * 1.0e-9
        if age > self.wrench_timeout_s:
            self._warn_throttled(
                'wrench_timeout',
                f'No wrench received for {age:.3f}s (timeout={self.wrench_timeout_s:.3f}s).',
            )
            return self._decay_torque()

        if self.latest_wrench_frame == self.base_frame:
            self.last_good_wrench_time = self.last_wrench_rx_time
            force = np.where(
                np.abs(self.latest_force_sensor) < self.force_deadband_n,
                0.0, self.latest_force_sensor,
            )
            force = force * self.wrench_force_scale
            tau_extra = self.force_to_torque_gain * np.cross(self.moment_arm, force)
            return self.latest_torque_sensor.copy() + tau_extra

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.latest_wrench_frame, rclpy.time.Time()
            )
            rot = tf.transform.rotation
            R   = _quat_xyzw_to_matrix(
                np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float64)
            )
            force_base = R @ np.where(
                np.abs(self.latest_force_sensor) < self.force_deadband_n,
                0.0, self.latest_force_sensor,
            )
            force_base = force_base * self.wrench_force_scale
            torque_base = R @ self.latest_torque_sensor
            self.last_good_wrench_time = self.last_wrench_rx_time
            r_base  = R @ self.moment_arm
            tau_extra = self.force_to_torque_gain * np.cross(r_base, force_base)
            return torque_base + tau_extra
        except TransformException as exc:
            self._warn_throttled('tf_lookup', f'TF lookup failed: {exc}')
            if self.last_good_wrench_time is None:
                return self._decay_torque()
            good_age = (now - self.last_good_wrench_time).nanoseconds * 1.0e-9
            if good_age > self.wrench_timeout_s:
                return self._decay_torque()
            # Return the already-scaled filtered joint torque back-projected to base
            # (approximate hold): just decay rather than invert.
            return self._decay_torque()

    def _decay_torque(self) -> np.ndarray:
        # Decay filtered joint torque and back-project — approximated as zero
        # base torque so the filter decays naturally via _lowpass on the next tick.
        return np.zeros(3, dtype=np.float64)

    def _lowpass(self, prev: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
        if self.torque_lowpass_cutoff_hz <= 0.0:
            return current
        tau   = 1.0 / (2.0 * math.pi * self.torque_lowpass_cutoff_hz)
        alpha = dt / (tau + dt)
        return alpha * current + (1.0 - alpha) * prev

    def _compute_damping(self, stiffness: np.ndarray) -> np.ndarray:
        return 2.0 * np.sqrt(self.inertia * stiffness) * self.damping_gain

    def _refresh_damping_from_current_state(self) -> None:
        self._base_damping = np.where(
            self._base_stiffness > 0.0,
            self._compute_damping(self._base_stiffness),
            self._base_damping,
        )
        self.damping = np.where(
            self.stiffness > 0.0,
            self._compute_damping(self.stiffness),
            self._base_damping,
        )
        if self.transparent_mode:
            self.damping = np.zeros(3, dtype=np.float64)

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish(self, now) -> None:
        traj = JointTrajectory()
        traj.header.stamp    = now.to_msg()
        traj.header.frame_id = self.base_frame
        traj.joint_names     = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions      = self.q_des.tolist()
        pt.velocities     = self.q_dot.tolist()
        pt.accelerations  = self.q_ddot.tolist()
        pt.time_from_start.nanosec = 10_000_000   # 10 ms look-ahead
        traj.points.append(pt)

        self.traj_pub.publish(traj)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _read_vec(self, name: str, positive: bool = False) -> np.ndarray:
        raw = self.get_parameter(name).value
        if len(raw) != 3:
            raise ValueError(f'Parameter "{name}" must contain exactly 3 values')
        vec = np.asarray(raw, dtype=np.float64)
        if positive and np.any(vec <= 0.0):
            raise ValueError(f'Parameter "{name}" must contain strictly positive values')
        return vec

    def _warn_throttled(self, key: str, message: str, period_s: float = 1.0) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        prev  = self._last_warn_time.get(key)
        if prev is None or (now_s - prev) > period_s:
            self.get_logger().warn(message)
            self._last_warn_time[key] = now_s

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_set_enabled(self, request, response):
        self._admittance_enabled = bool(request.enable)
        state = 'enabled' if self._admittance_enabled else 'disabled'
        self.get_logger().info(f'Admittance controller {state} via service.')
        response.success = True
        response.message = f'Admittance {state}.'
        return response

    def _srv_set_stiffness(self, request, response):
        new_k = np.asarray(request.stiffness, dtype=np.float64)
        if np.any(new_k < 0.0):
            response.success = False
            response.message = 'Stiffness values must be non-negative.'
            return response
        self.stiffness       = new_k
        self._base_stiffness = new_k.copy()
        # Where K=0, keep the existing damping so the system remains damped.
        new_d = np.where(new_k > 0.0, self._compute_damping(new_k), self._base_damping)
        self.damping         = new_d
        self._base_damping   = np.where(new_k > 0.0, new_d, self._base_damping)
        self.get_logger().info(f'Stiffness updated to {new_k.tolist()} via service.')
        response.success = True
        response.message = f'Stiffness set to {new_k.tolist()}.'
        return response

    def _srv_set_stiffness_scale(self, request, response):
        alpha = float(request.alpha)
        if alpha < 0.0:
            response.success = False
            response.message = 'alpha must be non-negative.'
            return response
        self._stiffness_scale = alpha
        self.stiffness        = self._base_stiffness * alpha
        # Where scaled K=0 (alpha=0 or base K=0), fall back to base damping.
        self.damping = np.where(
            self.stiffness > 0.0,
            self._compute_damping(self.stiffness),
            self._base_damping,
        )
        self.get_logger().info(
            f'Stiffness scale set to {alpha:.4f} → effective stiffness {self.stiffness.tolist()}, '
            f'damping {self.damping.tolist()}.'
        )
        response.success = True
        response.message = f'Stiffness scale set to {alpha:.4f}.'
        return response

    def _srv_set_damping_gain(self, request, response):
        gain = float(request.damping_gain)
        if not math.isfinite(gain) or gain <= 0.0:
            response.success = False
            response.message = 'damping_gain must be finite and > 0.'
            return response

        self.damping_gain = gain
        self._refresh_damping_from_current_state()
        self.get_logger().info(
            f'Damping gain set to {gain:.4f} → damping {self.damping.tolist()}.'
        )
        response.success = True
        response.message = f'Damping gain set to {gain:.4f}.'
        return response

    def _srv_set_inertia(self, request, response):
        new_inertia = np.asarray(request.inertia, dtype=np.float64)
        if np.any(~np.isfinite(new_inertia)) or np.any(new_inertia <= 0.0):
            response.success = False
            response.message = 'Inertia values must be finite and strictly positive.'
            return response

        self.inertia = new_inertia
        self._refresh_damping_from_current_state()
        self.get_logger().info(
            f'Inertia set to {new_inertia.tolist()} → damping {self.damping.tolist()}.'
        )
        response.success = True
        response.message = f'Inertia set to {new_inertia.tolist()}.'
        return response

    def _srv_set_reference_joint(self, request, response):
        positions = list(request.joint_positions)
        if len(positions) == 0:
            if self._current_q is None:
                response.success = False
                response.message = 'No joint state received yet; cannot snap to current position.'
                return response
            new_ref = self._current_q.copy()
        elif len(positions) == 3:
            new_ref = np.asarray(positions, dtype=np.float64)
        else:
            response.success = False
            response.message = f'joint_positions must be empty (snap to current) or length 3, got {len(positions)}.'
            return response
        self.q_ref = new_ref
        self.q_des = new_ref.copy()
        self.q_dot  = np.zeros(3, dtype=np.float64)
        self.q_ddot = np.zeros(3, dtype=np.float64)
        self.filtered_tau_joint = np.zeros(3, dtype=np.float64)
        self.get_logger().info(f'Reference joint position updated to {self.q_ref.tolist()} via service.')
        response.success = True
        response.message = f'Reference set to {self.q_ref.tolist()}.'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointSpaceAdmittanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
