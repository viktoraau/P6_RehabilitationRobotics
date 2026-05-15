"""Computed torque controller using KDL for rigid-body dynamics.

The URDF is parsed from /robot_description using only stdlib xml and PyKDL
(python3-pykdl, which ships with ROS Jazzy).  No extra pip packages required.

Dynamic matrices M, C, G are computed online by KDL's ChainDynParam solver.
Friction (viscous + Coulomb) and motor inertia are diagonal corrections
supplied via the YAML config file.  All controller gains default to 0.0 so
the operator can tune them without editing code.
"""

import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as kdl
import math 


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


# ──────────────────────────────────────────────────────────────────────────────
#  Minimal URDF → KDL chain builder  (no external deps beyond PyKDL)
# ──────────────────────────────────────────────────────────────────────────────

def _parse_vec3(text, default=(0.0, 0.0, 0.0)):
    if not text or not text.strip():
        return default
    vals = text.split()
    return tuple(float(v) for v in vals) if len(vals) == 3 else default


def _urdf_inertia_to_kdl(inertial_el):
    """Parse a URDF <inertial> element into a KDL RigidBodyInertia."""
    if inertial_el is None:
        return kdl.RigidBodyInertia()

    mass_el = inertial_el.find('mass')
    m = float(mass_el.get('value', '0')) if mass_el is not None else 0.0

    origin_el = inertial_el.find('origin')
    if origin_el is not None:
        xyz = _parse_vec3(origin_el.get('xyz'))
        rpy = _parse_vec3(origin_el.get('rpy'))
    else:
        xyz = rpy = (0.0, 0.0, 0.0)

    oc = kdl.Vector(*xyz)

    I_el = inertial_el.find('inertia')
    if I_el is not None:
        ixx = float(I_el.get('ixx', '0'))
        iyy = float(I_el.get('iyy', '0'))
        izz = float(I_el.get('izz', '0'))
        ixy = float(I_el.get('ixy', '0'))
        ixz = float(I_el.get('ixz', '0'))
        iyz = float(I_el.get('iyz', '0'))
    else:
        ixx = iyy = izz = ixy = ixz = iyz = 0.0

    Ic = kdl.RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz)

    # If the inertial frame is rotated wrt the link frame, rotate the tensor.
    if any(abs(r) > 1e-9 for r in rpy):
        R = kdl.Rotation.RPY(*rpy)
        Ic_mat = np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ])
        R_np = np.array([[R[i, j] for j in range(3)] for i in range(3)])
        Ic_rot = R_np @ Ic_mat @ R_np.T
        Ic = kdl.RotationalInertia(
            Ic_rot[0, 0], Ic_rot[1, 1], Ic_rot[2, 2],
            Ic_rot[0, 1], Ic_rot[0, 2], Ic_rot[1, 2],
        )

    return kdl.RigidBodyInertia(m, oc, Ic)


def _chain_from_urdf_string(urdf_string, base_link, tip_link, logger=None):
    """Build a KDL Chain from a URDF XML string.

    Follows the same convention as kdl_parser (C++):
    for each revolute joint with URDF origin (p, R), the KDL Joint is
    Joint(name, p, R*axis, RotAxis) with f_tip = Frame::Identity().
    """
    def _log(msg):
        if logger is not None:
            logger.info(msg)

    root = ET.fromstring(urdf_string)

    # link name → RigidBodyInertia
    link_inertia: dict[str, kdl.RigidBodyInertia] = {}
    for link_el in root.iter('link'):
        name = link_el.get('name', '')
        inertia = _urdf_inertia_to_kdl(link_el.find('inertial'))
        link_inertia[name] = inertia
        if inertia.getMass() > 0:
            _log(f'[URDF] link={name!r}  mass={inertia.getMass():.4f}  cog={inertia.getCOG()}')

    # child link → (parent_link, joint_name, joint_type, xyz, rpy, axis)
    child_to_parent: dict[str, tuple] = {}
    for joint_el in root.iter('joint'):
        jname = joint_el.get('name', '')
        jtype = joint_el.get('type', 'fixed')
        parent_el = joint_el.find('parent')
        child_el = joint_el.find('child')
        if parent_el is None or child_el is None:
            continue
        parent_lk = parent_el.get('link', '')
        child_lk = child_el.get('link', '')

        origin_el = joint_el.find('origin')
        if origin_el is not None:
            xyz = _parse_vec3(origin_el.get('xyz'))
            rpy = _parse_vec3(origin_el.get('rpy'))
        else:
            xyz = rpy = (0.0, 0.0, 0.0)

        axis_el = joint_el.find('axis')
        axis = _parse_vec3(
            axis_el.get('xyz') if axis_el is not None else None,
            default=(1.0, 0.0, 0.0),
        )
        child_to_parent[child_lk] = (parent_lk, jname, jtype, xyz, rpy, axis)

    # Trace path from tip back to base, then reverse
    path = []
    current = tip_link
    visited: set[str] = set()
    while current != base_link:
        if current in visited:
            raise ValueError(
                f'Loop detected tracing chain from {base_link!r} to {tip_link!r}'
            )
        visited.add(current)
        if current not in child_to_parent:
            raise ValueError(
                f'Link {current!r} has no parent joint – '
                f'cannot reach {base_link!r}'
            )
        entry = child_to_parent[current]
        path.append((entry, current))
        current = entry[0]  # parent link
    path.reverse()

    # Build KDL Chain
    chain = kdl.Chain()
    for (parent_lk, jname, jtype, xyz, rpy, axis), child_lk in path:
        F_origin = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))

        if jtype in ('revolute', 'continuous'):
            axis_in_parent = F_origin.M * kdl.Vector(*axis)
            kdl_joint = kdl.Joint(
                jname, F_origin.p, axis_in_parent, kdl.Joint.RotAxis
            )
        else:
            kdl_joint = kdl.Joint(jname, kdl.Joint.Fixed)

        I = link_inertia.get(child_lk, kdl.RigidBodyInertia())
        _log(f'[CHAIN] joint={jname!r}  child={child_lk!r}  mass={I.getMass():.4f}')
        if I.getMass() < 1e-9:
            _log(
                f'[CHAIN] WARNING: link {child_lk!r} has zero mass – '
                'check <inertial> in the URDF for this link'
            )
        chain.addSegment(
            kdl.Segment(jname, kdl_joint, kdl.Frame.Identity(), I)
        )

    return chain


class ComputedTorqueControllerKDL(Node):
    """Computed torque controller – dynamics from KDL, gains from config."""

    def __init__(self):
        super().__init__('computed_torque_controller_kdl')

        # ── Parameters ────────────────────────────────────────────────── #
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter(
            'desired_trajectory_topic',
            '/joint_trajectory_controller/JointTrajectoryController',
        )
        self.declare_parameter(
            'torque_command_topic',
            '/joint_group_effort_controller/commands',
        )
        self.declare_parameter(
            'joint_names', ['joint_1', 'joint_2', 'joint_3']
        )
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'RU_1')
        self.declare_parameter('gravity', [0.0, 0.0, -9.81])
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('kp', [12.0, 12.0, 12.0])
        self.declare_parameter('kd', [6.0, 6.0, 6.0])

        self.declare_parameter('viscous_friction', [0.0, 0.0, 0.0])
        self.declare_parameter('coulomb_friction', [0.0, 0.0, 0.0])
        self.declare_parameter('coulomb_tanh_scale', 10.0)
        self.declare_parameter('motor_inertia', [0.0, 0.0, 0.0])
        self.declare_parameter('use_motor_inertia', True)
        self.declare_parameter('torque_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('torque_limits', [65.0, 50.0, 30.0]) #justér
        self.declare_parameter('gravity_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('transparent_friction_scale', 1.0)
        self.declare_parameter('transparent_mode', False)
        self.declare_parameter('position_limits_lower', [math.radians(-65), math.radians(-60), math.radians(-30)] ) #justér
        self.declare_parameter('position_limits_upper', [math.radians(65), math.radians(60), math.radians(30)] ) #justér
        self.declare_parameter('joint_limit_avoidance_gain', 0.0)
        self.declare_parameter('joint_limit_buffer', 0.0)
        # 0.0 = no filtering; positive value = cutoff frequency in Hz
        self.declare_parameter('velocity_filter_cutoff_hz', 0.0)

        # ── Read parameters ───────────────────────────────────────────── #
        self.joint_names = list(
            self.get_parameter('joint_names')
            .get_parameter_value()
            .string_array_value
        )
        self.dof = len(self.joint_names)

        self._base_link = (
            self.get_parameter('base_link').get_parameter_value().string_value
        )
        self._tip_link = (
            self.get_parameter('tip_link').get_parameter_value().string_value
        )

        grav = list(
            self.get_parameter('gravity')
            .get_parameter_value()
            .double_array_value
        )
        self._kdl_gravity = kdl.Vector(*grav)

        self._ctrl_rate = float(self.get_parameter('control_rate_hz').value)
        self._kp = self._vec_param('kp')
        self._kd = self._vec_param('kd')
        self._viscous_b = self._vec_param('viscous_friction')
        self._coulomb_fc = self._vec_param('coulomb_friction')
        self._coulomb_scale = float(
            self.get_parameter('coulomb_tanh_scale').value
        )
        self._motor_inertia = self._vec_param('motor_inertia')
        self._use_motor_inertia = bool(
            self.get_parameter('use_motor_inertia').value
        )
        self._torque_scale = self._vec_param('torque_scale')
        self._torque_limits = np.abs(self._vec_param('torque_limits'))
        self._gravity_scale = self._vec_param('gravity_scale')
        self._transparent_friction_scale = float(
            self.get_parameter('transparent_friction_scale').value
        )
        self._transparent = bool(self.get_parameter('transparent_mode').value)
        self._pos_limits_lower = self._vec_param('position_limits_lower')
        self._pos_limits_upper = self._vec_param('position_limits_upper')
        self._limit_avoidance_gain = float(
            self.get_parameter('joint_limit_avoidance_gain').value
        )
        self._limit_buffer = float(
            self.get_parameter('joint_limit_buffer').value
        )
        cutoff_hz = float(self.get_parameter('velocity_filter_cutoff_hz').value)
        if cutoff_hz > 0.0:
            self._vel_filter_alpha = math.exp(
                -2.0 * math.pi * cutoff_hz / self._ctrl_rate
            )
        else:
            self._vel_filter_alpha = 0.0  # disabled

        # ── State ────────────────────────────────────────────────────── #
        self.q = np.zeros(self.dof)
        self.q_dot = np.zeros(self.dof)
        self.q_des = np.zeros(self.dof)
        self.q_dot_des = np.zeros(self.dof)
        self.q_ddot_des = np.zeros(self.dof)

        self.have_actual = False
        self.have_desired = False
        self._ready_logged = False
        self.last_tau = np.zeros(self.dof)

        # KDL solver – populated once robot_description arrives
        self._kdl_chain: kdl.Chain | None = None
        self._dyn_param: kdl.ChainDynParam | None = None

        # ── Topics ───────────────────────────────────────────────────── #
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            String, '/robot_description',
            self._robot_description_cb, latched,
        )

        joint_state_topic = (
            self.get_parameter('joint_state_topic')
            .get_parameter_value()
            .string_value
        )
        desired_topic = (
            self.get_parameter('desired_trajectory_topic')
            .get_parameter_value()
            .string_value
        )
        torque_topic = (
            self.get_parameter('torque_command_topic')
            .get_parameter_value()
            .string_value
        )

        self.create_subscription(
            JointState, joint_state_topic,
            self._joint_state_cb, 10,
        )
        self.create_subscription(
            JointTrajectory, desired_topic,
            self._desired_trajectory_cb, 10,
        )

        self._torque_pub = self.create_publisher(
            Float64MultiArray, torque_topic, 10
        )

        period = 1.0 / max(self._ctrl_rate, 1.0)
        self.create_timer(period, self._control_cb)

        mode_str = 'TRANSPARENT (gravity+friction compensation only)' if self._transparent else 'tracking'
        self.get_logger().info(
            f'Computed torque controller (KDL) started [{mode_str}]. '
            f'joints={self.joint_names}, '
            f'chain={self._base_link!r}->{self._tip_link!r}. '
            'Waiting for /robot_description …'
        )

    # ── URDF callback ─────────────────────────────────────────────────── #

    def _robot_description_cb(self, msg: String):
        if self._dyn_param is not None:
            return

        self.get_logger().info(
            f'[robot_description] received {len(msg.data)} chars; '
            f'building KDL chain {self._base_link!r} → {self._tip_link!r}'
        )

        try:
            chain = _chain_from_urdf_string(
                msg.data, self._base_link, self._tip_link,
                logger=self.get_logger(),
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to build KDL chain: {exc}')
            return

        n_joints = chain.getNrOfJoints()
        if n_joints != self.dof:
            self.get_logger().error(
                f'KDL chain {self._base_link!r}->{self._tip_link!r} '
                f'has {n_joints} joints but controller expects {self.dof}. '
                'Check base_link / tip_link parameters.'
            )
            return

        # Sanity-check: log every segment mass so a zero-mass URDF is obvious.
        total_mass = 0.0
        for i in range(chain.getNrOfSegments()):
            seg = chain.getSegment(i)
            m = seg.getInertia().getMass()
            total_mass += m
            self.get_logger().info(
                f'  [CHAIN verify] segment[{i}] {seg.getName()!r}: mass={m:.4f} kg'
            )
        if total_mass < 1e-6:
            self.get_logger().error(
                'KDL chain total mass is zero! '
                'The URDF published on /robot_description has no <inertial> '
                'data for the chain links – G, C, M will all be zero. '
                'Check which robot_description is being published.'
            )
            return

        grav_mag = (self._kdl_gravity[0]**2 + self._kdl_gravity[1]**2
                    + self._kdl_gravity[2]**2) ** 0.5
        self.get_logger().info(
            f'Gravity vector: [{self._kdl_gravity[0]:.3f}, '
            f'{self._kdl_gravity[1]:.3f}, {self._kdl_gravity[2]:.3f}] '
            f'(|g|={grav_mag:.3f} m/s²)'
        )

        # Keep the chain alive – ChainDynParam's internal solvers hold a
        # reference to the chain and return error -3 if it is GC'd.
        self._kdl_chain = chain
        self._dyn_param = kdl.ChainDynParam(self._kdl_chain, self._kdl_gravity)
        self.get_logger().info(
            f'KDL dynamics ready: {self._base_link} -> {self._tip_link} '
            f'({n_joints} joints, total_mass={total_mass:.3f} kg).'
        )

    # ── ROS callbacks ─────────────────────────────────────────────────── #

    def _joint_state_cb(self, msg: JointState):
        q = self._ordered(msg.name, msg.position, 'JointState.position')
        q_dot = self._ordered(msg.name, msg.velocity, 'JointState.velocity')
        if q is None or q_dot is None:
            return
        self.q = q
        alpha = self._vel_filter_alpha
        if alpha == 0.0 or not self.have_actual:
            self.q_dot = q_dot
        else:
            self.q_dot = alpha * self.q_dot + (1.0 - alpha) * q_dot
        self.have_actual = True
        self._log_ready_once()

    def _desired_trajectory_cb(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Desired trajectory has no points')
            return
        pt = msg.points[0]
        q_des = self._ordered(
            msg.joint_names, pt.positions, 'positions'
        )
        if q_des is None:
            return
        self.q_des = q_des
        self.q_dot_des = self._ordered(
            msg.joint_names, pt.velocities, 'velocities',
            required=False, default=0.0,
        )
        self.q_ddot_des = self._ordered(
            msg.joint_names, pt.accelerations, 'accelerations',
            required=False, default=0.0,
        )
        self.have_desired = True
        self._log_ready_once()

    def _control_cb(self):
        if self._dyn_param is None:
            self.get_logger().warn(
                'Not computing: KDL chain not ready – '
                'waiting for /robot_description.',
                throttle_duration_sec=5.0,
            )
            return
        if not self.have_actual:
            self.get_logger().warn(
                f'Not computing: no JointState received yet on '
                f'{self.get_parameter("joint_state_topic").get_parameter_value().string_value!r}.',
                throttle_duration_sec=5.0,
            )
            return
        if not self._transparent and not self.have_desired:
            self.get_logger().warn(
                f'Not computing: no desired trajectory received yet on '
                f'{self.get_parameter("desired_trajectory_topic").get_parameter_value().string_value!r}.',
                throttle_duration_sec=5.0,
            )
            return
        try:
            tau = self._compute_torque()
            tau = np.array([tau[0], tau[1], tau[2]])  # joint_1: URDF axis flipped to -Y, now matches hardware — no negate. joint_3: URDF axis flipped from -X to +X, KDL sign inverted — negate to restore hardware convention.
            tau = np.clip(tau, -self._torque_limits, self._torque_limits)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'_compute_torque raised an exception: {exc}',
                throttle_duration_sec=2.0,
            )
            return
        self.get_logger().debug(
            f'pos_err={self.q_des - self.q}, '
            f'vel_err={self.q_dot_des - self.q_dot}, '
            f'tau={tau}',
            throttle_duration_sec=1.0,
        )
        self.last_tau = tau
        cmd = Float64MultiArray()
        cmd.data = tau.tolist()
        self._torque_pub.publish(cmd)

    # ── Computed-torque law ──────────────────────────────────────────── #

    def _compute_torque(self) -> np.ndarray:
        if self._transparent:
            pos_err = np.zeros(self.dof)
            vel_err = np.zeros(self.dof)
            q_ddot_ref = np.zeros(self.dof)
        else:
            # Clamp desired position to limits so tracking law never pulls toward a limit.
            q_des_clamped = np.clip(
                self.q_des, self._pos_limits_lower, self._pos_limits_upper
            )
            pos_err = q_des_clamped - self.q
            vel_err = self.q_dot_des - self.q_dot
            q_ddot_ref = self.q_ddot_des + self._kd * vel_err + self._kp * pos_err

        # Pack into KDL arrays
        q_kdl = kdl.JntArray(self.dof)
        qd_kdl = kdl.JntArray(self.dof)
        for i in range(self.dof):
            q_kdl[i] = float(self.q[i])
            qd_kdl[i] = float(self.q_dot[i])

        # KDL solvers
        M_kdl = kdl.JntSpaceInertiaMatrix(self.dof)
        C_kdl = kdl.JntArray(self.dof)   # C(q,qd)*qd  – bias vector
        G_kdl = kdl.JntArray(self.dof)

        ret_m = self._dyn_param.JntToMass(q_kdl, M_kdl)
        ret_c = self._dyn_param.JntToCoriolis(q_kdl, qd_kdl, C_kdl)
        ret_g = self._dyn_param.JntToGravity(q_kdl, G_kdl)
        if ret_m != 0 or ret_c != 0 or ret_g != 0:
            self.get_logger().error(
                f'KDL solver error: JntToMass={ret_m} '
                f'JntToCoriolis={ret_c} JntToGravity={ret_g}',
                throttle_duration_sec=2.0,
            )

        # Convert to numpy
        M = np.array(
            [[M_kdl[i, j] for j in range(self.dof)] for i in range(self.dof)]
        )
        C = np.array([C_kdl[i] for i in range(self.dof)])
        G = np.array([G_kdl[i] for i in range(self.dof)])

        self.get_logger().info(
            f'[KDL] q={np.round(self.q, 3).tolist()}  '
            f'G={np.round(G, 4).tolist()}  '
            f'C={np.round(C, 4).tolist()}  '
            f'M_diag={np.round(np.diag(M), 4).tolist()}',
            throttle_duration_sec=1.0,
        )

        # Optional motor inertia correction (diagonal)
        if self._use_motor_inertia:
            M += np.diag(self._motor_inertia)

        # Per-joint gravity sign correction (use gravity_scale to flip if
        # hardware convention differs from KDL, e.g. -1.0 for a joint).
        G = G * self._gravity_scale

        # Friction compensation – scaled down in transparent mode to avoid
        # oscillations from velocity noise near zero velocity.
        friction_scale = self._transparent_friction_scale if self._transparent else 1.0
        viscous = friction_scale * self._viscous_b * self.q_dot
        coulomb = friction_scale * self._coulomb_fc * np.tanh(
            self._coulomb_scale * self.q_dot
        )

        # τ = M(q)·q̈_ref + C(q,q̇)·q̇ + G(q) + B·q̇ + Fc·tanh(α·q̇) + τ_avoid
        # In transparent mode q̈_ref=0 so the M term drops out entirely.
        tau_raw = M @ q_ddot_ref + C + G + viscous + coulomb
        tau_raw += self._joint_limit_avoidance_torque()
        self.get_logger().debug(
            f'M_diag={np.diag(M)}, G={G}, C={C}, '
            f'q_ddot_ref={q_ddot_ref}, tau_raw={tau_raw}'
        )
        return tau_raw * self._torque_scale

    # ── Helpers ─────────────────────────────────────────────────────────── #

    def _joint_limit_avoidance_torque(self) -> np.ndarray:
        """Return a restoring torque that ramps up linearly inside the buffer zone."""
        tau_avoid = np.zeros(self.dof)
        if self._limit_avoidance_gain == 0.0:
            return tau_avoid
        buf = self._limit_buffer
        for i in range(self.dof):
            q_i = self.q[i]
            lo, hi = self._pos_limits_lower[i], self._pos_limits_upper[i]
            if q_i < lo + buf:
                tau_avoid[i] = self._limit_avoidance_gain * (lo + buf - q_i)
            elif q_i > hi - buf:
                tau_avoid[i] = -self._limit_avoidance_gain * (q_i - (hi - buf))
        return tau_avoid

    def _vec_param(self, name: str) -> np.ndarray:
        raw = list(
            self.get_parameter(name).get_parameter_value().double_array_value
        )
        if len(raw) != self.dof:
            raise ValueError(
                f"Parameter '{name}' must have exactly {self.dof} values, "
                f'got {len(raw)}.'
            )
        return np.asarray(raw, dtype=np.float64)

    def _ordered(
        self,
        names,
        values,
        field: str,
        required: bool = True,
        default: float | None = None,
    ) -> np.ndarray | None:
        if len(names) == 0:
            if required:
                self.get_logger().warn(f'{field}: joint names are empty')
            return None
        if len(values) == 0:
            if default is not None:
                return np.full(self.dof, default, dtype=np.float64)
            if required:
                self.get_logger().warn(f'{field} is empty')
            return None

        name_to_idx = {n: i for i, n in enumerate(names)}
        missing = [n for n in self.joint_names if n not in name_to_idx]
        if missing:
            if required:
                self.get_logger().warn(
                    f'{field} missing joints: {missing}'
                )
            return None

        result = []
        for n in self.joint_names:
            idx = name_to_idx[n]
            if idx >= len(values):
                if default is not None:
                    result.append(default)
                    continue
                if required:
                    self.get_logger().warn(
                        f'{field}: no value for joint {n!r} at index {idx}'
                    )
                return None
            result.append(values[idx])
        return np.asarray(result, dtype=np.float64)

    def _log_ready_once(self):
        if self._ready_logged:
            return
        if not self.have_actual:
            return
        if not self._transparent and not self.have_desired:
            return
        self._ready_logged = True
        mode = 'transparent' if self._transparent else 'tracking'
        self.get_logger().info(
            f'Controller ready [{mode}]: publishing torques.'
        )


def main(args=None):
    """Run the KDL computed torque controller node."""
    rclpy.init(args=args)
    node = ComputedTorqueControllerKDL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
