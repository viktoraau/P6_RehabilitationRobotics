"""Computed torque controller node with packaged symbolic dynamics."""

import re
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray

from trajectory_msgs.msg import JointTrajectory


_PACKAGE_DIR = Path(__file__).resolve().parent
_DEFAULT_DYNAMIC_MODEL_DIR = _PACKAGE_DIR / 'dynamic_matrices'
_SAFE_EVAL_GLOBALS = {
    '__builtins__': {},
    'sin': np.sin,
    'cos': np.cos,
    'tanh': np.tanh,
    'pi': np.pi,
}


def _normalize_expression(expression):
    return expression.strip().replace('^', '**')


def _build_symbol_env(q, q_dot):
    env = {}
    for index, value in enumerate(q, start=1):
        env[f'theta{index}'] = float(value)
    for index, value in enumerate(q_dot, start=1):
        env[f'dtheta{index}'] = float(value)
    return env


class _SymbolicArray:
    def __init__(self, expressions, source_name):
        self.source_name = source_name
        self.shape = expressions.shape
        self._compiled = np.empty(self.shape, dtype=object)

        for index, expression in np.ndenumerate(expressions):
            self._compiled[index] = compile(
                _normalize_expression(expression),
                f'{source_name}{index}',
                'eval',
            )

    def evaluate(self, q, q_dot):
        env = _build_symbol_env(q, q_dot)
        values = np.empty(self.shape, dtype=np.float64)
        for index, code in np.ndenumerate(self._compiled):
            values[index] = float(eval(code, _SAFE_EVAL_GLOBALS, env))
        return values


def _load_symbolic_array(file_path, prefix, dof, shape_kind):
    entries = {}
    lines = file_path.read_text(encoding='utf-8').splitlines()

    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()
        if not line:
            continue

        if '=' not in line:
            raise ValueError(
                f'{file_path}:{line_number} is missing "=": {raw_line}'
            )

        name, expression = (part.strip() for part in line.split('=', 1))
        if shape_kind == 'matrix':
            match = re.fullmatch(rf'{prefix}(\d+)(\d+)', name)
            if match is None:
                raise ValueError(
                    f'{file_path}:{line_number} has invalid '
                    f'matrix entry {name!r}'
                )
            row, column = (int(value) - 1 for value in match.groups())
            entries[(row, column)] = expression
        else:
            match = re.fullmatch(rf'{prefix}(\d+)', name)
            if match is None:
                raise ValueError(
                    f'{file_path}:{line_number} has invalid '
                    f'vector entry {name!r}'
                )
            row = int(match.group(1)) - 1
            entries[(row,)] = expression

    if shape_kind == 'matrix':
        expressions = np.empty((dof, dof), dtype=object)
        expected_indices = [
            (row, column)
            for row in range(dof)
            for column in range(dof)
        ]
    else:
        expressions = np.empty((dof,), dtype=object)
        expected_indices = [(row,) for row in range(dof)]

    missing_indices = [
        index for index in expected_indices if index not in entries
    ]
    if missing_indices:
        raise ValueError(f'{file_path} is missing entries: {missing_indices}')

    for index in expected_indices:
        expressions[index] = entries[index]

    return _SymbolicArray(expressions, file_path.name)


class ComputedTorqueController(Node):
    """Computed torque controller for the three-joint rehabilitation arm."""

    def __init__(self):
        """Initialize subscriptions, model loading, and the torque loop."""
        super().__init__('computed_torque_controller')

        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter(
            'desired_trajectory_topic',
            '/joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter(
            'torque_command_topic',
            '/joint_group_effort_controller/commands',
        )
        self.declare_parameter(
            'joint_names',
            ['joint_1', 'joint_2', 'joint_3'],
        )
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('kp', [40.0, 40.0, 40.0])
        self.declare_parameter('kd', [12.0, 12.0, 12.0])
        self.declare_parameter('use_motor_inertia', True)
        self.declare_parameter(
            'dynamic_model_directory',
            str(_DEFAULT_DYNAMIC_MODEL_DIR),
        )

        self.joint_state_topic = (
            self.get_parameter('joint_state_topic')
            .get_parameter_value()
            .string_value
        )
        self.desired_trajectory_topic = (
            self.get_parameter('desired_trajectory_topic')
            .get_parameter_value()
            .string_value
        )
        self.torque_command_topic = (
            self.get_parameter('torque_command_topic')
            .get_parameter_value()
            .string_value
        )
        self.joint_names = list(
            self.get_parameter('joint_names')
            .get_parameter_value()
            .string_array_value
        )
        self.dof = len(self.joint_names)
        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )
        self.kp = self._read_vector_parameter('kp')
        self.kd = self._read_vector_parameter('kd')
        self.use_motor_inertia = bool(
            self.get_parameter('use_motor_inertia').value
        )
        self.dynamic_model_directory = Path(
            self.get_parameter('dynamic_model_directory').value
        )

        if self.dof != 3:
            raise ValueError(
                'The packaged computed torque model currently supports '
                'exactly 3 joints '
                f'but joint_names has {self.dof} entries.'
            )

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
        self.last_tau = np.zeros(self.dof)

        # --- Dynamic model ---
        self.mass_model = _load_symbolic_array(
            self.dynamic_model_directory / 'M_Values.txt',
            'M',
            self.dof,
            'matrix',
        )
        self.coriolis_model = _load_symbolic_array(
            self.dynamic_model_directory / 'C_Values.txt',
            'C',
            self.dof,
            'matrix',
        )
        self.gravity_model = _load_symbolic_array(
            self.dynamic_model_directory / 'G_Values.txt',
            'G',
            self.dof,
            'vector',
        )
        self.viscous_friction_model = _load_symbolic_array(
            self.dynamic_model_directory / 'B_Values.txt',
            'B',
            self.dof,
            'matrix',
        )
        self.coulomb_friction_model = _load_symbolic_array(
            self.dynamic_model_directory / 'Fc_Values.txt',
            'Fc',
            self.dof,
            'vector',
        )
        self.motor_inertia_model = _load_symbolic_array(
            self.dynamic_model_directory / 'Jm_Values.txt',
            'Jm',
            self.dof,
            'matrix',
        )

        # --- Subscribers ---
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            10,
        )

        self.create_subscription(
            JointTrajectory,
            self.desired_trajectory_topic,
            self._desired_trajectory_callback,
            10,
        )

        self.torque_command_publisher = self.create_publisher(
            Float64MultiArray,
            self.torque_command_topic,
            10,
        )

        control_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.control_timer = self.create_timer(
            control_period,
            self._control_callback,
        )

        self.get_logger().info(
            'Computed torque controller started. '
            f'actual={self.joint_state_topic!r}, '
            f'desired={self.desired_trajectory_topic!r}, '
            f'command={self.torque_command_topic!r}, '
            f'joints={self.joint_names}, '
            f'model_dir={str(self.dynamic_model_directory)!r}',
        )

    # --- Callbacks ---

    def _joint_state_callback(self, msg):
        q = self._ordered_values(msg.name, msg.position, 'JointState.position')
        q_dot = self._ordered_values(
            msg.name,
            msg.velocity,
            'JointState.velocity',
        )

        if q is None or q_dot is None:
            return

        self.q = q
        self.q_dot = q_dot
        self.q_effort = self._ordered_values(
            msg.name,
            msg.effort,
            'JointState.effort',
            required=False,
        )
        self.have_actual_state = True
        self._log_ready_once()

    def _desired_trajectory_callback(self, msg):
        if not msg.points:
            self.get_logger().warn('Desired trajectory message has no points')
            return

        point = msg.points[0]

        q_des = self._ordered_values(
            msg.joint_names,
            point.positions,
            'JointTrajectoryPoint.positions',
        )
        q_dot_des = self._ordered_values(
            msg.joint_names,
            point.velocities,
            'JointTrajectoryPoint.velocities',
            required=False,
            default_value=0.0,
        )
        q_ddot_des = self._ordered_values(
            msg.joint_names,
            point.accelerations,
            'JointTrajectoryPoint.accelerations',
            required=False,
            default_value=0.0,
        )

        if q_des is None:
            return

        self.q_des = q_des
        self.q_dot_des = q_dot_des
        self.q_ddot_des = q_ddot_des
        self.have_desired_state = True
        self._log_ready_once()

    def _control_callback(self):
        if not self.have_actual_state or not self.have_desired_state:
            return

        tau = self._compute_control_torque()
        self.last_tau = tau

        command = Float64MultiArray()
        command.data = tau.tolist()
        self.torque_command_publisher.publish(command)

    # --- Helpers ---

    def _read_vector_parameter(self, name):
        raw = self.get_parameter(name).value
        if len(raw) != self.dof:
            raise ValueError(
                f"Parameter '{name}' must contain exactly {self.dof} values."
            )
        return np.asarray(raw, dtype=np.float64)

    def _ordered_values(
        self,
        names,
        values,
        field_name,
        required=True,
        default_value=None,
    ):
        if len(names) == 0:
            if required:
                self.get_logger().warn(
                    f'{field_name} cannot be mapped: joint names are empty'
                )
            return None

        if len(values) == 0:
            if default_value is not None:
                return np.full(self.dof, default_value, dtype=np.float64)
            if required:
                self.get_logger().warn(f'{field_name} is empty')
            return None

        name_to_index = {name: index for index, name in enumerate(names)}
        missing_joints = [
            name for name in self.joint_names if name not in name_to_index
        ]
        if missing_joints:
            if required:
                self.get_logger().warn(
                    f'{field_name} missing expected joints: {missing_joints}'
                )
            return None

        ordered_values = []
        for name in self.joint_names:
            index = name_to_index[name]
            if index >= len(values):
                if default_value is not None:
                    ordered_values.append(default_value)
                    continue
                if required:
                    self.get_logger().warn(
                        f'{field_name} has no value for joint '
                        f"'{name}' at index {index}"
                    )
                return None
            ordered_values.append(values[index])

        return np.asarray(ordered_values, dtype=np.float64)

    def _compute_control_torque(self):
        position_error = self.q_des - self.q
        velocity_error = self.q_dot_des - self.q_dot
        reference_acceleration = (
            self.q_ddot_des
            + self.kd * velocity_error
            + self.kp * position_error
        )

        mass_matrix = self.mass_model.evaluate(self.q, self.q_dot)
        if self.use_motor_inertia:
            mass_matrix = mass_matrix + self.motor_inertia_model.evaluate(
                self.q,
                self.q_dot,
            )

        coriolis_matrix = self.coriolis_model.evaluate(self.q, self.q_dot)
        gravity_vector = self.gravity_model.evaluate(self.q, self.q_dot)
        viscous_term = self._apply_velocity_dependent_term(
            self.viscous_friction_model.evaluate(self.q, self.q_dot),
            self.q_dot,
        )
        coulomb_term = self.coulomb_friction_model.evaluate(self.q, self.q_dot)

        return (
            mass_matrix @ reference_acceleration
            + coriolis_matrix @ self.q_dot
            + gravity_vector
            + viscous_term
            + coulomb_term
        )

    def _apply_velocity_dependent_term(self, coefficients, velocity):
        if coefficients.ndim == 1:
            return coefficients * velocity

        diagonal = np.diag(coefficients)
        if np.allclose(coefficients, np.diag(diagonal)):
            return diagonal * velocity

        return coefficients @ velocity

    def _log_ready_once(self):
        if self.ready_logged:
            return

        if not self.have_actual_state or not self.have_desired_state:
            return

        self.ready_logged = True
        self.get_logger().info(
            'Controller ready: received actual joint state and desired '
            f'trajectory, publishing torques to '
            f'{self.torque_command_topic!r}.'
        )


def main(args=None):
    """Run the computed torque controller node."""
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
