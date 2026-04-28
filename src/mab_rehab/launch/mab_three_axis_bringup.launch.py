"""Bring-up launch for mab_rehab.

Thin rewrite of the old mab_ros2_control launch file. Per-joint tuning
(PID, safety, startup reference) lives in YAML and is injected here as
xacro args for the ros2_control joint params.
"""

import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def _as_bool(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _load_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}
    except OSError:
        return {}


def _config_root(config, key):
    value = config.get(key, config)
    return value if isinstance(value, dict) else {}


def _joint_config(config, joint_name):
    joints = config.get("joints", {})
    value = joints.get(joint_name, {})
    return value if isinstance(value, dict) else {}


def _append_xacro_arg(arguments, name, value):
    if isinstance(value, bool):
        value = str(value).lower()
    elif isinstance(value, (int, float)):
        value = str(value)
    arguments.extend([" ", f"{name}:=", value])


def _controller_spawner_args(controller_name, active_controller_name):
    arguments = [controller_name, "--controller-manager", "/controller_manager"]
    if controller_name != active_controller_name:
        arguments.append("--inactive")
    return arguments


def _stagger_actions(actions, initial_delay_sec, stagger_delay_sec):
    staggered = []
    for index, action in enumerate(actions):
        delay_sec = initial_delay_sec + (index * stagger_delay_sec)
        if delay_sec <= 0.0:
            staggered.append(action)
            continue
        staggered.append(TimerAction(period=delay_sec, actions=[action]))
    return staggered


def _build_launch_nodes(context, controllers_file, installed_share):
    # Resolve launch configurations once.
    enable_brake_chopper = _as_bool(
        LaunchConfiguration("enable_brake_chopper").perform(context)
    )
    pds_id = int(LaunchConfiguration("pds_id").perform(context))
    active_controller = LaunchConfiguration("active_controller").perform(context).strip().lower()
    active_controller_map = {
        "velocity": "joint_group_velocity_controller",
        "position": "joint_group_position_controller",
        "trajectory": "joint_trajectory_controller",
        "trajectory_velocity": "joint_trajectory_velocity_controller",
        "effort": "joint_group_effort_controller",
        "none": None,
    }
    active_controller_name = active_controller_map.get(active_controller)
    if active_controller_name is None and active_controller != "none":
        valid = ", ".join(active_controller_map.keys())
        raise RuntimeError(
            f"Invalid active_controller '{active_controller}'. Expected one of: {valid}"
        )

    spawn_inactive = _as_bool(
        LaunchConfiguration("spawn_inactive_controllers").perform(context))
    controller_rate_hz = int(
        LaunchConfiguration("controller_update_rate_hz").perform(context))
    controller_thread_priority = int(
        LaunchConfiguration("controller_thread_priority").perform(context))
    controller_spawn_initial_delay_sec = float(
        LaunchConfiguration("controller_spawn_initial_delay_sec").perform(context))
    controller_spawn_stagger_sec = float(
        LaunchConfiguration("controller_spawn_stagger_sec").perform(context))

    # Load the three YAML config files.
    startup_cfg = _config_root(
        _load_yaml(LaunchConfiguration("startup_reference_file").perform(context)),
        "startup_reference")
    safety_cfg = _config_root(
        _load_yaml(LaunchConfiguration("safety_limits_file").perform(context)),
        "safety_limits")
    pid_cfg = _config_root(
        _load_yaml(LaunchConfiguration("pid_tuning_file").perform(context)),
        "pid_tuning")

    # Write a tmp params file for controller_manager runtime settings.
    rate_params = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml", prefix="mab_cm_rate_")
    rate_params.write(
        "controller_manager:\n"
        "  ros__parameters:\n"
        f"    update_rate: {controller_rate_hz}\n"
        f"    thread_priority: {controller_thread_priority}\n")
    rate_params.flush()
    rate_params.close()

    # Assemble xacro args.
    xacro_args = [
        FindExecutable(name="xacro"),
        " ",
        installed_share + "/urdf/mab_three_axis.urdf.xacro",
    ]
    shared_args = {
        "bus": LaunchConfiguration("bus"),
        "data_rate": LaunchConfiguration("data_rate"),
        "use_pds": LaunchConfiguration("use_pds"),
        "use_regular_can_frames": LaunchConfiguration("use_regular_can_frames"),
        "pds_id": LaunchConfiguration("pds_id"),
        "power_stage_socket": LaunchConfiguration("power_stage_socket"),
        "fast_mode": LaunchConfiguration("fast_mode"),
        "md_id_1": LaunchConfiguration("md_id_1"),
        "md_id_2": LaunchConfiguration("md_id_2"),
        "md_id_3": LaunchConfiguration("md_id_3"),
        "md_config_1": LaunchConfiguration("md_config_1"),
        "md_config_2": LaunchConfiguration("md_config_2"),
        "md_config_3": LaunchConfiguration("md_config_3"),
        "safety_limits_enabled": safety_cfg.get("enabled", True),
        "startup_reference_enabled": startup_cfg.get("enabled", True),
    }
    for name, value in shared_args.items():
        _append_xacro_arg(xacro_args, name, value)

    # Per-joint: startup reference, safety envelope, PID/impedance gains.
    for index, joint_name in enumerate(("joint_1", "joint_2", "joint_3"), start=1):
        startup_joint = _joint_config(startup_cfg, joint_name)
        safety_joint = _joint_config(safety_cfg, joint_name)
        pid_joint = _joint_config(pid_cfg, joint_name)

        if "reference_position" in startup_joint:
            _append_xacro_arg(
                xacro_args, f"startup_reference_position_{index}",
                startup_joint["reference_position"])

        # Narrow the URDF joint limits if the safety envelope is tighter.
        if "position_min" in safety_joint:
            _append_xacro_arg(xacro_args, f"joint_{index}_lower", safety_joint["position_min"])
            _append_xacro_arg(
                xacro_args, f"safety_position_min_{index}", safety_joint["position_min"])
        if "position_max" in safety_joint:
            _append_xacro_arg(xacro_args, f"joint_{index}_upper", safety_joint["position_max"])
            _append_xacro_arg(
                xacro_args, f"safety_position_max_{index}", safety_joint["position_max"])
        if "max_velocity" in safety_joint:
            _append_xacro_arg(xacro_args, f"joint_{index}_velocity", safety_joint["max_velocity"])
            _append_xacro_arg(
                xacro_args, f"safety_max_velocity_{index}", safety_joint["max_velocity"])
        if "max_torque" in safety_joint:
            _append_xacro_arg(xacro_args, f"joint_{index}_effort", safety_joint["max_torque"])
            _append_xacro_arg(
                xacro_args, f"safety_max_torque_{index}", safety_joint["max_torque"])
        if "max_acceleration" in safety_joint:
            _append_xacro_arg(
                xacro_args, f"safety_max_acceleration_{index}",
                safety_joint["max_acceleration"])
        if "max_deceleration" in safety_joint:
            _append_xacro_arg(
                xacro_args, f"safety_max_deceleration_{index}",
                safety_joint["max_deceleration"])

        position_pid = pid_joint.get("position_pid", {})
        if isinstance(position_pid, dict):
            for key in ("kp", "ki", "kd", "windup"):
                if key in position_pid:
                    _append_xacro_arg(
                        xacro_args, f"position_pid_{key}_{index}", position_pid[key])

        velocity_pid = pid_joint.get("velocity_pid", {})
        if isinstance(velocity_pid, dict):
            for key in ("kp", "ki", "kd", "windup"):
                if key in velocity_pid:
                    _append_xacro_arg(
                        xacro_args, f"velocity_pid_{key}_{index}", velocity_pid[key])

        impedance = pid_joint.get("impedance", {})
        if isinstance(impedance, dict):
            for key in ("kp", "kd"):
                if key in impedance:
                    _append_xacro_arg(
                        xacro_args, f"impedance_{key}_{index}", impedance[key])

        friction = pid_joint.get("friction", {})
        if isinstance(friction, dict):
            for key in ("motor_friction", "motor_stiction"):
                if key in friction:
                    _append_xacro_arg(
                        xacro_args, f"{key}_{index}", friction[key])


    robot_description = {"robot_description": Command(xacro_args)}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            rate_params.name,
            {"inactive": True},
        ],
        additional_env={
            "MAB_SPI_SPEED_HZ": LaunchConfiguration("spi_speed_hz").perform(context),
            "MAB_SPI_PATH": LaunchConfiguration("spi_path").perform(context),
        },
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_trajectory_controller", active_controller_name),
        output="screen",
    )
    joint_trajectory_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_trajectory_velocity_controller", active_controller_name),
        output="screen",
    )
    joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_position_controller", active_controller_name),
        output="screen",
    )
    joint_group_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_velocity_controller", active_controller_name),
        output="screen",
    )
    joint_group_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_effort_controller", active_controller_name),
        output="screen",
    )

    controller_nodes = [joint_state_broadcaster]
    all_command_controllers = [
        joint_trajectory_controller,
        joint_trajectory_velocity_controller,
        joint_group_position_controller,
        joint_group_velocity_controller,
        joint_group_effort_controller,
    ]
    if spawn_inactive:
        controller_nodes.extend(all_command_controllers)
    elif active_controller_name == "joint_trajectory_controller":
        controller_nodes.append(joint_trajectory_controller)
    elif active_controller_name == "joint_trajectory_velocity_controller":
        controller_nodes.append(joint_trajectory_velocity_controller)
    elif active_controller_name == "joint_group_position_controller":
        controller_nodes.append(joint_group_position_controller)
    elif active_controller_name == "joint_group_velocity_controller":
        controller_nodes.append(joint_group_velocity_controller)
    elif active_controller_name == "joint_group_effort_controller":
        controller_nodes.append(joint_group_effort_controller)

    staggered_controller_nodes = _stagger_actions(
        controller_nodes,
        controller_spawn_initial_delay_sec,
        controller_spawn_stagger_sec,
    )

    launch_nodes = [robot_state_publisher, control_node, *staggered_controller_nodes]

    if enable_brake_chopper:
        launch_nodes.append(
            Node(
                package="mab_brake_chopper",
                executable="brake_chopper_node",
                name="brake_chopper",
                output="screen",
                parameters=[
                    {
                        "pds_id": pds_id,
                        "voltage_source": LaunchConfiguration(
                            "brake_voltage_source"
                        ).perform(context),
                        "trigger_voltage_v": float(
                            LaunchConfiguration("brake_trigger_voltage_v").perform(context)
                        ),
                        "telemetry_timeout_sec": float(
                            LaunchConfiguration(
                                "brake_telemetry_timeout_sec"
                            ).perform(context)
                        ),
                        "gpio_pin": int(LaunchConfiguration("brake_gpio_pin").perform(context)),
                        "gpio_backend": LaunchConfiguration(
                            "brake_gpio_backend"
                        ).perform(context),
                        "gpio_chip_label": LaunchConfiguration(
                            "brake_gpio_chip_label"
                        ).perform(context),
                        "gpio_chip_path": LaunchConfiguration(
                            "brake_gpio_chip_path"
                        ).perform(context),
                        "control_module_topic": f"pds/id_{pds_id}/control_module",
                        "dynamic_joint_states_topic": "/dynamic_joint_states",
                        "dynamic_joint_name": "mab_power_stage",
                        "dynamic_interface_name": "bus_voltage",
                    }
                ],
            )
        )

    return launch_nodes


def generate_launch_description():
    package_share = FindPackageShare("mab_rehab")
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "mab_three_axis.controllers.yaml"])
    installed_share = get_package_share_directory("mab_rehab")

    return LaunchDescription([
        DeclareLaunchArgument("bus", default_value="SPI"),
        DeclareLaunchArgument("data_rate", default_value="5M"),
        DeclareLaunchArgument("spi_speed_hz", default_value="12500000"),
        DeclareLaunchArgument("spi_path", default_value="/dev/spidev0.0"),
        DeclareLaunchArgument("use_pds", default_value="true"),
        DeclareLaunchArgument("use_regular_can_frames", default_value="false"),
        DeclareLaunchArgument("pds_id", default_value="100"),
        DeclareLaunchArgument("power_stage_socket", default_value="2"),
        DeclareLaunchArgument("fast_mode", default_value="false"),
        DeclareLaunchArgument("md_id_1", default_value="37"),
        DeclareLaunchArgument("md_id_2", default_value="940"),
        DeclareLaunchArgument("md_id_3", default_value="941"),
        DeclareLaunchArgument(
            "md_config_1", default_value=installed_share + "/config/motors/joint_1.cfg"),
        DeclareLaunchArgument(
            "md_config_2", default_value=installed_share + "/config/motors/joint_2.cfg"),
        DeclareLaunchArgument(
            "md_config_3", default_value=installed_share + "/config/motors/joint_3.cfg"),

        DeclareLaunchArgument("controller_update_rate_hz", default_value="100"),
        DeclareLaunchArgument("controller_thread_priority", default_value="99"),
        DeclareLaunchArgument(
            "startup_reference_file",
            default_value=installed_share + "/config/startup_reference.yaml"),
        DeclareLaunchArgument(
            "safety_limits_file",
            default_value=installed_share + "/config/safety_limits.yaml"),
        DeclareLaunchArgument(
            "pid_tuning_file",
            default_value=installed_share + "/config/pid_tuning.yaml"),
        DeclareLaunchArgument("active_controller", default_value="trajectory"),
        DeclareLaunchArgument("spawn_inactive_controllers", default_value="true"),
        DeclareLaunchArgument("controller_spawn_initial_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("controller_spawn_stagger_sec", default_value="1.0"),
        DeclareLaunchArgument("enable_brake_chopper", default_value="true"),
        DeclareLaunchArgument("brake_voltage_source", default_value="dynamic_joint_state"),
        DeclareLaunchArgument("brake_trigger_voltage_v", default_value="25.0"),
        DeclareLaunchArgument("brake_telemetry_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("brake_gpio_pin", default_value="17"),
        DeclareLaunchArgument("brake_gpio_backend", default_value="linux_char"),
        DeclareLaunchArgument("brake_gpio_chip_label", default_value="pinctrl-rp1"),
        DeclareLaunchArgument("brake_gpio_chip_path", default_value=""),
        OpaqueFunction(
            function=lambda context: _build_launch_nodes(
                context, controllers_file, installed_share)),
    ])
