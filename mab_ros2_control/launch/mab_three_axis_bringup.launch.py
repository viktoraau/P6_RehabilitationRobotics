from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import tempfile
import xml.etree.ElementTree as ET


def _as_bool(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _controller_spawner_args(controller_name, active_controller_name):
    arguments = [controller_name, "--controller-manager", "/controller_manager"]
    if controller_name != active_controller_name:
        arguments.append("--inactive")
    return arguments


def _build_launch_nodes(context, controllers_file, installed_share):
    use_xacro = _as_bool(LaunchConfiguration("use_xacro").perform(context))
    fast_mode = LaunchConfiguration("fast_mode").perform(context)
    telemetry_divider = LaunchConfiguration("telemetry_divider").perform(context)
    pds_id = LaunchConfiguration("pds_id").perform(context)
    enable_brake_chopper = _as_bool(
        LaunchConfiguration("enable_brake_chopper").perform(context)
    )
    power_stage_telemetry_divider = "1" if enable_brake_chopper else telemetry_divider
    spawn_inactive_controllers = _as_bool(
        LaunchConfiguration("spawn_inactive_controllers").perform(context)
    )
    controller_update_rate_hz = int(
        LaunchConfiguration("controller_update_rate_hz").perform(context)
    )
    update_rate_params = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml", prefix="mab_cm_rate_"
    )
    update_rate_params.write(
        "controller_manager:\n"
        "  ros__parameters:\n"
        f"    update_rate: {controller_update_rate_hz}\n"
    )
    update_rate_params.flush()
    update_rate_params.close()
    active_controller = LaunchConfiguration("active_controller").perform(context).strip().lower()
    active_controller_map = {
        "velocity": "joint_group_velocity_controller",
        "position": "joint_group_position_controller",
        "trajectory": "joint_trajectory_controller",
        "effort": "joint_group_effort_controller",
        "none": None,
        "joint_group_velocity_controller": "joint_group_velocity_controller",
        "joint_group_position_controller": "joint_group_position_controller",
        "joint_trajectory_controller": "joint_trajectory_controller",
        "joint_group_effort_controller": "joint_group_effort_controller",
    }
    active_controller_name = active_controller_map.get(active_controller)
    if active_controller_name is None and active_controller != "none":
        valid_values = ", ".join(["velocity", "position", "trajectory", "effort", "none"])
        raise RuntimeError(
            f"Invalid active_controller '{active_controller}'. Expected one of: {valid_values}"
        )

    if use_xacro:
        robot_description = {
            "robot_description": Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    installed_share + "/urdf/mab_three_axis.urdf.xacro",
                    " ",
                    "bus:=",
                    LaunchConfiguration("bus"),
                    " ",
                    "data_rate:=",
                    LaunchConfiguration("data_rate"),
                    " ",
                    "use_pds:=",
                    LaunchConfiguration("use_pds"),
                    " ",
                    "use_regular_can_frames:=",
                    LaunchConfiguration("use_regular_can_frames"),
                    " ",
                    "pds_id:=",
                    LaunchConfiguration("pds_id"),
                    " ",
                    "power_stage_socket:=",
                    LaunchConfiguration("power_stage_socket"),
                    " ",
                    "telemetry_divider:=",
                    power_stage_telemetry_divider,
                    " ",
                    "md_id_1:=",
                    LaunchConfiguration("md_id_1"),
                    " ",
                    "md_id_2:=",
                    LaunchConfiguration("md_id_2"),
                    " ",
                    "md_id_3:=",
                    LaunchConfiguration("md_id_3"),
                    " ",
                    "fast_mode:=",
                    LaunchConfiguration("fast_mode"),
                ]
            )
        }
    else:
        urdf_path = installed_share + "/urdf/mab_three_axis.default.urdf"
        urdf_tree = ET.parse(urdf_path)
        urdf_root = urdf_tree.getroot()
        hardware = urdf_root.find("./ros2_control[@name='mab_hardware']/hardware")
        overrides = {
            "bus": LaunchConfiguration("bus").perform(context),
            "data_rate": LaunchConfiguration("data_rate").perform(context),
            "use_pds": LaunchConfiguration("use_pds").perform(context),
            "use_regular_can_frames": LaunchConfiguration("use_regular_can_frames").perform(context),
            "pds_id": pds_id,
            "power_stage_socket": LaunchConfiguration("power_stage_socket").perform(context),
            "telemetry_divider": power_stage_telemetry_divider,
            "fast_mode": fast_mode,
        }
        joint_can_ids = {
            "joint_1": LaunchConfiguration("md_id_1").perform(context),
            "joint_2": LaunchConfiguration("md_id_2").perform(context),
            "joint_3": LaunchConfiguration("md_id_3").perform(context),
        }

        if hardware is not None:
            for param_name, value in overrides.items():
                param = hardware.find(f"./param[@name='{param_name}']")
                if param is not None:
                    param.text = value

            for joint_name, can_id in joint_can_ids.items():
                joint = urdf_root.find(
                    f"./ros2_control[@name='mab_hardware']/joint[@name='{joint_name}']"
                )
                if joint is None:
                    continue
                param = joint.find("./param[@name='can_id']")
                if param is not None:
                    param.text = can_id

        robot_description = {"robot_description": ET.tostring(urdf_root, encoding="unicode")}

    control_node = Node(
         package="controller_manager",
         executable="ros2_control_node",
         parameters=[
             robot_description,
             controllers_file,
             update_rate_params.name,
             {"inactive": True},
         ],
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
                   "--controller-manager", 
                   "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_trajectory_controller", active_controller_name
        ),
        output="screen",
    )

    joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_position_controller", active_controller_name
        ),
        output="screen",
    )

    joint_group_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_velocity_controller", active_controller_name
        ),
        output="screen",
    )

    joint_group_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=_controller_spawner_args(
            "joint_group_effort_controller", active_controller_name
        ),
        output="screen",
    )

    controller_nodes = [
        joint_state_broadcaster,
    ]
    all_command_controller_nodes = [
        joint_trajectory_controller,
        joint_group_position_controller,
        joint_group_velocity_controller,
        joint_group_effort_controller,
    ]

    if spawn_inactive_controllers:
        controller_nodes.extend(all_command_controller_nodes)
    elif active_controller_name == "joint_trajectory_controller":
        controller_nodes.append(joint_trajectory_controller)
    elif active_controller_name == "joint_group_position_controller":
        controller_nodes.append(joint_group_position_controller)
    elif active_controller_name == "joint_group_velocity_controller":
        controller_nodes.append(joint_group_velocity_controller)
    elif active_controller_name == "joint_group_effort_controller":
        controller_nodes.append(joint_group_effort_controller)

    launch_nodes = [
        robot_state_publisher,
        control_node,
        *controller_nodes,
    ]

    if enable_brake_chopper:
        launch_nodes.append(
            Node(
                package="mab_brake_chopper",
                executable="brake_chopper_node",
                name="brake_chopper",
                output="screen",
                parameters=[
                    {
                        "pds_id": int(pds_id),
                        "voltage_source": LaunchConfiguration("brake_voltage_source").perform(context),
                        "trigger_voltage_v": float(
                            LaunchConfiguration("brake_trigger_voltage_v").perform(context)
                        ),
                        "telemetry_timeout_sec": float(
                            LaunchConfiguration("brake_telemetry_timeout_sec").perform(context)
                        ),
                        "gpio_pin": int(
                            LaunchConfiguration("brake_gpio_pin").perform(context)
                        ),
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
    package_share = FindPackageShare("mab_ros2_control")
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "mab_three_axis.controllers.yaml"]
    )
    installed_share = get_package_share_directory("mab_ros2_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bus", default_value="SPI"),
            DeclareLaunchArgument("data_rate", default_value="5M"),
            DeclareLaunchArgument("use_pds", default_value="true"),
            DeclareLaunchArgument("use_regular_can_frames", default_value="false"),
            DeclareLaunchArgument("pds_id", default_value="100"),
            DeclareLaunchArgument("power_stage_socket", default_value="2"),
            DeclareLaunchArgument("telemetry_divider", default_value="50"),
            DeclareLaunchArgument("md_id_1", default_value="0"),
            DeclareLaunchArgument("md_id_2", default_value="37"),
            DeclareLaunchArgument("md_id_3", default_value="941"),
            DeclareLaunchArgument("fast_mode", default_value="false"),
            DeclareLaunchArgument("controller_update_rate_hz", default_value="100"),
            DeclareLaunchArgument("use_xacro", default_value="false"),
            DeclareLaunchArgument("active_controller", default_value="none"),
            DeclareLaunchArgument("spawn_inactive_controllers", default_value="true"),
            DeclareLaunchArgument("enable_brake_chopper", default_value="true"),
            DeclareLaunchArgument("brake_voltage_source", default_value="dynamic_joint_state"),
            DeclareLaunchArgument("brake_trigger_voltage_v", default_value="12.0"),
            DeclareLaunchArgument("brake_telemetry_timeout_sec", default_value="0.50"),
            DeclareLaunchArgument("brake_gpio_pin", default_value="17"),
            DeclareLaunchArgument("brake_gpio_backend", default_value="linux_char"),
            DeclareLaunchArgument("brake_gpio_chip_label", default_value="pinctrl-rp1"),
            DeclareLaunchArgument("brake_gpio_chip_path", default_value=""),
            OpaqueFunction(
                function=lambda context: _build_launch_nodes(
                    context, controllers_file, installed_share
                )
            ),
        ]
    )
