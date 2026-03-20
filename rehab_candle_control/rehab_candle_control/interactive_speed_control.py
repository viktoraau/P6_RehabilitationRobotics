import select
import sys
import termios
import tty
import ast
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from candle_ros2.msg import ControlModuleData, MotionCmd, PowerStageData
from candle_ros2.srv import AddDevices, Generic, GenericPds, SetMode


class InteractiveSpeedControl(Node):
    def __init__(self) -> None:
        super().__init__("interactive_speed_control")

        self.declare_parameter("pds_id", 100)
        self.declare_parameter("power_stage_socket", 2)
        self.declare_parameter("md_ids", [37, 941, 939])
        self.declare_parameter("speed_step", 0.5)
        self.declare_parameter("max_speed", 8.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("telemetry_rate_hz", 10.0)

        self.pds_id = int(self.get_parameter("pds_id").value)
        self.power_stage_socket = int(self.get_parameter("power_stage_socket").value)
        raw_md_ids = self.get_parameter("md_ids").value
        self.md_ids = self._parse_md_ids(raw_md_ids)
        if not self.md_ids:
            raise RuntimeError("md_ids parameter is empty")
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.telemetry_rate_hz = float(self.get_parameter("telemetry_rate_hz").value)

        self.speeds: Dict[int, float] = {md_id: 0.0 for md_id in self.md_ids}

        self.motion_pub = self.create_publisher(MotionCmd, "md/motion_command", 10)
        self.pds_telemetry_pub = self.create_publisher(
            Float64MultiArray, "rehab/telemetry/pds", 10
        )
        self.joint_telemetry_pubs: Dict[int, object] = {
            md_id: self.create_publisher(
                Float64MultiArray, f"rehab/telemetry/joint_{md_id}", 10
            )
            for md_id in self.md_ids
        }

        self.joint_state_sub = self.create_subscription(
            JointState,
            "md/joint_states",
            self._on_joint_states,
            10,
        )

        self.control_module_sub = self.create_subscription(
            ControlModuleData,
            f"pds/id_{self.pds_id}/control_module",
            self._on_control_module,
            10,
        )

        self.power_stage_sub = self.create_subscription(
            PowerStageData,
            f"pds/id_{self.pds_id}/power_stage_{self.power_stage_socket}",
            self._on_power_stage,
            10,
        )

        self.last_bus_voltage_v = None
        self.last_output_voltage_v = None
        self.last_load_current_a = None
        self.last_motor_state: Dict[int, Dict[str, float]] = {}

        self.add_pds_client = self.create_client(AddDevices, "/pds/add_pds")
        self.add_mds_client = self.create_client(AddDevices, "/md/add_mds")
        self.set_mode_client = self.create_client(SetMode, "/md/set_mode")
        self.enable_md_client = self.create_client(Generic, "/md/enable")

        power_stage_service = (
            f"/pds/id_{self.pds_id}/enable_ps_{self.power_stage_socket}"
        )
        self.enable_power_stage_client = self.create_client(GenericPds, power_stage_service)

        self._stdin_fd = None
        self._old_term_settings = None
        self._tty_file = None

        self._setup_complete = False
        self._setup_stage = "wait_services"
        self._setup_future = None
        self._setup_wait_log_once = False
        self._setup_wait_tick = 0
        self._required_services = {
            "/pds/add_pds": self.add_pds_client,
            power_stage_service: self.enable_power_stage_client,
            "/md/add_mds": self.add_mds_client,
            "/md/set_mode": self.set_mode_client,
            "/md/enable": self.enable_md_client,
        }
        self._print_controls_once()

        setup_timer_period = 0.5
        motion_timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        telemetry_timer_period = 1.0 / max(self.telemetry_rate_hz, 1.0)
        keyboard_timer_period = 0.05

        self.setup_timer = self.create_timer(setup_timer_period, self._try_setup)
        self.motion_timer = self.create_timer(motion_timer_period, self._publish_motion)
        self.telemetry_timer = self.create_timer(telemetry_timer_period, self._publish_telemetry)
        self.keyboard_timer = self.create_timer(keyboard_timer_period, self._poll_keyboard)

    def _print_controls_once(self) -> None:
        self.get_logger().info("Controls:")
        if len(self.md_ids) >= 1:
            self.get_logger().info(f"1/q -> motor {self.md_ids[0]} speed +/-")
        if len(self.md_ids) >= 2:
            self.get_logger().info(f"2/w -> motor {self.md_ids[1]} speed +/-")
        if len(self.md_ids) >= 3:
            self.get_logger().info(f"3/e -> motor {self.md_ids[2]} speed +/-")
        self.get_logger().info("space -> stop all motors")
        self.get_logger().info("x -> disable motors and quit")

    def _parse_md_ids(self, raw_value) -> List[int]:
        if isinstance(raw_value, list):
            return [int(x) for x in raw_value]

        if isinstance(raw_value, str):
            try:
                parsed = ast.literal_eval(raw_value)
            except (ValueError, SyntaxError) as exc:
                raise RuntimeError(f"Failed to parse md_ids parameter: {raw_value}") from exc

            if not isinstance(parsed, list):
                raise RuntimeError("md_ids must be a list")
            return [int(x) for x in parsed]

        raise RuntimeError("md_ids must be a list or YAML list string")

    def _try_setup(self) -> None:
        if self._setup_complete:
            return

        if self._setup_future is not None:
            return

        if self._setup_stage == "wait_services":
            if not self._setup_wait_log_once:
                self.get_logger().info("Waiting for PDS/MD services to become available...")
                self.get_logger().info(
                    "Expected services: " + ", ".join(self._required_services.keys())
                )
                self._setup_wait_log_once = True
            self._setup_wait_tick += 1

            ready = True
            ready = ready and self._wait_for_service(self.add_pds_client, "pds/add_pds")
            ready = ready and self._wait_for_service(
                self.enable_power_stage_client,
                f"pds/id_{self.pds_id}/enable_ps_{self.power_stage_socket}",
            )
            ready = ready and self._wait_for_service(self.add_mds_client, "md/add_mds")
            ready = ready and self._wait_for_service(self.set_mode_client, "md/set_mode")
            ready = ready and self._wait_for_service(self.enable_md_client, "md/enable")

            self.get_logger().info("All services ready. Starting setup sequence.")
            self._setup_stage = "add_pds"

        if self._setup_stage == "add_pds":
            req = AddDevices.Request()
            req.device_ids = [self.pds_id]
            self._start_setup_call(self.add_pds_client, req)
            return

        if self._setup_stage == "enable_power_stage":
            req = GenericPds.Request()
            self._start_setup_call(self.enable_power_stage_client, req)
            return

        if self._setup_stage == "add_mds":
            req = AddDevices.Request()
            req.device_ids = self.md_ids
            self._start_setup_call(self.add_mds_client, req)
            return

        if self._setup_stage == "set_mode":
            req = SetMode.Request()
            req.device_ids = self.md_ids
            req.mode = ["VELOCITY_PID"] * len(self.md_ids)
            self._start_setup_call(self.set_mode_client, req)
            return

        if self._setup_stage == "enable_mds":
            req = Generic.Request()
            req.device_ids = self.md_ids
            self._start_setup_call(self.enable_md_client, req)
            return

    def _start_setup_call(self, client, req) -> None:
        self._setup_future = client.call_async(req)
        self._setup_future.add_done_callback(self._on_setup_call_done)

    def _on_setup_call_done(self, future) -> None:
        self._setup_future = None

        try:
            rsp = future.result()
        except Exception as exc:
            self.get_logger().error(f"Setup stage '{self._setup_stage}' failed with exception: {exc}")
            return

        if rsp is None:
            self.get_logger().error(f"Setup stage '{self._setup_stage}' returned no response")
            return

        if self._setup_stage == "add_pds":
            if not rsp.success or not rsp.success[0]:
                self.get_logger().error(f"Failed to add PDS device {self.pds_id}: {rsp.success}")
                return
            self.get_logger().info("✓ PDS device added")
            self._setup_stage = "enable_power_stage"
            return

        if self._setup_stage == "enable_power_stage":
            if not rsp.success or not rsp.success[0]:
                self.get_logger().error(
                    f"Failed to enable power stage at socket {self.power_stage_socket}: {rsp.success}"
                )
                return
            self.get_logger().info("✓ Power stage enabled")
            self._setup_stage = "add_mds"
            return

        if self._setup_stage == "add_mds":
            if len(rsp.success) != len(self.md_ids) or not all(rsp.success):
                self.get_logger().error(f"Failed to add all MD devices: {rsp.success}")
                return
            self.get_logger().info("✓ MD motors added")
            self._setup_stage = "set_mode"
            return

        if self._setup_stage == "set_mode":
            if len(rsp.success) != len(self.md_ids) or not all(rsp.success):
                self.get_logger().error(f"Failed to set MD mode: {rsp.success}")
                return
            self.get_logger().info("✓ MD mode set to VELOCITY_PID")
            self._setup_stage = "enable_mds"
            return

        if self._setup_stage == "enable_mds":
            if len(rsp.success) != len(self.md_ids) or not all(rsp.success):
                self.get_logger().error(f"Failed to enable all MD devices: {rsp.success}")
                return
            self.get_logger().info("✓ MD motors enabled")
            self._enable_raw_keyboard()
            self._setup_complete = True
            self._setup_stage = "done"
            self.get_logger().info("Startup complete. Interactive speed control is active.")

    def _wait_for_service(self, client, service_name: str) -> bool:
        if client.wait_for_service(timeout_sec=0.2):
            return True
        return False

    def _call_add_pds(self) -> bool:
        req = AddDevices.Request()
        req.device_ids = [self.pds_id]
        rsp = self._sync_call(self.add_pds_client, req)
        if rsp is None:
            return False
        if not rsp.success or not rsp.success[0]:
            self.get_logger().error("Failed to add PDS device")
            return False
        return True

    def _call_enable_power_stage(self) -> bool:
        req = GenericPds.Request()
        rsp = self._sync_call(self.enable_power_stage_client, req)
        if rsp is None:
            self.get_logger().error(
                f"Failed to enable power stage: service call timed out"
            )
            return False
        if not rsp.success or not rsp.success[0]:
            self.get_logger().error(
                f"Failed to enable power stage at socket {self.power_stage_socket}"
            )
            return False
        self.get_logger().info(
            f"Enabled power stage at socket {self.power_stage_socket} for PDS {self.pds_id}"
        )
        return True

    def _call_add_mds(self) -> bool:
        req = AddDevices.Request()
        req.device_ids = self.md_ids
        rsp = self._sync_call(self.add_mds_client, req)
        if rsp is None:
            return False
        if len(rsp.success) != len(self.md_ids) or not all(rsp.success):
            self.get_logger().error(f"Failed to add all MD devices: {rsp.success}")
            return False
        return True

    def _call_set_mode_velocity_pid(self) -> bool:
        req = SetMode.Request()
        req.device_ids = self.md_ids
        req.mode = ["VELOCITY_PID"] * len(self.md_ids)
        rsp = self._sync_call(self.set_mode_client, req)
        if rsp is None:
            return False
        if len(rsp.success) != len(self.md_ids) or not all(rsp.success):
            self.get_logger().error(f"Failed to set MD mode: {rsp.success}")
            return False
        return True

    def _call_enable_mds(self) -> bool:
        req = Generic.Request()
        req.device_ids = self.md_ids
        rsp = self._sync_call(self.enable_md_client, req)
        if rsp is None:
            return False
        if len(rsp.success) != len(self.md_ids) or not all(rsp.success):
            self.get_logger().error(f"Failed to enable all MD devices: {rsp.success}")
            return False
        return True

    def _sync_call(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            self.get_logger().error("Service call timeout")
            return None
        if future.result() is None:
            self.get_logger().error("Service call returned no response")
            return None
        return future.result()

    def _publish_motion(self) -> None:
        if not self._setup_complete:
            return

        msg = MotionCmd()
        msg.device_ids = [int(x) for x in self.md_ids]
        msg.target_position = [0.0] * len(self.md_ids)
        msg.target_velocity = [float(self.speeds[md_id]) for md_id in self.md_ids]
        msg.target_torque = [0.0] * len(self.md_ids)
        self.motion_pub.publish(msg)

    def _on_joint_states(self, msg: JointState) -> None:
        n = min(len(msg.name), len(msg.position), len(msg.velocity), len(msg.effort))
        for i in range(n):
            motor_id = self._parse_joint_name_to_id(msg.name[i])
            if motor_id is None or motor_id not in self.md_ids:
                continue
            self.last_motor_state[motor_id] = {
                "position": float(msg.position[i]),
                "speed": float(msg.velocity[i]),
                "torque": float(msg.effort[i]),
            }

    def _on_control_module(self, msg: ControlModuleData) -> None:
        # The SDK values are in millivolts; convert to volts for plotting.
        self.last_bus_voltage_v = float(msg.bus_voltage) / 1000.0

    def _on_power_stage(self, msg: PowerStageData) -> None:
        # The SDK values are in millivolts/milliamps; convert to V/A for plotting.
        self.last_output_voltage_v = float(msg.output_voltage) / 1000.0
        self.last_load_current_a = float(msg.load_current) / 1000.0

    def _publish_telemetry(self) -> None:
        if not self._setup_complete:
            return

        pds_msg = Float64MultiArray()
        pds_msg.data = [
            self._num_or_nan(self.last_bus_voltage_v),
            self._num_or_nan(self.last_output_voltage_v),
            self._num_or_nan(self.last_load_current_a),
        ]
        self.pds_telemetry_pub.publish(pds_msg)

        for motor_id in self.md_ids:
            state = self.last_motor_state.get(motor_id)
            joint_msg = Float64MultiArray()
            joint_msg.data = [
                self._num_or_nan(None if state is None else state["position"]),
                self._num_or_nan(None if state is None else state["speed"]),
                self._num_or_nan(None if state is None else state["torque"]),
                float(self.speeds[motor_id]),
            ]
            self.joint_telemetry_pubs[motor_id].publish(joint_msg)

    def _num_or_nan(self, value) -> float:
        if value is None:
            return math.nan
        return float(value)

    def _parse_joint_name_to_id(self, joint_name: str):
        if not joint_name.startswith("Joint "):
            return None
        try:
            return int(joint_name.split(" ", 1)[1])
        except (ValueError, IndexError):
            return None

    def _enable_raw_keyboard(self) -> None:
        if self._stdin_fd is not None:
            return
        try:
            self._tty_file = open("/dev/tty", "r")
            self._stdin_fd = self._tty_file.fileno()
            self._old_term_settings = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
            self.get_logger().info("Keyboard input enabled (terminal in cbreak mode).")
        except (termios.error, OSError) as exc:
            self.get_logger().warn(f"Could not enable raw keyboard: {exc}")
            if self._tty_file is not None:
                self._tty_file.close()
                self._tty_file = None
            self._stdin_fd = None

    def _restore_keyboard(self) -> None:
        if self._stdin_fd is None or self._old_term_settings is None:
            return
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term_settings)
        self._stdin_fd = None
        self._old_term_settings = None
        if self._tty_file is not None:
            self._tty_file.close()
            self._tty_file = None

    def _poll_keyboard(self) -> None:
        if not self._setup_complete or self._stdin_fd is None:
            return

        if not select.select([self._tty_file], [], [], 0.0)[0]:
            return

        key = self._tty_file.read(1)
        if key == "1" and len(self.md_ids) >= 1:
            self._change_speed(self.md_ids[0], +self.speed_step)
        elif key == "q" and len(self.md_ids) >= 1:
            self._change_speed(self.md_ids[0], -self.speed_step)
        elif key == "2" and len(self.md_ids) >= 2:
            self._change_speed(self.md_ids[1], +self.speed_step)
        elif key == "w" and len(self.md_ids) >= 2:
            self._change_speed(self.md_ids[1], -self.speed_step)
        elif key == "3" and len(self.md_ids) >= 3:
            self._change_speed(self.md_ids[2], +self.speed_step)
        elif key == "e" and len(self.md_ids) >= 3:
            self._change_speed(self.md_ids[2], -self.speed_step)
        elif key == " ":
            for md_id in self.md_ids:
                self.speeds[md_id] = 0.0
            self.get_logger().info("All motor speeds set to 0")
        elif key == "x":
            self.get_logger().info("Exit requested: disabling motors.")
            self._disable_all_mds()
            raise KeyboardInterrupt

    def _change_speed(self, md_id: int, delta: float) -> None:
        new_speed = self.speeds[md_id] + delta
        new_speed = max(min(new_speed, self.max_speed), -self.max_speed)
        self.speeds[md_id] = new_speed
        self.get_logger().info(f"MD {md_id}: target_velocity = {new_speed:.2f} rad/s")

    def _disable_all_mds(self) -> None:
        req = Generic.Request()
        req.device_ids = self.md_ids
        disable_client = self.create_client(Generic, "md/disable")
        if not disable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("md/disable service not available during shutdown")
            return
        rsp = self._sync_call(disable_client, req)
        if rsp is None:
            return
        self.get_logger().info(f"Disable response: {rsp.success}")

    def destroy_node(self) -> bool:
        self._restore_keyboard()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractiveSpeedControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
