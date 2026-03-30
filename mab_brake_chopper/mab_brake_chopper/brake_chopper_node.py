import math
import os
import signal
import time
from typing import Optional

import rclpy
from candle_ros2.msg import ControlModuleData
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from mab_brake_chopper.gpio_backend import GpioError, LinuxCharGpioBackend, MockGpioBackend
from mab_brake_chopper.logic import BrakeChopperController, extract_dynamic_joint_state_value


class BrakeChopperNode(Node):
    def __init__(self) -> None:
        super().__init__("brake_chopper")

        self.declare_parameter("pds_id", 100)
        self.declare_parameter("voltage_source", "dynamic_joint_state")
        self.declare_parameter("control_module_topic", "")
        self.declare_parameter("dynamic_joint_states_topic", "/dynamic_joint_states")
        self.declare_parameter("dynamic_joint_name", "mab_power_stage")
        self.declare_parameter("dynamic_interface_name", "bus_voltage")
        self.declare_parameter("voltage_scale", 0.001)
        self.declare_parameter("trigger_voltage_v", 12.0)
        self.declare_parameter("telemetry_timeout_sec", 1.0)
        self.declare_parameter("gpio_pin", 17)
        self.declare_parameter("gpio_backend", "linux_char")
        self.declare_parameter("gpio_chip_path", "")
        self.declare_parameter("gpio_chip_label", "pinctrl-rp1")
        self.declare_parameter("gpio_consumer_label", "mab_brake_chopper")
        self.declare_parameter("active_high", True)

        self.pds_id = int(self.get_parameter("pds_id").value)
        self.voltage_source = str(self.get_parameter("voltage_source").value).strip().lower()
        self.control_module_topic = str(self.get_parameter("control_module_topic").value).strip()
        self.dynamic_joint_states_topic = str(
            self.get_parameter("dynamic_joint_states_topic").value
        ).strip()
        self.dynamic_joint_name = str(self.get_parameter("dynamic_joint_name").value).strip()
        self.dynamic_interface_name = str(
            self.get_parameter("dynamic_interface_name").value
        ).strip()
        self.voltage_scale = float(self.get_parameter("voltage_scale").value)
        self.trigger_voltage_v = float(self.get_parameter("trigger_voltage_v").value)
        self.telemetry_timeout_sec = float(
            self.get_parameter("telemetry_timeout_sec").value
        )
        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.gpio_backend_name = str(self.get_parameter("gpio_backend").value).strip().lower()
        self.gpio_chip_path = str(self.get_parameter("gpio_chip_path").value).strip()
        self.gpio_chip_label = str(self.get_parameter("gpio_chip_label").value).strip()
        self.gpio_consumer_label = str(
            self.get_parameter("gpio_consumer_label").value
        ).strip()
        self.active_high = bool(self.get_parameter("active_high").value)

        if self.voltage_source not in {"auto", "dynamic_joint_state", "control_module"}:
            raise RuntimeError(
                "voltage_source must be one of: auto, dynamic_joint_state, control_module"
            )
        if self.telemetry_timeout_sec <= 0.0:
            raise RuntimeError("telemetry_timeout_sec must be positive")
        if self.voltage_scale == 0.0:
            raise RuntimeError("voltage_scale must be non-zero")

        self.control_module_topic = (
            self.control_module_topic or f"pds/id_{self.pds_id}/control_module"
        )
        self.controller = BrakeChopperController(
            trigger_voltage_v=self.trigger_voltage_v,
        )

        self._last_voltage_v = math.nan
        self._last_voltage_monotonic: Optional[float] = None
        self._last_voltage_source = "none"
        self._seen_voltage_sources = set()
        self._telemetry_stale = False
        self._startup_monotonic = time.monotonic()
        self._startup_wait_warned = False

        self.enabled_pub = self.create_publisher(Bool, "brake_chopper/enabled", 10)
        self.voltage_pub = self.create_publisher(Float64, "brake_chopper/voltage_v", 10)

        self._gpio = self._create_gpio_backend()
        self._create_subscriptions()
        self._watchdog_timer = self.create_timer(0.1, self._check_voltage_timeout)

        self._publish_status()
        self.get_logger().info(
            "Brake chopper ready: "
            f"trigger={self.trigger_voltage_v:.2f} V, "
            f"gpio_pin={self.gpio_pin}, "
            f"gpio_backend={self.gpio_backend_name}, "
            f"chip={getattr(self._gpio, 'chip_path', 'unknown')}, "
            f"voltage_source={self.voltage_source}"
        )

    def _create_gpio_backend(self):
        if self.gpio_backend_name == "linux_char":
            backend = LinuxCharGpioBackend(
                chip_path=self.gpio_chip_path,
                chip_label=self.gpio_chip_label,
                line_offset=self.gpio_pin,
                consumer_label=self.gpio_consumer_label,
                active_high=self.active_high,
            )
            self.get_logger().info(
                f"Using GPIO character device {backend.chip_path} line {self.gpio_pin}"
            )
            return backend

        if self.gpio_backend_name == "mock":
            self.get_logger().warn("Using mock GPIO backend; no hardware pin will be driven.")
            return MockGpioBackend()

        raise RuntimeError("gpio_backend must be 'linux_char' or 'mock'")

    def _create_subscriptions(self) -> None:
        if self.voltage_source in {"auto", "control_module"}:
            self.control_module_sub = self.create_subscription(
                ControlModuleData,
                self.control_module_topic,
                self._on_control_module,
                10,
            )
            self.get_logger().info(
                f"Subscribed to control-module voltage topic '{self.control_module_topic}'"
            )

        if self.voltage_source in {"auto", "dynamic_joint_state"}:
            self.dynamic_joint_state_sub = self.create_subscription(
                DynamicJointState,
                self.dynamic_joint_states_topic,
                self._on_dynamic_joint_states,
                10,
            )
            self.get_logger().info(
                f"Subscribed to dynamic joint states topic '{self.dynamic_joint_states_topic}'"
            )

    def _on_control_module(self, msg: ControlModuleData) -> None:
        self._handle_voltage_sample(float(msg.bus_voltage), "control_module")

    def _on_dynamic_joint_states(self, msg: DynamicJointState) -> None:
        raw_value = extract_dynamic_joint_state_value(
            msg,
            joint_name=self.dynamic_joint_name,
            interface_name=self.dynamic_interface_name,
        )
        if raw_value is None:
            return

        self._handle_voltage_sample(raw_value, "dynamic_joint_state")

    def _handle_voltage_sample(self, raw_voltage: float, source_name: str) -> None:
        voltage_v = raw_voltage * self.voltage_scale
        if not math.isfinite(voltage_v):
            self.get_logger().warn(f"Ignoring non-finite voltage sample from {source_name}")
            return

        if self._telemetry_stale:
            self.get_logger().info("Voltage telemetry recovered.")

        if source_name not in self._seen_voltage_sources:
            self.get_logger().info(f"Voltage source active: {source_name}")
            self._seen_voltage_sources.add(source_name)

        self._telemetry_stale = False
        self._startup_wait_warned = False
        self._last_voltage_source = source_name
        self._last_voltage_v = voltage_v
        self._last_voltage_monotonic = time.monotonic()

        if self.controller.update(voltage_v):
            self._set_output(
                self.controller.enabled,
                f"voltage {voltage_v:.2f} V "
                f"{'fell below' if self.controller.enabled else 'rose above'} "
                f"the trigger threshold",
            )

        self._publish_status()

    def _set_output(self, enabled: bool, reason: str) -> None:
        try:
            self._gpio.set_output(enabled)
        except GpioError as exc:
            raise RuntimeError(f"Failed to update brake chopper GPIO: {exc}") from exc

        state_text = "ENABLED" if enabled else "DISABLED"
        self.get_logger().info(f"Brake chopper {state_text}: {reason}")

    def _publish_status(self) -> None:
        enabled_msg = Bool()
        enabled_msg.data = bool(self.controller.enabled)
        self.enabled_pub.publish(enabled_msg)

        voltage_msg = Float64()
        voltage_msg.data = float(self._last_voltage_v)
        self.voltage_pub.publish(voltage_msg)

    def _check_voltage_timeout(self) -> None:
        now = time.monotonic()

        if self._last_voltage_monotonic is None:
            if (
                not self._startup_wait_warned
                and now - self._startup_monotonic > self.telemetry_timeout_sec
            ):
                self._startup_wait_warned = True
                self.get_logger().warn(
                    "No voltage telemetry received yet; brake chopper remains OFF."
                )
            return

        elapsed = now - self._last_voltage_monotonic
        if elapsed <= self.telemetry_timeout_sec:
            return

        if self._telemetry_stale:
            return

        self._telemetry_stale = True
        self.get_logger().warn(
            f"Voltage telemetry is stale ({elapsed:.2f} s); forcing brake chopper OFF."
        )

        if self.controller.force_disable():
            self._set_output(False, "telemetry timeout")

        self._publish_status()

    def destroy_node(self):
        try:
            try:
                self.controller.force_disable()
                self._gpio.close()
            except Exception as exc:
                self.get_logger().warn(f"Brake chopper GPIO cleanup failed: {exc}")
        finally:
            return super().destroy_node()


def _kill_stale_instances() -> None:
    """Kill any leftover brake_chopper_node processes to free the GPIO line."""
    my_pid = os.getpid()
    try:
        result = os.popen("pgrep -f brake_chopper_node").read().strip()
        if not result:
            return
        stale_pids = []
        for line in result.splitlines():
            pid = int(line.strip())
            if pid != my_pid:
                stale_pids.append(pid)
        if not stale_pids:
            return

        # Send SIGTERM first
        for pid in stale_pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        # Wait up to 2 seconds for processes to die
        for _ in range(20):
            stale_pids = [p for p in stale_pids if _pid_alive(p)]
            if not stale_pids:
                break
            time.sleep(0.1)

        # Force kill anything still alive
        for pid in stale_pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        # Brief wait for kernel to release GPIO handles
        time.sleep(0.2)
    except Exception:
        pass


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except OSError:
        return True


def main(args=None) -> None:
    _kill_stale_instances()
    rclpy.init(args=args)
    node = None

    try:
        node = BrakeChopperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
