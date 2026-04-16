import json
import os
import threading
import collections
import statistics
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

from ft300_ros2.ft300_common import FT300


class FT300WrenchNode(Node):
    def __init__(self):
        super().__init__("ft300_wrench_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("timeout", 0.05)
        self.declare_parameter("topic_name", "/wrench_input")
        self.declare_parameter("frame_id", "tool0")

        self.declare_parameter("median_window", 7)
        self.declare_parameter("alpha_force", 0.3)
        self.declare_parameter("alpha_torque", 0.3)

        self.declare_parameter("force_deadband", 0.1)
        self.declare_parameter("torque_deadband", 0.003)

        self.declare_parameter("calibration_file", "")

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.topic_name = self.get_parameter("topic_name").value
        self.frame_id = self.get_parameter("frame_id").value

        self.median_window = int(self.get_parameter("median_window").value)
        self.alpha_force = float(self.get_parameter("alpha_force").value)
        self.alpha_torque = float(self.get_parameter("alpha_torque").value)

        self.force_deadband = float(self.get_parameter("force_deadband").value)
        self.torque_deadband = float(self.get_parameter("torque_deadband").value)

        self.calibration_file = self.get_parameter("calibration_file").value

        self.lock = threading.Lock()
        self.running = True

        self.raw_history = {
            "fx": collections.deque(maxlen=self.median_window),
            "fy": collections.deque(maxlen=self.median_window),
            "fz": collections.deque(maxlen=self.median_window),
            "mx": collections.deque(maxlen=self.median_window),
            "my": collections.deque(maxlen=self.median_window),
            "mz": collections.deque(maxlen=self.median_window),
        }

        self.filtered = {
            "fx": 0.0, "fy": 0.0, "fz": 0.0,
            "mx": 0.0, "my": 0.0, "mz": 0.0,
        }
        self.filter_initialized = False

        self.force_bias = np.zeros(3, dtype=float)
        self.torque_bias = np.zeros(3, dtype=float)
        self.tare_force = np.zeros(3, dtype=float)
        self.tare_torque = np.zeros(3, dtype=float)
        self.latest_wrench = np.zeros(6, dtype=float)

        self.load_calibration()

        self.pub = self.create_publisher(WrenchStamped, self.topic_name, 10)
        self.tare_srv = self.create_service(Trigger, "/ft300/tare", self.handle_tare)
        self.save_tare_srv = self.create_service(Trigger, "/ft300/save_tare", self.handle_save_tare)

        self.sensor = FT300(self.port, self.baudrate, self.timeout)
        self.sensor.start_stream()

        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(f"FT300 started on {self.port}")
        self.get_logger().info(f"Publishing WrenchStamped on {self.topic_name}")
        self.get_logger().info(f"Using frame_id '{self.frame_id}'")

    def load_calibration(self):
        if not self.calibration_file:
            self.get_logger().warn("No calibration_file provided. Using zero bias and zero tare.")
            return

        if not os.path.isfile(self.calibration_file):
            self.get_logger().warn(f"Calibration file not found: {self.calibration_file}")
            return

        try:
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.force_bias = np.array(data.get("force_bias_N", [0.0, 0.0, 0.0]), dtype=float)
            self.torque_bias = np.array(data.get("torque_bias_Nm", [0.0, 0.0, 0.0]), dtype=float)
            self.tare_force = np.array(data.get("tare_force_N", [0.0, 0.0, 0.0]), dtype=float)
            self.tare_torque = np.array(data.get("tare_torque_Nm", [0.0, 0.0, 0.0]), dtype=float)

            self.get_logger().info(f"Loaded calibration from: {self.calibration_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration file: {e}")

    def save_tare_to_file(self):
        if not self.calibration_file:
            raise RuntimeError("No calibration_file parameter set.")

        data = {}
        if os.path.isfile(self.calibration_file):
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                data = json.load(f)

        data["tare_force_N"] = self.tare_force.tolist()
        data["tare_torque_Nm"] = self.tare_torque.tolist()

        with open(self.calibration_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    @staticmethod
    def apply_deadband(value, threshold):
        return 0.0 if abs(value) < threshold else value

    def median_filter(self, key, value):
        self.raw_history[key].append(value)
        return statistics.median(self.raw_history[key])

    def low_pass(self, key, value, alpha):
        if not self.filter_initialized:
            self.filtered[key] = value
            return value

        self.filtered[key] = alpha * value + (1.0 - alpha) * self.filtered[key]
        return self.filtered[key]

    def process_sample(self, sample):
        wrench = np.array(sample, dtype=float)

        wrench[0:3] -= self.force_bias
        wrench[3:6] -= self.torque_bias

        wrench[0:3] -= self.tare_force
        wrench[3:6] -= self.tare_torque

        fx = self.median_filter("fx", wrench[0])
        fy = self.median_filter("fy", wrench[1])
        fz = self.median_filter("fz", wrench[2])
        mx = self.median_filter("mx", wrench[3])
        my = self.median_filter("my", wrench[4])
        mz = self.median_filter("mz", wrench[5])

        fx = self.low_pass("fx", fx, self.alpha_force)
        fy = self.low_pass("fy", fy, self.alpha_force)
        fz = self.low_pass("fz", fz, self.alpha_force)
        mx = self.low_pass("mx", mx, self.alpha_torque)
        my = self.low_pass("my", my, self.alpha_torque)
        mz = self.low_pass("mz", mz, self.alpha_torque)

        self.filter_initialized = True

        fx = self.apply_deadband(fx, self.force_deadband)
        fy = self.apply_deadband(fy, self.force_deadband)
        fz = self.apply_deadband(fz, self.force_deadband)
        mx = self.apply_deadband(mx, self.torque_deadband)
        my = self.apply_deadband(my, self.torque_deadband)
        mz = self.apply_deadband(mz, self.torque_deadband)

        self.latest_wrench[:] = [fx, fy, fz, mx, my, mz]

    def publish_wrench(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.wrench.force.x = float(self.latest_wrench[0])
        msg.wrench.force.y = float(self.latest_wrench[1])
        msg.wrench.force.z = float(self.latest_wrench[2])
        msg.wrench.torque.x = float(self.latest_wrench[3])
        msg.wrench.torque.y = float(self.latest_wrench[4])
        msg.wrench.torque.z = float(self.latest_wrench[5])

        self.pub.publish(msg)

    def read_loop(self):
        sample_counter = 0

        while self.running and rclpy.ok():
            try:
                sample = self.sensor.read_sample()
                if sample is None:
                    continue

                with self.lock:
                    self.process_sample(sample)
                    self.publish_wrench()
                    sample_counter += 1

                    if sample_counter % 100 == 0:
                        self.get_logger().info(
                            "Latest sample: "
                            f"Fx={self.latest_wrench[0]:+.3f}, "
                            f"Fy={self.latest_wrench[1]:+.3f}, "
                            f"Fz={self.latest_wrench[2]:+.3f}, "
                            f"Mx={self.latest_wrench[3]:+.4f}, "
                            f"My={self.latest_wrench[4]:+.4f}, "
                            f"Mz={self.latest_wrench[5]:+.4f}"
                        )

            except Exception as e:
                self.get_logger().error(f"Read loop error: {e}")

    def handle_tare(self, request, response):
        del request
        with self.lock:
            self.tare_force = self.latest_wrench[0:3].copy()
            self.tare_torque = self.latest_wrench[3:6].copy()

        response.success = True
        response.message = (
            f"Tare set. force={self.tare_force.tolist()}, "
            f"torque={self.tare_torque.tolist()}"
        )
        self.get_logger().info(response.message)
        return response

    def handle_save_tare(self, request, response):
        del request
        try:
            with self.lock:
                self.save_tare_to_file()
            response.success = True
            response.message = f"Saved tare to {self.calibration_file}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save tare: {e}"
            self.get_logger().error(response.message)
        return response

    def destroy_node(self):
        self.running = False
        try:
            self.sensor.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FT300WrenchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
