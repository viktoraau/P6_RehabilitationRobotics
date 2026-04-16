import json
import time
import numpy as np
import rclpy
from rclpy.node import Node

from ft300_ros2.ft300_common import FT300


G = 9.81


def skew(v):
    x, y, z = v
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,   0.0]
    ])


def solve_mass_and_force_bias(force_measurements, gravity_vectors):
    A_blocks = []
    b_blocks = []

    for f_i, g_i in zip(force_measurements, gravity_vectors):
        A_i = np.hstack([g_i.reshape(3, 1), np.eye(3)])
        A_blocks.append(A_i)
        b_blocks.append(f_i.reshape(3, 1))

    A = np.vstack(A_blocks)
    b = np.vstack(b_blocks).reshape(-1)

    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

    mass = x[0]
    force_bias = x[1:4]
    return mass, force_bias, residuals, rank, s


def solve_com_and_torque_bias(force_measurements_corrected, torque_measurements):
    A_blocks = []
    b_blocks = []

    for f_i, tau_i in zip(force_measurements_corrected, torque_measurements):
        A_i = np.hstack([-skew(f_i), np.eye(3)])
        A_blocks.append(A_i)
        b_blocks.append(tau_i.reshape(3, 1))

    A = np.vstack(A_blocks)
    b = np.vstack(b_blocks).reshape(-1)

    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

    r_com = x[0:3]
    torque_bias = x[3:6]
    return r_com, torque_bias, residuals, rank, s


def get_predefined_poses():
    return [
        {"name": "+X down", "g_sensor": np.array([+G, 0.0, 0.0])},
        {"name": "-X down", "g_sensor": np.array([-G, 0.0, 0.0])},
        {"name": "+Y down", "g_sensor": np.array([0.0, +G, 0.0])},
        {"name": "-Y down", "g_sensor": np.array([0.0, -G, 0.0])},
        {"name": "+Z down", "g_sensor": np.array([0.0, 0.0, +G])},
        {"name": "-Z down", "g_sensor": np.array([0.0, 0.0, -G])},
    ]


class FT300CalibrationNode(Node):
    def __init__(self):
        super().__init__("ft300_calibration_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("timeout", 0.1)
        self.declare_parameter("sample_time", 2.0)
        self.declare_parameter("settle_time", 2.0)
        self.declare_parameter("output_json", "ft300_calibration.json")

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.sample_time = float(self.get_parameter("sample_time").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.output_json = self.get_parameter("output_json").value

        self.sensor = FT300(self.port, self.baudrate, self.timeout)

    def average_wrench(self, duration_s):
        end_time = time.time() + duration_s
        samples = []

        while time.time() < end_time and rclpy.ok():
            sample = self.sensor.read_sample()
            if sample is not None:
                samples.append(sample)

        if not samples:
            raise RuntimeError("No valid samples received during averaging.")

        data = np.vstack(samples)
        mean = np.mean(data, axis=0)
        std = np.std(data, axis=0)
        return mean, std, len(samples)

    def run(self):
        self.get_logger().info("FT300 Center-of-Mass Calibration")
        self.get_logger().info(f"Connecting to {self.port}")

        poses = get_predefined_poses()
        measurements = []

        try:
            self.sensor.start_stream()
            self.get_logger().info("Streaming started.")
            print()
            print("Procedure:")
            print("1. Put the sensor/tool in each requested static orientation.")
            print("2. Make sure nothing touches the tool.")
            print("3. Minimize cable pull.")
            print("4. Press Enter when ready for each pose.")
            print()

            for i, pose in enumerate(poses, start=1):
                print(f"[{i}/{len(poses)}] Pose: {pose['name']}")
                input("Place the setup in this pose, then press Enter... ")

                print(f"Waiting {self.settle_time:.1f} s to settle...")
                time.sleep(self.settle_time)

                self.sensor.buffer.clear()
                self.sensor.ser.reset_input_buffer()

                print(f"Averaging for {self.sample_time:.1f} s...")
                mean, std, n = self.average_wrench(self.sample_time)

                measurements.append({
                    "name": pose["name"],
                    "g_sensor": pose["g_sensor"],
                    "mean": mean,
                    "std": std,
                    "samples": n,
                })

                fx, fy, fz, mx, my, mz = mean
                print(f"  Samples: {n}")
                print(
                    f"  Mean: Fx={fx:+.4f} N  Fy={fy:+.4f} N  Fz={fz:+.4f} N  "
                    f"Mx={mx:+.5f} Nm  My={my:+.5f} Nm  Mz={mz:+.5f} Nm"
                )
                print()

            gravity_vectors = [m["g_sensor"] for m in measurements]
            forces = [m["mean"][0:3] for m in measurements]
            torques = [m["mean"][3:6] for m in measurements]

            mass, force_bias, res_f, rank_f, _ = solve_mass_and_force_bias(forces, gravity_vectors)

            if mass < 0:
                gravity_vectors = [-g for g in gravity_vectors]
                mass, force_bias, res_f, rank_f, _ = solve_mass_and_force_bias(forces, gravity_vectors)

            forces_corrected = [f - force_bias for f in forces]
            com, torque_bias, res_t, rank_t, _ = solve_com_and_torque_bias(forces_corrected, torques)

            print()
            print("============================================================")
            print("CALIBRATION RESULT")
            print("============================================================")
            print(f"Estimated mass         : {mass:.6f} kg")
            print(f"Estimated weight       : {mass * G:.6f} N")
            print()
            print("Estimated force bias [N]:")
            print(f"  bFx = {force_bias[0]:+.6f}")
            print(f"  bFy = {force_bias[1]:+.6f}")
            print(f"  bFz = {force_bias[2]:+.6f}")
            print()
            print("Estimated torque bias [Nm]:")
            print(f"  bMx = {torque_bias[0]:+.6f}")
            print(f"  bMy = {torque_bias[1]:+.6f}")
            print(f"  bMz = {torque_bias[2]:+.6f}")
            print()
            print("Estimated center of mass relative to sensor origin:")
            print(f"  x = {com[0]:+.6f} m   ({com[0] * 1000:+.2f} mm)")
            print(f"  y = {com[1]:+.6f} m   ({com[1] * 1000:+.2f} mm)")
            print(f"  z = {com[2]:+.6f} m   ({com[2] * 1000:+.2f} mm)")

            output = {
                "port": self.port,
                "sample_time_s": self.sample_time,
                "settle_time_s": self.settle_time,
                "gravity_m_s2": G,
                "mass_kg": float(mass),
                "weight_N": float(mass * G),
                "force_bias_N": force_bias.tolist(),
                "torque_bias_Nm": torque_bias.tolist(),
                "com_m": com.tolist(),
                "com_mm": (com * 1000.0).tolist(),
                "tare_force_N": [0.0, 0.0, 0.0],
                "tare_torque_Nm": [0.0, 0.0, 0.0],
                "measurements": [
                    {
                        "name": m["name"],
                        "g_sensor": m["g_sensor"].tolist(),
                        "mean": m["mean"].tolist(),
                        "std": m["std"].tolist(),
                        "samples": int(m["samples"]),
                    }
                    for m in measurements
                ],
            }

            with open(self.output_json, "w", encoding="utf-8") as f:
                json.dump(output, f, indent=2)

            self.get_logger().info(f"Saved calibration to {self.output_json}")

            if len(res_f) > 0:
                self.get_logger().info(f"Force residual sum: {res_f[0]:.6e}")
            if len(res_t) > 0:
                self.get_logger().info(f"Torque residual sum: {res_t[0]:.6e}")
            self.get_logger().info(f"Force rank: {rank_f}, Torque rank: {rank_t}")

        finally:
            self.sensor.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    node = FT300CalibrationNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()