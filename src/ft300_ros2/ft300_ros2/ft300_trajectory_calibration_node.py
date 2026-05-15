"""FT300 Calibration + Publisher Node
=====================================

Architecture
------------
  robotiq_ft_sensor_standalone_node   (rq_fts_ros2_driver)
      └─ publishes: /robotiq_force_torque_sensor_broadcaster/wrench  (raw)

  THIS NODE  (ft300_trajectory_calibration_node)
      ├─ subscribes: raw_topic      → applies calibration
      ├─ publishes:  /ft300/wrench  → calibrated WrenchStamped  (always-on)
      ├─ service:    /ft300/force_torque_cali  (std_srvs/Trigger)
      │     └─ moves robot through calibration trajectory
      │        collects raw samples at each pose
      │        runs least-squares → hot-swaps calibration → saves JSON
      └─ action client: FollowJointTrajectory → joint_trajectory_controller

Usage
-----
  # Terminal 1 – raw sensor driver (owns serial port)
  ros2 run robotiq_ft_sensor_hardware robotiq_ft_sensor_standalone_node

  # Terminal 2 – calibration + calibrated publisher
  ros2 run ft300_ros2 ft300_trajectory_calibration_node \\
    --ros-args -p calibration_file:=/path/to/ft300_calibration.json

  # Trigger calibration
  ros2 service call /ft300/force_torque_cali std_srvs/srv/Trigger {}

Topics
------
  /robotiq_force_torque_sensor_broadcaster/wrench  (in)  raw WrenchStamped
  /ft300/wrench                                    (out) calibrated WrenchStamped
"""

import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory

import numpy as np
import rclpy
import rclpy.duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

G = 9.81
JOINT_NAMES = ["joint_1", "joint_2", "joint_3"]

# Hardware joint limits (mab_rehab/config/safety_limits.yaml):
#   joint_1: ±1.1344640137963142 rad  (≈ ±65°)
#   joint_2: ±1.0471975511965976 rad  (= ±π/3, ±60°)
#   joint_3: ±0.5235987755982988 rad  (= ±π/6, ±30°)
_J1 = math.radians(65)
_J2 = math.radians(60)
_J3 = math.radians(30)

DEFAULT_CALIB_POSES = [
    # name         joints [q1,   q2,   q3]   (rad)  duration_s
    #
    # NOTE: j2-only poses (j1=0, j2≠0) are intentionally omitted.
    # When j1=0, joint_2 rotates about an axis aligned with gravity so the
    # gravity vector in the sensor frame is identical to home — they add
    # measurement noise without new constraints and inflate the residual.
    # #
    ("home",       [ 0.0,   0.0,   0.0],             4.0),

    # j3 tilts the sensor forward/backward → gX/gZ diversity at j1=0
    ("j3_pos",     [ 0.0,   0.0,  +_J3],             5.0),
    ("j3_neg",     [ 0.0,   0.0,  -_J3],             5.0),
    # j1+j2 combinations → strong gY excitation (all four quadrants)
    ("j1p_j2p",    [+_J1,  +_J2,   0.0],             6.0),
    ("j1n_j2n",    [-_J1,  -_J2,   0.0],             6.0),
    ("j1p_j2n",    [+_J1,  -_J2,   0.0],             6.0),
    ("j1n_j2p",    [-_J1,  +_J2,   0.0],             6.0),

    #few random points within joint limits
    ("random_1",   [ math.radians(20), math.radians(-15), math.radians(10)], 5.0),
    ("random_2",   [ math.radians(-30), math.radians(10), math.radians(-20)], 5.0),
    ("random_3",   [ math.radians(45), math.radians(-30), math.radians(15)], 5.0),
    ("random_4",   [ math.radians(-45), math.radians(20), math.radians(-10)], 5.0),
    ("random_5",   [ math.radians(10), math.radians(30), math.radians(-25)], 5.0),
    ("random_6",   [ math.radians(-20), math.radians(-25), math.radians(20)], 5.0),
    ("random_7",   [ math.radians(30), math.radians(15), math.radians(-15)], 5.0),
]


# ─────────────────────────────────────────────────────────────────────────────
# TF2 helper
# ─────────────────────────────────────────────────────────────────────────────

def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion → 3×3 rotation matrix."""
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


# ─────────────────────────────────────────────────────────────────────────────
# Least-squares solvers
# ─────────────────────────────────────────────────────────────────────────────

def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[ 0.0, -z,    y],
                     [ z,    0.0, -x],
                     [-y,    x,    0.0]])


def _solve_mass_and_force_bias(forces, gravity_vectors):
    """Model: f_i = m*g_i + b_f  →  x = [m, bfx, bfy, bfz]^T"""
    A_blocks, b_blocks = [], []
    for f, g in zip(forces, gravity_vectors):
        A_blocks.append(np.hstack([g.reshape(3, 1), np.eye(3)]))
        b_blocks.append(f.reshape(3, 1))
    A = np.vstack(A_blocks)
    b = np.vstack(b_blocks).reshape(-1)
    x, res, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    return float(x[0]), x[1:4], res, int(rank)


def _solve_com_and_torque_bias(forces_corrected, torques):
    """Model: tau_i = -skew(f_i_corr)*r_com + b_tau"""
    A_blocks, b_blocks = [], []
    for f, tau in zip(forces_corrected, torques):
        A_blocks.append(np.hstack([-_skew(f), np.eye(3)]))
        b_blocks.append(tau.reshape(3, 1))
    A = np.vstack(A_blocks)
    b = np.vstack(b_blocks).reshape(-1)
    x, res, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    return x[0:3], x[3:6], res, int(rank)


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class FT300TrajectoryCalibNode(Node):
    """Subscribes to raw wrench from the Robotiq driver, publishes calibrated
    wrench, and auto-calibrates on service call."""

    def __init__(self):
        super().__init__("ft300_trajectory_calibration_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter(
            "raw_topic",
            "/robotiq_force_torque_sensor_broadcaster/wrench",
        )
        self.declare_parameter("calibrated_topic", "/ft300/wrench")
        self.declare_parameter("frame_id",         "RU_1")
        _default_calib = os.path.join(
            get_package_share_directory("ft300_ros2"),
            "config", "ft300_calibration.json",
        )
        self.declare_parameter("calibration_file", _default_calib)
        self.declare_parameter("settle_time",      1.0)
        self.declare_parameter("sample_time",      1.0)
        self.declare_parameter(
            "action_server",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        # trajectory_mode: "action" (default, FollowJointTrajectory action) or
        #                  "topic"  (publish JointTrajectory directly, for CTC)
        self.declare_parameter("trajectory_mode", "action")
        self.declare_parameter(
            "trajectory_topic",
            "joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("base_frame",   "base_link")
        self.declare_parameter("sensor_frame", "RU_1")
        self.declare_parameter("lp_cutoff_hz", 10.0)   # published output only; 0.0 = passthrough

        self._raw_topic   = self.get_parameter("raw_topic").value
        self._calib_topic = self.get_parameter("calibrated_topic").value
        self._frame_id    = self.get_parameter("frame_id").value
        self._calib_file  = os.path.abspath(
            self.get_parameter("calibration_file").value
        )
        self._settle_time      = float(self.get_parameter("settle_time").value)
        self._sample_time      = float(self.get_parameter("sample_time").value)
        self._action_ns        = self.get_parameter("action_server").value
        self._trajectory_mode  = self.get_parameter("trajectory_mode").value
        self._trajectory_topic = self.get_parameter("trajectory_topic").value
        self._base_frame  = self.get_parameter("base_frame").value
        self._sensor_frame = self.get_parameter("sensor_frame").value
        self._lp_cutoff  = float(self.get_parameter("lp_cutoff_hz").value)

        # ── Live calibration (protected by _lock) ─────────────────────────
        self._lock        = threading.Lock()
        self._force_bias  = np.zeros(3)
        self._torque_bias = np.zeros(3)
        self._tare_force  = np.zeros(3)
        self._tare_torque = np.zeros(3)
        self._mass        = 0.0
        self._com         = np.zeros(3)

        self._load_calibration()

        # ── Low-pass filter state (applied to published output only) ─────────
        # First-order IIR: y[k] = alpha*x[k] + (1-alpha)*y[k-1]
        # alpha = dt / (tau + dt),  tau = 1 / (2*pi*fc)
        self._lp_state = np.zeros(6)
        self._lp_ready : bool = False

        # ── Sample collection state ────────────────────────────────────────
        self._collecting  : bool = False
        self._collect_buf : list = []

        # ── Watchdog ──────────────────────────────────────────────────────
        self._msg_count    : int   = 0
        self._last_raw_time: float = 0.0

        # ── Joint state cache ─────────────────────────────────────────────
        self._joint_pos: dict = {n: 0.0 for n in JOINT_NAMES}

        # ── TF2 ───────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False,
        )

        # ── ReentrantCallbackGroup so the service handler can block-wait ──
        self._cbg = ReentrantCallbackGroup()

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(WrenchStamped, self._calib_topic, 10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            WrenchStamped,
            self._raw_topic,
            self._on_raw_wrench,
            10,
            callback_group=self._cbg,
        )

        self.create_subscription(
            JointState, "/joint_states",
            self._on_joint_state, 10,
            callback_group=self._cbg,
        )

        # ── Calibration service ────────────────────────────────────────────
        self.create_service(
            Trigger, "/ft300/force_torque_cali",
            self._handle_calibrate,
            callback_group=self._cbg,
        )

        # ── Watchdog timer: warn every 5 s if no raw messages arrive ──────
        self.create_timer(5.0, self._watchdog)

        # ── Trajectory action client (action mode) ──────────────────────────
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, self._action_ns,
            callback_group=self._cbg,
        )

        # ── Trajectory topic publisher (topic mode, for CTC) ──────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory, self._trajectory_topic, 10
        )

        self.get_logger().info(
            f"Subscribing to raw wrench on  '{self._raw_topic}'"
        )
        self.get_logger().info(
            f"Publishing calibrated wrench on '{self._calib_topic}'"
        )
        self.get_logger().info("Service '/ft300/force_torque_cali' ready")
        if self._trajectory_mode == "topic":
            self.get_logger().info(
                f"Trajectory mode: TOPIC  → publishing to '{self._trajectory_topic}'"
            )
        else:
            self.get_logger().info(
                f"Trajectory mode: ACTION → action server '{self._action_ns}'"
            )
        if self._lp_cutoff > 0.0:
            self.get_logger().info(
                f"Low-pass filter enabled: cutoff={self._lp_cutoff:.1f} Hz  "
                f"(tau={1.0/(2.0*math.pi*self._lp_cutoff)*1000:.1f} ms)  "
                "applied to published /ft300/wrench "
            )
        else:
            self.get_logger().info("Low-pass filter disabled (lp_cutoff_hz=0.0)")

    # ──────────────────────────────────────────────────────────────────────
    # Raw wrench callback → collect buffer + publish calibrated
    # ──────────────────────────────────────────────────────────────────────

    def _watchdog(self):
        if self._msg_count == 0:
            self.get_logger().warn(
                f"No raw wrench messages received yet on '{self._raw_topic}'. "
                "Check that the sensor driver is running and the topic name matches."
            )
        else:
            elapsed = time.time() - self._last_raw_time
            if elapsed > 4.9:
                self.get_logger().warn(
                    f"Raw wrench stream stalled — last message {elapsed:.1f} s ago "
                    f"(topic: '{self._raw_topic}')"
                )

    def _on_raw_wrench(self, msg: WrenchStamped):
        raw = np.array([
            msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ])

        # ── First-order IIR low-pass filter (published output only) ───────
        now = time.time()
        if not self._lp_ready:
            # seed filter on first message to avoid startup transient
            self._lp_state = raw.copy()
            self._lp_ready = True
        elif self._lp_cutoff > 0.0:
            dt    = max(1e-3, min(now - self._last_raw_time, 0.1))
            tau   = 1.0 / (2.0 * math.pi * self._lp_cutoff)
            alpha = dt / (tau + dt)
            self._lp_state = alpha * raw + (1.0 - alpha) * self._lp_state
        else:
            self._lp_state = raw  # passthrough

        self._msg_count += 1
        self._last_raw_time = now
        if self._msg_count == 1:
            self.get_logger().info(
                f"First raw wrench message received on '{self._raw_topic}' "
                f"— stream is alive."
            )

        with self._lock:
            if self._collecting:
                self._collect_buf.append(raw.copy())  # unfiltered for calibration LSQ
            fb = self._force_bias.copy()
            tb = self._torque_bias.copy()
            tf_ = self._tare_force.copy()
            tt  = self._tare_torque.copy()

        # apply calibration biases to the filtered signal
        f = self._lp_state[0:3] - fb - tf_
        t = self._lp_state[3:6] - tb - tt

        out = WrenchStamped()
        out.header.stamp    = msg.header.stamp        # preserve sensor timestamp
        out.header.frame_id = self._frame_id
        out.wrench.force.x  = float(f[0])
        out.wrench.force.y  = float(f[1])
        out.wrench.force.z  = float(f[2])
        out.wrench.torque.x = float(t[0])
        out.wrench.torque.y = float(t[1])
        out.wrench.torque.z = float(t[2])
        self._pub.publish(out)

    # ──────────────────────────────────────────────────────────────────────
    # Joint state subscriber
    # ──────────────────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_pos:
                self._joint_pos[name] = float(pos)

    # ──────────────────────────────────────────────────────────────────────
    # TF2 gravity helper
    # ──────────────────────────────────────────────────────────────────────

    def _gravity_in_sensor(self, timeout_s: float = 3.0) -> np.ndarray:
        """Return gravity vector expressed in sensor frame via live TF lookup.

        lookup_transform(sensor_frame, base_frame) gives R such that
          v_sensor = R * v_base
        g_base = [0, 0, -G]
        """
        try:
            ts = self._tf_buffer.lookup_transform(
                self._sensor_frame,
                self._base_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_s),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError(
                f"TF lookup '{self._base_frame}' -> '{self._sensor_frame}' "
                f"failed: {exc}"
            ) from exc

        q = ts.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        return R @ np.array([0.0, 0.0, -G])

    # ──────────────────────────────────────────────────────────────────────
    # Sample collection
    # ──────────────────────────────────────────────────────────────────────

    def _collect_samples(self, duration_s: float):
        """Arm collection, wait duration_s, return (mean, std, n)."""
        with self._lock:
            self._collect_buf.clear()
            self._collecting = True

        time.sleep(duration_s)

        with self._lock:
            self._collecting = False
            buf = list(self._collect_buf)

        if not buf:
            raise RuntimeError(
                "No samples received from raw wrench topic during measurement "
                f"window ({duration_s:.1f} s). "
                f"Is '{self._raw_topic}' being published?"
            )

        data = np.vstack(buf)
        # Use median to reject transient outliers (bumps, spikes)
        return np.median(data, axis=0), np.std(data, axis=0), len(buf)

    # ──────────────────────────────────────────────────────────────────────
    # Robot motion
    # ──────────────────────────────────────────────────────────────────────

    def _move_to_joints(self, positions: list, duration_s: float):
        """Send a trajectory goal and block until done.

        In 'action' mode (default): uses FollowJointTrajectory action.
        In 'topic' mode (CTC):      publishes JointTrajectory directly and
                                    waits duration_s for the robot to reach
                                    the target.
        """
        if self._trajectory_mode == "topic":
            self._move_to_joints_topic(positions, duration_s)
        else:
            self._move_to_joints_action(positions, duration_s)

    def _move_to_joints_topic(self, positions: list, duration_s: float):
        """Publish JointTrajectory to topic (used with CTC in effort mode).

        The IK node publishes continuously on the same topic at ~100 Hz, so a
        single publish would be immediately overwritten.  Instead we publish in
        a 100 Hz loop for the full duration to keep the calibration setpoint
        dominant.
        """
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions     = [float(p) for p in positions]
        pt.velocities    = [0.0] * len(positions)
        pt.accelerations = [0.0] * len(positions)
        pt.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1.0) * 1e9),
        )
        msg.points = [pt]

        self.get_logger().debug(
            f"[topic mode] Streaming trajectory to '{self._trajectory_topic}' "
            f"for {duration_s:.1f} s at 100 Hz"
        )
        end_t = time.time() + duration_s
        while time.time() < end_t:
            self._traj_pub.publish(msg)
            time.sleep(0.01)  # 100 Hz — dominates the IK node's stream

    def _move_to_joints_action(self, positions: list, duration_s: float):
        """Send a single-point FollowJointTrajectory goal and block until done."""
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f"Action server '{self._action_ns}' not available"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions  = [float(p) for p in positions]
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1.0) * 1e9),
        )
        goal.trajectory.points = [pt]

        done_event = threading.Event()
        outcome    = [None]

        def _on_goal_response(future):
            gh = future.result()
            if not gh.accepted:
                outcome[0] = "REJECTED"
                done_event.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            outcome[0] = future.result()
            done_event.set()

        self._traj_client.send_goal_async(goal).add_done_callback(
            _on_goal_response
        )

        if not done_event.wait(timeout=duration_s + 8.0):
            raise RuntimeError(f"Trajectory to {positions} timed out")
        if outcome[0] == "REJECTED":
            raise RuntimeError(
                f"Trajectory goal rejected for pose {positions}"
            )

    # ──────────────────────────────────────────────────────────────────────
    # Calibration pipeline
    # ──────────────────────────────────────────────────────────────────────

    def _run_calibration(self):
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info("  FT300 autonomous trajectory calibration")
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

        measurements = []

        for idx, (name, joints, duration) in enumerate(DEFAULT_CALIB_POSES, start=1):
            self.get_logger().info(
                f"[{idx}/{len(DEFAULT_CALIB_POSES)}] Pose '{name}'  "
                + "  ".join(f"q{i+1}={j:+.3f}" for i, j in enumerate(joints))
            )

            self._move_to_joints(joints, duration)

            self.get_logger().info(f"  Settling {self._settle_time:.1f} s ...")
            time.sleep(self._settle_time)

            g_s = self._gravity_in_sensor()
            q1  = self._joint_pos["joint_1"]
            q2  = self._joint_pos["joint_2"]
            q3  = self._joint_pos["joint_3"]
            self.get_logger().info(
                f"  Joints q1={q1:+.4f} q2={q2:+.4f} q3={q3:+.4f} rad"
            )
            self.get_logger().info(
                f"  g_sensor = [{g_s[0]:+.4f}, {g_s[1]:+.4f}, "
                f"{g_s[2]:+.4f}] N/kg"
            )

            self.get_logger().info(f"  Sampling {self._sample_time:.1f} s ...")
            mean, std, n = self._collect_samples(self._sample_time)
            fx, fy, fz, mx, my, mz = mean
            self.get_logger().info(
                f"  n={n}  Fx={fx:+.4f} Fy={fy:+.4f} Fz={fz:+.4f} N  "
                f"Mx={mx:+.5f} My={my:+.5f} Mz={mz:+.5f} Nm"
            )

            measurements.append(
                {"name": name, "g_sensor": g_s, "mean": mean, "std": std, "n": n}
            )

        # ── Least-squares ─────────────────────────────────────────────────
        self.get_logger().info("Running least-squares fit ...")

        grav_vecs = [m["g_sensor"] for m in measurements]
        forces    = [m["mean"][0:3] for m in measurements]
        torques   = [m["mean"][3:6] for m in measurements]

        mass, force_bias, res_f, rank_f = _solve_mass_and_force_bias(
            forces, grav_vecs
        )
        if mass < 0.0:
            grav_vecs = [-g for g in grav_vecs]
            mass, force_bias, res_f, rank_f = _solve_mass_and_force_bias(
                forces, grav_vecs
            )

        forces_corr = [f - force_bias for f in forces]
        com, torque_bias, res_t, rank_t = _solve_com_and_torque_bias(
            forces_corr, torques
        )

        # ── Summary ───────────────────────────────────────────────────────
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info("  CALIBRATION RESULT")
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info(
            f"  Mass           : {mass:.6f} kg  ({mass*G:.5f} N)"
        )
        self.get_logger().info(
            f"  Force bias [N] : [{force_bias[0]:+.6f}, "
            f"{force_bias[1]:+.6f}, {force_bias[2]:+.6f}]"
        )
        self.get_logger().info(
            f"  Torque bias[Nm]: [{torque_bias[0]:+.6f}, "
            f"{torque_bias[1]:+.6f}, {torque_bias[2]:+.6f}]"
        )
        self.get_logger().info(
            f"  CoM [mm]       : [{com[0]*1e3:+.2f}, "
            f"{com[1]*1e3:+.2f}, {com[2]*1e3:+.2f}]"
        )
        self.get_logger().info(
            f"  LSQ rank       : force={rank_f}  torque={rank_t}"
        )
        if len(res_f) > 0:
            self.get_logger().info(f"  Force residual : {res_f[0]:.4e}")
        if len(res_t) > 0:
            self.get_logger().info(f"  Torque residual: {res_t[0]:.4e}")

        # ── Per-pose force residuals (helps spot outlier poses) ───────────
        self.get_logger().info("  Per-pose force residuals [N]:")
        for m, g, f_corr in zip(
            measurements,
            grav_vecs,
            [f - force_bias - mass * g for f, g in zip(forces, grav_vecs)],
        ):
            r = np.linalg.norm(f_corr)
            self.get_logger().info(
                f"    {m['name']:<12} |res|={r:.4f} N  "
                f"({f_corr[0]:+.4f}, {f_corr[1]:+.4f}, {f_corr[2]:+.4f})"
            )

        # ── Hot-swap live calibration ─────────────────────────────────────
        with self._lock:
            self._mass        = float(mass)
            self._com         = com.copy()
            self._force_bias  = force_bias.copy()
            self._torque_bias = torque_bias.copy()

        self._save_calibration(mass, force_bias, torque_bias, com, measurements)
        self.get_logger().info("Calibration applied and saved.")
        return mass, force_bias, torque_bias, com

    # ──────────────────────────────────────────────────────────────────────
    # Service handler
    # ──────────────────────────────────────────────────────────────────────

    def _handle_calibrate(self, request, response):
        del request
        try:
            mass, fb, tb, com = self._run_calibration()
            response.success = True
            response.message = (
                f"Calibration OK  mass={mass:.4f} kg  "
                f"fb=[{fb[0]:+.4f},{fb[1]:+.4f},{fb[2]:+.4f}] N  "
                f"tb=[{tb[0]:+.5f},{tb[1]:+.5f},{tb[2]:+.5f}] Nm  "
                f"com=[{com[0]*1e3:+.2f},{com[1]*1e3:+.2f},{com[2]*1e3:+.2f}] mm"
            )
        except Exception as exc:
            response.success = False
            response.message = f"Calibration failed: {exc}"
            self.get_logger().error(response.message)
        return response

    # ──────────────────────────────────────────────────────────────────────
    # Persistence
    # ──────────────────────────────────────────────────────────────────────

    def _load_calibration(self):
        self.get_logger().info(f"Calibration file path: '{self._calib_file}'")
        if not self._calib_file or not os.path.isfile(self._calib_file):
            self.get_logger().warn(
                f"Calibration file not found: '{self._calib_file}'. "
                "Starting with zero bias — run /ft300/force_torque_cali to calibrate."
            )
            return
        try:
            with open(self._calib_file, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            fb = np.array(data.get("force_bias_N",   [0.0]*3))
            tb = np.array(data.get("torque_bias_Nm", [0.0]*3))
            # reject a file that has never been calibrated (all zeros means
            # no calibration was ever saved to this path)
            if np.all(fb == 0.0) and float(data.get("mass_kg", 0.0)) == 0.0:
                self.get_logger().warn(
                    f"Calibration file '{self._calib_file}' contains only zeros "
                    "(never calibrated). Run /ft300/force_torque_cali first."
                )
            self._force_bias  = fb
            self._torque_bias = tb
            self._tare_force  = np.array(data.get("tare_force_N",   [0.0]*3))
            self._tare_torque = np.array(data.get("tare_torque_Nm", [0.0]*3))
            self._mass        = float(data.get("mass_kg", 0.0))
            self._com         = np.array(data.get("com_m", [0.0]*3))
            self.get_logger().info(
                f"Loaded calibration  mass={self._mass:.4f} kg  "
                f"fb=[{fb[0]:+.4f},{fb[1]:+.4f},{fb[2]:+.4f}] N  "
                f"tb=[{tb[0]:+.5f},{tb[1]:+.5f},{tb[2]:+.5f}] Nm"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load calibration file: {exc}")

    def _save_calibration(
        self,
        mass:         float,
        force_bias:   np.ndarray,
        torque_bias:  np.ndarray,
        com:          np.ndarray,
        measurements: list,
    ):
        data: dict = {}
        if os.path.isfile(self._calib_file):
            try:
                with open(self._calib_file, "r", encoding="utf-8") as fh:
                    data = json.load(fh)
            except Exception:
                data = {}

        data.update({
            "mass_kg":        float(mass),
            "weight_N":       float(mass * G),
            "force_bias_N":   force_bias.tolist(),
            "torque_bias_Nm": torque_bias.tolist(),
            "com_m":          com.tolist(),
            "com_mm":         (com * 1000.0).tolist(),
            "tare_force_N":   data.get("tare_force_N",   [0.0]*3),
            "tare_torque_Nm": data.get("tare_torque_Nm", [0.0]*3),
            "measurements": [
                {
                    "name":     m["name"],
                    "g_sensor": m["g_sensor"].tolist(),
                    "mean":     m["mean"].tolist(),
                    "std":      m["std"].tolist(),
                    "samples":  int(m["n"]),
                }
                for m in measurements
            ],
        })

        with open(self._calib_file, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2)

        self.get_logger().info(f"Calibration saved to '{self._calib_file}'")  # absolute path


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FT300TrajectoryCalibNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
