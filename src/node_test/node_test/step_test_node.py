"""Step-response accuracy and readability test node.

Sends a sequence of random joint-space position commands to the computed
torque controller (via JointTrajectory on
``/joint_trajectory_controller/joint_trajectory``) and records the desired
vs. achieved joint positions.  When all waypoints are done the data is
written to a CSV file.

State machine
─────────────
  INIT       – wait for the first /joint_states message
  HOMING     – drive all joints to zero and wait to settle
  MOVING     – command the next random target; wait to settle
  RECORDING  – collect ``record_hold_s`` seconds of settled data, then advance
  DONE       – flush CSV and shut down

Each waypoint is considered "settled" once the maximum absolute joint error
has been below ``settle_threshold_rad`` continuously for ``settle_time_s``.

CSV columns (one row per waypoint):
  waypoint, elapsed_s,
  des_j1, des_j2, des_j3,
  ach_j1, ach_j2, ach_j3,
  err_j1, err_j2, err_j3
"""

import csv
import enum
import math
import os
import random
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ── State machine ─────────────────────────────────────────────────────────────

class _State(enum.Enum):
    INIT      = 'INIT'
    HOMING    = 'HOMING'
    MOVING    = 'MOVING'
    RECORDING = 'RECORDING'
    DONE      = 'DONE'


# ── Node ──────────────────────────────────────────────────────────────────────

class StepTestNode(Node):
    """Sends random step commands and logs desired vs. achieved positions."""

    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']

    # Hard joint limits [rad] – mirror controller_kdl.yaml position_limits
    _LIMITS_RAD = [
        math.radians(65.0),   # ±joint_1
        math.radians(60.0),   # ±joint_2
        math.radians(30.0),   # ±joint_3
    ]

    # Fraction of the hard limit used for random target generation.
    # Keeps the robot away from the physical stops during the test.
    _LIMIT_FRACTION = 0.80

    def __init__(self) -> None:
        super().__init__('step_test_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('n_waypoints', 10)
        self.declare_parameter('settle_threshold_rad', 0.015)
        self.declare_parameter('settle_time_s', 1.0)
        self.declare_parameter('record_hold_s', 0.5)
        self.declare_parameter('move_duration_s', 3.0)
        self.declare_parameter('republish_interval_s', 1.0)
        self.declare_parameter('timeout_s', 60.0)
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('random_seed', -1)            # -1 = truly random
        self.declare_parameter(
            'trajectory_topic',
            '/joint_trajectory_controller/joint_trajectory',
        )
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter(
            'output_csv',
            os.path.expanduser('~/step_test_results.csv'),
        )

        # ── Read parameters ───────────────────────────────────────────────────
        self._n_wp          = int(self.get_parameter('n_waypoints').value)
        self._settle_thr    = float(self.get_parameter('settle_threshold_rad').value)
        self._settle_time   = float(self.get_parameter('settle_time_s').value)
        self._record_hold   = float(self.get_parameter('record_hold_s').value)
        self._move_dur      = float(self.get_parameter('move_duration_s').value)
        self._republish_iv  = float(self.get_parameter('republish_interval_s').value)
        self._timeout       = float(self.get_parameter('timeout_s').value)
        self._ctrl_rate     = float(self.get_parameter('control_rate_hz').value)
        seed                = int(self.get_parameter('random_seed').value)
        traj_topic          = str(self.get_parameter('trajectory_topic').value)
        js_topic            = str(self.get_parameter('joint_state_topic').value)
        self._output_csv    = str(self.get_parameter('output_csv').value)

        # ── RNG ───────────────────────────────────────────────────────────────
        rng_seed = seed if seed >= 0 else None
        rng = random.Random(rng_seed)
        np.random.seed(rng_seed)

        # ── Generate random waypoints ─────────────────────────────────────────
        # Home position (all zeros) is always the first waypoint.
        self._waypoints: list[np.ndarray] = [np.zeros(3)]
        for _ in range(self._n_wp):
            target = np.array([
                rng.uniform(-lim * self._LIMIT_FRACTION,
                             lim * self._LIMIT_FRACTION)
                for lim in self._LIMITS_RAD
            ])
            self._waypoints.append(target)
        # Return home at the end
        self._waypoints.append(np.zeros(3))

        self.get_logger().info(
            f'Generated {len(self._waypoints)} waypoints '
            f'(home + {self._n_wp} random + home). '
            f'seed={rng_seed}'
        )

        # ── Internal state ────────────────────────────────────────────────────
        self._state          = _State.INIT
        self._wp_idx         = 0           # index into self._waypoints
        self._q_actual       = np.zeros(3)
        self._have_js        = False
        self._phase_t0       = 0.0         # wall-clock time entering current state
        self._settled_since  = None        # wall-clock time when settle began
        self._record_buf: list[np.ndarray] = []  # actual positions during RECORDING
        self._results: list[dict]          = []
        self._wp_t0          = 0.0         # wall-clock time when MOVING started
        self._last_pub_t     = 0.0         # wall-clock time of last trajectory publish

        # ── ROS interface ─────────────────────────────────────────────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory, traj_topic, 10
        )
        self._js_sub = self.create_subscription(
            JointState, js_topic,
            self._js_cb, 10,
        )

        dt = 1.0 / max(self._ctrl_rate, 1.0)
        self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'StepTestNode started.  '
            f'trajectory_topic={traj_topic!r}  '
            f'joint_state_topic={js_topic!r}  '
            f'output_csv={self._output_csv!r}'
        )

    # ── Subscriber ────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        """Cache the latest measured joint positions in JOINT_NAMES order."""
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, jn in enumerate(self.JOINT_NAMES):
            if jn in name_to_pos:
                self._q_actual[i] = name_to_pos[jn]
        self._have_js = True

    # ── Publisher helper ──────────────────────────────────────────────────────

    def _send_target(self, q_des: np.ndarray) -> None:
        """Publish a single-point JointTrajectory to the controller."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions    = q_des.tolist()
        pt.velocities   = [0.0] * 3
        pt.accelerations = [0.0] * 3

        # time_from_start: how long the controller has to reach the target
        sec_int = int(self._move_dur)
        nsec    = int((self._move_dur - sec_int) * 1e9)
        pt.time_from_start = Duration(sec=sec_int, nanosec=nsec)

        msg.points = [pt]
        self._traj_pub.publish(msg)

    # ── Tick (main state machine) ─────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()

        # ── INIT: wait for first joint-state ─────────────────────────────────
        if self._state == _State.INIT:
            if self._have_js:
                self.get_logger().info('Joint states received – homing …')
                self._transition_to_moving(now)
            return

        # ── MOVING: command sent, wait to settle ──────────────────────────────
        if self._state == _State.MOVING:
            q_des  = self._waypoints[self._wp_idx]
            error  = np.abs(self._q_actual - q_des)
            max_err = float(np.max(error))

            # Re-publish the target at regular intervals so the controller
            # always has a fresh setpoint (covers message drops and
            # time_from_start expiry on trajectory-tracking controllers).
            if (now - self._last_pub_t) >= self._republish_iv:
                self._send_target(q_des)
                self._last_pub_t = now

            if max_err < self._settle_thr:
                # Start (or continue) the settle timer
                if self._settled_since is None:
                    self._settled_since = now
                elif (now - self._settled_since) >= self._settle_time:
                    # Settled long enough → enter recording
                    self.get_logger().info(
                        f'[WP {self._wp_idx}] Settled.  '
                        f'max_err={math.degrees(max_err):.3f}°  '
                        f'→ recording …'
                    )
                    self._state = _State.RECORDING
                    self._record_buf.clear()
                    self._phase_t0 = now
            else:
                # Error grew again → reset settle timer
                self._settled_since = None

            # Timeout guard
            if (now - self._wp_t0) > self._timeout:
                self.get_logger().warn(
                    f'[WP {self._wp_idx}] Timeout after {self._timeout:.1f}s.  '
                    f'max_err={math.degrees(max_err):.3f}°  '
                    f'Recording best-effort position.'
                )
                # Record whatever position was achieved
                self._record_and_advance(now)
            return

        # ── RECORDING: accumulate samples for record_hold_s ──────────────────
        if self._state == _State.RECORDING:
            self._record_buf.append(self._q_actual.copy())
            if (now - self._phase_t0) >= self._record_hold:
                self._record_and_advance(now)
            return

        # ── DONE: nothing left to do ──────────────────────────────────────────
        if self._state == _State.DONE:
            return

    def _record_and_advance(self, now: float) -> None:
        """Store result for current waypoint and move to the next."""
        q_des = self._waypoints[self._wp_idx]

        if self._record_buf:
            q_ach = np.mean(self._record_buf, axis=0)
        else:
            q_ach = self._q_actual.copy()

        err = q_des - q_ach

        label = 'home' if self._wp_idx == 0 else (
            'return' if self._wp_idx == len(self._waypoints) - 1
            else str(self._wp_idx)
        )

        result = {
            'waypoint':  label,
            'elapsed_s': round(now - self._wp_t0, 3),
            'des_j1': round(float(q_des[0]), 6),
            'des_j2': round(float(q_des[1]), 6),
            'des_j3': round(float(q_des[2]), 6),
            'ach_j1': round(float(q_ach[0]), 6),
            'ach_j2': round(float(q_ach[1]), 6),
            'ach_j3': round(float(q_ach[2]), 6),
            'err_j1': round(float(err[0]), 6),
            'err_j2': round(float(err[1]), 6),
            'err_j3': round(float(err[2]), 6),
        }
        self._results.append(result)

        self.get_logger().info(
            f'[WP {label}]  '
            f'des=[{math.degrees(q_des[0]):.1f}, '
            f'{math.degrees(q_des[1]):.1f}, '
            f'{math.degrees(q_des[2]):.1f}]°  '
            f'ach=[{math.degrees(q_ach[0]):.1f}, '
            f'{math.degrees(q_ach[1]):.1f}, '
            f'{math.degrees(q_ach[2]):.1f}]°  '
            f'err=[{math.degrees(err[0]):.2f}, '
            f'{math.degrees(err[1]):.2f}, '
            f'{math.degrees(err[2]):.2f}]°'
        )

        # Advance to next waypoint or finish
        self._wp_idx += 1
        if self._wp_idx >= len(self._waypoints):
            self._finish()
        else:
            self._transition_to_moving(time.monotonic())

    def _transition_to_moving(self, now: float) -> None:
        """Send the next waypoint and enter MOVING state."""
        q_des = self._waypoints[self._wp_idx]
        self._send_target(q_des)
        self._state         = _State.MOVING
        self._wp_t0         = now
        self._last_pub_t    = now
        self._settled_since = None
        label = 'home' if self._wp_idx == 0 else (
            'return' if self._wp_idx == len(self._waypoints) - 1
            else str(self._wp_idx)
        )
        self.get_logger().info(
            f'[WP {label}]  Commanding  '
            f'[{math.degrees(q_des[0]):.1f}, '
            f'{math.degrees(q_des[1]):.1f}, '
            f'{math.degrees(q_des[2]):.1f}]° …'
        )

    def _finish(self) -> None:
        """Write CSV, log summary, transition to DONE."""
        self._state = _State.DONE
        self._save_csv()

        if self._results:
            errs = np.array([
                [r['err_j1'], r['err_j2'], r['err_j3']]
                for r in self._results
            ])
            abs_errs = np.abs(errs)
            rmse = np.sqrt(np.mean(errs ** 2, axis=0))
            self.get_logger().info('─' * 60)
            self.get_logger().info('STEP TEST COMPLETE – Accuracy Summary')
            self.get_logger().info(
                f'  RMSE  (deg): '
                f'j1={math.degrees(rmse[0]):.3f}  '
                f'j2={math.degrees(rmse[1]):.3f}  '
                f'j3={math.degrees(rmse[2]):.3f}'
            )
            self.get_logger().info(
                f'  Max |err| (deg): '
                f'j1={math.degrees(np.max(abs_errs[:, 0])):.3f}  '
                f'j2={math.degrees(np.max(abs_errs[:, 1])):.3f}  '
                f'j3={math.degrees(np.max(abs_errs[:, 2])):.3f}'
            )
            self.get_logger().info(f'  Results saved → {self._output_csv}')
            self.get_logger().info('─' * 60)

        # Ask ROS to shut down this node
        rclpy.shutdown()

    def _save_csv(self) -> None:
        """Write self._results to CSV."""
        if not self._results:
            self.get_logger().warn('No results to save.')
            return

        os.makedirs(os.path.dirname(os.path.abspath(self._output_csv)), exist_ok=True)
        fieldnames = [
            'waypoint', 'elapsed_s',
            'des_j1', 'des_j2', 'des_j3',
            'ach_j1', 'ach_j2', 'ach_j3',
            'err_j1', 'err_j2', 'err_j3',
        ]
        with open(self._output_csv, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._results)
        self.get_logger().info(f'CSV written: {self._output_csv}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
