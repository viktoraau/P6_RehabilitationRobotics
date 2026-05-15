"""Bode frequency-sweep test node.

Two modes:
  ctc          -- injects JointTrajectory sine waves directly into the CTC and
                  measures joint tracking bandwidth per joint.
  full_pipeline -- injects WrenchStamped sine torques into the admittance
                  controller and measures the end-to-end joint response.

Results are saved as CSV and an optional Bode plot PNG.
"""

import csv
import enum
import math
import os

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class _State(enum.Enum):
    INIT = 'INIT'
    PRE_FLIGHT = 'PRE_FLIGHT'
    HOMING = 'HOMING'
    SETTLING = 'SETTLING'
    MEASURING = 'MEASURING'
    ABORT_HOME = 'ABORT_HOME'  # coupling violation → home to zero, then finish
    DONE = 'DONE'


class BodeTestNode(Node):
    """Automated Bode frequency-sweep tester for the rehab robot controllers."""

    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']

    # Hard limits mirror controller_kdl.yaml position_limits_upper (deg → rad)
    _JOINT_LIMITS_RAD = [math.radians(65.0), math.radians(60.0), math.radians(30.0)]

    # Amplitude must stay below this fraction of the joint limit (CTC mode only)
    _SAFETY_FRACTION = 0.70

    # Torque axis driven per joint_index in full_pipeline mode.
    # Axis order: 0=x, 1=y, 2=z  (maps to WrenchStamped torque components)
    # joint_1 (axis -Y) → drive torque -Y  (sign=-1 so positive torque → positive joint motion)
    # joint_2 (axis -Z) → drive torque -Z  (sign=-1)
    # joint_3 (axis +X) → drive torque +X  (sign=+1)
    _TORQUE_AXIS = [1, 2, 0]
    _TORQUE_SIGN = [-1, -1, 1]  # +1 for positive axis, -1 for negative axis joints

    def __init__(self) -> None:
        super().__init__('bode_test_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('test_mode', 'ctc')
        self.declare_parameter('joint_index', 0)
        self.declare_parameter('amplitude', 0.70)
        self.declare_parameter('freq_start_hz', 0.1)
        self.declare_parameter('freq_end_hz', 5.0)
        self.declare_parameter('n_frequencies', 15)
        self.declare_parameter('settle_cycles', 3)
        self.declare_parameter('measure_cycles', 5)
        self.declare_parameter('output_csv', os.path.expanduser('~/bode_results.csv'))
        self.declare_parameter('plot_output', os.path.expanduser('~/bode_plot.png'))
        self.declare_parameter(
            'trajectory_topic', '/joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('wrench_topic', '/ft300/wrench')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('homing_duration_s', 8.0)
        self.declare_parameter('homing_threshold_rad', 0.01)
        self.declare_parameter('coupling_threshold_rad', 0.5)

        # ── Read parameters ───────────────────────────────────────────────────
        self._mode = str(self.get_parameter('test_mode').value)
        if self._mode not in ('ctc', 'full_pipeline'):
            raise ValueError(f"test_mode must be 'ctc' or 'full_pipeline', got '{self._mode}'")

        self._joint_idx = int(self.get_parameter('joint_index').value)
        if self._joint_idx not in (0, 1, 2):
            raise ValueError(f'joint_index must be 0, 1, or 2, got {self._joint_idx}')

        self._amplitude = float(self.get_parameter('amplitude').value)
        f_start = float(self.get_parameter('freq_start_hz').value)
        f_end = float(self.get_parameter('freq_end_hz').value)
        n_freq = int(self.get_parameter('n_frequencies').value)
        self._settle_cycles = int(self.get_parameter('settle_cycles').value)
        self._measure_cycles = int(self.get_parameter('measure_cycles').value)
        self._output_csv = str(self.get_parameter('output_csv').value)
        self._plot_output = str(self.get_parameter('plot_output').value)
        traj_topic = str(self.get_parameter('trajectory_topic').value)
        wrench_topic = str(self.get_parameter('wrench_topic').value)
        js_topic = str(self.get_parameter('joint_state_topic').value)
        self._ctrl_rate = float(self.get_parameter('control_rate_hz').value)
        self._homing_duration = float(self.get_parameter('homing_duration_s').value)
        self._homing_threshold = float(self.get_parameter('homing_threshold_rad').value)
        self._coupling_threshold = float(self.get_parameter('coupling_threshold_rad').value)

        # ── Frequency sweep (log-spaced) ──────────────────────────────────────
        self._freqs: list[float] = np.logspace(
            np.log10(f_start), np.log10(f_end), n_freq
        ).tolist()
        self._freq_idx = 0

        # ── State ─────────────────────────────────────────────────────────────
        self._state = _State.INIT
        self._q = np.zeros(3, dtype=np.float64)
        self._have_js = False

        self._t_state_start = 0.0  # wall time when current state was entered
        self._t_phase_start = 0.0  # wall time when SETTLING started (sine phase origin)

        self._measure_buf_t: list[float] = []
        self._measure_buf_q: list[float] = []
        self._q_freq_start = np.zeros(3, dtype=np.float64)  # snapshot at start of each freq

        # Results: list of dicts with keys freq, gain_db, phase_deg, A_out, A_in
        self._results: list[dict] = []

        # ── Publishers ────────────────────────────────────────────────────────
        if self._mode == 'ctc':
            self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        else:
            self._wrench_pub = self.create_publisher(WrenchStamped, wrench_topic, 10)

        # ── Service clients (full_pipeline only) ─────────────────────────────
        # reinit_reference resets q_ref = q_des = identity in the admittance
        # so the stiffness spring actively holds zero during homing.
        if self._mode == 'full_pipeline':
            self._reinit_client = self.create_client(Trigger, 'admittance/reinit_reference')

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(JointState, js_topic, self._js_cb, 10)

        # ── Control timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / max(self._ctrl_rate, 1.0), self._tick)

        self.get_logger().info(
            f'Bode test node ready. mode={self._mode}, '
            f'joint={self.JOINT_NAMES[self._joint_idx]}, '
            f'amplitude={self._amplitude:.4f} {"rad" if self._mode == "ctc" else "Nm"}, '
            f'sweep={f_start:.2f}–{f_end:.2f} Hz ({n_freq} points). '
            'Waiting for /joint_states …'
        )

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, jname in enumerate(self.JOINT_NAMES):
            if jname in name_to_pos:
                self._q[i] = name_to_pos[jname]
        self._have_js = True

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        t = self._now_s()

        if self._state == _State.INIT:
            if not self._have_js:
                return
            self.get_logger().info('Joint states received. Running pre-flight check …')
            self._state = _State.PRE_FLIGHT
            self._preflight()

        elif self._state == _State.PRE_FLIGHT:
            pass  # _preflight() transitions to HOMING or raises

        elif self._state == _State.HOMING:
            elapsed = t - self._t_state_start
            self._publish_homing()
            settled = bool(np.all(np.abs(self._q) < self._homing_threshold))
            if settled or elapsed >= self._homing_duration:
                if not settled:
                    self.get_logger().warn(
                        f'Homing timeout after {elapsed:.1f} s. '
                        f'Current q={np.degrees(self._q).round(1)} deg. Proceeding anyway.'
                    )
                else:
                    self.get_logger().info('Homed to zero. Starting frequency sweep.')
                self._enter_frequency(0)

        elif self._state == _State.SETTLING:
            f = self._freqs[self._freq_idx]
            t_rel = t - self._t_phase_start
            self._publish_sine(f, t_rel)
            if self._coupling_violated(f):
                return
            if t - self._t_state_start >= self._settle_cycles / f:
                self._state = _State.MEASURING
                self._t_state_start = t
                self._measure_buf_t.clear()
                self._measure_buf_q.clear()
                self.get_logger().info(
                    f'  [{self._freq_idx + 1}/{len(self._freqs)}] '
                    f'f={f:.3f} Hz — measuring …'
                )

        elif self._state == _State.MEASURING:
            f = self._freqs[self._freq_idx]
            t_rel = t - self._t_phase_start
            self._publish_sine(f, t_rel)
            if self._coupling_violated(f):
                return
            self._measure_buf_t.append(t_rel)
            self._measure_buf_q.append(float(self._q[self._joint_idx]))
            if t - self._t_state_start >= self._measure_cycles / f:
                self._analyze(f)

        elif self._state == _State.ABORT_HOME:
            elapsed = t - self._t_state_start
            self._publish_homing()
            settled = bool(np.all(np.abs(self._q) < self._homing_threshold))
            if settled or elapsed >= self._homing_duration:
                if not settled:
                    self.get_logger().warn(
                        f'Homing after abort timed out ({elapsed:.1f} s). '
                        'Robot may not be at zero — stop manually if needed.'
                    )
                else:
                    self.get_logger().info('Returned to zero after abort.')
                self._finish()

        elif self._state == _State.DONE:
            pass

    # ── State transitions ─────────────────────────────────────────────────────

    def _preflight(self) -> None:
        """Validate parameters; abort on safety violation."""
        if self._mode == 'ctc':
            limit = self._JOINT_LIMITS_RAD[self._joint_idx] * self._SAFETY_FRACTION
            if self._amplitude > limit:
                self.get_logger().fatal(
                    f'PRE-FLIGHT FAIL: amplitude={math.degrees(self._amplitude):.1f} deg '
                    f'exceeds safety limit {math.degrees(limit):.1f} deg '
                    f'({self._SAFETY_FRACTION * 100:.0f}% of '
                    f'{math.degrees(self._JOINT_LIMITS_RAD[self._joint_idx]):.0f} deg) '
                    f'for {self.JOINT_NAMES[self._joint_idx]}. '
                    'Reduce amplitude and restart.'
                )
                self._state = _State.DONE
                rclpy.shutdown()
                return

        self.get_logger().info(
            f'Pre-flight OK. Homing to zero (timeout={self._homing_duration:.1f} s) …'
        )
        if self._mode == 'full_pipeline':
            # Reset the admittance q_ref and q_des to identity (zero rotation)
            # so the stiffness spring actively drives q_des to zero during homing.
            # Without this, q_ref = q_des = TF-at-startup (possibly non-zero)
            # and zero wrench would simply freeze the robot at the startup pose.
            if self._reinit_client.service_is_ready():
                future = self._reinit_client.call_async(Trigger.Request())
                self.get_logger().info('Called admittance/reinit_reference — reference reset to zero.')
            else:
                self.get_logger().warn(
                    'admittance/reinit_reference not available — homing may fail if '
                    'the admittance was initialised at a non-zero pose.'
                )
        self._t_state_start = self._now_s()
        self._state = _State.HOMING

    def _enter_frequency(self, idx: int) -> None:
        f = self._freqs[idx]
        self._freq_idx = idx
        t = self._now_s()
        self._t_phase_start = t
        self._t_state_start = t
        self._q_freq_start = self._q.copy()
        self._state = _State.SETTLING
        self.get_logger().info(
            f'  [{idx + 1}/{len(self._freqs)}] f={f:.3f} Hz — settling '
            f'({self._settle_cycles} cycles = {self._settle_cycles / f:.1f} s) …'
        )

    def _coupling_violated(self, f: float) -> bool:
        """Check non-tested joints for motion relative to their position at frequency start."""
        for i, jname in enumerate(self.JOINT_NAMES):
            if i == self._joint_idx:
                continue
            deviation = abs(self._q[i] - self._q_freq_start[i])
            if deviation > self._coupling_threshold:
                self.get_logger().warn(
                    f'ABORT: {jname} moved {math.degrees(deviation):.1f} deg from its '
                    f'start position (threshold {math.degrees(self._coupling_threshold):.1f} deg) '
                    f'at f={f:.3f} Hz — homing to zero …'
                )
                self._t_state_start = self._now_s()
                self._state = _State.ABORT_HOME
                return True
        return False

    # ── Publishing ────────────────────────────────────────────────────────────

    def _publish_homing(self) -> None:
        if self._mode == 'ctc':
            self._publish_traj(
                positions=[0.0, 0.0, 0.0],
                velocities=[0.0, 0.0, 0.0],
                accelerations=[0.0, 0.0, 0.0],
            )
        else:
            # Zero wrench: the admittance stiffness spring (K*(q_des-q_ref))
            # now pulls q_des toward identity because reinit_reference was called
            # in preflight, so q_ref = q_des = identity = zero rotation.
            self._publish_wrench(tx=0.0, ty=0.0, tz=0.0)

    def _publish_sine(self, f: float, t_rel: float) -> None:
        omega = 2.0 * math.pi * f
        pos_val = self._amplitude * math.sin(omega * t_rel)
        vel_val = self._amplitude * omega * math.cos(omega * t_rel)
        acc_val = -self._amplitude * omega * omega * math.sin(omega * t_rel)

        if self._mode == 'ctc':
            positions = [0.0, 0.0, 0.0]
            velocities = [0.0, 0.0, 0.0]
            accelerations = [0.0, 0.0, 0.0]
            positions[self._joint_idx] = pos_val
            velocities[self._joint_idx] = vel_val
            accelerations[self._joint_idx] = acc_val
            self._publish_traj(positions, velocities, accelerations)
        else:
            torque = [0.0, 0.0, 0.0]
            torque[self._TORQUE_AXIS[self._joint_idx]] = (
                self._TORQUE_SIGN[self._joint_idx] * pos_val
            )
            self._publish_wrench(tx=torque[0], ty=torque[1], tz=torque[2])

    def _publish_traj(
        self,
        positions: list,
        velocities: list,
        accelerations: list,
    ) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.velocities = velocities
        pt.accelerations = accelerations
        pt.time_from_start = Duration(sec=0, nanosec=10_000_000)  # 10 ms
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def _publish_wrench(self, tx: float, ty: float, tz: float) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Use base_link frame so the admittance controller uses the torque
        # directly without a TF lookup that may fail at startup.
        msg.header.frame_id = 'base_link'
        msg.wrench.torque.x = tx
        msg.wrench.torque.y = ty
        msg.wrench.torque.z = tz
        self._wrench_pub.publish(msg)

    # ── Analysis ──────────────────────────────────────────────────────────────

    def _analyze(self, f: float) -> None:
        t_arr = np.asarray(self._measure_buf_t, dtype=np.float64)
        y_arr = np.asarray(self._measure_buf_q, dtype=np.float64)
        n = len(y_arr)

        if n < 4:
            self.get_logger().warn(f'Too few samples ({n}) at f={f:.3f} Hz — skipping.')
            self._advance()
            return

        # Remove DC offset (any static offset from gravity, etc.)
        y_arr = y_arr - np.mean(y_arr)

        # Sinusoidal correlation at test frequency.
        # For y(t) = A_out * sin(omega*t + phi):
        #   c = (2/N) * sum(y * sin(omega*t))  →  A_out * cos(phi)
        #   s = (2/N) * sum(y * cos(omega*t))  →  A_out * sin(phi)
        omega = 2.0 * math.pi * f
        sin_ref = np.sin(omega * t_arr)
        cos_ref = np.cos(omega * t_arr)
        c = 2.0 / n * float(np.dot(y_arr, sin_ref))
        s = 2.0 / n * float(np.dot(y_arr, cos_ref))

        A_out = math.sqrt(c * c + s * s)
        phase_rad = math.atan2(s, c)

        if A_out < 1e-9:
            self.get_logger().warn(f'Near-zero output amplitude at f={f:.3f} Hz.')
            gain_db = -99.0
        else:
            gain_db = 20.0 * math.log10(A_out / self._amplitude)

        phase_deg = math.degrees(phase_rad)

        self._results.append({
            'frequency_hz': f,
            'gain_db': gain_db,
            'phase_deg': phase_deg,
            'amplitude_out': A_out,
            'amplitude_in': self._amplitude,
            'joint_index': self._joint_idx,
            'test_mode': self._mode,
        })

        self.get_logger().info(
            f'    → gain={gain_db:+.2f} dB, phase={phase_deg:+.1f} deg, '
            f'A_out={A_out:.4f}'
        )
        self._advance()

    def _advance(self) -> None:
        next_idx = self._freq_idx + 1
        if next_idx < len(self._freqs):
            self._enter_frequency(next_idx)
        else:
            self._finish()

    def _finish(self) -> None:
        self._state = _State.DONE
        self.get_logger().info(
            f'Sweep complete ({len(self._results)} points). Saving results …'
        )
        self._save_csv()
        if self._plot_output:
            self._save_plot()
        self.get_logger().info('Done. You may now stop the node (Ctrl-C).')

    # ── Output ────────────────────────────────────────────────────────────────

    def _save_csv(self) -> None:
        fieldnames = [
            'frequency_hz', 'gain_db', 'phase_deg',
            'amplitude_out', 'amplitude_in', 'joint_index', 'test_mode',
        ]
        out_path = os.path.expanduser(self._output_csv)
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._results)
        self.get_logger().info(f'CSV saved → {out_path}')

    def _save_plot(self) -> None:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn('matplotlib not available — skipping plot.')
            return

        freqs = [r['frequency_hz'] for r in self._results]
        gains = [r['gain_db'] for r in self._results]
        # Unwrap phase to remove ±180° discontinuities caused by atan2 wrapping.
        phases = np.degrees(
            np.unwrap(np.radians([r['phase_deg'] for r in self._results]))
        ).tolist()

        joint_name = self.JOINT_NAMES[self._joint_idx]
        title = (
            f'Bode plot — {self._mode} mode, {joint_name}, '
            f'A={self._amplitude:.3f} {"rad" if self._mode == "ctc" else "Nm"}'
        )

        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        fig.suptitle(title, fontsize=11)

        ax1.semilogx(freqs, gains, 'b-o', markersize=5)
        ax1.axhline(0.0, color='k', linewidth=0.8, linestyle='--')
        ax1.axhline(-3.0, color='r', linewidth=0.8, linestyle='--', label='-3 dB')
        ax1.set_ylabel('Gain [dB]')
        ax1.grid(True, which='both', alpha=0.4)
        ax1.legend(fontsize=8)

        ax2.semilogx(freqs, phases, 'g-o', markersize=5)
        ax2.axhline(0.0, color='k', linewidth=0.8, linestyle='--')
        ax2.axhline(-90.0, color='r', linewidth=0.8, linestyle='--', label='-90°')
        ax2.set_ylabel('Phase [deg]')
        ax2.set_xlabel('Frequency [Hz]')
        ax2.grid(True, which='both', alpha=0.4)
        ax2.legend(fontsize=8)

        plt.tight_layout()
        out_path = os.path.expanduser(self._plot_output)
        plt.savefig(out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Bode plot saved → {out_path}')

    # ── Utility ───────────────────────────────────────────────────────────────

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BodeTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
