"""Gear calibration for the 3-joint rehabilitation robot.

Drives each joint to both mechanical end stops (detected via torque + stalled
velocity), records the reported end-stop positions, converts them back through
the current gear ratio to motor-side travel, then writes updated gearing and
limits to the motor .cfg.

  Physical output ranges: joint_1=130°  joint_2=60°  joint_3=120°
  Reference gear ratios:  joint_1=1/35  joint_2=0.1  joint_3=0.1
  Encoder location:       motor shaft / main encoder
                          reported positions are scaled by the current drive gear ratio
  Calibration order:      joint_3 → joint_2 → joint_1

Requires:
  - joint_group_velocity_controller active
  - /dynamic_joint_states published (joint_state_broadcaster)

Usage:
  ros2 run calibration_pkg gear_calibration
"""

import configparser
import math
import os
import sys
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class Phase(Enum):
    FIND_FIRST  = auto()   # drive in +ve direction until end stop
    FIND_SECOND = auto()   # reversed; drive to the opposite end stop
    CENTERING   = auto()   # bang-bang move to measured mid-point
    COMPLETE    = auto()   # this joint is done


# ---------------------------------------------------------------------------
# Per-joint config + runtime state
# ---------------------------------------------------------------------------

@dataclass
class JointData:
    # Static config
    name: str
    vel_axis: int            # index in the 3-element velocity-command array
    torque_limit_pos: float  # end-stop torque threshold (Nm) driving in +ve direction
    torque_limit_neg: float  # end-stop torque threshold (Nm) driving in -ve direction
    speed: float             # drive speed magnitude (motor-shaft rad/s)
    seek_speed_min: float    # minimum speed used by the seek ramp controller
    seek_kp: float           # seek velocity PI proportional gain
    seek_ki: float           # seek velocity PI integral gain
    seek_integral_limit: float  # clamp for the PI integral term
    seek_ramp_rate: float    # maximum speed ramp during end-stop seeking (rad/s^2)
    range_deg: float         # known physical output range (degrees, full sweep)
    tuned_gear_ratio: float  # reference gearbox ratio (output / motor)
    cfg_path: str            # absolute path to the .cfg file to overwrite

    # Interface indices — filled on first DynamicJointState message
    msg_idx:  int = -1
    pos_idx:  int = -1
    vel_idx:  int = -1
    eff_idx:  int = -1
    volt_idx: int = -1

    # State machine
    phase: Phase = Phase.FIND_FIRST
    extremities: List[Optional[float]] = field(default_factory=lambda: [None, None])

    # Calibration results (written by _write_cfg, read by _print_cfg_summary)
    cal_min: Optional[float] = None
    cal_max: Optional[float] = None
    cal_gear_ratio: Optional[float] = None
    cfg_update_result: Optional[str] = None

    # Timing / hysteresis helpers
    _sign:         int   = 1
    _last_event_t: float = field(default_factory=time.time)
    _prev_abs_vel: float = 0.0
    _last_seek_update_t: float = field(default_factory=time.time)
    _seek_cmd_mag: float = 0.0
    _seek_integral: float = 0.0

    @property
    def active_torque_limit(self) -> float:
        return self.torque_limit_pos if self._sign > 0 else self.torque_limit_neg


def _motor_cfg_dir_candidates() -> List[Tuple[Path, str]]:
    """Return likely motor cfg directories, preferring editable source trees."""
    module_path = Path(__file__).resolve()
    seen = set()
    candidates: List[Tuple[Path, str]] = []
    package_names = ("mab_rehab", "mab_ros2_control")

    def remember(path: Path, reason: str) -> None:
        normalized = os.path.normpath(str(path))
        if normalized in seen:
            return
        seen.add(normalized)
        candidates.append((Path(normalized), reason))

    for base in (module_path.parent, *module_path.parents):
        for package_name in package_names:
            remember(
                base / package_name / "config" / "motors",
                f"workspace search via {base}",
            )
            remember(
                base / "src" / package_name / "config" / "motors",
                f"workspace search via {base / 'src'}",
            )

    for package_name in package_names:
        try:
            share_dir = Path(get_package_share_directory(package_name)).resolve()
        except PackageNotFoundError:
            continue
        remember(
            share_dir / "config" / "motors",
            f"package share {share_dir}",
        )

    return candidates


def _discover_motor_cfg_dir() -> Tuple[str, str]:
    """Pick the first existing cfg directory, or return a best-effort fallback."""
    candidates = _motor_cfg_dir_candidates()
    for candidate, reason in candidates:
        if candidate.is_dir():
            return str(candidate), reason
    if candidates:
        candidate, reason = candidates[0]
        return str(candidate), f"{reason} (missing at startup)"
    fallback = Path(__file__).resolve().parent / "mab_rehab" / "config" / "motors"
    return str(fallback), "local fallback (missing at startup)"


# ---------------------------------------------------------------------------
# Calibration node
# ---------------------------------------------------------------------------

class GearCalibration(Node):

    # Detection thresholds
    _STALL_VEL  = 0.5   # rad/s  — "near-stalled" speed for end-stop detection
    _EVENT_GAP  = 2.0   # s      — quiet period after each direction change
    _CENTRE_TOL = 0.05  # rad    — acceptable centring error (motor-shaft rad)
    _MIN_RANGE_FACTOR = 0.5
    _MAX_RANGE_FACTOR = 1.5
    def __init__(self) -> None:
        super().__init__("gear_calibration")

        cfg_dir, cfg_origin = _discover_motor_cfg_dir()
        if os.path.isdir(cfg_dir):
            self.get_logger().info(
                f"Using motor cfg directory: {cfg_dir} ({cfg_origin})."
            )
        else:
            self.get_logger().warn(
                "Motor cfg directory was not found at startup. "
                "The node will search again before writing each cfg. "
                f"Initial candidate: {cfg_dir} ({cfg_origin})."
            )

        # Commands are published in joint order.
        self._joints: List[JointData] = [

            JointData(
                name="joint_2",  vel_axis=1,
                torque_limit_pos=4.5,  torque_limit_neg=5.2,
                speed=10.0, seek_speed_min=6.0, seek_kp=2.0, seek_ki=0.6,
                seek_integral_limit=8.0, seek_ramp_rate=5.5,
                range_deg=60.0, tuned_gear_ratio=0.1,
                cfg_path=os.path.join(cfg_dir, "joint_2.cfg"),
            ),
            JointData(
                name="joint_3",  vel_axis=2,
                torque_limit_pos=3.2,  torque_limit_neg=6.0,
                speed=10.0, seek_speed_min=4.0, seek_kp=1.0, seek_ki=0.6,
                seek_integral_limit=2.0, seek_ramp_rate=4.0,
                range_deg=120.0, tuned_gear_ratio=0.1,
                cfg_path=os.path.join(cfg_dir, "joint_3.cfg"),
            ),
            JointData(
                name="joint_1",  vel_axis=0,
                torque_limit_pos=1.5,  torque_limit_neg=7.6,
                speed=9.0, seek_speed_min=2.0, seek_kp=1.0, seek_ki=0.6,
                seek_integral_limit=5.0, seek_ramp_rate=2.0,
                range_deg=130.0, tuned_gear_ratio=1.0 / 36.0,
                cfg_path=os.path.join(cfg_dir, "joint_1.cfg"),
            ),


        ]
        for jd in self._joints:
            self._reset_seek_controller(jd)
        self._active: int = 2   # calibration starts with joint_3

        self._sub = self.create_subscription(
            DynamicJointState,
            "/dynamic_joint_states",
            self._on_state,
            10,
        )
        self._vel_pub = self.create_publisher(
            Float64MultiArray,
            "/joint_group_velocity_controller/commands",
            10,
        )
        self.get_logger().info(
            "GearCalibration node started.\n"
            "  Output ranges: joint_1=130°  joint_2=60°  joint_3=120°\n"
            "  Reference gear ratios: joint_1=1/35  joint_2=0.1  joint_3=0.1\n"
            "  Encoder: main encoder on the motor shaft — positions reported in radians.\n"
            "  Starting with joint_3."
        )

    # -------------------------------------------------------------------------
    # ROS callback
    # -------------------------------------------------------------------------

    def _on_state(self, msg: DynamicJointState) -> None:
        try:
            self._resolve_indices(msg)

            if self._active < 0:
                return  # all joints done

            jd = self._joints[self._active]
            if jd.msg_idx < 0:
                return  # joint not yet visible in any message

            iv = msg.interface_values[jd.msg_idx]
            #skip 941 and 940 only now 
            
            # Skip if the drive is absent (NaN voltage)
            if jd.volt_idx < 0 or math.isnan(iv.values[jd.volt_idx]):
                jd.cfg_update_result = "skipped: drive absent"
                self.get_logger().warn(f"{jd.name}: drive absent — skipping.")
                self._advance()
                return
            

            pos = iv.values[jd.pos_idx]
            vel = iv.values[jd.vel_idx]
            eff = iv.values[jd.eff_idx]

            self.get_logger().info(
                f"{jd.name} | {jd.phase.name:12s} | "
                f"pos={pos:+.4f}  vel={vel:+.4f}  eff={eff:+.4f}"
            )

            if jd.phase in (Phase.FIND_FIRST, Phase.FIND_SECOND):
                self._seek_end_stop(jd, pos, vel, eff)
            elif jd.phase is Phase.CENTERING:
                self._centre(jd, pos)

        except Exception as exc:
            self.get_logger().error(f"Callback error: {exc}")

    # -------------------------------------------------------------------------
    # End-stop detection
    # -------------------------------------------------------------------------

    def _seek_end_stop(
        self, jd: JointData, pos: float, vel: float, eff: float
    ) -> None:
        now      = time.time()
        abs_vel  = abs(vel)
        past_gap = now > jd._last_event_t + self._EVENT_GAP

        end_stop_hit = (
            past_gap
            and abs(eff) > jd.active_torque_limit
            and abs_vel  < self._STALL_VEL
            and jd._prev_abs_vel >= abs_vel   # velocity is not increasing
        )

        if end_stop_hit:
            slot = 0 if jd.phase is Phase.FIND_FIRST else 1
            jd.extremities[slot] = pos
            self.get_logger().info(
                f"{jd.name}: end stop [{slot}]  pos={pos:.4f} rad  eff={eff:.4f} Nm"
            )

            if jd.phase is Phase.FIND_FIRST:
                jd._sign = -1
                jd.phase  = Phase.FIND_SECOND
                self._reset_seek_controller(jd)
            else:
                jd.phase  = Phase.CENTERING

            jd._last_event_t = now
            self._pub_vel(jd, 0.0)
            time.sleep(0.3)   # brief settle before next command

        jd._prev_abs_vel = abs_vel

        # Issue drive command once the post-event quiet period has elapsed
        if jd.phase in (Phase.FIND_FIRST, Phase.FIND_SECOND) and past_gap:
            self._pub_vel(jd, self._compute_seek_velocity(jd, vel, now))

    # -------------------------------------------------------------------------
    # Centring
    # -------------------------------------------------------------------------

    def _centre(self, jd: JointData, pos: float) -> None:
        target = (jd.extremities[0] + jd.extremities[1]) / 2.0  # type: ignore[operator]
        error  = target - pos

        if abs(error) < self._CENTRE_TOL:
            self._pub_vel(jd, 0.0)
            self.get_logger().info(
                f"{jd.name}: centred at {pos:.4f} rad (motor shaft). "
                "Computing calibration..."
            )
            time.sleep(1.0)
            jd.phase = Phase.COMPLETE
            self._write_cfg(jd)
            self._advance()
            return

        # Bang-bang: constant speed toward target
        self._pub_vel(jd, jd.speed if error > 0 else -jd.speed)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _pub_vel(self, jd: JointData, speed: float) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        msg.data[jd.vel_axis] = speed
        self._vel_pub.publish(msg)

    def _reset_seek_controller(self, jd: JointData) -> None:
        jd._seek_cmd_mag = min(jd.seek_speed_min, jd.speed)
        jd._seek_integral = 0.0
        jd._last_seek_update_t = time.time()

    def _compute_seek_velocity(self, jd: JointData, vel: float, now: float) -> float:
        """Ramp the seek velocity while adding a small PI correction.

        A single fixed seek speed can leave the joint hovering at a value that is
        just low enough to stall. This controller ramps from a gentle minimum
        speed toward the configured maximum and integrates persistent lag so the
        command can push through stiction instead of sitting at a bad velocity.
        """
        dt = max(now - jd._last_seek_update_t, 1e-3)
        seek_speed_min = min(jd.seek_speed_min, jd.speed)
        target_mag = min(jd.speed, jd._seek_cmd_mag + jd.seek_ramp_rate * dt)
        directed_vel = max(0.0, jd._sign * vel)
        vel_error = target_mag - directed_vel
        jd._seek_integral += vel_error * dt
        jd._seek_integral = max(
            -jd.seek_integral_limit,
            min(jd.seek_integral_limit, jd._seek_integral),
        )
        cmd_mag = target_mag + jd.seek_kp * vel_error + jd.seek_ki * jd._seek_integral
        cmd_mag = max(seek_speed_min, min(jd.speed, cmd_mag))

        if cmd_mag <= seek_speed_min + 1e-6 or cmd_mag >= jd.speed - 1e-6:
            # Basic anti-windup: do not keep accumulating while the output is saturated.
            jd._seek_integral -= vel_error * dt

        jd._seek_cmd_mag = cmd_mag
        jd._last_seek_update_t = now
        return jd._sign * cmd_mag

    def _advance(self) -> None:
        self._active -= 1
        if self._active >= 0:
            self._reset_seek_controller(self._joints[self._active])
        if self._active < 0:
            self._finish_all()

    @staticmethod
    def _reported_to_motor_units(value: float, gear_ratio: float) -> float:
        return value / gear_ratio

    @staticmethod
    def _motor_to_reported_units(value: float, gear_ratio: float) -> float:
        return value * gear_ratio

    def _resolve_indices(self, msg: DynamicJointState) -> None:
        """Populate msg_idx and interface indices (runs once per joint)."""
        if all(jd.msg_idx >= 0 for jd in self._joints):
            return

        for jd in self._joints:
            if jd.msg_idx >= 0:
                continue
            try:
                jd.msg_idx = list(msg.joint_names).index(jd.name)
            except ValueError:
                continue
            names = list(msg.interface_values[jd.msg_idx].interface_names)
            jd.pos_idx  = names.index("position") if "position" in names else -1
            jd.vel_idx  = names.index("velocity") if "velocity" in names else -1
            jd.eff_idx  = names.index("effort")   if "effort"   in names else -1
            jd.volt_idx = names.index("voltage")  if "voltage"  in names else -1

        if all(jd.msg_idx >= 0 for jd in self._joints):
            self.get_logger().info(
                "Interface indices resolved: "
                + "  ".join(
                    f"{jd.name}[pos:{jd.pos_idx} vel:{jd.vel_idx} "
                    f"eff:{jd.eff_idx} volt:{jd.volt_idx}]"
                    for jd in self._joints
                )
            )

    # -------------------------------------------------------------------------
    # CFG update
    # -------------------------------------------------------------------------
    def _ensure_cfg_path(self, jd: JointData) -> bool:
        """Repair cfg_path when the module is running from install space."""
        if os.path.isfile(jd.cfg_path):
            return True

        original_path = jd.cfg_path
        for cfg_dir, origin in _motor_cfg_dir_candidates():
            candidate = cfg_dir / f"{jd.name}.cfg"
            if candidate.is_file():
                jd.cfg_path = str(candidate)
                self.get_logger().warn(
                    f"{jd.name}: cfg path {original_path} was not found; "
                    f"using {jd.cfg_path} ({origin})."
                )
                return True

        jd.cfg_update_result = f"skipped: cfg not found ({original_path})"
        self.get_logger().error(f"cfg not found: {original_path}")
        return False

    def _write_cfg(self, jd: JointData) -> None:
        """Overwrite only [motor] gear ratio in the .cfg file.

        The reported joint positions are already scaled by the currently
        configured gear ratio. To keep old gearing from poisoning the result,
        the update is done in three explicit steps:

        1. Undo the current gearing to recover motor-side positions.
        2. Compute the new gear ratio from the motor-side range.
        3. Re-apply the new gearing to those motor-side limits before writing.

        Only the gear ratio is written back to the cfg.
        Position limits are still computed and reported to the operator for
        reference, but left unchanged in the file.
        """
        if None in jd.extremities:
            jd.cfg_update_result = "skipped: incomplete extremities"
            self.get_logger().warn(f"{jd.name}: incomplete extremities — skipping cfg.")
            return

        if not self._ensure_cfg_path(jd):
            return

        # Read cfg so we can update only the gearing and calibrated limits.
        cfg = configparser.RawConfigParser()
        cfg.optionxform = str
        cfg.read(jd.cfg_path)

        try:
            previous_gear_ratio = float(cfg["motor"]["gear ratio"])
        except (KeyError, ValueError):
            previous_gear_ratio = jd.tuned_gear_ratio
            self.get_logger().warn(
                f"{jd.name}: could not read previous gear ratio from cfg; "
                f"assuming tuned value {jd.tuned_gear_ratio:.6f}."
            )

        if not math.isfinite(previous_gear_ratio) or previous_gear_ratio <= 0.0:
            self.get_logger().warn(
                f"{jd.name}: previous gear ratio {previous_gear_ratio!r} is invalid; "
                f"assuming tuned value {jd.tuned_gear_ratio:.6f}."
            )
            previous_gear_ratio = jd.tuned_gear_ratio

        reported_min_pos = min(jd.extremities)    # type: ignore[type-var]
        reported_max_pos = max(jd.extremities)    # type: ignore[type-var]
        reported_range = abs(reported_max_pos - reported_min_pos)

        if not math.isfinite(reported_range) or reported_range < 0.001:
            jd.cfg_update_result = (
                f"skipped: reported range {reported_range:.4f} rad is too small"
            )
            self.get_logger().error(
                f"{jd.name}: reported_range={reported_range:.4f} is suspiciously small "
                f"— skipping cfg update."
            )
            return

        motor_min_pos = self._reported_to_motor_units(reported_min_pos, previous_gear_ratio)
        motor_max_pos = self._reported_to_motor_units(reported_max_pos, previous_gear_ratio)
        motor_range = abs(motor_max_pos - motor_min_pos)

        if not math.isfinite(motor_range) or motor_range < 0.001:
            jd.cfg_update_result = (
                f"skipped: motor range {motor_range:.4f} rad is too small"
            )
            self.get_logger().error(
                f"{jd.name}: motor_range={motor_range:.4f} is suspiciously small after "
                "undoing the current gearing — skipping cfg update."
            )
            return

        output_range = math.radians(jd.range_deg)
        computed_gear_ratio = output_range / motor_range
        if not math.isfinite(computed_gear_ratio) or computed_gear_ratio <= 0.0:
            jd.cfg_update_result = (
                f"skipped: invalid computed gear ratio {computed_gear_ratio!r}"
            )
            self.get_logger().error(
                f"{jd.name}: computed gear ratio {computed_gear_ratio!r} is invalid — skipping cfg update."
            )
            return

        expected_motor_range = output_range / jd.tuned_gear_ratio
        suspicious_range = (
            motor_range < expected_motor_range * self._MIN_RANGE_FACTOR or
            motor_range > expected_motor_range * self._MAX_RANGE_FACTOR
        )
        suspicious_ratio = (
            computed_gear_ratio < jd.tuned_gear_ratio * self._MIN_RANGE_FACTOR or
            computed_gear_ratio > jd.tuned_gear_ratio * self._MAX_RANGE_FACTOR
        )

        self.get_logger().info(
            f"{jd.name}: reported_range={reported_range:.4f}  "
            f"motor_range={motor_range:.4f}  "
            f"output_range={output_range:.4f} rad ({jd.range_deg}°)  "
            f"old_gear_ratio={previous_gear_ratio:.6f}  "
            f"→ computed_gear_ratio={computed_gear_ratio:.6f}"
        )

        scaled_min_pos = self._motor_to_reported_units(motor_min_pos, computed_gear_ratio)
        scaled_max_pos = self._motor_to_reported_units(motor_max_pos, computed_gear_ratio)

        warning = None
        if suspicious_range:
            warning = (
                f"{jd.name}: motor_range={motor_range:.4f} rad is outside the sane band "
                f"for the reference gearing (expected about {expected_motor_range:.4f} rad)."
            )
        if warning is None and suspicious_ratio:
            warning = (
                f"{jd.name}: computed_gear_ratio={computed_gear_ratio:.6f} differs a lot from "
                f"the reference value {jd.tuned_gear_ratio:.6f}."
            )
        pid_rescale_preview = jd.tuned_gear_ratio / computed_gear_ratio
        if not self._confirm_cfg_write(
            jd=jd,
            previous_gear_ratio=previous_gear_ratio,
            reported_range=reported_range,
            motor_range=motor_range,
            expected_motor_range=expected_motor_range,
            measured_gear_ratio=computed_gear_ratio,
            min_pos=scaled_min_pos,
            max_pos=scaled_max_pos,
            pid_rescale=pid_rescale_preview,
            warning=warning,
        ):
            jd.cfg_update_result = "skipped: cfg update rejected by operator"
            self.get_logger().warn(f"{jd.name}: cfg update rejected by operator.")
            return

        cfg["motor"]["gear ratio"] = f"{computed_gear_ratio:.6f}"

        # Rescale PID gains from the reference gear ratio the PIDs were tuned for
        # (tuned_gear_ratio) to the newly computed gear ratio.
        # This is intentionally NOT relative to the previous cfg gear ratio,
        # because the gains were originally tuned at the nominal reference value.
        pid_rescale = jd.tuned_gear_ratio / computed_gear_ratio
        self.get_logger().info(
            f"{jd.name}: PID rescale factor = {pid_rescale:.4f} "
            f"(tuned={jd.tuned_gear_ratio:.6f} → new={computed_gear_ratio:.6f})"
        )
        _PID_GAIN_KEYS = ("kp", "ki", "kd")
        _PID_SECTIONS  = ("position pid", "velocity pid")
        rescaled_pids: dict = {}
        for section in _PID_SECTIONS:
            if section not in cfg:
                continue
            for key in _PID_GAIN_KEYS:
                if key not in cfg[section]:
                    continue
                try:
                    old_val = float(cfg[section][key])
                    new_val = old_val * pid_rescale
                    cfg[section][key] = f"{new_val:.6f}"
                    rescaled_pids[f"[{section}] {key}"] = (old_val, new_val)
                except ValueError:
                    pass

        try:
            with open(jd.cfg_path, "w") as fh:
                cfg.write(fh)
        except OSError as exc:
            jd.cfg_update_result = f"skipped: failed to write cfg ({exc})"
            self.get_logger().error(
                f"{jd.name}: failed to write cfg {jd.cfg_path}: {exc}"
            )
            return

        pid_lines = "\n".join(
            f"  {k:30s}: {v[0]:.6f} → {v[1]:.6f}"
            for k, v in rescaled_pids.items()
        )
        self.get_logger().info(
            f"Updated {jd.cfg_path}:\n"
            f"  [motor]  gear ratio   = {computed_gear_ratio:.6f}\n"
            f"  PID gains rescaled (factor={pid_rescale:.4f}):\n"
            f"{pid_lines}\n"
            f"  [info]   min position = {scaled_min_pos:.6f} (not written)\n"
            f"           max position = {scaled_max_pos:.6f} (not written)"
        )

        jd.cal_min        = scaled_min_pos
        jd.cal_max        = scaled_max_pos
        jd.cal_gear_ratio = computed_gear_ratio
        jd.cfg_update_result = f"written: {jd.cfg_path}"

    def _confirm_cfg_write(
        self,
        *,
        jd: JointData,
        previous_gear_ratio: float,
        reported_range: float,
        motor_range: float,
        expected_motor_range: float,
        measured_gear_ratio: float,
        min_pos: float,
        max_pos: float,
        pid_rescale: float,
        warning: Optional[str],
    ) -> bool:
        """Ask the operator to accept or reject the pending cfg update."""
        lines = [
            "",
            "=" * 70,
            f"CALIBRATION REVIEW: {jd.name}",
            f"  reported range @ old gearing : {reported_range:.6f} rad",
            f"  motor range after undoing old gearing : {motor_range:.6f} rad",
            f"  expected motor range : {expected_motor_range:.6f} rad",
            f"  computed gear ratio  : {measured_gear_ratio:.6f}",
            f"  old gear ratio       : {previous_gear_ratio:.6f}",
            f"  reference gear ratio : {jd.tuned_gear_ratio:.6f}",
            f"  new gear ratio       : {measured_gear_ratio:.6f}",
            f"  PID rescale factor   : {pid_rescale:.4f}  "
            f"(gains tuned at {jd.tuned_gear_ratio:.6f} will be multiplied by {pid_rescale:.4f})",
            f"  min position (info)  : {min_pos:.6f} (not written)",
            f"  max position (info)  : {max_pos:.6f} (not written)",
        ]
        if warning is not None:
            lines.extend(
                [
                    "",
                    "WARNING:",
                    f"  {warning}",
                ]
            )
        lines.append("=" * 70)
        self.get_logger().info("\n".join(lines))

        if not sys.stdin.isatty():
            self.get_logger().warn(
                f"{jd.name}: no interactive terminal available, so cfg update will not be written."
            )
            return False

        while True:
            try:
                answer = input(f"Accept cfg update for {jd.name}? [y/N]: ").strip().lower()
            except EOFError:
                self.get_logger().warn(f"{jd.name}: input closed while waiting for confirmation.")
                return False
            if answer in ("y", "yes"):
                return True
            if answer in ("", "n", "no"):
                return False
            print("Please answer 'y' or 'n'.")
    # -------------------------------------------------------------------------
    # Completion
    # -------------------------------------------------------------------------

    def _finish_all(self) -> None:
        self._print_cfg_summary()
        if any(jd.cal_min is not None for jd in self._joints):
            self.get_logger().info(
                "All joints calibrated. "
                "Rebuild with 'colcon build --packages-select mab_rehab' "
                "to pick up any updated .cfg files on the next launch."
            )
        else:
            self.get_logger().warn(
                "All joints calibrated, but no .cfg files were updated. "
                "Check the summary above before rebuilding."
            )
        rclpy.try_shutdown()

    def _print_cfg_summary(self) -> None:
        """Log a compact summary of which cfg updates were actually written."""
        lines = [
            "",
            "=" * 70,
            "CFG UPDATE SUMMARY",
            "Only accepted updates with a resolved writable cfg file were written.",
            "=" * 70,
            "",
        ]
        for jd in self._joints:
            if jd.cal_min is not None:
                lines += [
                    f"  {jd.name}: gear_ratio={jd.cal_gear_ratio:.6f}  "
                    f"min_position_info={jd.cal_min:.6f}  max_position_info={jd.cal_max:.6f}  "
                    f"path={jd.cfg_path}",
                    "",
                ]
                continue
            result = jd.cfg_update_result or "no cfg update recorded"
            lines += [
                f"  {jd.name}: {result}",
                "",
            ]
        lines.append("=" * 70)
        self.get_logger().info("\n".join(lines))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GearCalibration()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
