"""
Calibration ROS 2 node.

Service  : wrist_games/start_calibration  (StartCalibration)
Parameter: data_dir    – where to write patient ROM files (default ~/wrist_games_data)
Parameter: joint_topic – joint_states topic             (default /joint_states)
Parameter: demo_mode   – use keyboard instead of robot  (default false)

The service call blocks until the pygame window closes, then returns the
path to the saved JSON file (or an error string).
"""

import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from wrist_games_interfaces.srv import StartCalibration

from wrist_games.calibration_ui import run as run_calibration_ui
from wrist_games.rom_utils import (
    JOINT_FE, JOINT_RU, JOINT_PS, save_rom
)


class CalibrationNode(Node):

    def __init__(self):
        super().__init__("calibration_node")

        self.declare_parameter("data_dir",    "~/wrist_games_data")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("demo_mode",   False)

        data_dir    = self.get_parameter("data_dir").value
        joint_topic = self.get_parameter("joint_topic").value
        self._demo  = self.get_parameter("demo_mode").value
        self._data_dir = Path(data_dir).expanduser()

        # Callback group that allows subscription + service to run concurrently
        _cb_group = ReentrantCallbackGroup()

        # Joint state tracking
        self._angles = {JOINT_FE: 0.0, JOINT_RU: 0.0, JOINT_PS: 0.0}
        self._angles_lock = threading.Lock()
        self.create_subscription(
            JointState, joint_topic, self._joint_cb, 10,
            callback_group=_cb_group,
        )

        # Service
        self.create_service(
            StartCalibration,
            "wrist_games/start_calibration",
            self._service_cb,
            callback_group=_cb_group,
        )

        # Synchronisation between the service thread and the pygame main thread
        self._pending_patient: str | None = None
        self._pending_lock = threading.Lock()
        self._start_event  = threading.Event()
        self._done_event   = threading.Event()
        self._svc_result   = {}   # filled by main thread before done_event.set()

        self.get_logger().info(
            f"calibration_node ready  |  data_dir={self._data_dir}  "
            f"demo={self._demo}"
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        with self._angles_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._angles:
                    self._angles[name] = math.degrees(pos)

    def _service_cb(self, request, response):
        with self._pending_lock:
            if self._start_event.is_set():
                response.success = False
                response.message = "Calibration already in progress"
                response.rom_file = ""
                return response
            self._pending_patient = request.patient_id
            self._done_event.clear()
            self._start_event.set()

        self.get_logger().info(
            f"Calibration requested for patient '{request.patient_id}'"
        )
        # Block until the main thread signals completion (or 10-min timeout)
        self._done_event.wait(timeout=600)

        response.success  = self._svc_result.get("success", False)
        response.message  = self._svc_result.get("message", "")
        response.rom_file = self._svc_result.get("rom_file", "")
        return response

    # ── Pygame interface ───────────────────────────────────────────────────────

    def get_angles(self) -> dict:
        with self._angles_lock:
            return dict(self._angles)

    def _kb_update(self, dt: float):
        """Keyboard reader for demo mode – called every pygame frame."""
        import pygame
        from wrist_games.rom_utils import DEFAULT_RANGES
        speed = 35.0 * dt
        keys  = pygame.key.get_pressed()
        with self._angles_lock:
            if keys[pygame.K_w]: self._angles[JOINT_FE] = min(self._angles[JOINT_FE] + speed,  90)
            if keys[pygame.K_s]: self._angles[JOINT_FE] = max(self._angles[JOINT_FE] - speed, -90)
            if keys[pygame.K_d]: self._angles[JOINT_RU] = min(self._angles[JOINT_RU] + speed,  40)
            if keys[pygame.K_a]: self._angles[JOINT_RU] = max(self._angles[JOINT_RU] - speed, -40)
            if keys[pygame.K_e]: self._angles[JOINT_PS] = min(self._angles[JOINT_PS] + speed,  90)
            if keys[pygame.K_q]: self._angles[JOINT_PS] = max(self._angles[JOINT_PS] - speed, -90)


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    # Spin ROS in a background thread so the main thread stays free for pygame.
    # MultiThreadedExecutor is required: the service callback blocks on _done_event
    # while the joint-state subscription must keep running concurrently.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            # Wait for a service call to arrive
            triggered = node._start_event.wait(timeout=0.1)
            if not triggered:
                continue

            node._start_event.clear()
            patient_id = node._pending_patient

            node.get_logger().info(f"Opening calibration window for '{patient_id}'")

            kb_fn = node._kb_update if node._demo else None
            ui_result = run_calibration_ui(node.get_angles, kb_fn, patient_id)

            if ui_result["success"]:
                try:
                    path = save_rom(patient_id, ui_result["results"], node._data_dir)
                    node._svc_result = {
                        "success":  True,
                        "message":  f"ROM saved for {patient_id}",
                        "rom_file": str(path),
                    }
                    node.get_logger().info(f"ROM saved → {path}")
                except Exception as exc:
                    node._svc_result = {
                        "success":  False,
                        "message":  f"Save failed: {exc}",
                        "rom_file": "",
                    }
                    node.get_logger().error(f"Save failed: {exc}")
            else:
                node._svc_result = {
                    "success":  False,
                    "message":  "Calibration cancelled by user",
                    "rom_file": "",
                }
                node.get_logger().warn("Calibration cancelled")

            node._done_event.set()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
