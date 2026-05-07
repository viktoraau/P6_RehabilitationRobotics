"""
Game ROS 2 node.

Service  : wrist_games/start_game  (StartGame)
Parameter: data_dir    – where to read patient ROM files (default ~/wrist_games_data)
Parameter: joint_topic – joint_states topic             (default /joint_states)
Parameter: demo_mode   – use keyboard instead of robot  (default false)
Parameter: num_targets – targets per round              (default 8)

The service call blocks until the game window closes, then returns the
patient's score and total time.
"""

import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from wrist_games_interfaces.srv import StartGame

from wrist_games.game_ui import run as run_game_ui
from wrist_games.rom_utils import (
    JOINT_FE, JOINT_RU, JOINT_PS, DEFAULT_RANGES, load_latest_rom
)


class GameNode(Node):

    def __init__(self):
        super().__init__("game_node")

        self.declare_parameter("data_dir",    "~/wrist_games_data")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("demo_mode",   False)
        self.declare_parameter("num_targets", 8)

        data_dir        = self.get_parameter("data_dir").value
        joint_topic     = self.get_parameter("joint_topic").value
        self._demo      = self.get_parameter("demo_mode").value
        self._num_tgts  = self.get_parameter("num_targets").value
        self._data_dir  = Path(data_dir).expanduser()

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
            StartGame,
            "wrist_games/start_game",
            self._service_cb,
            callback_group=_cb_group,
        )

        # Synchronisation
        self._pending_patient: str | None = None
        self._pending_lock = threading.Lock()
        self._start_event  = threading.Event()
        self._done_event   = threading.Event()
        self._svc_result   = {}

        self.get_logger().info(
            f"game_node ready  |  data_dir={self._data_dir}  "
            f"demo={self._demo}  num_targets={self._num_tgts}"
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
                response.success      = False
                response.message      = "Game already in progress"
                response.score        = 0
                response.total_time_s = 0.0
                return response
            self._pending_patient = request.patient_id
            self._done_event.clear()
            self._start_event.set()

        self.get_logger().info(
            f"Game requested for patient '{request.patient_id}'"
        )
        self._done_event.wait(timeout=3600)   # max 1 h

        response.success      = self._svc_result.get("success", False)
        response.message      = self._svc_result.get("message", "")
        response.score        = self._svc_result.get("score", 0)
        response.total_time_s = self._svc_result.get("total_time_s", 0.0)
        return response

    # ── Pygame interface ───────────────────────────────────────────────────────

    def get_angles(self) -> dict:
        with self._angles_lock:
            return dict(self._angles)

    def _kb_update(self, dt: float):
        """Keyboard reader for demo mode – called every pygame frame."""
        import pygame
        speed = 30.0 * dt
        with self._angles_lock:
            keys = pygame.key.get_pressed()
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
    node = GameNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            triggered = node._start_event.wait(timeout=0.1)
            if not triggered:
                continue

            node._start_event.clear()
            patient_id = node._pending_patient

            # Load patient ROM (fall back to defaults if not found)
            ranges, rom_path = load_latest_rom(patient_id, node._data_dir)
            if rom_path:
                node.get_logger().info(f"Loaded ROM: {rom_path}")
            else:
                node.get_logger().warn(
                    f"No ROM found for '{patient_id}', using defaults"
                )

            node.get_logger().info(
                f"Opening game window for '{patient_id}'"
            )

            kb_fn = node._kb_update if node._demo else None
            ui_result = run_game_ui(
                node.get_angles, kb_fn,
                patient_id, ranges,
                node._num_tgts,
            )

            node._svc_result = {
                "success":      ui_result.get("success", False),
                "message":      (
                    f"Score {ui_result.get('score', 0)}/{node._num_tgts}"
                    if ui_result.get("success")
                    else "Game closed before completion"
                ),
                "score":        ui_result.get("score", 0),
                "total_time_s": ui_result.get("total_time_s", 0.0),
            }

            node.get_logger().info(
                f"Game finished: {node._svc_result['message']}  "
                f"({node._svc_result['total_time_s']:.1f}s)"
            )
            node._done_event.set()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
