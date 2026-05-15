"""
Airplane Game ROS 2 node.

Service  : wrist_games/start_airplane_game  (StartAirplaneGame)
Parameter: data_dir    – patient ROM directory   (default ~/wrist_games_data)
Parameter: joint_topic – joint_states topic      (default /joint_states)
Parameter: demo_mode   – keyboard control        (default false)

The service call blocks while the pygame window is open, then returns
score, rings, orbs, and total time.
"""

import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from wrist_games_interfaces.srv import StartAirplaneGame

from wrist_games.airplane_game_ui import run as run_airplane_ui
from wrist_games.rom_utils import JOINT_FE, JOINT_RU, JOINT_PS, load_latest_rom


class AirplaneNode(Node):

    def __init__(self):
        super().__init__("airplane_node")

        self.declare_parameter("data_dir",    "~/wrist_games_data")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("demo_mode",   False)

        self._data_dir  = Path(
            self.get_parameter("data_dir").value).expanduser()
        self._demo      = self.get_parameter("demo_mode").value
        joint_topic     = self.get_parameter("joint_topic").value

        # Callback group that allows subscription + service to run concurrently
        _cb_group = ReentrantCallbackGroup()

        # Joint state tracking
        self._angles      = {JOINT_FE: 0.0, JOINT_RU: 0.0, JOINT_PS: 0.0}
        self._angles_lock = threading.Lock()
        self.create_subscription(
            JointState, joint_topic, self._joint_cb, 10,
            callback_group=_cb_group,
        )

        # Service
        self.create_service(
            StartAirplaneGame,
            "wrist_games/start_airplane_game",
            self._service_cb,
            callback_group=_cb_group,
        )

        # Thread synchronisation
        self._pending_patient: str | None = None
        self._pending_route:   str        = "easy"
        self._pending_lock = threading.Lock()
        self._start_event  = threading.Event()
        self._done_event   = threading.Event()
        self._svc_result   = {}

        self.get_logger().info(
            f"airplane_node ready  |  data_dir={self._data_dir}  "
            f"demo={self._demo}"
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        with self._angles_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._angles:
                    deg = math.degrees(pos)
                    if name == JOINT_PS:
                        deg = -deg
                    self._angles[name] = deg

    def _service_cb(self, request, response):
        with self._pending_lock:
            if self._start_event.is_set():
                response.success      = False
                response.message      = "Game already in progress"
                response.score        = 0
                response.rings_hit    = 0
                response.rings_total  = 0
                response.orbs_caught  = 0
                response.total_time_s = 0.0
                return response
            self._pending_patient = request.patient_id
            self._pending_route   = request.route if request.route else "easy"
            self._done_event.clear()
            self._start_event.set()

        self.get_logger().info(
            f"Airplane game requested  patient='{request.patient_id}'  "
            f"route='{self._pending_route}'"
        )
        self._done_event.wait(timeout=3600)

        response.success      = self._svc_result.get("success",      False)
        response.message      = self._svc_result.get("message",      "")
        response.score        = self._svc_result.get("score",        0)
        response.rings_hit    = self._svc_result.get("rings_hit",    0)
        response.rings_total  = self._svc_result.get("rings_total",  0)
        response.orbs_caught  = self._svc_result.get("orbs_caught",  0)
        response.total_time_s = self._svc_result.get("total_time_s", 0.0)
        return response

    # ── Pygame interface ───────────────────────────────────────────────────────

    def get_angles(self) -> dict:
        with self._angles_lock:
            return dict(self._angles)

    def _kb_update(self, dt: float):
        import pygame
        speed = 32.0 * dt
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
    node = AirplaneNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            if not node._start_event.wait(timeout=0.1):
                continue

            node._start_event.clear()
            patient_id = node._pending_patient
            route      = node._pending_route

            ranges, rom_path = load_latest_rom(patient_id, node._data_dir)
            if rom_path:
                node.get_logger().info(f"Loaded ROM: {rom_path}")
            else:
                node.get_logger().warn(
                    f"No ROM found for '{patient_id}', using defaults"
                )

            node.get_logger().info(
                f"Opening airplane game  patient='{patient_id}'  route='{route}'"
            )

            kb_fn     = node._kb_update if node._demo else None
            ui_result = run_airplane_ui(
                node.get_angles, kb_fn, patient_id, ranges, route
            )

            node._svc_result = {
                "success":      ui_result.get("success", False),
                "message": (
                    f"Rings {ui_result.get('rings_hit', 0)}/"
                    f"{ui_result.get('rings_total', 0)}  "
                    f"Orbs {ui_result.get('orbs_caught', 0)}"
                    if ui_result.get("success")
                    else "Game closed before route finished"
                ),
                "score":        ui_result.get("score",        0),
                "rings_hit":    ui_result.get("rings_hit",    0),
                "rings_total":  ui_result.get("rings_total",  0),
                "orbs_caught":  ui_result.get("orbs_caught",  0),
                "total_time_s": ui_result.get("total_time_s", 0.0),
            }

            node.get_logger().info(
                f"Game finished: {node._svc_result['message']}  "
                f"score={node._svc_result['score']}  "
                f"time={node._svc_result['total_time_s']:.1f}s"
            )
            node._done_event.set()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
