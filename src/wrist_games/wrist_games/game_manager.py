"""
game_manager.py
---------------
Persistent ROS2 node that exposes one Trigger service per game.
Launch with ros2 run, then call services to start/stop games.

Available services:
  /wrist_games/start_airplane   std_srvs/srv/Trigger
  /wrist_games/start_catcher    std_srvs/srv/Trigger
  /wrist_games/start_jedi       std_srvs/srv/Trigger
  /wrist_games/start_octagon    std_srvs/srv/Trigger
  /wrist_games/start_pendulum   std_srvs/srv/Trigger
  /wrist_games/start_tunnel     std_srvs/srv/Trigger
  /wrist_games/start_xwing      std_srvs/srv/Trigger
  /wrist_games/stop_game        std_srvs/srv/Trigger

ROS2 parameters (set via --ros-args -p key:=value):
  ros_topic            (string,  default "/joint_states")
  joint_names          (string,  default "")   -- comma-separated names; empty = use indices
  joint_v_index        (int,     default 1)    -- y joint   (FE  - flex/extend)
  joint_h_index        (int,     default 2)    -- x joint   (RUD - radial/ulnar deviation)
  joint_yaw_index      (int,     default 0)    -- turn joint (PS  - pronation/supination)
  control_joint_index  (int,     default 2)    -- x joint for 1-D games (catcher)
  control_gain         (double,  default 1.0)
  start_lives          (int,     default 3)
  points_per_catch     (int,     default 10)
"""

import subprocess
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GameManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("wrist_game_manager")

        self.declare_parameter("ros_topic",           "/joint_states")
        self.declare_parameter("joint_names",         "")   # comma-separated joint names; empty = use indices
        self.declare_parameter("control_joint_index", 2)    # joint index for 1-D x games (catcher)
        self.declare_parameter("joint_v_index",       1)    # joint index for y / vertical    (FE)
        self.declare_parameter("joint_h_index",       2)    # joint index for x / horizontal  (RUD)
        self.declare_parameter("joint_yaw_index",     0)    # joint index for turn / rotation (PS)
        self.declare_parameter("control_gain",        1.0)
        self.declare_parameter("start_lives",         3)
        self.declare_parameter("points_per_catch",    10)

        self._game_proc: Optional[subprocess.Popen] = None

        # mode: "2d" = v+h joints, "1d_v" = single v joint, "1d_h" = single h joint, "3d" = v+h+yaw
        games = {
            "airplane": ("wrist_games.airplane_game", "2d"),
            "catcher":  ("wrist_games.catcher_game",  "1d_v"),
            "jedi":     ("wrist_games.jedi_game",     "2d"),
            "octagon":  ("wrist_games.octagon_game",  "2d"),
            "pendulum": ("wrist_games.pendulum_game", "1d_h"),
            "tunnel":   ("wrist_games.tunnel_game",   "3d"),
            "xwing":    ("wrist_games.xwing_game",    "2d"),
        }
        for name, (module, mode) in games.items():
            self.create_service(
                Trigger,
                f"/wrist_games/start_{name}",
                lambda req, res, m=module, md=mode: self._start_game(req, res, m, md),
            )

        self.create_service(Trigger, "/wrist_games/stop_game", self._stop_game)

        self.get_logger().info(
            "Wrist Game Manager ready. Services:\n"
            + "\n".join(f"  /wrist_games/start_{n}" for n in games)
            + "\n  /wrist_games/stop_game"
        )

    def _is_running(self) -> bool:
        return self._game_proc is not None and self._game_proc.poll() is None

    def _p(self, name: str):
        return self.get_parameter(name).get_parameter_value()

    def _base_args(self) -> list:
        return [
            "--ros-topic",        self._p("ros_topic").string_value,
            "--joint-names",      self._p("joint_names").string_value,
            "--control-gain",     str(self._p("control_gain").double_value),
            "--start-lives",      str(self._p("start_lives").integer_value),
            "--points-per-catch", str(self._p("points_per_catch").integer_value),
        ]

    def _build_cmd(self, module: str, mode: str) -> list:
        cmd = [sys.executable, "-m", module] + self._base_args()
        if mode == "2d":
            cmd += [
                "--joint-v-index", str(self._p("joint_v_index").integer_value),
                "--joint-h-index", str(self._p("joint_h_index").integer_value),
            ]
        elif mode == "3d":
            cmd += [
                "--joint-v-index",   str(self._p("joint_v_index").integer_value),
                "--joint-h-index",   str(self._p("joint_h_index").integer_value),
                "--joint-yaw-index", str(self._p("joint_yaw_index").integer_value),
            ]
        elif mode == "1d_v":
            cmd += ["--control-joint-index", str(self._p("control_joint_index").integer_value)]
        elif mode == "1d_h":
            cmd += ["--joint-h-index", str(self._p("joint_h_index").integer_value)]
        return cmd

    def _start_game(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        module: str,
        mode: str,
    ) -> Trigger.Response:
        if self._is_running():
            response.success = False
            response.message = "A game is already running. Call /wrist_games/stop_game first."
            return response

        cmd = self._build_cmd(module, mode)
        self._game_proc = subprocess.Popen(cmd)
        response.success = True
        response.message = f"{module.split('.')[-1]} started (PID {self._game_proc.pid})."
        self.get_logger().info(response.message)
        return response

    def _stop_game(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._is_running():
            response.success = False
            response.message = "No game is currently running."
            return response

        self._game_proc.terminate()  # type: ignore[union-attr]
        try:
            self._game_proc.wait(timeout=3.0)  # type: ignore[union-attr]
        except subprocess.TimeoutExpired:
            self._game_proc.kill()  # type: ignore[union-attr]

        self._game_proc = None
        response.success = True
        response.message = "Game stopped."
        self.get_logger().info(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = GameManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._is_running():
            node._game_proc.terminate()  # type: ignore[union-attr]
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
