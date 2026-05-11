"""
octagon_game.py
---------------
Keep a cursor inside a slowly shrinking, rotating octagon.

Joint mapping:
  Joint 1 (FE - Flex/Extend)    →  horizontal X
  Joint 2 (RUD - Radial/Ulnar)  →  vertical   Y

Score: +points every second while cursor is inside the octagon.
Lives: lost for each second spent outside.
"""

import argparse
import math
from pathlib import Path
from typing import List, Tuple

import pygame
import rclpy

from wrist_games.joint_state_bridge import JointStateBridge
from wrist_games.score_sound import LevelManager, ScoreBoard, SoundManager

W, H = 900, 600
CX, CY = W // 2, H // 2


def _octagon_pts(cx: float, cy: float, r: float, rot: float) -> List[Tuple[int, int]]:
    pts = []
    for i in range(8):
        a = 2 * math.pi / 8 * i + rot
        pts.append((int(cx + r * math.cos(a)), int(cy + r * math.sin(a))))
    return pts


def _in_convex_poly(px: float, py: float, pts: List[Tuple[int, int]]) -> bool:
    n = len(pts)
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        if (bx - ax) * (py - ay) - (by - ay) * (px - ax) < 0:
            return False
    return True


class OctagonGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Octagon Squeeze – Wrist Control (ROS2)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 24)
        self.font_big = pygame.font.SysFont("consolas", 48, bold=True)

        rclpy.init(args=None)
        jnames = [n.strip() for n in args.joint_names.split(",")] if args.joint_names else None
        self.bridge = JointStateBridge(args.ros_topic, jnames)
        self.h_joint = args.joint_h_index
        self.v_joint = args.joint_v_index
        self.gain = args.control_gain
        self.scoreboard = ScoreBoard(args.points_per_catch)
        self.level_manager = LevelManager(points_per_level=50)
        self.sound = SoundManager(Path(__file__).resolve().parent / "assets")

        self.start_lives = args.start_lives
        self.lives = args.start_lives
        self.cursor_x = float(CX)
        self.cursor_y = float(CY)

        self.oct_r = 200.0
        self.oct_rot = 0.0
        self.oct_rot_speed = 0.12      # rad/s; increases with level
        self.oct_shrink_rate = 3.0     # px per point; increases with level
        self.score_timer = 0.0
        self.outside_timer = 0.0
        self.outside_penalized = False
        self._levelup_flash = 0.0

        self.sound.play("start")

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        vals = self.bridge.get_normalized(self.gain)
        self.cursor_x = CX - vals[self.h_joint] * (W / 2 - 60)
        self.cursor_y = CY + vals[self.v_joint] * (H / 2 - 60)

    def _on_level_up(self, level: int) -> None:
        self.sound.play("level_up")
        self.oct_rot_speed = min(0.6, self.oct_rot_speed + 0.05)
        self.oct_shrink_rate = min(8.0, self.oct_shrink_rate + 1.0)
        self._levelup_flash = 1.2

    def run(self) -> None:
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            self._update_control()

            self.oct_rot += dt * self.oct_rot_speed
            pts = _octagon_pts(CX, CY, self.oct_r, self.oct_rot)
            inside = _in_convex_poly(self.cursor_x, self.cursor_y, pts)

            if inside:
                self.outside_timer = 0.0
                self.outside_penalized = False
                self.score_timer += dt
                if self.score_timer >= 1.0:
                    self.scoreboard.on_catch()
                    self.sound.play("score")
                    self.score_timer = 0.0
                    self.oct_r = max(58.0, self.oct_r - self.oct_shrink_rate)
                    if self.level_manager.check(self.scoreboard.score):
                        self._on_level_up(self.level_manager.current(self.scoreboard.score))
            else:
                self.score_timer = 0.0
                self.outside_timer += dt
                if self.outside_timer >= 1.5 and not self.outside_penalized:
                    self.outside_penalized = True
                    self.lives -= 1
                    self.scoreboard.on_miss()
                    self.sound.play("miss")
                    if self.lives <= 0:
                        self.lives = self.start_lives
                        self.oct_r = 200.0

            self._levelup_flash = max(0.0, self._levelup_flash - dt)
            level = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            # ── Draw ────────────────────────────────────────────────────
            self.screen.fill((10, 12, 35))

            pygame.draw.polygon(self.screen, (0, 35, 75), pts)
            outline_col = (0, 200, 100) if inside else (220, 60, 60)
            pygame.draw.polygon(self.screen, outline_col, pts, 3)

            cx_i, cy_i = int(self.cursor_x), int(self.cursor_y)
            cursor_col = (0, 255, 120) if inside else (255, 90, 60)
            pygame.draw.circle(self.screen, cursor_col, (cx_i, cy_i), 14)
            pygame.draw.circle(self.screen, (255, 255, 255), (cx_i, cy_i), 14, 2)

            if self._levelup_flash > 0.0:
                alpha = int(min(180, self._levelup_flash * 150))
                flash = pygame.Surface((W, H), pygame.SRCALPHA)
                flash.fill((80, 255, 180, alpha))
                self.screen.blit(flash, (0, 0))
                lbl = self.font_big.render(f"LEVEL {level}!", True, (255, 255, 255))
                self.screen.blit(lbl, (W // 2 - lbl.get_width() // 2, H // 2 - 30))

            hud = self.font.render(
                f"Score: {self.scoreboard.score}  Best: {self.scoreboard.high_score}"
                f"  Lv:{level}  {hearts}  Size:{int(self.oct_r)}",
                True, (255, 255, 255))
            self.screen.blit(hud, (18, 18))

            status = "INSIDE" if inside else f"OUTSIDE  ({max(0.0, 1.5 - self.outside_timer):.1f}s)"
            status_surf = self.font.render(status, True, cursor_col)
            self.screen.blit(status_surf, (W // 2 - status_surf.get_width() // 2, H - 36))

            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Octagon cursor squeeze game (ROS2).")
    p.add_argument("--ros-topic", default="/joint_states")
    p.add_argument("--joint-names", default="")
    p.add_argument("--joint-v-index", type=int, default=1, choices=[0, 1, 2])
    p.add_argument("--joint-h-index", type=int, default=2, choices=[0, 1, 2])
    p.add_argument("--control-gain", type=float, default=1.0)
    p.add_argument("--start-lives", type=int, default=3)
    p.add_argument("--points-per-catch", type=int, default=10)
    return p


def main() -> None:
    args, _ = _parser().parse_known_args()
    OctagonGame(args).run()


if __name__ == "__main__":
    main()
