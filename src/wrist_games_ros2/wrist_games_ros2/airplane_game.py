"""
airplane_game.py
----------------
Fly a plane through scrolling gates using wrist flexion/extension.

Joint mapping:
  Joint 1 (FE - Flex/Extend)  →  pitch  (up / down)

Score: +points per gate passed.  Miss: life lost when hitting pillar.
"""

import argparse
import random
from pathlib import Path
from typing import List

import pygame
import rclpy

from wrist_games_ros2.joint_state_bridge import JointStateBridge
from wrist_games_ros2.score_sound import LevelManager, ScoreBoard, SoundManager

W, H = 900, 600
PLANE_X = 145


class Gate:
    WIDTH = 38

    def __init__(self, gap_center: float, gap_half: float) -> None:
        self.x = float(W + 50)
        self.gap_center = gap_center
        self.gap_half = gap_half
        self.scored = False

    def update(self, speed: float, dt: float) -> None:
        self.x -= speed * dt

    @property
    def rect_top(self) -> pygame.Rect:
        return pygame.Rect(int(self.x) - self.WIDTH // 2, 0, self.WIDTH, int(self.gap_center - self.gap_half))

    @property
    def rect_bot(self) -> pygame.Rect:
        bot_y = int(self.gap_center + self.gap_half)
        return pygame.Rect(int(self.x) - self.WIDTH // 2, bot_y, self.WIDTH, H - bot_y)


class AirplaneGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Airplane  Wrist Control (ROS2)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 24)
        self.font_big = pygame.font.SysFont("consolas", 48, bold=True)

        rclpy.init(args=None)
        jnames = [n.strip() for n in args.joint_names.split(",")] if args.joint_names else None
        self.bridge = JointStateBridge(args.ros_topic, jnames)
        self.v_joint = args.joint_v_index
        self.gain = args.control_gain
        self.scoreboard = ScoreBoard(args.points_per_catch)
        self.level_manager = LevelManager(points_per_level=50)
        self.sound = SoundManager(Path(__file__).resolve().parent / "assets")

        self.start_lives = args.start_lives
        self.lives = args.start_lives
        self.plane_y = float(H // 2)
        self.gates: List[Gate] = []
        self.speed = 220.0
        self.gap_half = 85.0
        self.gate_timer = 0.0
        self.gate_interval = 2.2
        self._levelup_flash = 0.0

        self.sound.play("start")

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        vals = self.bridge.get_normalized(self.gain)
        target_y = H / 2 - vals[self.v_joint] * (H / 2 - 55)
        self.plane_y += (target_y - self.plane_y) * 0.18

    def _on_level_up(self, level: int) -> None:
        self.sound.play("level_up")
        self.speed = min(550.0, self.speed + 20.0)
        self.gap_half = max(50.0, self.gap_half - 5.0)
        self.gate_interval = max(1.2, self.gate_interval - 0.1)
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

            self.gate_timer += dt
            if self.gate_timer >= self.gate_interval:
                cy = random.uniform(100, H - 100)
                self.gates.append(Gate(cy, self.gap_half))
                self.gate_timer = 0.0

            plane_rect = pygame.Rect(PLANE_X - 22, int(self.plane_y) - 11, 44, 22)

            for gate in self.gates:
                gate.update(self.speed, dt)
                if not gate.scored and gate.x < PLANE_X:
                    gate.scored = True
                    if (not plane_rect.colliderect(gate.rect_top)
                            and not plane_rect.colliderect(gate.rect_bot)):
                        self.scoreboard.on_catch()
                        self.sound.play("score")
                        self.speed = min(550.0, self.speed + 8.0)
                        self.gap_half = max(50.0, self.gap_half - 1.5)
                        if self.level_manager.check(self.scoreboard.score):
                            self._on_level_up(self.level_manager.current(self.scoreboard.score))
                    else:
                        self.lives -= 1
                        self.scoreboard.on_miss()
                        self.sound.play("miss")
                        if self.lives <= 0:
                            self.lives = self.start_lives

            self.gates = [g for g in self.gates if g.x > -60]

            self._levelup_flash = max(0.0, self._levelup_flash - dt)
            level = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            # ── Draw ────────────────────────────────────────────────────
            self.screen.fill((30, 100, 185))
            for cx_c, cy_c in [(200, 80), (500, 50), (760, 110)]:
                pygame.draw.ellipse(self.screen, (220, 235, 255), (cx_c, cy_c, 110, 45))
            pygame.draw.rect(self.screen, (30, 50, 30), (0, H - 40, W, 40))

            for gate in self.gates:
                pygame.draw.rect(self.screen, (50, 160, 60), gate.rect_top)
                pygame.draw.rect(self.screen, (50, 160, 60), gate.rect_bot)
                cap_x = gate.rect_top.x - 4
                pygame.draw.rect(self.screen, (40, 200, 50),
                                 (cap_x, gate.rect_top.bottom - 14, Gate.WIDTH + 8, 14))
                pygame.draw.rect(self.screen, (40, 200, 50),
                                 (cap_x, gate.rect_bot.top, Gate.WIDTH + 8, 14))

            px, py = PLANE_X, int(self.plane_y)
            pygame.draw.polygon(self.screen, (230, 230, 255),
                                [(px + 30, py), (px - 22, py - 13), (px - 22, py + 13)])
            pygame.draw.polygon(self.screen, (180, 180, 220),
                                [(px - 5, py - 10), (px - 28, py - 28), (px - 36, py - 12)])
            pygame.draw.polygon(self.screen, (180, 180, 220),
                                [(px - 5, py + 10), (px - 28, py + 28), (px - 36, py + 12)])

            if self._levelup_flash > 0.0:
                alpha = int(min(180, self._levelup_flash * 150))
                flash = pygame.Surface((W, H), pygame.SRCALPHA)
                flash.fill((80, 255, 180, alpha))
                self.screen.blit(flash, (0, 0))
                lbl = self.font_big.render(f"LEVEL {level}!", True, (255, 255, 255))
                self.screen.blit(lbl, (W // 2 - lbl.get_width() // 2, H // 2 - 30))

            hud = self.font.render(
                f"Score: {self.scoreboard.score}  Best: {self.scoreboard.high_score}"
                f"  Lv:{level}  {hearts}",
                True, (255, 255, 255))
            self.screen.blit(hud, (18, 18))
            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Wrist-controlled airplane game (ROS2).")
    p.add_argument("--ros-topic", default="/joint_states")
    p.add_argument("--joint-names", default="")
    p.add_argument("--joint-v-index", type=int, default=1, choices=[0, 1, 2])
    p.add_argument("--control-gain", type=float, default=1.0)
    p.add_argument("--start-lives", type=int, default=3)
    p.add_argument("--points-per-catch", type=int, default=10)
    return p


def main() -> None:
    args, _ = _parser().parse_known_args()
    AirplaneGame(args).run()


if __name__ == "__main__":
    main()
