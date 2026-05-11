"""
xwing_game.py
-------------
Pilot an X-Wing through space, destroying TIE fighters.

Joint mapping:
  Joint 1 (FE  - Flex/Extend)    →  ship vertical   position (up / down)
  Joint 2 (RUD - Radial/Ulnar)   →  ship horizontal position (left / right)

Lasers fire automatically.  Enemy TIE fighters home in on the ship.
Score: +points per TIE destroyed.  Lives lost on collision.
"""

import argparse
import math
import random
from pathlib import Path
from typing import List, Tuple

import pygame
import rclpy

from wrist_games.joint_state_bridge import JointStateBridge
from wrist_games.score_sound import LevelManager, ScoreBoard, SoundManager

W, H = 900, 600
SHIP_X_MID  = 180
SHIP_X_HALF = 100
SHIP_Y_HALF = H // 2 - 55


class Enemy:
    R = 20

    def __init__(self, speed_bonus: float = 0.0) -> None:
        self.x = float(W + 40)
        self.y = random.uniform(55, H - 55)
        self.vx = random.uniform(-165, -120) - speed_bonus
        self.vy = 0.0
        self.alive = True

    def update(self, dt: float, ship_y: float) -> None:
        self.x += self.vx * dt
        dy = ship_y - self.y
        self.vy += dy * 0.6 * dt
        self.vy *= 0.97
        self.y = max(30.0, min(H - 30.0, self.y + self.vy * dt))
        if self.x < -50:
            self.alive = False


class Laser:
    def __init__(self, x: float, y: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.alive = True

    def update(self, dt: float) -> None:
        self.x += 720.0 * dt
        if self.x > W + 20:
            self.alive = False


_STARS: List[Tuple[int, int, int]] = [
    (random.randint(0, W), random.randint(0, H), random.choice([1, 1, 1, 2]))
    for _ in range(140)
]


class XWingGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("X-Wing – Wrist Control (ROS2)")
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
        self.ship_x = float(SHIP_X_MID)
        self.ship_y = float(H // 2)

        self.enemies: List[Enemy] = []
        self.lasers: List[Laser] = []
        self.spawn_timer = 0.0
        self.spawn_interval = 1.8
        self.laser_timer = 0.0
        self.laser_interval = 0.22
        self.enemy_speed_bonus = 0.0
        self._levelup_flash = 0.0

        self.sound.play("start")

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        vals = self.bridge.get_normalized(self.gain)
        target_x = SHIP_X_MID + vals[self.h_joint] * SHIP_X_HALF
        target_y = H / 2        - vals[self.v_joint] * SHIP_Y_HALF
        self.ship_x += (target_x - self.ship_x) * 0.20
        self.ship_y += (target_y - self.ship_y) * 0.20

    def _draw_xwing(self, cx: int, cy: int) -> None:
        pygame.draw.polygon(self.screen, (210, 215, 230),
                            [(cx + 32, cy), (cx - 28, cy - 9), (cx - 28, cy + 9)])
        pygame.draw.polygon(self.screen, (175, 180, 200),
                            [(cx - 4, cy - 5), (cx - 32, cy - 38),
                             (cx - 40, cy - 26), (cx - 4, cy)])
        pygame.draw.polygon(self.screen, (175, 180, 200),
                            [(cx - 4, cy + 5), (cx - 32, cy + 38),
                             (cx - 40, cy + 26), (cx - 4, cy)])
        pygame.draw.circle(self.screen, (255, 130, 0), (cx - 38, cy - 29), 5)
        pygame.draw.circle(self.screen, (255, 130, 0), (cx - 38, cy + 29), 5)

    def _draw_tie(self, cx: int, cy: int) -> None:
        pygame.draw.circle(self.screen, (110, 115, 130), (cx, cy), 13)
        pygame.draw.rect(self.screen, (50, 175, 55), (cx - 32, cy - 26, 13, 52))
        pygame.draw.rect(self.screen, (50, 175, 55), (cx + 19, cy - 26, 13, 52))
        for gy in range(-22, 26, 11):
            pygame.draw.line(self.screen, (30, 110, 35),
                             (cx - 32, cy + gy), (cx - 19, cy + gy), 1)
            pygame.draw.line(self.screen, (30, 110, 35),
                             (cx + 19, cy + gy), (cx + 32, cy + gy), 1)

    def _on_level_up(self, level: int) -> None:
        self.sound.play("level_up")
        self.spawn_interval = max(0.4, self.spawn_interval - 0.15)
        self.enemy_speed_bonus = min(120.0, self.enemy_speed_bonus + 20.0)
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

            self.spawn_timer += dt
            if self.spawn_timer >= self.spawn_interval:
                self.enemies.append(Enemy(speed_bonus=self.enemy_speed_bonus))
                self.spawn_timer = 0.0

            self.laser_timer += dt
            if self.laser_timer >= self.laser_interval:
                sx, sy = int(self.ship_x), int(self.ship_y)
                self.lasers.append(Laser(sx + 30, sy - 5))
                self.lasers.append(Laser(sx + 30, sy + 5))
                self.laser_timer = 0.0

            for e in self.enemies:
                e.update(dt, self.ship_y)
            for la in self.lasers:
                la.update(dt)

            ship_rect = pygame.Rect(int(self.ship_x) - 28, int(self.ship_y) - 36, 56, 72)

            for la in self.lasers:
                if not la.alive:
                    continue
                lr = pygame.Rect(int(la.x) - 12, int(la.y) - 3, 24, 6)
                for e in self.enemies:
                    if e.alive and lr.inflate(4, 4).collidepoint(int(e.x), int(e.y)):
                        e.alive = False
                        la.alive = False
                        self.scoreboard.on_catch()
                        self.sound.play("score")
                        if self.level_manager.check(self.scoreboard.score):
                            self._on_level_up(self.level_manager.current(self.scoreboard.score))
                        break

            for e in self.enemies:
                if not e.alive:
                    continue
                if ship_rect.collidepoint(int(e.x), int(e.y)):
                    e.alive = False
                    self.lives -= 1
                    self.scoreboard.on_miss()
                    self.sound.play("miss")
                    if self.lives <= 0:
                        self.lives = self.start_lives

            self.enemies = [e for e in self.enemies if e.alive]
            self.lasers  = [la for la in self.lasers if la.alive]

            self._levelup_flash = max(0.0, self._levelup_flash - dt)
            level = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            # ── Draw ────────────────────────────────────────────────────
            self.screen.fill((4, 4, 14))
            for sx, sy, sr in _STARS:
                pygame.draw.circle(self.screen, (190, 195, 205), (sx, sy), sr)

            for la in self.lasers:
                pygame.draw.line(self.screen, (255, 70, 70),
                                 (int(la.x) - 16, int(la.y)),
                                 (int(la.x) + 4, int(la.y)), 3)

            for e in self.enemies:
                self._draw_tie(int(e.x), int(e.y))

            self._draw_xwing(int(self.ship_x), int(self.ship_y))

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
    p = argparse.ArgumentParser(description="X-Wing space shooter game (ROS2).")
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
    XWingGame(args).run()


if __name__ == "__main__":
    main()
