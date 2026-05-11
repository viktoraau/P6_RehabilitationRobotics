"""
jedi_game.py
------------
Deflect enemy blaster bolts with a lightsaber controlled by wrist rotation.

Joint mapping:
  Joint 1 (FE - Flex/Extend)    →  saber vertical angle
  Joint 2 (RUD - Radial/Ulnar)  →  saber horizontal angle

Score: +points per deflected bolt.  Miss: life lost when bolt reaches player.
"""

import argparse
import math
import random
from pathlib import Path
from typing import List, Tuple

import pygame
import rclpy

from wrist_games_ros2.joint_state_bridge import JointStateBridge
from wrist_games_ros2.score_sound import LevelManager, ScoreBoard, SoundManager

W, H = 900, 600
CX, CY = W // 2, H // 2
SABER_BACK = 70
SABER_TIP  = 175
BOLT_R     = 9
DEFLECT_THRESH = BOLT_R + 9


def _dist_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float
) -> float:
    dx, dy = bx - ax, by - ay
    if dx == dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class Bolt:
    def __init__(self, speed_bonus: float = 0.0) -> None:
        angle = random.uniform(0, 2 * math.pi)
        r = max(W, H) * 0.75
        self.x = CX + r * math.cos(angle)
        self.y = CY + r * math.sin(angle)
        speed = random.uniform(240.0, 320.0) + speed_bonus
        dist = math.hypot(CX - self.x, CY - self.y)
        self.vx = speed * (CX - self.x) / dist
        self.vy = speed * (CY - self.y) / dist
        self.color = (255, random.randint(80, 200), 0)
        self.alive = True

    def update(self, dt: float) -> None:
        self.x += self.vx * dt
        self.y += self.vy * dt

    def at_player(self) -> bool:
        return math.hypot(self.x - CX, self.y - CY) < BOLT_R + 16

    def off_screen(self) -> bool:
        return not (-50 < self.x < W + 50 and -50 < self.y < H + 50)


_STARS = [(random.randint(0, W), random.randint(0, H), random.choice([1, 1, 2])) for _ in range(160)]


class JediGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Jedi Deflect – Wrist Control (ROS2)")
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
        self.saber_angle = 0.0
        self.bolts: List[Bolt] = []
        self.spawn_timer = 0.0
        self.spawn_interval = 2.0
        self.bolt_speed_bonus = 0.0
        self._levelup_flash = 0.0

        self.sound.play("start")

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        vals = self.bridge.get_normalized(self.gain)
        h, v = vals[self.h_joint], -vals[self.v_joint]
        if abs(h) > 0.05 or abs(v) > 0.05:
            self.saber_angle = math.atan2(v, h)

    def _saber_endpoints(self) -> Tuple[float, float, float, float]:
        cos_a, sin_a = math.cos(self.saber_angle), math.sin(self.saber_angle)
        return (
            CX - SABER_BACK * cos_a, CY - SABER_BACK * sin_a,
            CX + SABER_TIP * cos_a,  CY + SABER_TIP * sin_a,
        )

    def _on_level_up(self, level: int) -> None:
        self.sound.play("level_up")
        self.spawn_interval = max(0.3, self.spawn_interval - 0.1)
        self.bolt_speed_bonus = min(200.0, self.bolt_speed_bonus + 20.0)
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
                self.bolts.append(Bolt(speed_bonus=self.bolt_speed_bonus))
                self.spawn_timer = 0.0

            ax, ay, bx, by = self._saber_endpoints()
            surviving: List[Bolt] = []
            for bolt in self.bolts:
                bolt.update(dt)
                dist = _dist_point_to_segment(bolt.x, bolt.y, ax, ay, bx, by)
                if dist < DEFLECT_THRESH:
                    self.scoreboard.on_catch()
                    self.sound.play("score")
                    if self.level_manager.check(self.scoreboard.score):
                        self._on_level_up(self.level_manager.current(self.scoreboard.score))
                elif bolt.at_player():
                    self.lives -= 1
                    self.scoreboard.on_miss()
                    self.sound.play("miss")
                    if self.lives <= 0:
                        self.lives = self.start_lives
                elif bolt.alive and not bolt.off_screen():
                    surviving.append(bolt)
            self.bolts = surviving

            self._levelup_flash = max(0.0, self._levelup_flash - dt)
            level = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            # ── Draw ────────────────────────────────────────────────────
            self.screen.fill((5, 5, 20))
            for sx, sy, sr in _STARS:
                pygame.draw.circle(self.screen, (200, 200, 210), (sx, sy), sr)

            for bolt in self.bolts:
                trail_x = int(bolt.x - bolt.vx * 0.04)
                trail_y = int(bolt.y - bolt.vy * 0.04)
                pygame.draw.line(self.screen,
                                 (bolt.color[0] // 2, bolt.color[1] // 3, 0),
                                 (int(bolt.x), int(bolt.y)), (trail_x, trail_y), 4)
                pygame.draw.circle(self.screen, bolt.color, (int(bolt.x), int(bolt.y)), BOLT_R)

            iax, iay, ibx, iby = int(ax), int(ay), int(bx), int(by)
            pygame.draw.line(self.screen, (0, 60, 100), (iax, iay), (ibx, iby), 10)
            pygame.draw.line(self.screen, (0, 190, 255), (iax, iay), (ibx, iby), 5)
            pygame.draw.line(self.screen, (200, 240, 255), (iax, iay), (ibx, iby), 1)

            pygame.draw.circle(self.screen, (100, 70, 40), (CX, CY), 20)
            pygame.draw.circle(self.screen, (180, 140, 90), (CX, CY), 16)

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

            tip_label = self.font.render(
                "Tilt wrist to rotate lightsaber — deflect the bolts!", True, (160, 160, 180))
            self.screen.blit(tip_label, (W // 2 - tip_label.get_width() // 2, H - 36))

            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Jedi lightsaber deflect game (ROS2).")
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
    JediGame(args).run()


if __name__ == "__main__":
    main()
