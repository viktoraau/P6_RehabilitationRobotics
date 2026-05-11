"""
pendulum_game.py
----------------
Balance an inverted pendulum by moving the cart with your wrist.

Joint mapping:
  Joint 2 (RUD - Radial/Ulnar Dev.)  →  cart target X position

Physics: classic cart-pole inverted pendulum.
Score: +points every second the pole stays upright.
Lives: lost when the pole falls past the fall angle threshold.
"""

import argparse
import math
import random
from pathlib import Path

import pygame
import rclpy

from wrist_games_ros2.joint_state_bridge import JointStateBridge
from wrist_games_ros2.score_sound import LevelManager, ScoreBoard, SoundManager

W, H = 900, 600
TRACK_Y = H - 70
TRACK_MARGIN = 70

CART_W, CART_H = 80, 26
POLE_L_PX = 190
POLE_L_M  = 1.5

g   = 9.81
M   = 1.0
m   = 0.15
MU  = 0.05
FALL_ANGLE   = math.radians(42)
SCORE_PERIOD = 1.0


class PendulumGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Balance Pendulum – Wrist Control (ROS2)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 24)
        self.font_big = pygame.font.SysFont("consolas", 48, bold=True)

        rclpy.init(args=None)
        jnames = [n.strip() for n in args.joint_names.split(",")] if args.joint_names else None
        self.bridge = JointStateBridge(args.ros_topic, jnames)
        self.h_joint = args.joint_h_index
        self.gain = args.control_gain
        self.scoreboard = ScoreBoard(args.points_per_catch)
        self.level_manager = LevelManager(points_per_level=50)
        self.sound = SoundManager(Path(__file__).resolve().parent / "assets")

        self.start_lives = args.start_lives
        self.lives = args.start_lives
        self.cart_target = float(W // 2)
        self.score_timer = 0.0
        self.score_angle = math.radians(10)   # tightens with level
        self.disturbance_timer = 0.0
        self.disturbance_interval = 8.0       # seconds between random nudges
        self._levelup_flash = 0.0
        self._reset()

        self.sound.play("start")

    def _reset(self) -> None:
        self.cart_x   = float(W // 2)
        self.cart_vel = 0.0
        self.theta     = random.uniform(-1, 1) * 0.06
        self.theta_dot = 0.0

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        vals = self.bridge.get_normalized(self.gain)
        self.cart_target = W / 2 + vals[self.h_joint] * (W / 2 - TRACK_MARGIN - CART_W / 2)

    def _physics(self, dt: float) -> None:
        Kp, Kd = 900.0, 85.0
        F = Kp * (self.cart_target - self.cart_x) - Kd * self.cart_vel
        F = max(-2500.0, min(2500.0, F))
        F -= MU * self.cart_vel

        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        denom = POLE_L_M * (4.0 / 3.0 - m * cos_t ** 2 / (M + m))
        theta_ddot = (
            g * sin_t
            - cos_t * (F + m * POLE_L_M * self.theta_dot ** 2 * sin_t) / (M + m)
        ) / denom
        cart_ddot = (
            F + m * POLE_L_M * (self.theta_dot ** 2 * sin_t - theta_ddot * cos_t)
        ) / (M + m)

        self.theta_dot += theta_ddot * dt
        self.theta     += self.theta_dot * dt
        self.cart_vel  += cart_ddot * dt
        self.cart_x    += self.cart_vel * dt

        lo = TRACK_MARGIN + CART_W / 2
        hi = W - TRACK_MARGIN - CART_W / 2
        if self.cart_x < lo:
            self.cart_x = lo
            self.cart_vel *= -0.25
        elif self.cart_x > hi:
            self.cart_x = hi
            self.cart_vel *= -0.25

    def _on_level_up(self, level: int) -> None:
        self.sound.play("level_up")
        self.score_angle = max(math.radians(8), self.score_angle - math.radians(2))
        self.disturbance_interval = max(3.0, self.disturbance_interval - 0.5)
        self._levelup_flash = 1.2

    def run(self) -> None:
        running = True
        while running:
            dt = min(self.clock.tick(60) / 1000.0, 0.033)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            self._update_control()

            # Random disturbance
            self.disturbance_timer += dt
            if self.disturbance_timer >= self.disturbance_interval:
                self.theta_dot += random.uniform(-1.5, 1.5)
                self.disturbance_timer = 0.0

            self._physics(dt)

            if abs(self.theta) < self.score_angle:
                self.score_timer += dt
                if self.score_timer >= SCORE_PERIOD:
                    self.scoreboard.on_catch()
                    self.sound.play("score")
                    self.score_timer = 0.0
                    if self.level_manager.check(self.scoreboard.score):
                        self._on_level_up(self.level_manager.current(self.scoreboard.score))
            else:
                self.score_timer = 0.0

            if abs(self.theta) > FALL_ANGLE:
                self.lives -= 1
                self.scoreboard.on_miss()
                self.sound.play("miss")
                if self.lives <= 0:
                    self.lives = self.start_lives
                self._reset()

            self._levelup_flash = max(0.0, self._levelup_flash - dt)
            level = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            # ── Draw ────────────────────────────────────────────────────
            self.screen.fill((18, 22, 35))

            pygame.draw.rect(self.screen, (70, 75, 95),
                             (TRACK_MARGIN, TRACK_Y + CART_H - 5, W - 2 * TRACK_MARGIN, 5))

            tgt_x = int(self.cart_target)
            pygame.draw.rect(self.screen, (40, 60, 120),
                             (tgt_x - CART_W // 2, TRACK_Y, CART_W, CART_H), 2)

            cx = int(self.cart_x)
            pygame.draw.rect(self.screen, (70, 135, 210),
                             (cx - CART_W // 2, TRACK_Y, CART_W, CART_H), border_radius=5)
            for wx in (cx - CART_W // 2 + 12, cx + CART_W // 2 - 12):
                pygame.draw.circle(self.screen, (50, 80, 130), (wx, TRACK_Y + CART_H), 8)

            tip_x = cx + POLE_L_PX * math.sin(self.theta)
            tip_y = TRACK_Y - POLE_L_PX * math.cos(self.theta)
            angle_deg = abs(math.degrees(self.theta))
            if angle_deg < math.degrees(self.score_angle):
                pole_col = (50, 230, 80)
            elif angle_deg < 25:
                pole_col = (255, 210, 0)
            else:
                pole_col = (255, 60, 60)

            pygame.draw.line(self.screen, (20, 40, 20),
                             (cx, TRACK_Y), (int(tip_x), int(tip_y)), 11)
            pygame.draw.line(self.screen, pole_col,
                             (cx, TRACK_Y), (int(tip_x), int(tip_y)), 6)
            pygame.draw.circle(self.screen, (255, 210, 0), (int(tip_x), int(tip_y)), 13)
            pygame.draw.circle(self.screen, (180, 180, 200), (cx, TRACK_Y), 6)

            if self._levelup_flash > 0.0:
                alpha = int(min(180, self._levelup_flash * 150))
                flash = pygame.Surface((W, H), pygame.SRCALPHA)
                flash.fill((80, 255, 180, alpha))
                self.screen.blit(flash, (0, 0))
                lbl = self.font_big.render(f"LEVEL {level}!", True, (255, 255, 255))
                self.screen.blit(lbl, (W // 2 - lbl.get_width() // 2, H // 2 - 30))

            hud = self.font.render(
                f"Score: {self.scoreboard.score}  Best: {self.scoreboard.high_score}"
                f"  Lv:{level}  {hearts}  Angle:{math.degrees(self.theta):+.1f}°",
                True, (255, 255, 255))
            self.screen.blit(hud, (18, 18))

            tip_label = self.font.render(
                "Flex/extend wrist to move cart — keep the pole upright!",
                True, (160, 165, 185))
            self.screen.blit(tip_label, (W // 2 - tip_label.get_width() // 2, 50))

            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Inverted pendulum balance game (ROS2).")
    p.add_argument("--ros-topic", default="/joint_states")
    p.add_argument("--joint-names", default="")
    p.add_argument("--joint-h-index", type=int, default=2, choices=[0, 1, 2])
    p.add_argument("--control-gain", type=float, default=1.0)
    p.add_argument("--start-lives", type=int, default=3)
    p.add_argument("--points-per-catch", type=int, default=10)
    return p


def main() -> None:
    args, _ = _parser().parse_known_args()
    PendulumGame(args).run()


if __name__ == "__main__":
    main()
