import argparse
import random
from pathlib import Path

import pygame
import rclpy

from wrist_games_ros2.joint_state_bridge import JointStateBridge
from wrist_games_ros2.score_sound import LevelManager, ScoreBoard, SoundManager

WIDTH = 900
HEIGHT = 600


class WristCatcherGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Wrist Catcher (ROS2)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 24)
        self.font_big = pygame.font.SysFont("consolas", 48, bold=True)

        rclpy.init(args=None)
        joint_names = [n.strip() for n in args.joint_names.split(",")] if args.joint_names else None
        self.bridge = JointStateBridge(topic_name=args.ros_topic, joint_names=joint_names)

        self.control_joint_index = args.control_joint_index
        self.control_gain = args.control_gain
        self.start_lives = args.start_lives
        self.lives = args.start_lives
        self.scoreboard = ScoreBoard(points_per_catch=args.points_per_catch)
        self.level_manager = LevelManager(points_per_level=50)
        self.sound = SoundManager(Path(__file__).resolve().parent / "assets")

        self.paddle_w = 160
        self.paddle_h = 18
        self.paddle_y = HEIGHT - 60
        self.paddle_x = (WIDTH - self.paddle_w) * 0.5

        self.ball_r = 12
        self.ball_speed = 260.0
        self.ball_speed_base = 260.0
        self.ball_x = WIDTH * 0.5
        self.ball_y = 40.0
        self._levelup_flash = 0.0

        self.sound.play("start")

    def _reset_ball(self) -> None:
        self.ball_x = random.uniform(self.ball_r, WIDTH - self.ball_r)
        self.ball_y = float(self.ball_r + 5)

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        values = self.bridge.get_normalized(gain=self.control_gain)
        idx = max(0, min(2, self.control_joint_index))
        x01 = (values[idx] + 1.0) * 0.5
        self.paddle_x = x01 * (WIDTH - self.paddle_w)

    def _on_level_up(self) -> None:
        self.sound.play("level_up")
        self.paddle_w = max(60, self.paddle_w - 8)
        self.ball_speed_base = min(500.0, self.ball_speed_base + 15.0)
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
            self.ball_y += self.ball_speed * dt

            paddle_rect = pygame.Rect(int(self.paddle_x), self.paddle_y, self.paddle_w, self.paddle_h)
            ball_rect = pygame.Rect(
                int(self.ball_x - self.ball_r), int(self.ball_y - self.ball_r),
                self.ball_r * 2, self.ball_r * 2,
            )

            if paddle_rect.colliderect(ball_rect) and self.ball_y < self.paddle_y + self.paddle_h:
                self.scoreboard.on_catch()
                self.sound.play("score")
                self._reset_ball()
                self.ball_speed = min(620.0, self.ball_speed + 12.0)
                if self.level_manager.check(self.scoreboard.score):
                    self._on_level_up()
            elif self.ball_y - self.ball_r > HEIGHT:
                self.lives -= 1
                self.scoreboard.on_miss()
                self.sound.play("miss")
                self.ball_speed = max(self.ball_speed_base, self.ball_speed - 10.0)
                self._reset_ball()
                if self.lives <= 0:
                    self.lives = self.start_lives

            self._levelup_flash = max(0.0, self._levelup_flash - dt)
            level = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            self.screen.fill((15, 18, 30))

            if self._levelup_flash > 0.0:
                alpha = int(min(180, self._levelup_flash * 150))
                flash = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                flash.fill((80, 255, 180, alpha))
                self.screen.blit(flash, (0, 0))
                lbl = self.font_big.render(f"LEVEL {level}!", True, (255, 255, 255))
                self.screen.blit(lbl, (WIDTH // 2 - lbl.get_width() // 2, HEIGHT // 2 - 30))

            pygame.draw.rect(self.screen, (90, 220, 240), paddle_rect, border_radius=6)
            pygame.draw.circle(self.screen, (255, 196, 90), (int(self.ball_x), int(self.ball_y)), self.ball_r)
            label = self.font.render(
                f"Score: {self.scoreboard.score}  Best: {self.scoreboard.high_score}"
                f"  Lv:{level}  {hearts}",
                True, (240, 240, 240),
            )
            self.screen.blit(label, (22, 20))
            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Wrist-controlled catcher game over ROS2 JointState.")
    parser.add_argument("--ros-topic", default="/joint_states")
    parser.add_argument("--joint-names", default="")
    parser.add_argument("--control-joint-index", type=int, default=2, choices=[0, 1, 2])
    parser.add_argument("--control-gain", type=float, default=1.0)
    parser.add_argument("--start-lives", type=int, default=3)
    parser.add_argument("--points-per-catch", type=int, default=10)
    return parser


def main() -> None:
    args, _ = _parser().parse_known_args()
    WristCatcherGame(args).run()


if __name__ == "__main__":
    main()
