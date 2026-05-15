"""
xwing_game.py
-------------
360-degree dogfight around a centered aircraft.

Joint mapping:
    Joint 0 (PS  - Pron/Supination)    -> turn left/right (yaw rate)
    Joint 1 (FE  - Flex/Extend)        -> move forward/backward
    Joint 2 (RUD - Radial/Ulnar)       -> strafe right/left

The aircraft stays centered on screen while enemies spawn around you and
close in from all directions.
"""

import argparse
import math
import random
from pathlib import Path
from typing import List, Tuple

import pygame
import rclpy

from wrist_games_ros2.joint_state_bridge import JointStateBridge
from wrist_games_ros2.rom_utils import JOINT_FE, JOINT_PS, JOINT_RU, load_latest_rom
from wrist_games_ros2.score_sound import LevelManager, ScoreBoard, SoundManager

W, H = 900, 600
CX, CY = W // 2, H // 2

TURN_RATE = 2.9          # rad/s at full PS input
FWD_SPEED = 245.0        # world-units/s at full FE input
STRAFE_SPEED = 220.0     # world-units/s at full RU input
LASER_SPEED = 630.0

ENEMY_SPAWN_RADIUS_MIN = 500.0
ENEMY_SPAWN_RADIUS_MAX = 690.0
ENEMY_DESPAWN_RADIUS = 980.0


class Enemy:
    R = 18

    def __init__(self, ship_x: float, ship_y: float, speed_bonus: float = 0.0) -> None:
        ang = random.uniform(0.0, math.tau)
        dist = random.uniform(ENEMY_SPAWN_RADIUS_MIN, ENEMY_SPAWN_RADIUS_MAX)
        self.x = ship_x + math.cos(ang) * dist
        self.y = ship_y + math.sin(ang) * dist
        self.base_speed = random.uniform(92.0, 138.0) + speed_bonus
        self.orbit = random.uniform(-55.0, 55.0)
        self.alive = True

    def update(self, dt: float, ship_x: float, ship_y: float) -> None:
        dx = ship_x - self.x
        dy = ship_y - self.y
        dist = math.hypot(dx, dy)
        if dist < 1.0:
            dist = 1.0

        ux, uy = dx / dist, dy / dist
        tx, ty = -uy, ux
        vx = ux * self.base_speed + tx * self.orbit
        vy = uy * self.base_speed + ty * self.orbit
        self.x += vx * dt
        self.y += vy * dt

        # Cull enemies that drift very far out of the combat volume.
        if dist > ENEMY_DESPAWN_RADIUS:
            self.alive = False


class Laser:
    def __init__(self, x: float, y: float, vx: float, vy: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.vx = float(vx)
        self.vy = float(vy)
        self.ttl = 1.35
        self.alive = True

    def update(self, dt: float) -> None:
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.ttl -= dt
        if self.ttl <= 0.0:
            self.alive = False


_STARS: List[Tuple[float, float, int]] = [
    (
        random.uniform(-2600.0, 2600.0),
        random.uniform(-2600.0, 2600.0),
        random.choice([1, 1, 1, 2]),
    )
    for _ in range(220)
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
        self.yaw_joint = args.joint_yaw_index
        self.h_joint = args.joint_h_index
        self.v_joint = args.joint_v_index
        self.gain = args.control_gain
        self.scoreboard = ScoreBoard(args.points_per_catch)
        self.level_manager = LevelManager(points_per_level=50)
        self.sound = SoundManager(Path(__file__).resolve().parent / "assets")

        ranges, rom_path = load_latest_rom(args.rom_patient_id, Path(args.rom_data_dir).expanduser())
        self.rom_ranges = ranges
        if rom_path:
            print(f"[xwing_game] Using ROM file: {rom_path}")
        else:
            print("[xwing_game] No ROM file found, using default ROM ranges")

        self.start_lives = args.start_lives
        self.lives = args.start_lives
        self.ship_x = 0.0
        self.ship_y = 0.0
        self.ship_heading = 0.0
        self.ship_vx = 0.0
        self.ship_vy = 0.0

        self.enemies: List[Enemy] = []
        self.lasers: List[Laser] = []
        self.spawn_timer = 0.0
        self.spawn_interval = 1.25
        self.laser_timer = 0.0
        self.laser_interval = 0.20
        self.enemy_speed_bonus = 0.0
        self._levelup_flash = 0.0

        self.sound.play("start")

    @staticmethod
    def _norm_rom(angle_deg: float, lo: float, hi: float) -> float:
        if angle_deg >= 0.0:
            return min(1.0, angle_deg / hi) if hi > 0.0 else 0.0
        return max(-1.0, angle_deg / abs(lo)) if lo < 0.0 else 0.0

    def _update_control(self, dt: float) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        raw = self.bridge.get_positions()
        deg = [math.degrees(v) for v in raw]

        turn_cmd = self._norm_rom(
            deg[self.yaw_joint],
            self.rom_ranges[JOINT_PS][0],
            self.rom_ranges[JOINT_PS][1],
        )
        fe_cmd = self._norm_rom(
            deg[self.v_joint],
            self.rom_ranges[JOINT_FE][0],
            self.rom_ranges[JOINT_FE][1],
        )
        ru_cmd = self._norm_rom(
            deg[self.h_joint],
            self.rom_ranges[JOINT_RU][0],
            self.rom_ranges[JOINT_RU][1],
        )

        self.ship_heading = (self.ship_heading + turn_cmd * TURN_RATE * dt) % math.tau

        fx, fy = math.cos(self.ship_heading), math.sin(self.ship_heading)
        rx, ry = -math.sin(self.ship_heading), math.cos(self.ship_heading)

        target_vx = fx * (fe_cmd * FWD_SPEED) + rx * (ru_cmd * STRAFE_SPEED)
        target_vy = fy * (fe_cmd * FWD_SPEED) + ry * (ru_cmd * STRAFE_SPEED)
        self.ship_vx += (target_vx - self.ship_vx) * 0.24
        self.ship_vy += (target_vy - self.ship_vy) * 0.24

        self.ship_x += self.ship_vx * dt
        self.ship_y += self.ship_vy * dt

    def _rot(self, x: float, y: float) -> Tuple[int, int]:
        c, s = math.cos(self.ship_heading), math.sin(self.ship_heading)
        return (int(CX + x * c - y * s), int(CY + x * s + y * c))

    def _draw_xwing(self) -> None:
        nose = self._rot(36, 0)
        aft_u = self._rot(-30, -10)
        aft_d = self._rot(-30, 10)
        pygame.draw.polygon(self.screen, (220, 225, 238), [nose, aft_u, aft_d])

        wing_l = [self._rot(-6, -4), self._rot(-36, -40), self._rot(-46, -28), self._rot(-10, -1)]
        wing_r = [self._rot(-6, 4), self._rot(-36, 40), self._rot(-46, 28), self._rot(-10, 1)]
        pygame.draw.polygon(self.screen, (175, 182, 202), wing_l)
        pygame.draw.polygon(self.screen, (175, 182, 202), wing_r)

        eng_l = self._rot(-42, -30)
        eng_r = self._rot(-42, 30)
        pygame.draw.circle(self.screen, (255, 145, 20), eng_l, 5)
        pygame.draw.circle(self.screen, (255, 145, 20), eng_r, 5)

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
        self.spawn_interval = max(0.45, self.spawn_interval - 0.10)
        self.enemy_speed_bonus = min(120.0, self.enemy_speed_bonus + 20.0)
        self._levelup_flash = 1.2

    def _spawn_lasers(self) -> None:
        fx, fy = math.cos(self.ship_heading), math.sin(self.ship_heading)
        rx, ry = -math.sin(self.ship_heading), math.cos(self.ship_heading)
        for offset in (-7.0, 7.0):
            lx = self.ship_x + fx * 34.0 + rx * offset
            ly = self.ship_y + fy * 34.0 + ry * offset
            lvx = fx * LASER_SPEED + self.ship_vx * 0.3
            lvy = fy * LASER_SPEED + self.ship_vy * 0.3
            self.lasers.append(Laser(lx, ly, lvx, lvy))

    def _to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        return int(CX + (wx - self.ship_x)), int(CY + (wy - self.ship_y))

    def _draw_radar(self) -> None:
        rr = 72
        rx, ry = W - 95, 92
        pygame.draw.circle(self.screen, (45, 55, 80), (rx, ry), rr)
        pygame.draw.circle(self.screen, (95, 110, 145), (rx, ry), rr, 2)
        pygame.draw.circle(self.screen, (95, 110, 145), (rx, ry), rr // 2, 1)

        for e in self.enemies:
            dx = e.x - self.ship_x
            dy = e.y - self.ship_y
            dist = max(1.0, math.hypot(dx, dy))
            ux, uy = dx / dist, dy / dist
            blip_r = min(rr - 6, dist * 0.08)
            bx = int(rx + ux * blip_r)
            by = int(ry + uy * blip_r)
            pygame.draw.circle(self.screen, (255, 90, 90), (bx, by), 3)

        pygame.draw.circle(self.screen, (150, 230, 255), (rx, ry), 4)

    def run(self) -> None:
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            self._update_control(dt)

            self.spawn_timer += dt
            if self.spawn_timer >= self.spawn_interval:
                self.enemies.append(
                    Enemy(self.ship_x, self.ship_y, speed_bonus=self.enemy_speed_bonus)
                )
                self.spawn_timer = 0.0

            self.laser_timer += dt
            if self.laser_timer >= self.laser_interval:
                self._spawn_lasers()
                self.laser_timer = 0.0

            for e in self.enemies:
                e.update(dt, self.ship_x, self.ship_y)
            for la in self.lasers:
                la.update(dt)

            for la in self.lasers:
                if not la.alive:
                    continue
                for e in self.enemies:
                    if e.alive and math.hypot(la.x - e.x, la.y - e.y) < (Enemy.R + 8):
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
                if math.hypot(e.x - self.ship_x, e.y - self.ship_y) < 36:
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
                px = int((sx - self.ship_x * 0.08) % W)
                py = int((sy - self.ship_y * 0.08) % H)
                pygame.draw.circle(self.screen, (190, 195, 205), (px, py), sr)

            for la in self.lasers:
                lx, ly = self._to_screen(la.x, la.y)
                pygame.draw.circle(self.screen, (255, 70, 70), (lx, ly), 2)

            for e in self.enemies:
                ex, ey = self._to_screen(e.x, e.y)
                if -80 < ex < W + 80 and -80 < ey < H + 80:
                    self._draw_tie(ex, ey)

            self._draw_xwing()
            self._draw_radar()

            # Heading marker
            hx = int(CX + math.cos(self.ship_heading) * 62)
            hy = int(CY + math.sin(self.ship_heading) * 62)
            pygame.draw.line(self.screen, (120, 170, 255), (CX, CY), (hx, hy), 2)

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

            tip = self.font.render(
                "PS=turn   FE=forward/back   RU=right/left",
                True, (165, 175, 205),
            )
            self.screen.blit(tip, (W // 2 - tip.get_width() // 2, H - 32))

            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="X-Wing space shooter game (ROS2).")
    p.add_argument("--ros-topic", default="/joint_states")
    p.add_argument("--joint-names", default="")
    p.add_argument("--joint-yaw-index", type=int, default=0, choices=[0, 1, 2])
    p.add_argument("--joint-v-index", type=int, default=1, choices=[0, 1, 2])
    p.add_argument("--joint-h-index", type=int, default=2, choices=[0, 1, 2])
    p.add_argument("--control-gain", type=float, default=1.0)
    p.add_argument("--rom-patient-id", default="default")
    p.add_argument("--rom-data-dir", default="~/wrist_games_data")
    p.add_argument("--start-lives", type=int, default=3)
    p.add_argument("--points-per-catch", type=int, default=10)
    return p


def main() -> None:
    args, _ = _parser().parse_known_args()
    XWingGame(args).run()


if __name__ == "__main__":
    main()
