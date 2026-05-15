"""
tunnel_game.py
--------------
"Human Hole" – fit your shape through an approaching wall.

3-DOF wrist control:
  Joint 0 (PS  - Pron/Supination)    →  shape rotation (yaw)
    Joint 2 (RUD - Radial/Ulnar Dev.)  →  shape vertical position (Y)
    Joint 1 (FE  - Flex/Extend)        →  shape horizontal nudge (X)

A wall with a custom-shaped hole scrolls toward you from the right.
You must align your shape's position AND rotation to pass through the hole.
Each success gives you a new, harder shape. Speed and precision demands
increase with each level.

Score: +points per wall passed.  Miss: life lost when misaligned at impact.
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

# Layout constants
PLAYER_ANCHOR_X = 160   # fixed x-anchor for player shape
PLAYER_RANGE_Y  = H // 2 - 70   # ± pixels for vertical movement
PLAYER_RANGE_X  = 60             # ± pixels for horizontal nudge

WALL_W          = 90    # wall thickness in pixels
SHAPE_SIZE      = 52    # shape drawn at this radius (pixels)
HOLE_CLEARANCE_START = 28  # clearance between shape edge and hole edge (px)


# ---------------------------------------------------------------------------
# Shape definitions – (x, y) normalised so max radius ≈ 1.0
# ---------------------------------------------------------------------------

def _reg_poly(n: int, start: float = 0.0) -> List[Tuple[float, float]]:
    return [(math.cos(2 * math.pi * i / n + start),
             math.sin(2 * math.pi * i / n + start)) for i in range(n)]


def _star(n: int, r_out: float = 1.0, r_in: float = 0.42) -> List[Tuple[float, float]]:
    pts = []
    for i in range(n * 2):
        r = r_out if i % 2 == 0 else r_in
        a = -math.pi / 2 + i * math.pi / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


_HALF = math.pi / 2

SHAPE_DEFS: dict = {
    "square":   [(-0.71, -0.71), ( 0.71, -0.71), ( 0.71,  0.71), (-0.71,  0.71)],
    "triangle": _reg_poly(3, -_HALF),
    "cross": [
        (-0.28, -1.00), ( 0.28, -1.00), ( 0.28, -0.28), ( 1.00, -0.28),
        ( 1.00,  0.28), ( 0.28,  0.28), ( 0.28,  1.00), (-0.28,  1.00),
        (-0.28,  0.28), (-1.00,  0.28), (-1.00, -0.28), (-0.28, -0.28),
    ],
    "pentagon": _reg_poly(5, -_HALF),
    "hexagon":  _reg_poly(6, -_HALF),
    "star":     _star(5),
    "octagon":  _reg_poly(8, -_HALF + math.pi / 8),
    "arrow": [
        ( 0.00, -1.00), ( 0.60, -0.25), ( 0.30, -0.25),
        ( 0.30,  0.90), (-0.30,  0.90), (-0.30, -0.25), (-0.60, -0.25),
    ],
}

SHAPE_ORDER = ["square", "triangle", "cross", "pentagon", "hexagon", "star", "octagon", "arrow"]

# Colour palette  (player tint, wall colour)
SHAPE_COLOURS = {
    "square":   (90,  220, 240),
    "triangle": (255, 180,  60),
    "cross":    (200,  90, 255),
    "pentagon": ( 90, 240, 160),
    "hexagon":  (240, 230,  80),
    "star":     (255, 100, 140),
    "octagon":  (110, 200, 255),
    "arrow":    (255, 160,  50),
}

BG_COLOUR   = (6, 6, 22)
WALL_COLOUR = (55, 60, 80)
HOLE_OUTLINE = (230, 230, 255)

_STARS: List[Tuple[float, float, int]] = [
    (random.uniform(0, W), random.uniform(0, H), random.choice([1, 1, 2]))
    for _ in range(180)
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rotate_scaled(
    pts: List[Tuple[float, float]],
    cx: float, cy: float,
    angle: float,
    scale: float,
) -> List[Tuple[int, int]]:
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    out = []
    for x, y in pts:
        rx = x * cos_a - y * sin_a
        ry = x * sin_a + y * cos_a
        out.append((int(cx + rx * scale), int(cy + ry * scale)))
    return out


def _norm_angle(a: float) -> float:
    """Wrap angle into [-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class TunnelGame:
    def __init__(self, args: argparse.Namespace) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Shape Tunnel – Wrist Control (ROS2)")
        self.clock  = pygame.time.Clock()
        self.font   = pygame.font.SysFont("consolas", 24)
        self.font_sm = pygame.font.SysFont("consolas", 18)
        self.font_big = pygame.font.SysFont("consolas", 52, bold=True)

        rclpy.init(args=None)
        jnames = [n.strip() for n in args.joint_names.split(",")] if args.joint_names else None
        self.bridge = JointStateBridge(args.ros_topic, jnames)

        self.v_joint   = args.joint_v_index
        self.h_joint   = args.joint_h_index
        self.yaw_joint = args.joint_yaw_index
        self.gain      = args.control_gain

        ranges, rom_path = load_latest_rom(args.rom_patient_id, Path(args.rom_data_dir).expanduser())
        self.rom_ranges = ranges
        if rom_path:
            print(f"[tunnel_game] Using ROM file: {rom_path}")
        else:
            print("[tunnel_game] No ROM file found, using default ROM ranges")

        self.scoreboard   = ScoreBoard(args.points_per_catch)
        self.level_manager = LevelManager(points_per_level=50)
        self.sound        = SoundManager(Path(__file__).resolve().parent / "assets")

        self.start_lives = args.start_lives
        self.lives       = args.start_lives

        # Player state
        self.player_y   = float(H / 2)
        self.player_x   = float(PLAYER_ANCHOR_X)
        self.player_rot = 0.0

        # Shape progress
        self.shape_idx  = 0
        self._shape_key = SHAPE_ORDER[0]

        # Wall state
        self.wall_speed    = 120.0
        self.tolerance_pos = float(HOLE_CLEARANCE_START)
        self.tolerance_rot = 28.0   # degrees
        self.hole_size     = SHAPE_SIZE + HOLE_CLEARANCE_START

        self._new_wall()

        # Flash/feedback state
        self._flash_timer = 0.0
        self._flash_type  = ""    # "success" | "miss" | "levelup"
        self._levelup_flash = 0.0
        self._score_pop: List[Tuple[float, float, str, float]] = []  # x,y,text,age

        # Star scroll
        self._star_offset = 0.0

        self.sound.play("start")

    # ------------------------------------------------------------------
    # Wall management
    # ------------------------------------------------------------------

    def _new_wall(self) -> None:
        margin = 90
        self.wall_x      = float(W + 20)
        self.hole_y      = random.uniform(margin, H - margin)
        self.hole_rot    = random.uniform(-math.pi / 2, math.pi / 2)
        self._checked    = False

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def _update_control(self) -> None:
        rclpy.spin_once(self.bridge, timeout_sec=0.0)
        raw = self.bridge.get_positions()
        deg = [math.degrees(v) for v in raw]

        ps_n = self._norm_rom(
            deg[self.yaw_joint],
            self.rom_ranges[JOINT_PS][0],
            self.rom_ranges[JOINT_PS][1],
        )
        ru_n = self._norm_rom(
            deg[self.v_joint],
            self.rom_ranges[JOINT_RU][0],
            self.rom_ranges[JOINT_RU][1],
        )
        fe_n = self._norm_rom(
            deg[self.h_joint],
            self.rom_ranges[JOINT_FE][0],
            self.rom_ranges[JOINT_FE][1],
        )

        # RU controls Y
        target_y = H / 2 - ru_n * PLAYER_RANGE_Y
        self.player_y += (target_y - self.player_y) * 0.22

        # FE controls small X nudge
        target_x = PLAYER_ANCHOR_X + fe_n * PLAYER_RANGE_X
        self.player_x += (target_x - self.player_x) * 0.22

        # PS controls yaw  (tanh maps to [-1,1] → [-π, π])
        self.player_rot = ps_n * math.pi

    @staticmethod
    def _norm_rom(angle_deg: float, lo: float, hi: float) -> float:
        if angle_deg >= 0.0:
            return min(1.0, angle_deg / hi) if hi > 0.0 else 0.0
        return max(-1.0, angle_deg / abs(lo)) if lo < 0.0 else 0.0

    # ------------------------------------------------------------------
    # Level
    # ------------------------------------------------------------------

    def _on_level_up(self, level: int) -> None:
        self.sound.play("level_up")
        self.wall_speed    = min(380.0, self.wall_speed + 18.0)
        self.tolerance_pos = max(10.0,  self.tolerance_pos - 3.0)
        self.tolerance_rot = max(8.0,   self.tolerance_rot - 2.5)
        self.hole_size     = max(SHAPE_SIZE + 10, self.hole_size - 3)
        # advance to next shape when level pushes past shape boundary
        new_idx = min(len(SHAPE_ORDER) - 1, level - 1)
        if new_idx > self.shape_idx:
            self.shape_idx  = new_idx
            self._shape_key = SHAPE_ORDER[self.shape_idx]
        self._levelup_flash = 1.5

    # ------------------------------------------------------------------
    # Check alignment
    # ------------------------------------------------------------------

    def _check_alignment(self) -> bool:
        dy   = abs(self.player_y - self.hole_y)
        dx   = abs(self.player_x - PLAYER_ANCHOR_X)
        drot = abs(_norm_angle(self.player_rot - self.hole_rot))
        return (dy   < self.tolerance_pos
                and dx  < self.tolerance_pos * 1.5
                and drot < math.radians(self.tolerance_rot))

    # ------------------------------------------------------------------
    # Draw helpers
    # ------------------------------------------------------------------

    def _draw_wall(self) -> None:
        shape_key = self._shape_key
        pts_norm  = SHAPE_DEFS[shape_key]
        wx = int(self.wall_x)

        # Draw wall rectangle (full height) minus hole area
        # Strategy: draw full rect, then punch hole using background tinted colour
        wall_rect = pygame.Rect(wx, 0, WALL_W, H)
        pygame.draw.rect(self.screen, WALL_COLOUR, wall_rect)

        # Punch hole (draw hole shape in background colour to "erase" wall)
        hole_pts = _rotate_scaled(pts_norm, wx + WALL_W / 2, self.hole_y,
                                  self.hole_rot, self.hole_size)
        pygame.draw.polygon(self.screen, BG_COLOUR, hole_pts)

        # Hole outline (white glow)
        pygame.draw.polygon(self.screen, HOLE_OUTLINE, hole_pts, 3)

        # Wall edge bevels
        pygame.draw.line(self.screen, (100, 105, 130), (wx, 0), (wx, H), 3)
        pygame.draw.line(self.screen, (100, 105, 130), (wx + WALL_W - 1, 0), (wx + WALL_W - 1, H), 3)

    def _draw_player(self) -> None:
        shape_key = self._shape_key
        pts_norm  = SHAPE_DEFS[shape_key]
        col       = SHAPE_COLOURS[shape_key]

        poly = _rotate_scaled(pts_norm, self.player_x, self.player_y,
                               self.player_rot, SHAPE_SIZE)
        pygame.draw.polygon(self.screen, col, poly)
        pygame.draw.polygon(self.screen, (255, 255, 255), poly, 2)

        # Small rotation guide: line from center in forward direction
        fwd_x = int(self.player_x + math.cos(self.player_rot) * (SHAPE_SIZE + 12))
        fwd_y = int(self.player_y + math.sin(self.player_rot) * (SHAPE_SIZE + 12))
        pygame.draw.line(self.screen, (255, 255, 255, 180),
                          (int(self.player_x), int(self.player_y)), (fwd_x, fwd_y), 2)

    def _draw_target_guide(self) -> None:
        """Ghost outline at hole position on screen to help player aim."""
        if self.wall_x < W - 50:
            # Show a faint guide circle/indicator where the hole will be
            guide_alpha_factor = max(0.0, min(1.0, (W - self.wall_x) / (W * 0.6)))
            if guide_alpha_factor > 0.05:
                col_mod = int(60 * guide_alpha_factor)
                pygame.draw.circle(self.screen, (col_mod, col_mod, col_mod + 20),
                                   (PLAYER_ANCHOR_X, int(self.hole_y)), 6)

    def _draw_progress_bar(self) -> None:
        """Horizontal bar showing how far the wall is."""
        bar_w = 200
        bar_x = W - bar_w - 18
        bar_y = H - 36
        bar_h = 14
        frac = max(0.0, min(1.0, (self.wall_x - PLAYER_ANCHOR_X) / (W - PLAYER_ANCHOR_X)))
        pygame.draw.rect(self.screen, (40, 42, 60), (bar_x, bar_y, bar_w, bar_h), border_radius=4)
        fill_w = int(frac * bar_w)
        if fill_w > 0:
            col = (255, int(200 * frac), 0) if frac < 0.4 else (0, 200, 100)
            pygame.draw.rect(self.screen, col, (bar_x, bar_y, fill_w, bar_h), border_radius=4)
        pygame.draw.rect(self.screen, (120, 125, 160), (bar_x, bar_y, bar_w, bar_h), 2, border_radius=4)
        dist_lbl = self.font_sm.render("WALL", True, (160, 165, 200))
        self.screen.blit(dist_lbl, (bar_x - dist_lbl.get_width() - 8, bar_y))

    def _draw_alignment_indicator(self) -> None:
        """Small indicator showing how close to aligned the player is."""
        shape_key = self._shape_key
        pts_norm  = SHAPE_DEFS[shape_key]
        col       = SHAPE_COLOURS[shape_key]

        # Draw small hint: target hole ghost at PLAYER_ANCHOR_X in the correct position/rotation
        hint_scale = SHAPE_SIZE * 0.55
        hint_pts = _rotate_scaled(pts_norm, PLAYER_ANCHOR_X, int(self.hole_y),
                                  self.hole_rot, hint_scale)
        # Draw with low alpha simulation (dim colour)
        hint_col = tuple(max(0, c // 4) for c in col)
        pygame.draw.polygon(self.screen, hint_col, hint_pts)
        pygame.draw.polygon(self.screen, (80, 80, 110), hint_pts, 1)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            # While flashing (brief pause after check), don't move wall
            if self._flash_timer > 0.0:
                self._flash_timer -= dt
                if self._flash_timer <= 0.0:
                    self._new_wall()
            else:
                self._update_control()

                # Move wall
                self.wall_x -= self.wall_speed * dt

                # Check alignment when midpoint of wall crosses player
                mid = self.wall_x + WALL_W / 2
                if mid <= PLAYER_ANCHOR_X and not self._checked:
                    self._checked = True
                    if self._check_alignment():
                        self.scoreboard.on_catch()
                        self.sound.play("score")
                        self._flash_type  = "success"
                        self._flash_timer = 0.7
                        # Score pop
                        self._score_pop.append(
                            (self.player_x, self.player_y - 30,
                             f"+{self.scoreboard.points_per_catch}", 1.0))
                        if self.level_manager.check(self.scoreboard.score):
                            self._on_level_up(self.level_manager.current(self.scoreboard.score))
                    else:
                        self.lives -= 1
                        self.scoreboard.on_miss()
                        self.sound.play("miss")
                        self._flash_type  = "miss"
                        self._flash_timer = 0.7
                        if self.lives <= 0:
                            self.lives = self.start_lives

                # Wall passed entirely off-screen without check (shouldn't happen but safety)
                if self.wall_x + WALL_W < -40 and not self._checked:
                    self._checked = True
                    self.lives -= 1
                    self.scoreboard.on_miss()
                    self.sound.play("miss")
                    self._flash_type  = "miss"
                    self._flash_timer = 0.5
                    if self.lives <= 0:
                        self.lives = self.start_lives

            # Decay level-up flash
            self._levelup_flash = max(0.0, self._levelup_flash - dt)

            # Advance star parallax scroll
            self._star_offset = (self._star_offset + self.wall_speed * 0.04 * dt) % W

            level  = self.level_manager.current(self.scoreboard.score)
            hearts = "\u2665" * self.lives

            # ── Draw ──────────────────────────────────────────────────
            self.screen.fill(BG_COLOUR)

            # Parallax stars (two layers)
            for sx, sy, sr in _STARS:
                bx = (sx - self._star_offset) % W
                pygame.draw.circle(self.screen, (180, 185, 200), (int(bx), int(sy)), sr)

            # Alignment guide (ghost target at player X)
            self._draw_alignment_indicator()

            # Wall
            if self.wall_x + WALL_W > 0:
                self._draw_wall()

            # Player shape
            self._draw_player()

            # Flash overlays
            if self._flash_timer > 0.0 and self._flash_type == "success":
                alpha = int(min(160, self._flash_timer * 220))
                flash = pygame.Surface((W, H), pygame.SRCALPHA)
                flash.fill((50, 255, 120, alpha))
                self.screen.blit(flash, (0, 0))
            elif self._flash_timer > 0.0 and self._flash_type == "miss":
                alpha = int(min(160, self._flash_timer * 220))
                flash = pygame.Surface((W, H), pygame.SRCALPHA)
                flash.fill((255, 50, 50, alpha))
                self.screen.blit(flash, (0, 0))

            if self._levelup_flash > 0.0:
                alpha = int(min(180, self._levelup_flash * 120))
                flash = pygame.Surface((W, H), pygame.SRCALPHA)
                flash.fill((80, 180, 255, alpha))
                self.screen.blit(flash, (0, 0))
                lbl = self.font_big.render(f"LEVEL {level}!", True, (255, 255, 255))
                self.screen.blit(lbl, (W // 2 - lbl.get_width() // 2, H // 2 - 40))
                shape_lbl = self.font.render(
                    f"New shape: {self._shape_key.upper()}", True, (220, 240, 255))
                self.screen.blit(shape_lbl, (W // 2 - shape_lbl.get_width() // 2, H // 2 + 20))

            # Score pop-ups
            still = []
            for sx, sy, txt, age in self._score_pop:
                if age > 0:
                    a = int(255 * min(1.0, age))
                    col = (80 + int(175 * age), 255, 80 + int(80 * age))
                    lbl = self.font.render(txt, True, col)
                    self.screen.blit(lbl, (int(sx) - lbl.get_width() // 2, int(sy - (1.0 - age) * 40)))
                    still.append((sx, sy, txt, age - dt * 1.5))
            self._score_pop = [(sx, sy, t, a) for sx, sy, t, a in still if a > 0]

            self._draw_progress_bar()

            # HUD
            hud = self.font.render(
                f"Score: {self.scoreboard.score}  Best: {self.scoreboard.high_score}"
                f"  Lv:{level}  {hearts}",
                True, (240, 240, 240))
            self.screen.blit(hud, (18, 18))

            shape_info = self.font_sm.render(
                f"Shape: {self._shape_key.upper()}  "
                f"| Tolerance: ±{int(self.tolerance_pos)}px  ±{int(self.tolerance_rot)}°",
                True, (140, 145, 180))
            self.screen.blit(shape_info, (18, 50))

            tip = self.font_sm.render(
                "RU=up/down  FE=left/right  PS=rotate  — fit through the hole!",
                True, (100, 105, 140))
            self.screen.blit(tip, (W // 2 - tip.get_width() // 2, H - 28))

            pygame.display.flip()

        self.bridge.destroy_node()
        rclpy.shutdown()
        pygame.quit()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="3-DOF wrist shape tunnel game (ROS2).")
    p.add_argument("--ros-topic", default="/joint_states")
    p.add_argument("--joint-names", default="")
    p.add_argument("--joint-v-index",   type=int, default=2, choices=[0, 1, 2],
                   help="RUD joint – controls up/down (default 2).")
    p.add_argument("--joint-h-index",   type=int, default=1, choices=[0, 1, 2],
                   help="FE joint – controls left/right nudge (default 1).")
    p.add_argument("--joint-yaw-index", type=int, default=0, choices=[0, 1, 2],
                   help="PS joint  – controls shape rotation (default 0).")
    p.add_argument("--control-gain",    type=float, default=1.0)
    p.add_argument("--rom-patient-id", default="default")
    p.add_argument("--rom-data-dir", default="~/wrist_games_data")
    p.add_argument("--start-lives",     type=int,   default=3)
    p.add_argument("--points-per-catch", type=int,  default=10)
    return p


def main() -> None:
    args, _ = _parser().parse_known_args()
    TunnelGame(args).run()


if __name__ == "__main__":
    main()
