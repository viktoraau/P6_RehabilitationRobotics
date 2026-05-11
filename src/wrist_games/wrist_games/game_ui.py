"""
Pure-pygame reach game UI.

Called by the ROS2 node (or standalone scripts).  No ROS 2 imports here.

run(angles_fn, kb_update_fn, patient_id, ranges, num_targets) -> dict
  angles_fn()       -> {'wrist_fe': deg, 'wrist_ru': deg, 'wrist_ps': deg}
  kb_update_fn(dt)  -> None | None (pass None when using real hardware)
  patient_id        -> str | None
  ranges            -> {joint: (min_deg, max_deg), ...}
  num_targets       -> int
  returns           -> {'success': bool, 'score': int, 'total_time_s': float}
"""

import math
import random

import pygame

from wrist_games.rom_utils import JOINT_FE, JOINT_RU, JOINT_PS

# ── Screen & game constants ────────────────────────────────────────────────────
W, H          = 960, 720
TARGET_RADIUS = 55
PS_TOLERANCE  = 15.0
DWELL_TIME    = 1.0
MARGIN_X      = TARGET_RADIUS + 30
MARGIN_Y      = TARGET_RADIUS + 80

# ── Colours ───────────────────────────────────────────────────────────────────
C_BG          = ( 14,  20,  36)
C_GRID        = ( 30,  40,  60)
C_TARGET_IDLE = ( 50, 160,  90)
C_TARGET_ON   = (100, 230, 140)
C_TARGET_RING = (200, 255, 180)
C_CURSOR      = (220, 220,  55)
C_CURSOR_OFF  = (180,  80,  60)
C_TEXT        = (200, 205, 215)
C_DIM         = (100, 110, 130)
C_BAR_BG      = ( 35,  45,  65)
C_BAR_FG      = ( 80, 150, 230)
C_BAR_OK      = (100, 230, 140)
C_WARN        = (230, 170,  50)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _lerp(a, b, t):
    return a + (b - a) * max(0.0, min(1.0, t))


def _map(v, lo, hi, out_lo, out_hi):
    return _lerp(out_lo, out_hi, (v - lo) / (hi - lo))


def _random_target(ranges):
    x  = random.randint(MARGIN_X, W - MARGIN_X)
    y  = random.randint(MARGIN_Y, H - 40)
    ps = random.uniform(ranges[JOINT_PS][0] * 0.7,
                        ranges[JOINT_PS][1] * 0.7)
    return x, y, ps


def _draw_grid(surf):
    for x in range(0, W, 80):
        pygame.draw.line(surf, C_GRID, (x, 0), (x, H))
    for y in range(0, H, 80):
        pygame.draw.line(surf, C_GRID, (0, y), (W, y))


def _draw_target(surf, tx, ty, tps, active, dwell_pct):
    col = C_TARGET_ON if active else C_TARGET_IDLE
    pygame.draw.circle(surf, col, (tx, ty), TARGET_RADIUS)
    pygame.draw.circle(surf, C_BG, (tx, ty), TARGET_RADIUS - 6)
    pygame.draw.circle(surf, col, (tx, ty), TARGET_RADIUS - 6, 2)
    a  = math.radians(tps)
    ex = tx + (TARGET_RADIUS - 14) * math.sin(a)
    ey = ty - (TARGET_RADIUS - 14) * math.cos(a)
    pygame.draw.line(surf, col, (tx, ty), (int(ex), int(ey)), 4)
    if active and dwell_pct > 0:
        rect  = pygame.Rect(tx - TARGET_RADIUS, ty - TARGET_RADIUS,
                            TARGET_RADIUS * 2, TARGET_RADIUS * 2)
        start = math.pi / 2
        pygame.draw.arc(surf, C_TARGET_RING, rect,
                        start, start + dwell_pct * 2 * math.pi, 6)


def _draw_cursor(surf, cx, cy, ps, on_target):
    col = C_CURSOR if on_target else C_CURSOR_OFF
    pygame.draw.circle(surf, col, (int(cx), int(cy)), 14)
    a  = math.radians(ps)
    ex = cx + 22 * math.sin(a)
    ey = cy - 22 * math.cos(a)
    pygame.draw.line(surf, (30, 30, 30), (int(cx), int(cy)), (int(ex), int(ey)), 3)
    pygame.draw.circle(surf, col, (int(cx), int(cy)), 14, 2)


def _draw_ps_gauge(surf, font, current_ps, target_ps, ps_ok, ps_range):
    gx, gy, gw, gh = 280, 12, 400, 22
    pygame.draw.rect(surf, C_BAR_BG, (gx, gy, gw, gh), border_radius=4)

    def px(v):
        return int(gx + _map(v, ps_range[0], ps_range[1], 0, gw))

    zx  = px(target_ps - PS_TOLERANCE)
    zx2 = px(target_ps + PS_TOLERANCE)
    pygame.draw.rect(surf, C_BAR_OK if ps_ok else C_BAR_FG,
                     (zx, gy, zx2 - zx, gh), border_radius=4)
    mk = px(current_ps)
    pygame.draw.rect(surf, C_CURSOR, (mk - 3, gy - 3, 6, gh + 6), border_radius=3)
    lbl = font.render("PS", True, C_DIM)
    surf.blit(lbl, (gx - 30, gy + 2))


# ── Main entry point ──────────────────────────────────────────────────────────

def run(angles_fn, kb_update_fn, patient_id, ranges, num_targets: int = 8) -> dict:
    """Open game window, block until round finishes or window is closed."""
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    title  = f"Wrist Reach Game  |  {patient_id}" if patient_id else "Wrist Reach Game"
    pygame.display.set_caption(title)
    clock = pygame.time.Clock()

    font_hud = pygame.font.SysFont("segoeui", 20)
    font_big = pygame.font.SysFont("segoeui", 52, bold=True)
    font_med = pygame.font.SysFont("segoeui", 28)
    font_sm  = pygame.font.SysFont("segoeui", 18)

    demo = kb_update_fn is not None

    def reset_state():
        return dict(targets_left=num_targets, score=0,
                    target=_random_target(ranges),
                    dwell_start=None, game_over=False,
                    start_ticks=pygame.time.get_ticks())

    gs      = reset_state()
    debug   = False
    result  = {"success": False, "score": 0, "total_time_s": 0.0}

    while True:
        dt = clock.tick(60) / 1000.0
        if demo:
            kb_update_fn(dt)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return result
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return result
                elif event.key == pygame.K_r and gs["game_over"]:
                    gs = reset_state()
                elif event.key == pygame.K_d:
                    debug = not debug

        ang = angles_fn()
        fe  = ang[JOINT_FE]
        ru  = ang[JOINT_RU]
        ps  = ang[JOINT_PS]

        cx = _map(fe, ranges[JOINT_FE][0], ranges[JOINT_FE][1], MARGIN_X, W - MARGIN_X)
        cy = _map(ru, ranges[JOINT_RU][1], ranges[JOINT_RU][0], MARGIN_Y, H - 40)

        tx, ty, tps = gs["target"]
        spatial_ok  = math.hypot(cx - tx, cy - ty) < TARGET_RADIUS
        ps_ok       = abs(ps - tps) < PS_TOLERANCE
        on_target   = spatial_ok and ps_ok

        dwell_pct = 0.0
        if not gs["game_over"]:
            if on_target:
                if gs["dwell_start"] is None:
                    gs["dwell_start"] = pygame.time.get_ticks()
                elapsed   = (pygame.time.get_ticks() - gs["dwell_start"]) / 1000.0
                dwell_pct = min(elapsed / DWELL_TIME, 1.0)
                if elapsed >= DWELL_TIME:
                    gs["score"]       += 1
                    gs["targets_left"] -= 1
                    gs["dwell_start"]   = None
                    if gs["targets_left"] <= 0:
                        gs["game_over"] = True
                        total_s = (pygame.time.get_ticks() - gs["start_ticks"]) / 1000.0
                        result  = {"success": True,
                                   "score": gs["score"],
                                   "total_time_s": round(total_s, 2)}
                    else:
                        gs["target"] = _random_target(ranges)
                        tx, ty, tps  = gs["target"]
            else:
                gs["dwell_start"] = None

        # ── Draw ──────────────────────────────────────────────────────────────
        screen.fill(C_BG)
        _draw_grid(screen)

        if not gs["game_over"]:
            _draw_target(screen, tx, ty, tps, on_target, dwell_pct)
            _draw_cursor(screen, cx, cy, ps, on_target)
            _draw_ps_gauge(screen, font_hud, ps, tps, ps_ok, ranges[JOINT_PS])

            elapsed_s = (pygame.time.get_ticks() - gs["start_ticks"]) / 1000.0
            hud = font_hud.render(
                f"Score: {gs['score']}    "
                f"Remaining: {gs['targets_left']}    "
                f"Time: {elapsed_s:.1f}s",
                True, C_TEXT)
            screen.blit(hud, (W - hud.get_width() - 20, 14))

            bottom = None
            if debug:
                bottom = (
                    f"FE {fe:+.1f}°  RU {ru:+.1f}°  PS {ps:+.1f}°  "
                    f"spatial={spatial_ok}  ps_ok={ps_ok}"
                )
            elif demo:
                bottom = "DEMO  W/S=FE  A/D=RU  Q/E=PS  D=debug  R=restart  ESC=quit"
            if bottom:
                screen.blit(font_sm.render(bottom, True, C_DIM), (10, H - 24))

        else:
            total_s = result["total_time_s"]
            msgs = [
                (font_big, "Well done!",                                C_TARGET_RING),
                (font_med, f"Reached {gs['score']} / {num_targets}",   C_TEXT),
                (font_med, f"Total time: {total_s:.1f} s",             C_TEXT),
                (font_sm,  "Press R to restart   ESC to close",        C_DIM),
            ]
            y = H // 2 - 90
            for f, txt, col in msgs:
                s = f.render(txt, True, col)
                screen.blit(s, (W // 2 - s.get_width() // 2, y))
                y += s.get_height() + 16

        pygame.display.flip()
