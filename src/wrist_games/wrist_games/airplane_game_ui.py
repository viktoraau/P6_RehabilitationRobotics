"""
Airplane Game UI  –  rail-flight rehabilitation game.

The plane flies forward automatically on a route of rings.
All three wrist DOF are used:
  PS (pronation/supination) → bank left/right  → lateral drift
    RU (radial/ulnar)         → pitch up/down    → altitude
    FE (flexion/extension)    → yaw nudge        → fine lateral trim

run(angles_fn, kb_update_fn, patient_id, ranges, route_name) -> dict
  angles_fn()       -> {'wrist_fe': deg, 'wrist_ru': deg, 'wrist_ps': deg}
  kb_update_fn(dt)  -> None  (pass None for real hardware)
  returns           -> {success, score, rings_hit, rings_total,
                        orbs_caught, total_time_s}
"""

import math
import random

import pygame

from wrist_games.rom_utils import JOINT_FE, JOINT_RU, JOINT_PS

# ── Screen ────────────────────────────────────────────────────────────────────
W, H   = 1024, 720
CX     = W // 2
CY     = H // 2
PLANE_CY = CY + 90     # plane drawn below screen centre (looks forward)

# ── 3-D projection ────────────────────────────────────────────────────────────
FOCAL         = 460

# ── World / physics ───────────────────────────────────────────────────────────
FORWARD_SPEED = 160    # world-units / second
Z_SPACING     = 850    # distance between consecutive rings
MAX_X         = 300    # lateral bounds for plane
MAX_Y         = 190    # altitude bounds

BANK_SPEED    = 210    # world-units/s lateral  (PS at full ROM)
YAW_SPEED     = 90     # world-units/s lateral  (FE at full ROM)
PITCH_SPEED   = 155    # world-units/s vertical (RU at full ROM)

# ── Detection ─────────────────────────────────────────────────────────────────
RING_DETECT_Z = 80     # z at which ring is evaluated (passed / missed)
ORB_DETECT_Z  = 80
ORB_HIT_DIST  = 42     # world-space distance to catch an orb
FLASH_TIME    = 0.45   # seconds ring flashes after hit / miss

# ── Scoring ───────────────────────────────────────────────────────────────────
RING_SCORE = 10
ORB_SCORE  = 3

# ── Colours ───────────────────────────────────────────────────────────────────
C_SKY_TOP    = ( 10,  30,  80)
C_SKY_BOT    = ( 65, 125, 185)
C_GROUND     = ( 34,  72,  26)
C_GRID       = ( 48,  92,  38)
C_RING       = ( 70, 170, 255)
C_RING_NEAR  = (110, 240, 200)
C_RING_HIT   = (160, 255,  90)
C_RING_MISS  = (255,  75,  70)
C_ORB        = (255, 220,  45)
C_ORB_NEAR   = (255, 255, 160)
C_PLANE      = (210, 215, 235)
C_ENGINE     = (160, 200, 255)
C_ROUTE      = ( 75, 130, 210)
C_TEXT       = (220, 225, 235)
C_DIM        = ( 95, 110, 130)

# ── Routes (x, y, ring_radius)  world-space waypoints ─────────────────────────
ROUTES = {
    "easy": [
        (   0,   0, 88), (   0,   0, 86),
        (  85,   0, 82), (  85,  50, 80),
        (   0,  50, 78), (   0,   0, 76),
        ( -85,   0, 74), ( -85, -50, 72),
        (   0, -50, 70), (   0,   0, 68),
    ],
    "medium": [
        (   0,   0, 74), ( 120,   0, 70),
        ( 120,  70, 66), (   0,  70, 63),
        (-120,  70, 60), (-120,   0, 58),
        (   0,   0, 56), (   0, -70, 53),
        ( 120, -70, 50), ( 120,   0, 48),
        (   0,   0, 46),
    ],
}


# ═════════════════════════════════════════════════════════════════════════════
# Helpers
# ═════════════════════════════════════════════════════════════════════════════

def _norm(angle, lo, hi):
    """Normalise angle to [-1, 1] using patient ROM bounds."""
    if angle >= 0:
        return min(1.0, angle / hi) if hi > 0 else 0.0
    else:
        return max(-1.0, angle / abs(lo)) if lo < 0 else 0.0


def _project(wx, wy, wz, px, py):
    """Perspective-project world point onto screen.  Returns (sx, sy, scale)."""
    if wz <= 1:
        return None, None, 0.0
    s  = FOCAL / wz
    sx = CX + (wx - px) * s
    sy = CY - (wy - py) * s
    return sx, sy, s


def _mk_sky(w, h):
    """Pre-render a sky gradient surface (called once at startup)."""
    surf = pygame.Surface((w, h))
    for y in range(h):
        t = y / h
        r = int(C_SKY_TOP[0] + (C_SKY_BOT[0] - C_SKY_TOP[0]) * t)
        g = int(C_SKY_TOP[1] + (C_SKY_BOT[1] - C_SKY_TOP[1]) * t)
        b = int(C_SKY_TOP[2] + (C_SKY_BOT[2] - C_SKY_TOP[2]) * t)
        pygame.draw.line(surf, (r, g, b), (0, y), (w, y))
    return surf


# ═════════════════════════════════════════════════════════════════════════════
# Draw functions
# ═════════════════════════════════════════════════════════════════════════════

def _draw_world(surf, sky_surf, plane_y, scroll_t):
    """Background sky + ground + perspective grid.  Returns horizon_y."""
    # Horizon shifts with altitude: fly higher → horizon moves down
    horizon_y = int(CY - plane_y * 0.75)
    horizon_y = max(60, min(H - 60, horizon_y))

    sky_h = horizon_y
    if sky_h > 0:
        scaled = pygame.transform.scale(sky_surf, (W, sky_h))
        surf.blit(scaled, (0, 0))

    # Ground
    pygame.draw.rect(surf, C_GROUND, (0, horizon_y, W, H - horizon_y))

    # Converging grid lines (vertical)
    vp = (CX, horizon_y)
    for x_off in range(-660, 661, 110):
        pygame.draw.line(surf, C_GRID, vp, (CX + x_off, H), 1)

    # Scrolling horizontal depth stripes
    for i in range(7):
        t = ((i / 7) + scroll_t) % 1.0
        y = int(horizon_y + (H - horizon_y) * t ** 1.7)
        if horizon_y < y < H:
            pygame.draw.line(surf, C_GRID, (0, y), (W, y), 1)

    return horizon_y


def _draw_ring(surf, ring, px, py):
    sx, sy, scale = _project(ring['x'], ring['y'], ring['z'], px, py)
    if sx is None:
        return
    sr = max(4, int(ring['radius'] * scale))
    thickness = max(3, sr // 7)

    state = ring['state']
    if state == 'active':
        col = C_RING_NEAR if ring['z'] < 280 else C_RING
    elif state == 'hit':
        col = C_RING_HIT
    elif state == 'miss':
        col = C_RING_MISS
    else:
        return

    pygame.draw.circle(surf, col, (int(sx), int(sy)), sr, thickness)

    # Inner dot marker at ring centre (navigation aid)
    if state == 'active' and sr > 14:
        pygame.draw.circle(surf, col, (int(sx), int(sy)), 3)


def _draw_orb(surf, orb, px, py):
    if orb['state'] != 'active':
        return
    sx, sy, scale = _project(orb['x'], orb['y'], orb['z'], px, py)
    if sx is None:
        return
    sr  = max(3, int(16 * scale))
    col = C_ORB_NEAR if orb['z'] < 200 else C_ORB
    pygame.draw.circle(surf, col, (int(sx), int(sy)), sr)
    if sr > 5:
        pygame.draw.circle(surf, (255, 255, 220), (int(sx), int(sy)), max(2, sr // 3))


def _draw_route_line(surf, rings, px, py):
    pts = []
    for r in rings:
        if r['state'] == 'active' and r['z'] > RING_DETECT_Z:
            sx, sy, _ = _project(r['x'], r['y'], r['z'], px, py)
            if sx is not None and -50 < sx < W + 50 and -50 < sy < H + 50:
                pts.append((int(sx), int(sy)))
    if len(pts) >= 2:
        pygame.draw.lines(surf, C_ROUTE, False, pts, 1)
        for p in pts:
            pygame.draw.circle(surf, C_ROUTE, p, 3)


def _draw_plane(surf, bank_norm, cx, cy):
    """Back-view plane silhouette.  bank_norm in [-1, 1] drives roll."""
    angle = math.radians(-bank_norm * 42)
    ca, sa = math.cos(angle), math.sin(angle)

    def rp(x, y):
        return (int(cx + x * ca - y * sa),
                int(cy + x * sa + y * ca))

    body  = [rp(-4, -22), rp(4, -22), rp(5, 10), rp(0, 17), rp(-5, 10)]
    lwing = [rp(-4, 2), rp(-50, 18), rp(-46, 24), rp(-6, 11)]
    rwing = [rp( 4, 2), rp( 50, 18), rp( 46, 24), rp( 6, 11)]
    ltail = [rp(-5, 11), rp(-24, 17), rp(-22, 22), rp(-5, 15)]
    rtail = [rp( 5, 11), rp( 24, 17), rp( 22, 22), rp( 5, 15)]

    for pts in (body, lwing, rwing, ltail, rtail):
        pygame.draw.polygon(surf, C_PLANE, pts)

    # Engine exhaust glow at nose
    nose = rp(0, -24)
    pygame.draw.circle(surf, C_ENGINE, nose, 5)
    pygame.draw.circle(surf, (220, 235, 255), nose, 2)


def _draw_hud(surf, font_hud, font_sm,
              score, rings_hit, rings_total, orbs_caught,
              plane_y, ps_n, demo):
    # ── Top-left score strip
    s = font_hud.render(
        f"Score: {score}   Rings: {rings_hit}/{rings_total}   Orbs: {orbs_caught}",
        True, C_TEXT)
    surf.blit(s, (16, 14))

    # ── Altitude bar (right edge)
    bx, by, bw, bh = W - 26, CY - 110, 14, 220
    pygame.draw.rect(surf, (25, 35, 55), (bx, by, bw, bh), border_radius=5)
    t_alt  = (plane_y + MAX_Y) / (2 * MAX_Y)
    fill_h = int(bh * max(0.0, min(1.0, t_alt)))
    pygame.draw.rect(surf, (70, 150, 230),
                     (bx, by + bh - fill_h, bw, fill_h), border_radius=5)
    # Zero line
    pygame.draw.line(surf, C_DIM,
                     (bx - 4, by + bh // 2), (bx + bw + 4, by + bh // 2), 1)
    alt_lbl = font_sm.render("ALT", True, C_DIM)
    surf.blit(alt_lbl, (bx - 2, by - 18))

    # ── Artificial horizon strip (bottom-centre, shows bank)
    aix, aiy = CX, H - 34
    pygame.draw.line(surf, C_DIM, (aix - 70, aiy), (aix + 70, aiy), 1)
    bank_rad = math.radians(-ps_n * 42)
    ca, sa   = math.cos(bank_rad), math.sin(bank_rad)
    pygame.draw.line(surf, (200, 180, 75),
                     (int(aix - 58 * ca), int(aiy + 58 * sa)),
                     (int(aix + 58 * ca), int(aiy - 58 * sa)), 2)
    pygame.draw.circle(surf, (200, 180, 75), (aix, aiy), 4)
    bank_lbl = font_sm.render("BANK", True, C_DIM)
    surf.blit(bank_lbl, (aix - bank_lbl.get_width() // 2, H - 56))

    if demo:
        d = font_sm.render(
            "DEMO  W/S = yaw   A/D = pitch   Q/E = bank   ESC = quit",
            True, C_DIM)
        surf.blit(d, (10, H - 22))


# ═════════════════════════════════════════════════════════════════════════════
# Main entry point
# ═════════════════════════════════════════════════════════════════════════════

def run(angles_fn, kb_update_fn, patient_id, ranges, route_name: str = "easy") -> dict:
    """Open the airplane game window and block until the route finishes."""
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(
        f"Airplane Game  |  {patient_id or 'Demo'}  |  Route: {route_name.capitalize()}")
    clock    = pygame.time.Clock()
    sky_surf = _mk_sky(W, 400)   # pre-render once
    demo     = kb_update_fn is not None

    font_big = pygame.font.SysFont("segoeui", 52, bold=True)
    font_med = pygame.font.SysFont("segoeui", 28)
    font_hud = pygame.font.SysFont("segoeui", 20)
    font_sm  = pygame.font.SysFont("segoeui", 18)

    # ── Build world from route ────────────────────────────────────────────────
    route_wps = ROUTES.get(route_name, ROUTES["easy"])
    rings: list[dict] = []
    orbs:  list[dict] = []

    for i, (wx, wy, wr) in enumerate(route_wps):
        z = Z_SPACING * (i + 1)
        rings.append(dict(x=wx, y=wy, z=float(z), radius=wr,
                          state='active', flash_t=0.0))
        if i + 1 < len(route_wps):
            nx, ny, _ = route_wps[i + 1]
            for j in range(1, 4):
                t  = j / 4
                oz = z + Z_SPACING * t
                ox = wx + (nx - wx) * t + random.uniform(-28, 28)
                oy = wy + (ny - wy) * t + random.uniform(-28, 28)
                orbs.append(dict(x=ox, y=oy, z=float(oz), state='active'))

    # ── Game state ────────────────────────────────────────────────────────────
    plane_x, plane_y = 0.0, 0.0
    scroll_t         = 0.0
    score            = 0
    rings_hit        = 0
    orbs_caught      = 0
    rings_total      = len(rings)
    game_over        = False
    start_ticks      = pygame.time.get_ticks()
    result           = dict(success=False, score=0, rings_hit=0,
                            rings_total=rings_total, orbs_caught=0,
                            total_time_s=0.0)

    while True:
        dt = min(clock.tick(60) / 1000.0, 0.05)
        if demo:
            kb_update_fn(dt)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return result
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                return result

        # ── Read & normalise angles ───────────────────────────────────────────
        ang  = angles_fn()
        fe_n = _norm(ang[JOINT_FE], ranges[JOINT_FE][0], ranges[JOINT_FE][1])
        ru_n = _norm(ang[JOINT_RU], ranges[JOINT_RU][0], ranges[JOINT_RU][1])
        ps_n = _norm(ang[JOINT_PS], ranges[JOINT_PS][0], ranges[JOINT_PS][1])

        if not game_over:
            # ── Update plane position ─────────────────────────────────────────
            plane_x += (ps_n * BANK_SPEED + fe_n * YAW_SPEED) * dt
            plane_y += ru_n * PITCH_SPEED * dt
            plane_x  = max(-MAX_X, min(MAX_X, plane_x))
            plane_y  = max(-MAX_Y, min(MAX_Y, plane_y))

            # ── Scroll world forward ──────────────────────────────────────────
            move      = FORWARD_SPEED * dt
            scroll_t  = (scroll_t + move / 180.0) % 1.0

            for r in rings:
                r['z'] -= move
                if r['flash_t'] > 0:
                    r['flash_t'] -= dt
                    if r['flash_t'] <= 0 and r['state'] in ('hit', 'miss'):
                        r['state'] = 'gone'
                if r['state'] == 'active' and r['z'] < RING_DETECT_Z:
                    dx = abs(plane_x - r['x'])
                    dy = abs(plane_y - r['y'])
                    if dx < r['radius'] * 0.88 and dy < r['radius'] * 0.88:
                        r['state']  = 'hit'
                        r['flash_t'] = FLASH_TIME
                        score      += RING_SCORE
                        rings_hit  += 1
                    else:
                        r['state']  = 'miss'
                        r['flash_t'] = FLASH_TIME

            for orb in orbs:
                orb['z'] -= move
                if orb['state'] == 'active' and orb['z'] < ORB_DETECT_Z:
                    if math.hypot(plane_x - orb['x'], plane_y - orb['y']) < ORB_HIT_DIST:
                        orb['state']  = 'caught'
                        score        += ORB_SCORE
                        orbs_caught  += 1
                    else:
                        orb['state'] = 'gone'

            # ── Check route complete ──────────────────────────────────────────
            if all(r['state'] != 'active' for r in rings):
                # Wait for last flash to finish
                if all(r['flash_t'] <= 0 for r in rings):
                    total_s = (pygame.time.get_ticks() - start_ticks) / 1000.0
                    result  = dict(success=True, score=score,
                                   rings_hit=rings_hit, rings_total=rings_total,
                                   orbs_caught=orbs_caught,
                                   total_time_s=round(total_s, 2))
                    game_over = True

        # ── Draw ──────────────────────────────────────────────────────────────
        _draw_world(screen, sky_surf, plane_y, scroll_t)

        # Route preview line
        _draw_route_line(screen,
                         [r for r in rings if r['state'] == 'active'],
                         plane_x, plane_y)

        # Objects sorted back-to-front (painter's algorithm)
        draw_objs = sorted(
            [r for r in rings if r['state'] in ('active', 'hit', 'miss') and r['z'] > 0] +
            [o for o in orbs  if o['state'] == 'active' and o['z'] > 0],
            key=lambda o: o['z'], reverse=True
        )
        for obj in draw_objs:
            if 'radius' in obj:
                _draw_ring(screen, obj, plane_x, plane_y)
            else:
                _draw_orb(screen, obj, plane_x, plane_y)

        _draw_plane(screen, ps_n, CX, PLANE_CY)
        _draw_hud(screen, font_hud, font_sm,
                  score, rings_hit, rings_total, orbs_caught,
                  plane_y, ps_n, demo)

        if game_over:
            overlay = pygame.Surface((W, H), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 110))
            screen.blit(overlay, (0, 0))
            msgs = [
                (font_big, "Route Complete!",
                           (180, 255, 160)),
                (font_med, f"Score: {score}  |  Rings: {rings_hit}/{rings_total}  |  Orbs: {orbs_caught}",
                           C_TEXT),
                (font_med, f"Time: {result['total_time_s']:.1f} s",
                           C_TEXT),
                (font_sm,  "ESC to close",
                           C_DIM),
            ]
            y = H // 2 - 100
            for f, txt, col in msgs:
                s = f.render(txt, True, col)
                screen.blit(s, (W // 2 - s.get_width() // 2, y))
                y += s.get_height() + 18

        pygame.display.flip()
