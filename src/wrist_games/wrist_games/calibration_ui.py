"""
Pure-pygame calibration UI.

Called by the ROS2 node (or standalone scripts).  No ROS 2 imports here.

run(angles_fn, kb_update_fn, patient_id) -> dict
  angles_fn()       -> {'wrist_fe': deg, 'wrist_ru': deg, 'wrist_ps': deg}
  kb_update_fn(dt)  -> None | None (pass None when using real hardware)
  patient_id        -> str, shown on screen
  returns           -> {'success': bool,
                        'results': {joint: {'min': float, 'max': float}, ...}}
"""

import math

import pygame

from wrist_games.rom_utils import JOINT_FE, JOINT_RU, JOINT_PS

# ── Timing ────────────────────────────────────────────────────────────────────
COUNTDOWN_S = 3.0
RECORDING_S = 4.0

# ── Screen ────────────────────────────────────────────────────────────────────
W, H = 960, 680

# ── Colours ───────────────────────────────────────────────────────────────────
C_BG     = ( 14,  20,  36)
C_TEXT   = (210, 215, 225)
C_DIM    = (100, 110, 130)
C_ACCENT = ( 80, 160, 240)
C_OK     = ( 80, 220, 130)
C_WARN   = (230, 170,  50)
C_BAR_BG = ( 35,  45,  65)
C_BAR_FG = ( 80, 150, 230)
C_PEAK   = (220, 220,  55)

# ── Calibration phases ────────────────────────────────────────────────────────
_PHASES = [
    dict(joint=JOINT_FE, direction="max", title="FLEXION",
         instruction="Bend your wrist forward\n(palm toward forearm)", icon="↑"),
    dict(joint=JOINT_FE, direction="min", title="EXTENSION",
         instruction="Bend your wrist backward\n(back of hand toward forearm)", icon="↓"),
    dict(joint=JOINT_RU, direction="max", title="RADIAL DEVIATION",
         instruction="Tilt your wrist toward your thumb\n(radial side)", icon="→"),
    dict(joint=JOINT_RU, direction="min", title="ULNAR DEVIATION",
         instruction="Tilt your wrist toward your pinky\n(ulnar side)", icon="←"),
    dict(joint=JOINT_PS, direction="max", title="PRONATION",
         instruction="Rotate your forearm\nso palm faces down", icon="↻"),
    dict(joint=JOINT_PS, direction="min", title="SUPINATION",
         instruction="Rotate your forearm\nso palm faces up", icon="↺"),
]

_BAR_RANGE = {JOINT_FE: (-90, 90), JOINT_RU: (-40, 40), JOINT_PS: (-90, 90)}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _centred(surf, font, text, y, colour):
    s = font.render(text, True, colour)
    surf.blit(s, (W // 2 - s.get_width() // 2, y))
    return s.get_height()


def _multiline(surf, font, text, y, colour):
    for line in text.split("\n"):
        y += _centred(surf, font, line, y, colour) + 6
    return y


def _bar(surf, value, lo, hi, bx, by, bw, bh, peak=None):
    pygame.draw.rect(surf, C_BAR_BG, (bx, by, bw, bh), border_radius=6)
    t = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    fw = int(bw * t)
    if fw > 0:
        pygame.draw.rect(surf, C_BAR_FG, (bx, by, fw, bh), border_radius=6)
    zero_x = bx + int(bw * (-lo) / (hi - lo))
    pygame.draw.line(surf, C_DIM, (zero_x, by - 4), (zero_x, by + bh + 4), 2)
    if peak is not None:
        px = bx + int(bw * max(0.0, min(1.0, (peak - lo) / (hi - lo))))
        pygame.draw.rect(surf, C_PEAK, (px - 3, by - 4, 6, bh + 8), border_radius=3)


# ── Main entry point ──────────────────────────────────────────────────────────

def run(angles_fn, kb_update_fn, patient_id: str) -> dict:
    """Open calibration window, block until done or closed.  Returns result dict."""
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"ROM Calibration  |  {patient_id}")
    clock  = pygame.time.Clock()

    f_title = pygame.font.SysFont("segoeui", 48, bold=True)
    f_big   = pygame.font.SysFont("segoeui", 38, bold=True)
    f_med   = pygame.font.SysFont("segoeui", 26)
    f_sm    = pygame.font.SysFont("segoeui", 19)

    demo = kb_update_fn is not None

    # State
    st          = "INTRO"
    phase_idx   = 0
    results     = {}
    phase_peak  = None
    phase_timer = 0.0

    while True:
        dt = clock.tick(60) / 1000.0
        if demo:
            kb_update_fn(dt)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return {"success": False, "results": {}}
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return {"success": False, "results": {}}

                if event.key == pygame.K_SPACE:
                    if st == "INTRO":
                        st, phase_timer = "COUNTDOWN", COUNTDOWN_S

                    elif st == "RESULT":
                        phase_idx += 1
                        if phase_idx >= len(_PHASES):
                            st = "SUMMARY"
                        else:
                            st, phase_timer = "COUNTDOWN", COUNTDOWN_S

                    elif st == "SUMMARY":
                        pygame.quit()
                        return {"success": True, "results": results}

        ang = angles_fn()
        cur = ang[_PHASES[phase_idx]["joint"]] if phase_idx < len(_PHASES) else 0.0

        # State updates
        if st == "COUNTDOWN":
            phase_timer -= dt
            if phase_timer <= 0:
                st, phase_timer, phase_peak = "RECORDING", RECORDING_S, None

        elif st == "RECORDING":
            phase = _PHASES[phase_idx]
            cur   = ang[phase["joint"]]
            if phase["direction"] == "max":
                phase_peak = cur if phase_peak is None else max(phase_peak, cur)
            else:
                phase_peak = cur if phase_peak is None else min(phase_peak, cur)
            phase_timer -= dt
            if phase_timer <= 0:
                joint = phase["joint"]
                if joint not in results:
                    results[joint] = {}
                results[joint][phase["direction"]] = phase_peak
                st = "RESULT"

        # ── Draw ──────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        if st == "INTRO":
            _centred(screen, f_title, "ROM Calibration", 80, C_ACCENT)
            _centred(screen, f_med, f"Patient: {patient_id}", 160, C_TEXT)
            _centred(screen, f_med, "You will perform 6 movements, one at a time.", 220, C_TEXT)
            _centred(screen, f_med, "Move as far as comfortable and hold.", 260, C_TEXT)
            if demo:
                _centred(screen, f_sm, "DEMO  W/S=FE  A/D=RU  Q/E=PS", 340, C_WARN)
            _centred(screen, f_big, "Press SPACE to begin", 430, C_OK)
            _centred(screen, f_sm, "ESC = cancel", H - 40, C_DIM)

        elif st in ("COUNTDOWN", "RECORDING"):
            phase = _PHASES[phase_idx]
            joint = phase["joint"]
            lo, hi = _BAR_RANGE[joint]
            cur    = ang[joint]

            _centred(screen, f_title, phase["title"], 50, C_ACCENT)
            _centred(screen, f_sm, f"Phase {phase_idx + 1} / {len(_PHASES)}", 120, C_DIM)
            _multiline(screen, f_med, phase["instruction"], 160, C_TEXT)
            _centred(screen, f_title, phase["icon"], 255, C_TEXT)

            bx, by, bw, bh = W // 2 - 220, 360, 440, 36
            _bar(screen, cur, lo, hi, bx, by, bw, bh, phase_peak)
            _centred(screen, f_med, f"{cur:+.1f}°", by + bh + 12, C_TEXT)
            if phase_peak is not None:
                _centred(screen, f_sm, f"Peak: {phase_peak:+.1f}°", by + bh + 48, C_PEAK)

            t = math.ceil(phase_timer)
            if st == "COUNTDOWN":
                _centred(screen, f_big, f"Get ready… {t}", 480, C_WARN)
            else:
                _centred(screen, f_big, f"Recording… {t}s", 480, C_OK)

        elif st == "RESULT":
            phase = _PHASES[phase_idx]
            _centred(screen, f_title, phase["title"], 100, C_ACCENT)
            _centred(screen, f_med, "Recorded:", 210, C_TEXT)
            _centred(screen, f_big, f"{phase_peak:+.1f}°", 260, C_OK)
            nxt = "Press SPACE for next movement" \
                if phase_idx < len(_PHASES) - 1 else "Press SPACE for summary"
            _centred(screen, f_med, nxt, 380, C_TEXT)

        elif st == "SUMMARY":
            _centred(screen, f_title, "Summary", 40, C_ACCENT)
            _centred(screen, f_med, f"Patient: {patient_id}", 110, C_DIM)
            rows = [
                ("Flexion",     results.get(JOINT_FE, {}).get("max")),
                ("Extension",   results.get(JOINT_FE, {}).get("min")),
                ("Radial Dev.", results.get(JOINT_RU, {}).get("max")),
                ("Ulnar Dev.",  results.get(JOINT_RU, {}).get("min")),
                ("Pronation",   results.get(JOINT_PS, {}).get("max")),
                ("Supination",  results.get(JOINT_PS, {}).get("min")),
            ]
            y = 170
            for label, val in rows:
                ls = f_med.render(f"{label}:", True, C_TEXT)
                vs = f_med.render(f"{val:+.1f}°" if val is not None else "—", True, C_OK)
                screen.blit(ls, (W // 2 - 180, y))
                screen.blit(vs, (W // 2 + 60, y))
                y += ls.get_height() + 8
            _centred(screen, f_big, "Press SPACE to save & close", y + 30, C_ACCENT)
            _centred(screen, f_sm, "ESC = discard", H - 40, C_DIM)

        pygame.display.flip()
