import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import pygame

try:
    import winsound
except ImportError:  # pragma: no cover
    winsound = None


@dataclass
class ScoreBoard:
    points_per_catch: int = 10
    score: int = 0
    high_score: int = 0

    def on_catch(self) -> None:
        self.score += self.points_per_catch
        self.high_score = max(self.high_score, self.score)

    def on_miss(self) -> None:
        self.score = 0


class LevelManager:
    """Score-based level tracker.

    Level number = score // points_per_level + 1  (starts at 1).
    Call check(score) each time the score changes; it returns True exactly
    once per level threshold crossing so the caller can trigger level-up
    effects (sounds, difficulty bumps, etc.).
    """

    def __init__(self, points_per_level: int = 50) -> None:
        self.points_per_level = points_per_level
        self._prev_level: int = 1

    def check(self, score: int) -> bool:
        """Return True the first time each new level is reached."""
        lvl = self.current(score)
        if lvl > self._prev_level:
            self._prev_level = lvl
            return True
        return False

    def current(self, score: int) -> int:
        """Return the current level number (>= 1)."""
        return score // max(1, self.points_per_level) + 1


class SoundManager:
    def __init__(self, assets_dir: Path) -> None:
        self._sounds: Dict[str, pygame.mixer.Sound] = {}
        self._mixer_ready = False
        if os.environ.get("WRIST_GAMES_SOUND", "1") == "0":
            return
        try:
            pygame.mixer.init()
            self._mixer_ready = True
        except pygame.error:
            self._mixer_ready = False
            return

        for name in ("start", "score", "miss", "level_up"):
            wav_file = assets_dir / f"{name}.wav"
            if wav_file.exists():
                try:
                    self._sounds[name] = pygame.mixer.Sound(str(wav_file))
                except pygame.error:
                    continue

    def play(self, name: str) -> None:
        sound = self._sounds.get(name)
        if sound is not None:
            sound.play()
            return
        if winsound is None:
            return
        beep_map = {
            "score":    winsound.MB_OK,
            "miss":     winsound.MB_ICONHAND,
            "level_up": winsound.MB_ICONEXCLAMATION,
        }
        winsound.MessageBeep(beep_map.get(name, winsound.MB_ICONASTERISK))
