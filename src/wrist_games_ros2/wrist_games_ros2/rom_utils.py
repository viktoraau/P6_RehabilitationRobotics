"""Shared ROM file I/O utilities."""

import json
from datetime import datetime
from pathlib import Path

JOINT_PS = "joint_1"
JOINT_FE = "joint_2"
JOINT_RU = "joint_3"

DEFAULT_RANGES = {
    JOINT_FE: (-60.0, 60.0),
    JOINT_RU: (-30.0, 30.0),
    JOINT_PS: (-65.0, 65.0),
}


def save_rom(patient_id: str, results: dict, data_dir: Path) -> Path:
    """
    Persist calibration results to JSON.

    results: { joint_name: {'min': float, 'max': float}, ... }
    Returns the path of the written file.
    """
    out_dir = Path(data_dir) / patient_id
    out_dir.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    path = out_dir / f"rom_{ts}.json"

    labels = {
        JOINT_FE: "Flexion / Extension",
        JOINT_RU: "Radial / Ulnar Deviation",
        JOINT_PS: "Pronation / Supination",
    }
    payload = {
        "patient_id": patient_id,
        "session_date": datetime.now().strftime("%Y-%m-%d"),
        "session_time": datetime.now().strftime("%H:%M:%S"),
        "joints": {
            j: {
                "min": round(results[j]["min"], 2),
                "max": round(results[j]["max"], 2),
                "label": labels[j],
            }
            for j in (JOINT_FE, JOINT_RU, JOINT_PS)
        },
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    return path


def load_latest_rom(patient_id: str, data_dir: Path):
    """
    Return (ranges_dict, path_str) for the most recent ROM file,
    or (DEFAULT_RANGES, None) when no file is found.
    """
    folder = Path(data_dir) / patient_id
    if folder.exists():
        files = sorted(folder.glob("rom_*.json"))
        if files:
            latest = files[-1]
            with open(latest) as f:
                data = json.load(f)
            joints = data["joints"]
            ranges = {
                JOINT_FE: (joints[JOINT_FE]["min"], joints[JOINT_FE]["max"]),
                JOINT_RU: (joints[JOINT_RU]["min"], joints[JOINT_RU]["max"]),
                JOINT_PS: (joints[JOINT_PS]["min"], joints[JOINT_PS]["max"]),
            }
            return ranges, str(latest)

    return dict(DEFAULT_RANGES), None
