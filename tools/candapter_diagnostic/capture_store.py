"""Persistent CSV storage for CAN frame captures."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path

from protocol import CANFrame


CAPTURE_DIRECTORY = Path(__file__).with_name("captures")
CAPTURE_COLUMNS = (
    "timestamp",
    "arbitration_id",
    "data",
    "is_extended",
)


def save_capture(
    frames: list[CANFrame],
    directory: Path = CAPTURE_DIRECTORY,
    filename_prefix: str = "capture",
) -> Path:
    """Save a non-empty frame snapshot and return its CSV path."""
    if not frames:
        raise ValueError("Cannot save an empty capture")
    if filename_prefix not in {"capture", "fault_capture"}:
        raise ValueError("Unsupported capture filename prefix")

    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fZ")
    path = directory / f"{filename_prefix}_{stamp}.csv"
    temporary_path = path.with_suffix(".tmp")

    with temporary_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CAPTURE_COLUMNS)
        writer.writeheader()
        for frame in frames:
            writer.writerow(
                {
                    "timestamp": repr(frame.timestamp),
                    "arbitration_id": f"0x{frame.arbitration_id:X}",
                    "data": frame.data.hex().upper(),
                    "is_extended": int(frame.is_extended),
                }
            )

    temporary_path.replace(path)
    return path


def list_captures(
    directory: Path = CAPTURE_DIRECTORY,
) -> list[Path]:
    """Return saved captures from newest to oldest."""
    if not directory.exists():
        return []
    return sorted(
        directory.glob("*capture_*.csv"),
        key=lambda path: (path.stat().st_mtime_ns, path.name),
        reverse=True,
    )


def load_capture(path: Path) -> list[CANFrame]:
    """Load and validate CAN frames from a saved capture."""
    frames: list[CANFrame] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != list(CAPTURE_COLUMNS):
            raise ValueError("Capture has an unsupported CSV header")

        for row_number, row in enumerate(reader, start=2):
            try:
                is_extended = row["is_extended"] == "1"
                if row["is_extended"] not in {"0", "1"}:
                    raise ValueError("invalid extended-frame flag")
                arbitration_id = int(row["arbitration_id"], 0)
                maximum_id = 0x1FFFFFFF if is_extended else 0x7FF
                if not 0 <= arbitration_id <= maximum_id:
                    raise ValueError("arbitration ID is out of range")
                data = bytes.fromhex(row["data"])
                if len(data) > 8:
                    raise ValueError("payload exceeds 8 bytes")
                frames.append(
                    CANFrame(
                        arbitration_id=arbitration_id,
                        data=data,
                        timestamp=float(row["timestamp"]),
                        is_extended=is_extended,
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid capture row {row_number}: {exc}"
                ) from exc

    return frames
