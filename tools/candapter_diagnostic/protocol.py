"""SC2 powertrain CAN protocol definitions and decoders."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import struct
from typing import Any


BPS_STATUS_CAN_ID = 0x100
BPS_TEMPERATURE_CAN_ID = 0x108
BPS_ELECTRICAL_CAN_ID = 0x109
POWERTRAIN_FAULT_CAN_ID = 0x505
FAULT_CLEAR_CAN_ID = 0x7EB

FAULT_CLEAR_PAYLOAD = bytes.fromhex("03 7F 20 22 00 00 00 00")

CELL_VOLTAGE_MIN_V = 2.52
CELL_VOLTAGE_MAX_V = 4.18
PACK_CURRENT_MAX_A = 35.0
TEMPERATURE_MIN_C = 4.0
TEMPERATURE_MAX_C = 56.0

CELL_VOLTAGE_SCALE_V = 0.0001
PACK_CURRENT_ZERO_RAW = 0x8000
PACK_CURRENT_SCALE_A = 0.1
STATE_OF_CHARGE_SCALE_PERCENT = 0.1


@dataclass(frozen=True)
class CANFrame:
    arbitration_id: int
    data: bytes
    timestamp: float
    is_extended: bool = False

    @property
    def dlc(self) -> int:
        return len(self.data)

    @property
    def id_hex(self) -> str:
        width = 8 if self.is_extended else 3
        return f"{self.arbitration_id:0{width}X}"

    @property
    def data_hex(self) -> str:
        return self.data.hex(" ").upper()


def _u16_be(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "big", signed=False)


def decode_frame(frame: CANFrame) -> dict[str, Any]:
    """Decode a known frame, retaining raw data for unknown IDs."""
    result: dict[str, Any] = {
        "timestamp": datetime.fromtimestamp(frame.timestamp).strftime(
            "%H:%M:%S.%f"
        )[:-3],
        "id": f"0x{frame.id_hex}",
        "dlc": frame.dlc,
        "data": frame.data_hex,
        "name": "Unknown",
        "fields": {},
        "error": "",
    }

    try:
        if frame.arbitration_id == BPS_STATUS_CAN_ID:
            if frame.dlc != 8:
                raise ValueError("expected DLC 8")
            result["name"] = "BPS pack status"
            result["fields"] = {
                "State of charge": (
                    _u16_be(frame.data, 0)
                    * STATE_OF_CHARGE_SCALE_PERCENT
                )
            }
        elif frame.arbitration_id == BPS_TEMPERATURE_CAN_ID:
            if frame.dlc != 4:
                raise ValueError("expected DLC 4")
            result["name"] = "BPS temperatures"
            result["fields"] = {
                "Lowest temperature": _u16_be(frame.data, 0),
                "Highest temperature": _u16_be(frame.data, 2),
            }
        elif frame.arbitration_id == BPS_ELECTRICAL_CAN_ID:
            if frame.dlc != 6:
                raise ValueError("expected DLC 6")
            raw_current = _u16_be(frame.data, 4)
            result["name"] = "BPS electrical"
            result["fields"] = {
                "Highest cell voltage": (
                    _u16_be(frame.data, 0) * CELL_VOLTAGE_SCALE_V
                ),
                "Lowest cell voltage": (
                    _u16_be(frame.data, 2) * CELL_VOLTAGE_SCALE_V
                ),
                "Pack current": (
                    abs(raw_current - PACK_CURRENT_ZERO_RAW)
                    * PACK_CURRENT_SCALE_A
                ),
            }
        elif frame.arbitration_id == POWERTRAIN_FAULT_CAN_ID:
            if frame.dlc != 1:
                raise ValueError("expected DLC 1")
            active = bool(frame.data[0] & 0x01)
            result["name"] = "Powertrain fault status"
            result["fields"] = {
                "Fault active": active,
                "Status": "FAULT" if active else "Healthy",
            }
        elif frame.arbitration_id == FAULT_CLEAR_CAN_ID:
            result["name"] = "Powertrain fault-clear command"
            result["fields"] = {
                "Exact command": frame.data == FAULT_CLEAR_PAYLOAD
            }
        elif 0x500 <= frame.arbitration_id <= 0x504:
            if frame.dlc != 4:
                raise ValueError("expected DLC 4")
            names = {
                0x500: "12 V current",
                0x501: "12 V voltage",
                0x502: "Supplemental current",
                0x503: "Battery current",
                0x504: "Supplemental voltage",
            }
            result["name"] = names[frame.arbitration_id]
            result["fields"] = {
                "Value": struct.unpack("<f", frame.data)[0]
            }
    except (IndexError, struct.error, ValueError) as exc:
        result["error"] = str(exc)

    return result


def fault_status_rows(latest: dict[int, CANFrame]) -> list[dict[str, Any]]:
    """Return every known fault condition with its current status."""
    rows: list[dict[str, Any]] = []

    def add(
        fault: str,
        source: str,
        value: str,
        is_fault: bool | None,
        limit: str,
    ) -> None:
        rows.append(
            {
                "Fault": fault,
                "Source": source,
                "Value": value,
                "Limit": limit,
                "Status": (
                    "UNKNOWN"
                    if is_fault is None
                    else ("FAULT" if is_fault else "OK")
                ),
            }
        )

    temp = latest.get(BPS_TEMPERATURE_CAN_ID)
    if temp is None or temp.dlc != 4:
        for label in ("Low temperature", "High temperature"):
            add(
                label,
                "0x108",
                "—",
                None,
                f"{TEMPERATURE_MIN_C:g}–{TEMPERATURE_MAX_C:g} °C",
            )
    else:
        low_temp = float(_u16_be(temp.data, 0))
        high_temp = float(_u16_be(temp.data, 2))
        add(
            "Low temperature",
            "0x108",
            f"{low_temp:.0f} °C",
            not TEMPERATURE_MIN_C <= low_temp <= TEMPERATURE_MAX_C,
            f"{TEMPERATURE_MIN_C:g}–{TEMPERATURE_MAX_C:g} °C",
        )
        add(
            "High temperature",
            "0x108",
            f"{high_temp:.0f} °C",
            not TEMPERATURE_MIN_C <= high_temp <= TEMPERATURE_MAX_C,
            f"{TEMPERATURE_MIN_C:g}–{TEMPERATURE_MAX_C:g} °C",
        )

    electrical = latest.get(BPS_ELECTRICAL_CAN_ID)
    if electrical is None or electrical.dlc != 6:
        add(
            "High cell over/undervoltage",
            "0x109",
            "—",
            None,
            f"{CELL_VOLTAGE_MIN_V:.2f}–{CELL_VOLTAGE_MAX_V:.2f} V",
        )
        add(
            "Low cell over/undervoltage",
            "0x109",
            "—",
            None,
            f"{CELL_VOLTAGE_MIN_V:.2f}–{CELL_VOLTAGE_MAX_V:.2f} V",
        )
        add(
            "Pack overcurrent",
            "0x109",
            "—",
            None,
            f"≤ {PACK_CURRENT_MAX_A:g} A",
        )
    else:
        high_v = _u16_be(electrical.data, 0) * CELL_VOLTAGE_SCALE_V
        low_v = _u16_be(electrical.data, 2) * CELL_VOLTAGE_SCALE_V
        raw_current = _u16_be(electrical.data, 4)
        current = (
            abs(raw_current - PACK_CURRENT_ZERO_RAW)
            * PACK_CURRENT_SCALE_A
        )
        add(
            "High cell over/undervoltage",
            "0x109",
            f"{high_v:.4f} V",
            not CELL_VOLTAGE_MIN_V <= high_v <= CELL_VOLTAGE_MAX_V,
            f"{CELL_VOLTAGE_MIN_V:.2f}–{CELL_VOLTAGE_MAX_V:.2f} V",
        )
        add(
            "Low cell over/undervoltage",
            "0x109",
            f"{low_v:.4f} V",
            not CELL_VOLTAGE_MIN_V <= low_v <= CELL_VOLTAGE_MAX_V,
            f"{CELL_VOLTAGE_MIN_V:.2f}–{CELL_VOLTAGE_MAX_V:.2f} V",
        )
        add(
            "Pack overcurrent",
            "0x109",
            f"{current:.1f} A",
            current > PACK_CURRENT_MAX_A,
            f"≤ {PACK_CURRENT_MAX_A:g} A",
        )

    status = latest.get(POWERTRAIN_FAULT_CAN_ID)
    if status is None or status.dlc != 1:
        add("Powertrain combined fault", "0x505 bit 0", "—", None, "0")
    else:
        active = bool(status.data[0] & 0x01)
        add(
            "Powertrain combined fault",
            "0x505 bit 0",
            str(int(active)),
            active,
            "0",
        )

    return rows


def parse_hex_data(value: str) -> bytes:
    compact = (
        value.replace("0x", "")
        .replace("0X", "")
        .replace(" ", "")
        .replace(",", "")
        .replace("_", "")
    )
    if len(compact) % 2:
        raise ValueError("Data must contain complete byte pairs")
    data = bytes.fromhex(compact)
    if len(data) > 8:
        raise ValueError("Classic CAN payloads are limited to 8 bytes")
    return data
