"""Threaded serial driver for the Ewert Energy Systems CANdapter."""

from __future__ import annotations

from collections import deque
import queue
import threading
import time

import serial
from serial.tools import list_ports

from protocol import CANFrame


CAN_BITRATE_COMMANDS = {
    10_000: "S0",
    20_000: "S1",
    50_000: "S2",
    100_000: "S3",
    125_000: "S4",
    250_000: "S5",
    500_000: "S6",
    800_000: "S7",
    1_000_000: "S8",
}

ACK = 0x06
BELL = 0x07


class CANdapterError(RuntimeError):
    pass


class CANdapter:
    def __init__(
        self,
        port: str,
        serial_baudrate: int = 9600,
        can_bitrate: int = 250_000,
        max_frames: int = 5_000,
    ) -> None:
        if can_bitrate not in CAN_BITRATE_COMMANDS:
            raise ValueError(f"Unsupported CAN bitrate: {can_bitrate}")
        self.port = port
        self.serial_baudrate = serial_baudrate
        self.can_bitrate = can_bitrate
        self._serial: serial.Serial | None = None
        self._frames: deque[CANFrame] = deque(maxlen=max_frames)
        self._frame_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._responses: queue.Queue[int] = queue.Queue()
        self._errors: deque[str] = deque(maxlen=100)
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None

    @staticmethod
    def available_ports() -> list[str]:
        return [port.device for port in list_ports.comports()]

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        if self.connected:
            return
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.serial_baudrate,
            timeout=0.1,
            write_timeout=1.0,
        )
        try:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self._initialize_command(CAN_BITRATE_COMMANDS[self.can_bitrate])
            self._initialize_command("O")
        except Exception:
            self._serial.close()
            self._serial = None
            raise

        self._stop.clear()
        self._reader = threading.Thread(
            target=self._reader_loop,
            name="candapter-reader",
            daemon=True,
        )
        self._reader.start()

    def _initialize_command(self, command: str, timeout: float = 1.0) -> None:
        if self._serial is None:
            raise CANdapterError("Serial port is not open")
        self._serial.write(command.encode("ascii") + b"\r")
        self._serial.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self._serial.read(1)
            if response == bytes([ACK]):
                return
            if response == bytes([BELL]):
                raise CANdapterError(
                    f"CANdapter rejected initialization command {command}"
                )
        raise CANdapterError(
            f"CANdapter did not acknowledge initialization command {command}"
        )

    def send_frame(
        self,
        arbitration_id: int,
        data: bytes,
        is_extended: bool = False,
        timeout: float = 1.0,
    ) -> None:
        if not self.connected or self._serial is None:
            raise CANdapterError("CANdapter is not connected")
        if len(data) > 8:
            raise ValueError("Classic CAN payloads are limited to 8 bytes")
        if is_extended:
            if not 0 <= arbitration_id <= 0x1FFFFFFF:
                raise ValueError("Extended CAN ID is out of range")
            command = (
                f"X{arbitration_id:08X}{len(data):X}{data.hex().upper()}"
            )
        else:
            if not 0 <= arbitration_id <= 0x7FF:
                raise ValueError("Standard CAN ID is out of range")
            command = (
                f"T{arbitration_id:03X}{len(data):X}{data.hex().upper()}"
            )

        with self._write_lock:
            while True:
                try:
                    self._responses.get_nowait()
                except queue.Empty:
                    break
            self._serial.write(command.encode("ascii") + b"\r")
            self._serial.flush()
            try:
                response = self._responses.get(timeout=timeout)
            except queue.Empty as exc:
                raise CANdapterError(
                    "Timed out waiting for CANdapter transmit response"
                ) from exc
            if response != ACK:
                raise CANdapterError("CANdapter rejected the CAN frame")

    def frames_snapshot(self) -> list[CANFrame]:
        with self._frame_lock:
            return list(self._frames)

    def clear_frames(self) -> None:
        with self._frame_lock:
            self._frames.clear()

    def errors_snapshot(self) -> list[str]:
        return list(self._errors)

    def _reader_loop(self) -> None:
        line = bytearray()
        while not self._stop.is_set():
            try:
                if self._serial is None:
                    return
                chunk = self._serial.read(1)
                if not chunk:
                    continue
                value = chunk[0]
                if value in (ACK, BELL):
                    self._responses.put(value)
                elif value == 0x0D:
                    if line:
                        self._consume_line(line.decode("ascii", errors="ignore"))
                        line.clear()
                elif value != 0x0A:
                    line.append(value)
            except (OSError, serial.SerialException) as exc:
                self._errors.append(str(exc))
                return

    def _consume_line(self, line: str) -> None:
        try:
            frame = parse_candapter_frame(line)
        except ValueError as exc:
            self._errors.append(f"{line!r}: {exc}")
            return
        if frame is not None:
            with self._frame_lock:
                self._frames.append(frame)

    def close(self) -> None:
        serial_port = self._serial
        if serial_port is None:
            return
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=0.5)
        try:
            if serial_port.is_open:
                serial_port.write(b"C\r")
                serial_port.flush()
                time.sleep(0.05)
                serial_port.close()
        finally:
            self._serial = None
            self._reader = None


def parse_candapter_frame(line: str) -> CANFrame | None:
    """Parse CANdapter T/X receive lines into a CANFrame."""
    line = line.strip()
    if not line:
        return None
    frame_type = line[0].upper()
    if frame_type == "T":
        id_width = 3
        is_extended = False
    elif frame_type == "X":
        id_width = 8
        is_extended = True
    else:
        return None

    minimum_length = 1 + id_width + 1
    if len(line) < minimum_length:
        raise ValueError("truncated frame")
    arbitration_id = int(line[1 : 1 + id_width], 16)
    dlc = int(line[1 + id_width], 16)
    payload_text = line[minimum_length:]
    if dlc > 8 or len(payload_text) != dlc * 2:
        raise ValueError("DLC does not match payload")
    return CANFrame(
        arbitration_id=arbitration_id,
        data=bytes.fromhex(payload_text),
        timestamp=time.time(),
        is_extended=is_extended,
    )
