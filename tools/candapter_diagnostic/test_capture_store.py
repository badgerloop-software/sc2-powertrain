import tempfile
import time
import unittest
from pathlib import Path

from capture_store import list_captures, load_capture, save_capture
from protocol import CANFrame


class CaptureStoreTests(unittest.TestCase):
    def test_capture_round_trip(self) -> None:
        frames = [
            CANFrame(
                arbitration_id=0x101,
                data=bytes.fromhex("00 00 00 00 64 00 00 00"),
                timestamp=time.time(),
            ),
            CANFrame(
                arbitration_id=0x18FF50E5,
                data=bytes.fromhex("01 02 03"),
                timestamp=time.time() + 1,
                is_extended=True,
            ),
        ]

        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            saved_path = save_capture(frames, directory)

            self.assertEqual(list_captures(directory), [saved_path])
            self.assertEqual(load_capture(saved_path), frames)

    def test_empty_capture_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            with self.assertRaises(ValueError):
                save_capture([], Path(temporary_directory))

    def test_fault_capture_uses_distinct_filename(self) -> None:
        frame = CANFrame(0x505, b"\x01", time.time())
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            saved_path = save_capture(
                [frame],
                directory,
                filename_prefix="fault_capture",
            )

            self.assertTrue(saved_path.name.startswith("fault_capture_"))
            self.assertIn(saved_path, list_captures(directory))
            self.assertEqual(load_capture(saved_path), [frame])


if __name__ == "__main__":
    unittest.main()
