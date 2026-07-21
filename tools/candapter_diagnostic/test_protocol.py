import time
import unittest

from candapter import parse_candapter_frame
from protocol import (
    CANFrame,
    decode_frame,
    fault_status_rows,
)


class ProtocolTests(unittest.TestCase):
    def test_temperature_decode(self) -> None:
        frame = CANFrame(0x108, bytes.fromhex("00 1B 00 1D"), time.time())
        decoded = decode_frame(frame)
        self.assertEqual(decoded["fields"]["Lowest temperature"], 27)
        self.assertEqual(decoded["fields"]["Highest temperature"], 29)

    def test_electrical_decode(self) -> None:
        frame = CANFrame(
            0x109,
            bytes.fromhex("85 CF 83 3F 7F FF"),
            time.time(),
        )
        decoded = decode_frame(frame)
        self.assertAlmostEqual(
            decoded["fields"]["Highest cell voltage"],
            3.4255,
        )
        self.assertAlmostEqual(
            decoded["fields"]["Lowest cell voltage"],
            3.3599,
        )
        self.assertAlmostEqual(decoded["fields"]["Pack current"], 0.1)

    def test_fault_status_decode(self) -> None:
        frame = CANFrame(0x505, b"\x01", time.time())
        decoded = decode_frame(frame)
        self.assertTrue(decoded["fields"]["Fault active"])

    def test_candapter_standard_frame_parse(self) -> None:
        frame = parse_candapter_frame("T109685CF833F7FFF")
        self.assertIsNotNone(frame)
        assert frame is not None
        self.assertEqual(frame.arbitration_id, 0x109)
        self.assertEqual(frame.data, bytes.fromhex("85 CF 83 3F 7F FF"))

    def test_broadcast_soc(self) -> None:
        frame = CANFrame(
            0x100,
            bytes.fromhex("01 F4 00 00 00 00 40 00"),
            time.time(),
        )
        decoded = decode_frame(frame)
        self.assertEqual(decoded["fields"]["State of charge"], 50.0)

    def test_normal_frames_have_no_value_faults(self) -> None:
        now = time.time()
        latest = {
            0x108: CANFrame(0x108, bytes.fromhex("00 1B 00 1D"), now),
            0x109: CANFrame(
                0x109,
                bytes.fromhex("85 CF 83 3F 7F FF"),
                now,
            ),
            0x505: CANFrame(0x505, b"\x00", now),
        }
        rows = fault_status_rows(latest)
        self.assertTrue(all(row["Status"] == "OK" for row in rows))


if __name__ == "__main__":
    unittest.main()
