"""Tests for the host side of the basestation UART protocol; no hardware.

    pixi run python -m unittest discover -s basestation/host
"""

import unittest

from basestation_proto import (
    RESP_ACK,
    RESP_NAK,
    RESP_OTA_DONE,
    RESP_PROGRESS,
    RESP_RECV_MSG,
    Basestation,
    BasestationError,
    build_frame,
)


class FakeSerial:
    """Serves the given bytes to read() and returns b"" once they run out,
    as a serial port with a timeout does when nothing arrives."""

    def __init__(self, incoming: bytes):
        self.incoming = bytearray(incoming)
        self.written = bytearray()

    def read(self, n: int = 1) -> bytes:
        out = bytes(self.incoming[:n])
        del self.incoming[:n]
        return out

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)


def station(incoming: bytes) -> Basestation:
    bs = Basestation.__new__(Basestation)
    bs.ser = FakeSerial(incoming)
    bs.timeout = 0.2
    return bs


IDLE_PROGRESS = build_frame(RESP_PROGRESS, bytes(6))
HEARTBEAT = build_frame(RESP_RECV_MSG, bytes([2, 0xE9, 0xFF]) + b"V=0 B=0 I=0 D=0 i")


class ExpectAck(unittest.TestCase):
    def test_ack_alone(self):
        station(build_frame(RESP_ACK)).send_fw_data(b"x")

    def test_skips_progress_and_mesh_messages_before_the_ack(self):
        bs = station(IDLE_PROGRESS + HEARTBEAT + IDLE_PROGRESS + build_frame(RESP_ACK))
        bs.send_fw_data(b"x")
        self.assertEqual(bs.ser.incoming, b"")

    def test_nak_after_a_progress_frame_still_raises(self):
        bs = station(IDLE_PROGRESS + build_frame(RESP_NAK, bytes([0x05])))
        with self.assertRaisesRegex(BasestationError, "0x05"):
            bs.send_fw_data(b"x")

    def test_other_reply_raises(self):
        bs = station(build_frame(RESP_OTA_DONE, bytes([0])))
        with self.assertRaisesRegex(BasestationError, "Unexpected"):
            bs.send_fw_data(b"x")

    def test_only_unsolicited_frames_time_out(self):
        bs = station(IDLE_PROGRESS + HEARTBEAT)
        with self.assertRaisesRegex(BasestationError, "Timeout"):
            bs.send_fw_data(b"x")


if __name__ == "__main__":
    unittest.main()
