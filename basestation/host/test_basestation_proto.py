"""Tests for the host side of the basestation UART protocol; no hardware.

    pixi run python -m unittest discover -s basestation/host
"""

import unittest

from basestation_proto import (
    CMD_SEND_BROADCAST,
    CMD_SEND_MSG,
    RESP_ACK,
    RESP_NAK,
    RESP_OTA_DONE,
    RESP_PROGRESS,
    RESP_RECV_MSG,
    Basestation,
    BasestationError,
    build_frame,
    parse_frame,
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


class Resync(unittest.TestCase):
    """The bridge drops bytes while the port is closed, so a newly opened
    port can start with truncated frames; their bogus lengths must not
    swallow the frames after them."""

    def test_truncated_frame_before_the_ack(self):
        # SYNC, a length of 64, then only a few bytes of payload.
        stale = bytes([0xAA, 0x40, 0x00, 0x83, 0x00, 0x00])
        bs = station(stale + build_frame(RESP_ACK))
        bs.send_fw_data(b"x")

    def test_sync_byte_inside_a_cut_frame(self):
        # A cut PROGRESS frame whose last kept byte is 0xAA, the SYNC value.
        cut = IDLE_PROGRESS[:5] + b"\xaa"
        bs = station(cut + HEARTBEAT + build_frame(RESP_ACK))
        bs.send_fw_data(b"x")

    def test_frame_split_across_reads_is_not_lost(self):
        whole = build_frame(RESP_ACK)
        bs = station(whole[:2])
        self.assertIsNone(parse_frame(bs.ser, timeout=0.05))
        bs.ser.incoming += whole[2:]
        frame = parse_frame(bs.ser, timeout=0.2)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.cmd, RESP_ACK)


    def test_frame_with_sync_in_its_payload_split_across_reads(self):
        whole = build_frame(RESP_RECV_MSG, bytes([2, 0xAA, 0xFF]) + b"\xaa\x00\x00x")
        bs = station(whole[:9])
        self.assertIsNone(parse_frame(bs.ser, timeout=0.05))
        bs.ser.incoming += whole[9:]
        frame = parse_frame(bs.ser, timeout=0.2)
        self.assertEqual((frame.cmd, frame.payload), (RESP_RECV_MSG, whole[4:-1]))


class SendWithoutAck(unittest.TestCase):
    """data_relay sends this way so that only its receive thread reads."""

    def test_sends_the_frame_and_reads_nothing(self):
        bs = station(build_frame(RESP_ACK))
        bs.send_mesh_msg(2, b"hi", wait_ack=False)
        bs.send_broadcast(b"hello", wait_ack=False)
        self.assertEqual(
            bytes(bs.ser.written),
            build_frame(CMD_SEND_MSG, b"\x02hi") + build_frame(CMD_SEND_BROADCAST, b"hello"),
        )
        self.assertEqual(bytes(bs.ser.incoming), build_frame(RESP_ACK))


if __name__ == "__main__":
    unittest.main()
