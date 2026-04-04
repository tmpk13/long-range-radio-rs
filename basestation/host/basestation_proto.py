"""
UART frame protocol for basestation communication.

Frame format:
    [SYNC 0xAA] [LEN_LO] [LEN_HI] [CMD] [PAYLOAD: LEN bytes] [CRC8]

LEN is payload length only.  CRC8 covers CMD + PAYLOAD.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass

import serial

SYNC = 0xAA
MAX_PAYLOAD = 256

# ── Command IDs ─────────────────────────────────────────────────────────────

# Host -> Basestation (OTA)
CMD_START_OTA = 0x01
CMD_FW_DATA = 0x02
CMD_BEGIN_TRANSFER = 0x03
CMD_QUERY_STATUS = 0x04
CMD_ABORT_OTA = 0x05

# Host -> Basestation (Data)
CMD_SEND_MSG = 0x10
CMD_SEND_BROADCAST = 0x11

# Basestation -> Host
RESP_ACK = 0x81
RESP_NAK = 0x82
RESP_PROGRESS = 0x83
RESP_OTA_DONE = 0x84
RESP_RECV_MSG = 0x90

# OTA states
OTA_IDLE = 0x00
OTA_RECEIVING_FW = 0x01
OTA_TRANSFERRING = 0x02
OTA_COMPLETE = 0x03
OTA_FAILED = 0x04

# Error codes
ERR_INVALID_STATE = 0x01
ERR_BAD_SIZE = 0x02
ERR_FLASH_ERROR = 0x03
ERR_BAD_FRAME = 0x04
ERR_NO_FW_DATA = 0x05


# ── CRC-8 ───────────────────────────────────────────────────────────────────

def crc8(data: bytes) -> int:
    """CRC-8 with polynomial 0x07 (CRC-8/ITU)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ── Frame building / parsing ────────────────────────────────────────────────

def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """Build a complete UART frame."""
    plen = len(payload)
    header = bytes([SYNC, plen & 0xFF, (plen >> 8) & 0xFF, cmd])
    crc = crc8(bytes([cmd]) + payload)
    return header + payload + bytes([crc])


@dataclass
class Frame:
    cmd: int
    payload: bytes


def parse_frame(ser: serial.Serial, timeout: float = 2.0) -> Frame | None:
    """Read one frame from serial.  Returns None on timeout."""
    deadline = time.monotonic() + timeout

    # Wait for sync byte
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == SYNC:
            break
    else:
        return None

    # Read LEN (2 bytes)
    raw_len = ser.read(2)
    if len(raw_len) < 2:
        return None
    plen = raw_len[0] | (raw_len[1] << 8)
    if plen > MAX_PAYLOAD:
        return None

    # Read CMD (1 byte) + PAYLOAD (plen bytes) + CRC (1 byte)
    remaining = 1 + plen + 1
    data = ser.read(remaining)
    if len(data) < remaining:
        return None

    cmd = data[0]
    payload = data[1 : 1 + plen]
    received_crc = data[-1]

    expected_crc = crc8(bytes([cmd]) + payload)
    if received_crc != expected_crc:
        return None

    return Frame(cmd=cmd, payload=payload)


# ── High-level protocol wrapper ─────────────────────────────────────────────

class BasestationError(Exception):
    pass


class Basestation:
    """High-level interface to a basestation node over serial."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.timeout = timeout
        # Flush stale data
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _send(self, cmd: int, payload: bytes = b""):
        self.ser.write(build_frame(cmd, payload))

    def _recv(self, timeout: float | None = None) -> Frame | None:
        return parse_frame(self.ser, timeout or self.timeout)

    def _expect_ack(self, timeout: float | None = None):
        frame = self._recv(timeout)
        if frame is None:
            raise BasestationError("Timeout waiting for response")
        if frame.cmd == RESP_NAK:
            code = frame.payload[0] if frame.payload else 0
            raise BasestationError(f"NAK received: error code 0x{code:02X}")
        if frame.cmd != RESP_ACK:
            raise BasestationError(
                f"Unexpected response: cmd=0x{frame.cmd:02X}"
            )

    def start_ota(self, target_addr: int, fw_size: int, fw_version: int):
        """Send START_OTA command."""
        payload = struct.pack("<BIH", target_addr, fw_size, fw_version)
        self._send(CMD_START_OTA, payload)
        self._expect_ack()

    def send_fw_data(self, data: bytes):
        """Send a chunk of firmware data."""
        self._send(CMD_FW_DATA, data)
        self._expect_ack()

    def begin_transfer(self):
        """Start the mesh OTA transfer."""
        self._send(CMD_BEGIN_TRANSFER)
        self._expect_ack()

    def query_status(self) -> tuple[int, int, int, int] | None:
        """Query OTA status.  Returns (state, pct, sent, total) or None."""
        self._send(CMD_QUERY_STATUS)
        frame = self._recv()
        if frame is None:
            return None
        if frame.cmd == RESP_PROGRESS and len(frame.payload) >= 6:
            state = frame.payload[0]
            pct = frame.payload[1]
            sent = struct.unpack_from("<H", frame.payload, 2)[0]
            total = struct.unpack_from("<H", frame.payload, 4)[0]
            return (state, pct, sent, total)
        return None

    def abort_ota(self):
        """Abort current OTA transfer."""
        self._send(CMD_ABORT_OTA)
        self._expect_ack()

    def send_mesh_msg(self, dest: int, data: bytes):
        """Send a message to a specific mesh node."""
        self._send(CMD_SEND_MSG, bytes([dest]) + data)
        self._expect_ack()

    def send_broadcast(self, data: bytes):
        """Broadcast a message to all mesh nodes."""
        self._send(CMD_SEND_BROADCAST, data)
        self._expect_ack()

    def recv_mesh_msg(
        self, timeout: float = 1.0
    ) -> tuple[int, int, bytes] | None:
        """
        Receive a mesh message from the basestation.
        Returns (src_addr, rssi, data) or None on timeout.
        """
        frame = self._recv(timeout)
        if frame is None:
            return None
        if frame.cmd == RESP_RECV_MSG and len(frame.payload) >= 3:
            src = frame.payload[0]
            rssi = struct.unpack_from("<h", frame.payload, 1)[0]
            data = frame.payload[3:]
            return (src, rssi, data)
        # Could be a progress frame; skip it
        return None

    def drain_progress(self) -> tuple[int, int, int, int] | None:
        """Read any pending PROGRESS frame (non-blocking)."""
        frame = self._recv(timeout=0.1)
        if frame and frame.cmd == RESP_PROGRESS and len(frame.payload) >= 6:
            state = frame.payload[0]
            pct = frame.payload[1]
            sent = struct.unpack_from("<H", frame.payload, 2)[0]
            total = struct.unpack_from("<H", frame.payload, 4)[0]
            return (state, pct, sent, total)
        return None

    def wait_ota_done(
        self, timeout: float = 3600.0
    ) -> tuple[bool, int]:
        """
        Wait for OTA_DONE response.
        Returns (success: bool, result_byte: int).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._recv(timeout=2.0)
            if frame is None:
                continue
            if frame.cmd == RESP_OTA_DONE:
                result = frame.payload[0] if frame.payload else 0xFF
                return (result == 0x00, result)
            if frame.cmd == RESP_PROGRESS:
                # Yield progress info to caller
                continue
        raise BasestationError("Timeout waiting for OTA completion")
