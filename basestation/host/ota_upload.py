#!/usr/bin/env python3
"""
Upload firmware to a mesh node via the basestation.

Usage:
    python ota_upload.py --port /dev/ttyUSB0 --target 2 --version 3 firmware.bin

The tool:
  1. Sends the firmware binary to the basestation over UART
  2. Basestation writes it to its DFU flash partition
  3. Basestation sends the firmware to the target node over LoRa mesh
  4. Shows progress bars for both upload and mesh transfer phases
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

from tqdm import tqdm

from basestation_proto import (
    RESP_OTA_DONE,
    RESP_PROGRESS,
    Basestation,
    BasestationError,
    OTA_COMPLETE,
    OTA_FAILED,
    OTA_TRANSFERRING,
    parse_frame,
)

# Maximum chunk size for UART firmware upload
UART_CHUNK_SIZE = 128


def upload_firmware(bs: Basestation, fw_data: bytes, target: int, version: int):
    """Upload firmware to basestation and initiate mesh OTA."""
    fw_size = len(fw_data)
    print(f"Firmware: {fw_size} bytes, target node: {target}, version: {version}")

    # Phase 1: Send START_OTA
    print("Sending OTA start...")
    bs.start_ota(target, fw_size, version)

    # Phase 2: Upload firmware data to basestation
    print("Uploading firmware to basestation...")
    chunks = (fw_size + UART_CHUNK_SIZE - 1) // UART_CHUNK_SIZE
    with tqdm(total=fw_size, unit="B", unit_scale=True, desc="Upload") as pbar:
        for i in range(chunks):
            offset = i * UART_CHUNK_SIZE
            chunk = fw_data[offset : offset + UART_CHUNK_SIZE]
            bs.send_fw_data(chunk)
            pbar.update(len(chunk))

    # Phase 3: Begin mesh transfer
    print("Starting mesh OTA transfer...")
    bs.begin_transfer()

    # Phase 4: Monitor mesh transfer progress
    print("Transferring over mesh (this will take several minutes)...")
    pbar = tqdm(total=100, unit="%", desc="Mesh OTA", bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt}% [{elapsed}<{remaining}]")
    last_pct = 0

    deadline = time.monotonic() + 7200  # 2 hour max
    while time.monotonic() < deadline:
        frame = parse_frame(bs.ser, timeout=2.0)
        if frame is None:
            continue

        if frame.cmd == RESP_PROGRESS and len(frame.payload) >= 6:
            state = frame.payload[0]
            pct = frame.payload[1]
            sent = struct.unpack_from("<H", frame.payload, 2)[0]
            total = struct.unpack_from("<H", frame.payload, 4)[0]

            if pct > last_pct:
                pbar.update(pct - last_pct)
                last_pct = pct

            if total > 0:
                pbar.set_postfix(chunks=f"{sent}/{total}")

            if state == OTA_COMPLETE:
                pbar.update(100 - last_pct)
                pbar.close()
                print("OTA transfer complete!")
                return
            if state == OTA_FAILED:
                pbar.close()
                print("OTA transfer FAILED on basestation side.", file=sys.stderr)
                sys.exit(1)

        elif frame.cmd == RESP_OTA_DONE:
            pbar.close()
            result = frame.payload[0] if frame.payload else 0xFF
            if result == 0x00:
                print("OTA transfer successful! Target node will reboot.")
            else:
                print(
                    f"OTA transfer finished with result: 0x{result:02X}",
                    file=sys.stderr,
                )
                sys.exit(1)
            return

    pbar.close()
    print("Timeout waiting for OTA completion.", file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Upload firmware to a mesh node via basestation"
    )
    parser.add_argument("firmware", help="Path to firmware .bin file")
    parser.add_argument(
        "--port", "-p", required=True, help="Serial port (e.g., /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--target", "-t", type=int, required=True, help="Target node address (1-255)"
    )
    parser.add_argument(
        "--version", "-v", type=int, required=True, help="Firmware version number"
    )
    args = parser.parse_args()

    # Validate
    if not 1 <= args.target <= 255:
        print("Target address must be 1-255", file=sys.stderr)
        sys.exit(1)
    if not 0 <= args.version <= 65535:
        print("Version must be 0-65535", file=sys.stderr)
        sys.exit(1)

    # Read firmware
    try:
        with open(args.firmware, "rb") as f:
            fw_data = f.read()
    except FileNotFoundError:
        print(f"File not found: {args.firmware}", file=sys.stderr)
        sys.exit(1)

    if len(fw_data) == 0:
        print("Firmware file is empty", file=sys.stderr)
        sys.exit(1)

    max_size = 56 * 2048  # 112 KB
    if len(fw_data) > max_size:
        print(
            f"Firmware too large: {len(fw_data)} bytes (max {max_size})",
            file=sys.stderr,
        )
        sys.exit(1)

    # Connect and upload
    try:
        bs = Basestation(args.port, args.baud)
        upload_firmware(bs, fw_data, args.target, args.version)
    except BasestationError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if "bs" in dir():
            bs.close()


if __name__ == "__main__":
    import serial

    main()
