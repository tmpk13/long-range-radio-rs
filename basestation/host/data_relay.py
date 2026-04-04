#!/usr/bin/env python3
"""
Relay mesh data between the basestation and stdout/stdin.

This acts as a compatibility layer for the daemon — it reads mesh
messages from the basestation's data UART and prints them as JSON
lines to stdout.  Messages from stdin are sent to the mesh.

Usage:
    python data_relay.py --port /dev/ttyUSB1

Input format (stdin, one JSON per line):
    {"dest": 2, "data": "aGVsbG8="}       # base64-encoded, to specific node
    {"broadcast": true, "data": "aGVsbG8="} # broadcast

Output format (stdout, one JSON per line):
    {"src": 3, "rssi": -45, "data": "aGVsbG8=", "timestamp": 1711900000.0}
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
import threading
import time

import serial

from basestation_proto import (
    RESP_RECV_MSG,
    Basestation,
    BasestationError,
    parse_frame,
)


def rx_thread(bs: Basestation, running: threading.Event):
    """Read mesh messages from basestation and print as JSON to stdout."""
    while running.is_set():
        try:
            frame = parse_frame(bs.ser, timeout=1.0)
            if frame is None:
                continue
            if frame.cmd == RESP_RECV_MSG and len(frame.payload) >= 3:
                src = frame.payload[0]
                rssi = int.from_bytes(frame.payload[1:3], "little", signed=True)
                data = frame.payload[3:]
                msg = {
                    "src": src,
                    "rssi": rssi,
                    "data": base64.b64encode(data).decode("ascii"),
                    "timestamp": time.time(),
                }
                print(json.dumps(msg), flush=True)
        except Exception as e:
            print(
                json.dumps({"error": str(e), "timestamp": time.time()}),
                file=sys.stderr,
                flush=True,
            )


def tx_thread(bs: Basestation, running: threading.Event):
    """Read JSON messages from stdin and send to mesh via basestation."""
    while running.is_set():
        try:
            line = sys.stdin.readline()
            if not line:
                running.clear()
                break
            line = line.strip()
            if not line:
                continue

            msg = json.loads(line)
            data = base64.b64decode(msg["data"])

            if msg.get("broadcast"):
                bs.send_broadcast(data)
            elif "dest" in msg:
                bs.send_mesh_msg(msg["dest"], data)
            else:
                print(
                    json.dumps({"error": "Missing 'dest' or 'broadcast' field"}),
                    file=sys.stderr,
                    flush=True,
                )
        except json.JSONDecodeError as e:
            print(
                json.dumps({"error": f"Invalid JSON: {e}"}),
                file=sys.stderr,
                flush=True,
            )
        except BasestationError as e:
            print(
                json.dumps({"error": f"Send failed: {e}"}),
                file=sys.stderr,
                flush=True,
            )
        except Exception as e:
            print(
                json.dumps({"error": str(e)}),
                file=sys.stderr,
                flush=True,
            )


def main():
    parser = argparse.ArgumentParser(
        description="Relay mesh data between basestation and stdin/stdout"
    )
    parser.add_argument(
        "--port", "-p", required=True, help="Data UART serial port (e.g., /dev/ttyUSB1)"
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    args = parser.parse_args()

    try:
        bs = Basestation(args.port, args.baud)
    except serial.SerialException as e:
        print(f"Cannot open serial port: {e}", file=sys.stderr)
        sys.exit(1)

    running = threading.Event()
    running.set()

    rx = threading.Thread(target=rx_thread, args=(bs, running), daemon=True)
    tx = threading.Thread(target=tx_thread, args=(bs, running), daemon=True)

    rx.start()
    tx.start()

    print(
        json.dumps({"status": "connected", "port": args.port}),
        file=sys.stderr,
        flush=True,
    )

    try:
        while running.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()
        bs.close()
        print(
            json.dumps({"status": "disconnected"}),
            file=sys.stderr,
            flush=True,
        )


if __name__ == "__main__":
    main()
