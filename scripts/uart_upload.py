#!/usr/bin/env python3
"""
UART uploader for CAN_Tx OTA sender.

Protocol sent to CAN_Tx:
- 1 byte: 'U' (0x55)
- 4 bytes: image size (uint32 little-endian)
- N bytes: raw binary image payload
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("Error: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Upload firmware .bin to CAN_Tx over UART using U + size + data protocol"
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--bin", required=True, dest="bin_path", help="Path to firmware .bin file")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate (default: 115200)")
    parser.add_argument("--chunk", type=int, default=256, help="Chunk bytes per write (default: 256)")
    parser.add_argument(
        "--chunk-delay",
        type=float,
        default=0.002,
        help="Delay between chunk writes in seconds (default: 0.002)",
    )
    parser.add_argument("--timeout", type=float, default=2.0, help="Serial timeout in seconds (default: 2.0)")
    parser.add_argument(
        "--post-wait",
        type=float,
        default=0.5,
        help="Wait time after transfer in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--open-wait",
        type=float,
        default=1.0,
        help="Wait after opening serial port before send (default: 1.0)",
    )
    parser.add_argument(
        "--status-check",
        action="store_true",
        help="Send 'Q' after upload and print target stored-image status",
    )
    return parser.parse_args()


def human_size(num: int) -> str:
    units = ["B", "KB", "MB", "GB"]
    value = float(num)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            return f"{value:.2f} {unit}"
        value /= 1024.0
    return f"{num} B"


def load_binary(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()


def wait_for_token(ser: serial.Serial, token: bytes, timeout_s: float) -> bytes:
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(128)
        if chunk:
            buf.extend(chunk)
            if token in buf:
                return bytes(buf)
    return bytes(buf)


def upload(
    port: str,
    baud: int,
    payload: bytes,
    chunk_size: int,
    chunk_delay: float,
    timeout: float,
    post_wait: float,
    open_wait: float,
    status_check: bool,
) -> None:
    total = len(payload)
    header = b"U" + struct.pack("<I", total)

    with serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=timeout) as ser:
        # Avoid unintended resets on some USB-UART adapters.
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass

        if open_wait > 0:
            time.sleep(open_wait)

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(header)
        ser.flush()

        # Wait until target finishes flash erase and announces READY.
        ready_buf = wait_for_token(ser, b"READY", 8.0)
        if b"READY" not in ready_buf:
            text = ready_buf.decode(errors="replace")
            raise RuntimeError(f"Target did not send READY before payload. Output: {text}")

        sent = 0
        start = time.time()
        while sent < total:
            part = payload[sent : sent + chunk_size]
            ser.write(part)
            ser.flush()

            # Wait for per-chunk ACK from target before sending next bytes.
            ack_buf = wait_for_token(ser, b"K", 2.0)
            if b"K" not in ack_buf:
                text = ack_buf.decode(errors="replace")
                raise RuntimeError(f"Target missing per-chunk ACK K. Output: {text}")

            sent += len(part)

            if chunk_delay > 0:
                time.sleep(chunk_delay)

            percent = (sent * 100.0) / total if total else 100.0
            sys.stdout.write(f"\rSent {sent}/{total} bytes ({percent:.1f}%)")
            sys.stdout.flush()

        ser.flush()
        elapsed = time.time() - start
        if post_wait > 0:
            time.sleep(post_wait)

        # Wait for explicit upload completion marker.
        ok_buf = wait_for_token(ser, b"UPLOAD_OK", 6.0)
        if b"UPLOAD_OK" not in ok_buf:
            text = ok_buf.decode(errors="replace")
            raise RuntimeError(f"Target did not confirm UPLOAD_OK. Output: {text}")

        if status_check:
            ser.write(b"Q")
            ser.flush()
            deadline = time.time() + 2.5
            chunks: list[bytes] = []
            while time.time() < deadline:
                out = ser.read(256)
                if out:
                    chunks.append(out)
                    continue
                # idle read; break once we have at least some output
                if chunks:
                    break

            out = b"".join(chunks)
            if out:
                print("Target reply:")
                print(out.decode(errors="replace"))
            else:
                print("Target reply: <none>")

    sys.stdout.write("\n")
    print(f"Upload complete in {elapsed:.2f}s")


def main() -> int:
    args = parse_args()

    if not os.path.isfile(args.bin_path):
        print(f"Error: file not found: {args.bin_path}", file=sys.stderr)
        return 1

    if args.chunk <= 0:
        print("Error: --chunk must be > 0", file=sys.stderr)
        return 1

    payload = load_binary(args.bin_path)
    size = len(payload)
    if size == 0:
        print("Error: binary file is empty", file=sys.stderr)
        return 1

    if size > 0xFFFFFFFF:
        print("Error: binary too large for 32-bit size field", file=sys.stderr)
        return 1

    print("UART upload start")
    print(f"Port     : {args.port}")
    print(f"Baud     : {args.baud}")
    print(f"Binary   : {args.bin_path}")
    print(f"Size     : {size} bytes ({human_size(size)})")
    print(f"Chunk    : {args.chunk} bytes")
    print(f"Chunk dly: {args.chunk_delay:.3f}s")
    print(f"Open wait: {args.open_wait:.2f}s")

    try:
        upload(
            port=args.port,
            baud=args.baud,
            payload=payload,
            chunk_size=args.chunk,
            chunk_delay=args.chunk_delay,
            timeout=args.timeout,
            post_wait=args.post_wait,
            open_wait=args.open_wait,
            status_check=args.status_check,
        )
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"Protocol error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"OS error: {exc}", file=sys.stderr)
        return 1

    print("Protocol sent: 'U' + <size_u32_le> + <raw_bin>")
    print("Now press the user button on CAN_Tx board to start CAN OTA transfer.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
