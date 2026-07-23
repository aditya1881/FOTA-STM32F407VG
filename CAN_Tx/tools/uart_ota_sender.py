#!/usr/bin/env python3
import argparse
import binascii
import struct
import sys
import time

import serial

OTA_UART_MAGIC = 0x3141544F  # 'OTA1' little-endian in stream


def crc32_file(path: str) -> tuple[bytes, int]:
    with open(path, "rb") as f:
        data = f.read()
    return data, (binascii.crc32(data) & 0xFFFFFFFF)


def main() -> int:
    parser = argparse.ArgumentParser(description="Send firmware .bin to CAN_Tx UART OTA streamer")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM6")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--bin", required=True, dest="bin_path", help="Path to firmware .bin")
    parser.add_argument("--chunk-delay-ms", type=float, default=0.0, help="Delay between UART chunks")
    parser.add_argument("--uart-chunk", type=int, default=64, help="UART write chunk size")
    args = parser.parse_args()

    image, image_crc = crc32_file(args.bin_path)
    image_size = len(image)
    if image_size == 0:
        print("Error: empty bin file", file=sys.stderr)
        return 2

    header = struct.pack("<III", OTA_UART_MAGIC, image_size, image_crc)

    print(f"Sending: {args.bin_path}")
    print(f"Size   : {image_size} bytes")
    print(f"CRC32  : 0x{image_crc:08X}")

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(header)
        sent = 0
        while sent < image_size:
            n = min(args.uart_chunk, image_size - sent)
            ser.write(image[sent:sent + n])
            sent += n
            if args.chunk_delay_ms > 0:
                time.sleep(args.chunk_delay_ms / 1000.0)

        ser.flush()

        # Optional: print device debug output for a short period.
        deadline = time.time() + 3.0
        print("Waiting for device logs...")
        while time.time() < deadline:
            line = ser.readline()
            if line:
                try:
                    print(line.decode("utf-8", errors="replace").rstrip())
                except Exception:
                    pass

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
