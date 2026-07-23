#!/usr/bin/env python3
"""
Create CAN_Tx storage partition image for preloading CAN_Rx firmware.

Storage format at partition base:
- magic    (u32 LE): 0x3241544F ('OTA2')
- size     (u32 LE): firmware size in bytes
- crc32    (u32 LE): IEEE CRC-32 of firmware bytes
- reserved (u32 LE): 0xFFFFFFFF
- payload  : raw firmware bytes
- padding  : 0xFF to full partition size
"""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

MAGIC_OTA2 = 0x3241544F
DEFAULT_PARTITION_SIZE = 128 * 1024
HEADER_SIZE = 16


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pack CAN_Rx bin into CAN_Tx storage partition image")
    parser.add_argument("--input", required=True, help="Input firmware binary (e.g., CAN_Rx/build/CAN_Rx.bin)")
    parser.add_argument("--output", required=True, help="Output partition image binary")
    parser.add_argument(
        "--partition-size",
        type=int,
        default=DEFAULT_PARTITION_SIZE,
        help=f"Partition size in bytes (default: {DEFAULT_PARTITION_SIZE})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.is_file():
        print(f"Error: input not found: {input_path}")
        return 1

    fw = input_path.read_bytes()
    fw_size = len(fw)
    max_payload = args.partition_size - HEADER_SIZE

    if fw_size == 0:
        print("Error: input firmware is empty")
        return 1

    if fw_size > max_payload:
        print(
            f"Error: firmware too large ({fw_size} bytes). "
            f"Max allowed for partition is {max_payload} bytes"
        )
        return 1

    crc = zlib.crc32(fw) & 0xFFFFFFFF
    header = struct.pack("<IIII", MAGIC_OTA2, fw_size, crc, 0xFFFFFFFF)

    image = bytearray(args.partition_size)
    image[:] = b"\xFF" * args.partition_size
    image[0:HEADER_SIZE] = header
    image[HEADER_SIZE : HEADER_SIZE + fw_size] = fw

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(image)

    print("Partition image created")
    print(f"Input   : {input_path}")
    print(f"Output  : {output_path}")
    print(f"Size    : {fw_size} bytes")
    print(f"CRC32   : 0x{crc:08X}")
    print(f"Image   : {args.partition_size} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
