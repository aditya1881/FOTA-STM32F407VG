#!/usr/bin/env python3
"""Generate a simple firmware manifest for CAN_Tx SIM7670 downloads.

The manifest format is plain key=value lines:
- size: firmware size in bytes
- crc: CRC32 of the binary, hex formatted
- active_slot: target slot marker (A or B)
"""

from __future__ import annotations

import argparse
import zlib
from pathlib import Path

DEFAULT_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate manifest.txt for CAN_Rx firmware")
    parser.add_argument("--input", required=True, help="Input firmware binary path")
    parser.add_argument("--output", required=True, help="Output manifest path")
    parser.add_argument("--version", type=int, default=DEFAULT_VERSION, help="Firmware version number")
    parser.add_argument("--slot", choices=("A", "B"), default=None, help="Active firmware slot (A or B); auto-detects from filename if omitted")
    return parser.parse_args()


def detect_slot(input_path: Path, requested_slot: str | None) -> str:
    if requested_slot is not None:
        return requested_slot

    name = input_path.name.lower()
    if "slotb" in name or "slot_b" in name or "_b" in name:
        return "B"
    return "A"


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.is_file():
        print(f"Error: input not found: {input_path}")
        return 1

    data = input_path.read_bytes()
    size = len(data)
    crc = zlib.crc32(data) & 0xFFFFFFFF
    slot = detect_slot(input_path, args.slot)

    manifest = f"size={size}\ncrc=0x{crc:08X}\nactive_slot={slot}\n"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(manifest, encoding="utf-8")

    print("Manifest created")
    print(f"Input   : {input_path}")
    print(f"Output  : {output_path}")
    print(f"Version : {args.version}")
    print(f"Size    : {size} bytes")
    print(f"CRC32   : 0x{crc:08X}")
    print(f"Slot    : {slot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
