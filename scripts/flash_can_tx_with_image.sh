#!/usr/bin/env bash
set -euo pipefail

# Build and flash CAN_Tx app partition + CAN_Tx storage partition (preloaded CAN_Rx image).
# Storage partition is reserved at 0x080E0000 (128KB).

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage:
  ./flash_can_tx_with_image.sh [path-to-can-rx-bin]

Defaults:
  CAN_Rx bin path: ../CAN_Rx/build/CAN_Rx_slotB.bin

Environment variables:
  STM32_CLI     Path to STM32_Programmer_CLI.exe
  STLINK_FREQ   SWD frequency in kHz (default: 4000)
  STLINK_SN     Optional ST-LINK serial number

What it does:
  1) Builds CAN_Tx firmware
  2) Ensures CAN_Rx Slot-B .bin exists (builds CAN_Rx_slotB if needed)
  3) Packs storage partition image with header+crc
  4) Flashes CAN_Tx app to 0x08000000
  5) Flashes storage image to 0x080E0000
  6) Resets target
EOF
  exit 0
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="$WORKSPACE_ROOT/scripts"
CAN_TX_DIR="$WORKSPACE_ROOT/CAN_Tx"
CAN_RX_DIR="$WORKSPACE_ROOT/CAN_Rx"

RX_BIN_INPUT="${1:-$CAN_RX_DIR/build/CAN_Rx_slotB.bin}"
PACKED_STORAGE_BIN="$SCRIPTS_DIR/can_tx_storage_image.bin"
SLOT_B_BASE=0x08040000
SLOT_B_LIMIT=0x08080000

STM32_CLI="${STM32_CLI:-C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe}"
if [[ ! -x "$STM32_CLI" ]]; then
  echo "Error: STM32 CLI not found/executable: $STM32_CLI" >&2
  exit 2
fi

STLINK_FREQ="${STLINK_FREQ:-4000}"
STLINK_SN="${STLINK_SN:-}"
CONNECT_ARGS=("port=SWD" "freq=$STLINK_FREQ")
if [[ -n "$STLINK_SN" ]]; then
  CONNECT_ARGS+=("sn=$STLINK_SN")
fi

echo "Building CAN_Tx..."
pushd "$CAN_TX_DIR" >/dev/null
make clean all
popd >/dev/null

if [[ ! -f "$RX_BIN_INPUT" ]]; then
  echo "CAN_Rx bin not found at: $RX_BIN_INPUT"
  echo "Building CAN_Rx Slot-B image to generate bin..."
  pushd "$CAN_RX_DIR" >/dev/null
  make clean all TARGET=CAN_Rx_slotB LDSCRIPT=STM32F407VGTX_APP_B_FLASH.ld
  popd >/dev/null
fi

if [[ ! -f "$RX_BIN_INPUT" ]]; then
  echo "Error: CAN_Rx bin still not found: $RX_BIN_INPUT" >&2
  exit 3
fi

# Verify the payload really targets Slot B. The 2nd vector entry is reset handler address.
RESET_HANDLER_HEX="$(python3 - "$RX_BIN_INPUT" <<'PY'
import struct
import sys

path = sys.argv[1]
with open(path, 'rb') as f:
    data = f.read(8)
if len(data) < 8:
    print("", end="")
    sys.exit(0)
_, reset = struct.unpack('<II', data)
print(f"0x{reset:08X}")
PY
)"

if [[ -z "$RESET_HANDLER_HEX" ]]; then
  echo "Error: Invalid CAN_Rx bin (too small): $RX_BIN_INPUT" >&2
  exit 5
fi

RESET_HANDLER_DEC=$((RESET_HANDLER_HEX))
if (( (RESET_HANDLER_DEC & 1) == 0 )) || (( RESET_HANDLER_DEC < SLOT_B_BASE )) || (( RESET_HANDLER_DEC >= SLOT_B_LIMIT )); then
  echo "Error: Provided CAN_Rx image is not linked for Slot B." >&2
  echo "  File         : $RX_BIN_INPUT" >&2
  echo "  Reset handler: $RESET_HANDLER_HEX" >&2
  echo "  Expected     : 0x08040001 .. 0x0807FFFF" >&2
  echo "Use: ../CAN_Rx/build/CAN_Rx_slotB.bin (or run script without args)." >&2
  exit 6
fi

echo "Packing CAN_Tx storage partition image..."
python3 "$SCRIPTS_DIR/pack_can_tx_storage.py" --input "$RX_BIN_INPUT" --output "$PACKED_STORAGE_BIN"

TX_ELF="$CAN_TX_DIR/build/CAN_Tx.elf"
if [[ ! -f "$TX_ELF" ]]; then
  echo "Error: CAN_Tx ELF not found: $TX_ELF" >&2
  exit 4
fi

echo "CAN_Tx ELF   : $TX_ELF"
echo "CAN_Rx BIN   : $RX_BIN_INPUT"
echo "Storage image: $PACKED_STORAGE_BIN"
echo "ST-LINK freq : ${STLINK_FREQ} kHz"
if [[ -n "$STLINK_SN" ]]; then
  echo "ST-LINK SN   : $STLINK_SN"
fi

read -r -p "Connect CAN_Tx board and press Enter to continue (Ctrl+C to abort)... "

echo "Flashing CAN_Tx app partition (0x08000000)..."
"$STM32_CLI" -c "${CONNECT_ARGS[@]}" mode=UR -w "$TX_ELF" 0x08000000 -v

echo "Flashing CAN_Tx storage partition (0x080E0000)..."
"$STM32_CLI" -c "${CONNECT_ARGS[@]}" mode=UR -w "$PACKED_STORAGE_BIN" 0x080E0000 -v

echo "Resetting target..."
"$STM32_CLI" -c "${CONNECT_ARGS[@]}" -rst

echo "Done. CAN_Tx now has app + preloaded CAN_Rx image in storage partition."
