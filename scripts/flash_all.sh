#!/usr/bin/env bash
set -euo pipefail

# Build and flash Bootloader and Application (CAN_Rx) using ST-LINK + STM32CubeProgrammer CLI
# Requires STM32_Programmer_CLI to be installed.

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOOT_DIR="$WORKSPACE_ROOT/Bootloader"
APP_DIR="$WORKSPACE_ROOT/CAN_Rx"
APP_SLOT_B_LD="STM32F407VGTX_FLASH_SLOT_B.ld"

STM32_CLI="${STM32_CLI:-C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe}"
if [ ! -x "$STM32_CLI" ]; then
  echo "Error: $STM32_CLI not found or not executable. Install STM32CubeProgrammer CLI or set STM32_CLI." >&2
  exit 2
fi

STLINK_FREQ="${STLINK_FREQ:-4000}"
STLINK_SN="${STLINK_SN:-}"

CONNECT_ARGS=("port=SWD" "freq=$STLINK_FREQ")
if [ -n "$STLINK_SN" ]; then
  CONNECT_ARGS+=("sn=$STLINK_SN")
fi

echo "Building Bootloader..."
pushd "$BOOT_DIR" >/dev/null
make clean all
popd >/dev/null

echo "Building Application (CAN_Rx)..."
pushd "$APP_DIR" >/dev/null
make clean all

if [ ! -f "$APP_SLOT_B_LD" ]; then
  echo "Error: missing $APP_DIR/$APP_SLOT_B_LD" >&2
  echo "Create Slot B linker script before running this automation." >&2
  exit 4
fi

echo "Building OTA image linked for Slot B..."
make all TARGET=CAN_Rx_slot_b LDSCRIPT="$APP_SLOT_B_LD"
popd >/dev/null

echo "Generating CAN_Rx manifest..."
python3 "$WORKSPACE_ROOT/scripts/generate_manifest.py" \
  --input "$APP_DIR/build/CAN_Rx_slot_b.bin" \
  --output "$APP_DIR/build/manifest.txt"

BOOT_ELF="$BOOT_DIR/build/Bootloader.elf"
APP_ELF="$APP_DIR/build/CAN_Rx.elf"
OTA_BIN="$APP_DIR/build/CAN_Rx_slot_b.bin"

if [ ! -f "$BOOT_ELF" ]; then
  BOOT_ELF="$(find "$BOOT_DIR/build" -maxdepth 1 -type f -name '*.elf' | head -n 1 || true)"
fi
if [ ! -f "$APP_ELF" ]; then
  APP_ELF="$(find "$APP_DIR/build" -maxdepth 1 -type f -name '*.elf' | head -n 1 || true)"
fi

if [ -z "$BOOT_ELF" ] || [ -z "$APP_ELF" ]; then
  echo "Could not find built ELF files in build/ folders." >&2
  echo "Expected in: $BOOT_DIR/build and $APP_DIR/build" >&2
  exit 3
fi

echo "Boot ELF: $BOOT_ELF"
echo "App ELF:  $APP_ELF"
echo "OTA BIN (Slot B): $OTA_BIN"
echo "ST-LINK frequency: ${STLINK_FREQ} kHz"
if [ -n "$STLINK_SN" ]; then
  echo "ST-LINK serial: $STLINK_SN"
fi

read -p "Connect target and press Enter to continue (or Ctrl+C to abort)..."

echo "Flashing Bootloader to 0x08000000"
"$STM32_CLI" -c "${CONNECT_ARGS[@]}" mode=UR -w "$BOOT_ELF" 0x08000000 -v

echo "Flashing App (CAN_Rx) to 0x08010000"
"$STM32_CLI" -c "${CONNECT_ARGS[@]}" mode=UR -w "$APP_ELF" 0x08010000 -v

echo "Resetting target"
"$STM32_CLI" -c "${CONNECT_ARGS[@]}" -rst

echo "Optionally, initialize metadata region if needed."
echo "Done."
