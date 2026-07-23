# OTA Update over CAN (STM32F407)

This project demonstrates a college-level OTA firmware update system over CAN using STM32F407.

It contains 3 firmware projects:
- `Bootloader`: decides which application slot to boot and safely jumps to app.
- `CAN_Tx`: OTA sender node (stores image in its own flash and sends to receiver on button press).
- `CAN_Rx`: OTA receiver node (writes image to non-active slot, validates CRC, updates metadata, reboots).

## 1) High-level Architecture

- Bootloader region starts at `0x08000000`.
- Application slots:
  - Slot A start: `0x08010000`
  - Slot B start: `0x08040000`
- Metadata region: `0x08080000`

Flow:
1. `CAN_Tx` stores firmware image in its own flash area.
2. User presses button on `CAN_Tx`.
3. `CAN_Tx` sends OTA frames over CAN: `SYNC -> START -> DATA(seq) -> END`.
4. `CAN_Rx` receives frames, selects non-active slot, erases and writes data.
5. `CAN_Rx` validates full image by CRC and size.
6. `CAN_Rx` sets metadata to pending update and reboots.
7. Bootloader reads metadata and jumps to updated slot.

## 2) OTA CAN Protocol

CAN standard IDs:
- `0x320`: SYNC
- `0x321`: START
- `0x322`: DATA
- `0x323`: END
- `0x324`: ACK
- `0x325`: NACK

Frame format:
- `SYNC` (8 bytes): ASCII token `OTASYNC!`
- `START` (8 bytes):
  - bytes [0..3] = image size (uint32 LE)
  - bytes [4..7] = CRC32 (uint32 LE)
- `DATA` (3..8 bytes):
  - bytes [0..1] = sequence number (uint16 LE)
  - bytes [2..7] = up to 6 payload bytes
- `END` (8 bytes):
  - bytes [0..3] = footer magic `0x454E4431` (`END1`)
  - bytes [4..7] = image size (uint32 LE)

Reliability:
- Sender waits ACK/NACK per frame.
- Sender retries failed frame up to configured limit.
- Receiver checks sequence number and can NACK unexpected sequence.

## 3) Metadata and Boot Decision

Metadata structure is in:
- `CAN_Rx/Core/Inc/ota_metadata.h`
- `Bootloader/Core/Inc/ota_metadata.h`

Important metadata fields:
- `activeSlot`, `bootSlot`, `confirmedSlot`
- `appASizeBytes`, `appACrc32`, `appAState`
- `appBSizeBytes`, `appBCrc32`, `appBState`
- `updateInProgress`, `bootAttemptCount`, `maxBootAttempts`

Receiver behavior on successful update:
- writes target slot info into metadata (`pending` state)
- stores size + CRC
- sets update flags
- schedules reboot

Bootloader behavior:
- validates candidate application vector table
- handles boot attempts / rollback logic
- de-inits system state and jumps to selected app reset handler

## 4) Project Folder Map

- `Bootloader/` boot manager firmware
- `CAN_Tx/` sender firmware
- `CAN_Rx/` receiver firmware
- `scripts/` build and flash helper scripts

## 5) Build Instructions

Build each project:

```bash
cd Bootloader
make clean all

cd ../CAN_Rx
make clean all

cd ../CAN_Tx
make clean all
```

## 6) Flash Instructions

### Option A: Use provided script (Bootloader + CAN_Rx)

```bash
cd scripts
./flash_all.sh
```

This script currently builds and flashes:
- `Bootloader` at `0x08000000`
- `CAN_Rx` app at `0x08010000`

### Option B: Flash CAN_Tx separately

```bash
cd CAN_Tx
make flash
```

Or use your own ST-LINK / CubeProgrammer command for `CAN_Tx` ELF.

## 7) How to Load Firmware Image into CAN_Tx Flash

`CAN_Tx` supports UART upload command:
- 1 byte command: `'U'` (0x55)
- 4 bytes image size (uint32 little-endian)
- raw image bytes (`size` bytes)

When upload succeeds, `CAN_Tx` stores image in its flash and prints confirmation on UART.

Notes:
- Keep image size within configured max.
- Upload binary image (`.bin`) generated from target application.

## 8) OTA Demo Procedure (End-to-End)

1. Flash receiver board with `Bootloader + CAN_Rx`.
2. Flash sender board with `CAN_Tx`.
3. Connect CANH/CANL/GND between boards with proper termination.
4. Open UART terminal(s) at `115200` baud.
5. Upload new firmware binary to `CAN_Tx` using UART `U + size + data` protocol.
6. Press user button on `CAN_Tx`.
7. Observe logs:
   - `CAN_Tx`: sends SYNC/START/DATA/END and receives ACK.
   - `CAN_Rx`: receives, writes non-active slot, validates CRC.
8. `CAN_Rx` reboots automatically after successful metadata update.
9. Bootloader jumps to updated application slot.

## 9) Validation Checklist (for Viva / Demo)

- [ ] CAN bus communication stable (no persistent error LED).
- [ ] Receiver chooses non-active slot.
- [ ] Full image received byte count equals header size.
- [ ] Final CRC on receiver matches header CRC.
- [ ] Metadata updated to pending/valid state.
- [ ] Receiver reboots automatically after OTA success.
- [ ] Bootloader jumps to new application image.
- [ ] Previous slot remains available for rollback concept.

## 10) Common Troubleshooting

1. No ACK/NACK on sender:
- Check CAN bit timing config on both boards (must match).
- Check wiring and 120 ohm termination.
- Confirm standard IDs are identical in both projects.

2. Receiver NACK for protocol:
- Ensure sender sequence starts from 0.
- Ensure frame order is SYNC -> START -> DATA -> END.

3. CRC mismatch:
- Confirm the exact binary bytes are uploaded to sender.
- Confirm START CRC is computed from same binary image.

4. Flash write/erase errors:
- Verify flash sector mapping and image size boundaries.
- Ensure target slot does not overlap metadata region.

5. Bootloader does not jump:
- Verify app vector table (initial SP + reset handler) is valid.
- Verify metadata points to expected slot.

## 11) Important Files

- Bootloader jump logic: `Bootloader/Core/Src/main.c`
- Receiver OTA logic: `CAN_Rx/Core/Src/main.c`
- Sender OTA logic: `CAN_Tx/Core/Src/main.c`
- Sender flash helpers: `CAN_Tx/Core/Src/flash_if.c`
- Metadata layout: `CAN_Rx/Core/Inc/ota_metadata.h`
- Build/flash script: `scripts/flash_all.sh`

## 12) Scope and Level

This implementation is intentionally medium-level and suitable for college demonstration:
- clear protocol framing
- explicit CRC32 validation
- slot-based update strategy
- simple reliability retries
- practical bootloader jump handling

For production systems, add stronger security (signed images, anti-rollback, encryption, watchdog-safe state machine, power-fail recovery hardening).
