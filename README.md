# OTA Update over CAN (STM32F407)

This project demonstrates a college-level OTA firmware update system over CAN using STM32F407.

It contains 3 firmware projects:
- `Bootloader`: decides which application slot to boot and safely jumps to app.
- `CAN_Tx`: OTA sender node (stores image in its own flash and sends to receiver on button press).
- `CAN_Rx`: OTA receiver node (writes image to non-active slot, validates CRC, updates metadata, reboots).

## Quick Start (Do / Don't)

Do this:
- Build + flash receiver base using `scripts/flash_all.sh`.
- Preload sender with Slot-B OTA payload using `scripts/flash_can_tx_with_image.sh`.
- Use `CAN_Rx_slotB.bin` as OTA image when target slot is B.

Don't do this:
- Do not pass `CAN_Rx/build/CAN_Rx.bin` as Slot-B OTA payload.
- Do not mix Slot-A linked image with Slot-B write target.

One-command safe preload:

```bash
cd scripts
bash flash_can_tx_with_image.sh
```

The script validates reset-handler address and aborts if image is not linked for Slot B.

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
- `DATA` (6 bytes):
  - bytes [0..1] = sequence number (uint16 LE)
  - bytes [2..5] = 4 payload bytes
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

# Build receiver image linked for Slot B (required OTA payload for Slot B updates)
make clean all TARGET=CAN_Rx_slotB LDSCRIPT=STM32F407VGTX_APP_B_FLASH.ld

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

### Option C: Flash CAN_Tx + preload OTA payload in its storage partition

```bash
cd scripts
bash flash_can_tx_with_image.sh
```

Default behavior:
- Uses `../CAN_Rx/build/CAN_Rx_slotB.bin` as OTA payload.
- If missing, builds it automatically using `STM32F407VGTX_APP_B_FLASH.ld`.
- Flashes `CAN_Tx` app at `0x08000000` and storage image at `0x080E0000`.

Safety check:
- Script verifies payload reset handler address is in Slot B range (`0x08040001..0x0807FFFF`).
- If you accidentally pass `../CAN_Rx/build/CAN_Rx.bin` (Slot A linked), script aborts with an error.

## 7) How to Load Firmware Image into CAN_Tx Flash

`CAN_Tx` supports UART upload command:
- 1 byte command: `'U'` (0x55)
- 4 bytes image size (uint32 little-endian)
- raw image bytes (`size` bytes)

When upload succeeds, `CAN_Tx` stores image in its flash and prints confirmation on UART.

Notes:
- Keep image size within configured max.
- Upload binary image (`.bin`) generated from target application.

## 7A) Download Firmware via SIM7670 (New)

Current CAN_Tx firmware now supports downloading the OTA payload over cellular and storing it in partition-2 (`0x080E0000`) without changing CAN_Rx or Bootloader.

Wiring used by firmware:
- Debug/UART console: `USART2` (`PA2/PA3`) at `115200`
- SIM7670 modem UART: `USART3` (`PB10 TX`, `PB11 RX`) at `115200`

Runtime UART commands on debug console:
- `R` : check SIM7670 AT responsiveness (`AT`, `ATE0`)
- `W` : set firmware URL at runtime (no rebuild)
- `T` : set manifest URL at runtime (no rebuild)
- `M` : fetch and parse manifest (size/crc, version optional)
- `G` : download firmware from URL and store into CAN_Tx storage partition
- `Q` : print stored image status
- `S` : send stored image over CAN (existing flow)

Default modem configuration in code (`CAN_Tx/Core/Src/main.c`):
- `SIM7670_APN` = `internet`
- `SIM7670_DEFAULT_URL` = `https://raw.githubusercontent.com/aditya1881/FOTA-STM32F407VG/main/CAN_Rx/build/CAN_Rx.bin`
- `SIM7670_DEFAULT_MANIFEST_URL` = `https://raw.githubusercontent.com/aditya1881/FOTA-STM32F407VG/main/CAN_Rx/build/manifest.txt`

Note: GitHub blob links are not direct binary downloads. The code uses the raw GitHub URL form so SIM7670 can fetch the `.bin` file directly.

You should change `SIM7670_DEFAULT_URL` to your real hosted binary URL (recommended: GitHub Release asset direct URL).

End-to-end with modem:
1. Power up CAN_Tx with SIM7670 connected on `USART3`.
2. On debug UART, send `R` and verify `SIM7670 ready`.
3. Optional: send `T` to set manifest URL and `W` to set fallback direct URL.
4. Send `M` to fetch manifest and verify printed version/size/crc/url.
5. Send `G` to download and store firmware into partition-2.
4. Send `Q` to confirm stored size/CRC.
5. Send `S` (or press button) to run existing OTA-over-CAN transfer.

Manifest format accepted by current parser:
- Key-value lines:
  - `size=14788`
  - `crc=0x1234ABCD`
- Optional lines if you want them:
  - `version=12`
  - `url=https://.../CAN_Rx_slotB.bin`

`G` behavior:
- First tries to fetch manifest and use its size/CRC.
- If manifest fetch fails, falls back to direct firmware URL.
- If manifest includes a URL, that URL overrides the direct firmware URL.
- If manifest is available, downloaded binary is checked against manifest size and CRC before storage header is committed.

## 8) OTA Demo Procedure (End-to-End)

1. Flash receiver board with `Bootloader + CAN_Rx`.
2. Flash sender board with `CAN_Tx`.
3. Connect CANH/CANL/GND between boards with proper termination.
4. Open UART terminal(s) at `115200` baud.
5. Upload new firmware binary to `CAN_Tx` using UART `U + size + data` protocol.
  - For a Slot B OTA target, use Slot B linked binary (`CAN_Rx_slotB.bin`).
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

5. Bootloader does not jump or rolls back to Slot A:
- Verify app vector table (initial SP + reset handler) is valid.
- Verify metadata points to expected slot.
- Ensure OTA payload is Slot-B linked (`CAN_Rx_slotB.bin`) when writing to Slot B.
- Do not use `CAN_Rx.bin` as Slot-B OTA payload.

6. Boot jumps but app resets quickly (then rollback):
- Ensure receiver image sets vector table correctly after jump.
- Current code sets `SCB->VTOR` to the image vector table in `CAN_Rx/Core/Src/system_stm32f4xx.c`.

## 11) Important Files

- Bootloader jump logic: `Bootloader/Core/Src/main.c`
- Receiver OTA logic: `CAN_Rx/Core/Src/main.c`
- Sender OTA logic: `CAN_Tx/Core/Src/main.c`
- Sender flash helpers: `CAN_Tx/Core/Src/flash_if.c`
- Metadata layout: `CAN_Rx/Core/Inc/ota_metadata.h`
- Build/flash script: `scripts/flash_all.sh`
- CAN_Tx preload script: `scripts/flash_can_tx_with_image.sh`
- Slot B linker script: `CAN_Rx/STM32F407VGTX_APP_B_FLASH.ld`

## 12) Scope and Level

This implementation is intentionally medium-level and suitable for college demonstration:
- clear protocol framing
- explicit CRC32 validation
- slot-based update strategy
- simple reliability retries
- practical bootloader jump handling

For production systems, add stronger security (signed images, anti-rollback, encryption, watchdog-safe state machine, power-fail recovery hardening).
