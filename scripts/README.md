# Build and Flash scripts

This folder contains helper scripts to build and flash the Bootloader and Application (CAN_Rx) from the workspace.

Requirements:
- `STM32_Programmer_CLI` (part of STM32CubeProgrammer) available on `PATH`.
- `make` available (STM32CubeIDE makefiles are used).
- ST-LINK connected over SWD.

Usage:

1. Build and flash both images with ST-LINK:

```bash
cd scripts
./flash_all.sh
```

If you accidentally type `./flsh_all.sh`, a compatibility wrapper is also provided.

Optional ST-LINK selection:

```bash
# Set SWD frequency in kHz (default: 4000)
STLINK_FREQ=24000 ./flash_all.sh

# Select a specific ST-LINK by serial number
STLINK_SN=066CFF525152717867183721 ./flash_all.sh
```

The script will:
- run `make clean all` in `Bootloader` and `CAN_Rx` folders
- locate built `.elf` files under `build/`
- flash Bootloader at `0x08000000`
- flash App at `0x08010000`
- reset the target

OpenOCD / st-flash alternative:

If you prefer `openocd` or `st-flash`, replace the `STM32_Programmer_CLI` commands in `flash_all.sh` with appropriate commands. Example using `st-flash` (ST-Link v2):

```bash
# st-flash write <binfile> 0x08000000
```

Notes:
- Adjust addresses in `flash_all.sh` if your linker scripts use different app slot addresses.
- The script assumes `Debug` output; if you use `Release`, update the paths accordingly.
