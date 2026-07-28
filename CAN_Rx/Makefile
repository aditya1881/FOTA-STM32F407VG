TARGET ?= CAN_Rx
BUILD_DIR := build
CAN_NODE_ID ?= 7

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size
STM32_PROGRAMMER_CLI ?= C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe
STM32_SWD_FREQ ?= 1000
STM32_STLINK_SN ?=
APP_A_ADDRESS ?= 0x08010000
APP_B_ADDRESS ?= 0x08040000
APP_VARIANT ?=
ifeq ($(APP_VARIANT),A)
  APP_SUFFIX := _app_a
  LDSCRIPT ?= STM32F407VGTX_FLASH.ld
  FLASH_ADDRESS ?= $(APP_A_ADDRESS)
else ifeq ($(APP_VARIANT),B)
  APP_SUFFIX := _app_b
  LDSCRIPT ?= STM32F407VGTX_APP_B_FLASH.ld
  FLASH_ADDRESS ?= $(APP_B_ADDRESS)
else
  APP_SUFFIX :=
  LDSCRIPT ?= STM32F407VGTX_FLASH.ld
  FLASH_ADDRESS ?= $(APP_A_ADDRESS)
endif
STM32_CONNECT := port=SWD freq=$(STM32_SWD_FREQ)
ifneq ($(strip $(STM32_STLINK_SN)),)
STM32_CONNECT += sn=$(STM32_STLINK_SN)
endif

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

DEFS := \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-DCAN_NODE_ID=$(CAN_NODE_ID)

INCLUDES := \
-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

C_SOURCES := $(wildcard Core/Src/*.c) $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)
ASM_SOURCES := Core/Startup/startup_stm32f407vgtx.s

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
DEPS := $(OBJECTS:.o=.d)

OPT ?= -Og
DEBUG_FLAGS ?= -g3 -DDEBUG

CFLAGS := $(MCU) $(DEFS) $(DEBUG_FLAGS) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -MMD -MP
ASFLAGS := $(MCU) $(DEBUG_FLAGS) -x assembler-with-cpp -MMD -MP
OUTPUT_NAME := $(TARGET)$(APP_SUFFIX)
LDFLAGS := $(MCU) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(OUTPUT_NAME).map,--cref -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs

ELF = $(BUILD_DIR)/$(OUTPUT_NAME).elf
HEX = $(BUILD_DIR)/$(OUTPUT_NAME).hex
BIN = $(BUILD_DIR)/$(OUTPUT_NAME).bin

.PHONY: all clean flash size A B build_app_a build_app_b

all: $(if $(filter A,$(MAKECMDGOALS)),build_app_a,$(if $(filter B,$(MAKECMDGOALS)),build_app_b,build_app_a))

A: build_app_a
B: build_app_b

build_app_a: APP_VARIANT := A
build_app_a: $(ELF) $(HEX) $(BIN)
	@echo "Built APP A firmware: $(ELF)"
	$(SZ) $(ELF)

build_app_b: APP_VARIANT := B
build_app_b: $(ELF) $(HEX) $(BIN)
	@echo "Built APP B firmware: $(ELF)"
	$(SZ) $(ELF)

$(ELF): $(OBJECTS) $(LDSCRIPT) | $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(HEX): $(ELF)
	$(CP) -O ihex $< $@

$(BIN): $(ELF)
	$(CP) -O binary -S $< $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR):
	@mkdir -p $@

size: $(ELF)
	$(SZ) $(ELF)

flash: $(ELF)
	powershell.exe -NoProfile -ExecutionPolicy Bypass -Command '$$programmer = "$(STM32_PROGRAMMER_CLI)"; $$connect = "$(STM32_CONNECT)" -split " "; & $$programmer -c $$connect -w "$(BIN)" $(FLASH_ADDRESS) -rst'

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
