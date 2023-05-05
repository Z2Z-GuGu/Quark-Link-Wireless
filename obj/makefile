################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include StdPeriphDriver/subdir.mk
-include Startup/subdir.mk
-include RVMSIS/subdir.mk
-include HAL/subdir.mk
-include APP/usb_task/subdir.mk
-include APP/uart_task/subdir.mk
-include APP/boardbase/subdir.mk
-include APP/ble_task/subdir.mk
-include APP/app_drv_fifo/subdir.mk
-include APP/Storage/subdir.mk
-include APP/Main/subdir.mk
-include APP/Blinker/subdir.mk
-include APP/Application/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
11-CH573F-BLE-UART-USB.hex \

SECONDARY_LIST += \
11-CH573F-BLE-UART-USB.lst \

SECONDARY_SIZE += \
11-CH573F-BLE-UART-USB.siz \


# All Target
all: 11-CH573F-BLE-UART-USB.elf secondary-outputs

# Tool invocations
11-CH573F-BLE-UART-USB.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -T "C:\Project\CH573F\11-CH573F-BLE-UART-USB\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -L"C:\Project\CH573F\11-CH573F-BLE-UART-USB\StdPeriphDriver" -L"C:\Project\CH573F\11-CH573F-BLE-UART-USB\LIB" -Xlinker --print-memory-usage -Wl,-Map,"11-CH573F-BLE-UART-USB.map" --specs=nano.specs --specs=nosys.specs -o "11-CH573F-BLE-UART-USB.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
11-CH573F-BLE-UART-USB.hex: 11-CH573F-BLE-UART-USB.elf
	@	riscv-none-embed-objcopy -O ihex "11-CH573F-BLE-UART-USB.elf"  "11-CH573F-BLE-UART-USB.hex"
	@	@
11-CH573F-BLE-UART-USB.lst: 11-CH573F-BLE-UART-USB.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "11-CH573F-BLE-UART-USB.elf" > "11-CH573F-BLE-UART-USB.lst"
	@	@
11-CH573F-BLE-UART-USB.siz: 11-CH573F-BLE-UART-USB.elf
	@	riscv-none-embed-size --format=berkeley "11-CH573F-BLE-UART-USB.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) 11-CH573F-BLE-UART-USB.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
