################################################################################
# MRS Version: {"version":"1.8.5","date":"2023/05/22"}
# 自动生成的文件。不要编辑！
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include StdPeriphDriver/subdir.mk
-include Startup/subdir.mk
-include RVMSIS/subdir.mk
-include QLW_lib/usb_task/subdir.mk
-include QLW_lib/uart_task/subdir.mk
-include QLW_lib/boardbase/subdir.mk
-include QLW_lib/ble_task/subdir.mk
-include QLW_lib/app_drv_fifo/subdir.mk
-include QLW_lib/Storage/subdir.mk
-include QLW_lib/SYS_Config/subdir.mk
-include Main/subdir.mk
-include HAL/subdir.mk
-include APP/subdir.mk
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
Quark_Link_Wireless.hex \

SECONDARY_LIST += \
Quark_Link_Wireless.lst \

SECONDARY_SIZE += \
Quark_Link_Wireless.siz \


# 所有目标
all: Quark_Link_Wireless.elf secondary-outputs

# 工具调用
Quark_Link_Wireless.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -T "C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -L"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\StdPeriphDriver" -L"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\LIB" -Xlinker --print-memory-usage -Wl,-Map,"Quark_Link_Wireless.map" --specs=nano.specs --specs=nosys.specs -o "Quark_Link_Wireless.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
Quark_Link_Wireless.hex: Quark_Link_Wireless.elf
	@	riscv-none-embed-objcopy -O ihex "Quark_Link_Wireless.elf"  "Quark_Link_Wireless.hex"
	@	@
Quark_Link_Wireless.lst: Quark_Link_Wireless.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "Quark_Link_Wireless.elf" > "Quark_Link_Wireless.lst"
	@	@
Quark_Link_Wireless.siz: Quark_Link_Wireless.elf
	@	riscv-none-embed-size --format=berkeley "Quark_Link_Wireless.elf"
	@	@
# 其他目标
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Quark_Link_Wireless.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
