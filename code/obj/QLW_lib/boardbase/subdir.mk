################################################################################
# MRS Version: {"version":"1.8.5","date":"2023/05/22"}
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../QLW_lib/boardbase/boardbase.c 

OBJS += \
./QLW_lib/boardbase/boardbase.o 

C_DEPS += \
./QLW_lib/boardbase/boardbase.d 


# Each subdirectory must supply rules for building sources it contributes
QLW_lib/boardbase/%.o: ../QLW_lib/boardbase/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\StdPeriphDriver\inc" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\RVMSIS" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\HAL\include" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\LIB" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\Startup" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\APP" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

