################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/MCU.c \
../HAL/RTC.c \
../HAL/SLEEP.c 

OBJS += \
./HAL/MCU.o \
./HAL/RTC.o \
./HAL/SLEEP.o 

C_DEPS += \
./HAL/MCU.d \
./HAL/RTC.d \
./HAL/SLEEP.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/%.o: ../HAL/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -I"C:\Project\CH573F\Quark_Link_Wireless\StdPeriphDriver\inc" -I"C:\Project\CH573F\Quark_Link_Wireless\RVMSIS" -I"C:\Project\CH573F\Quark_Link_Wireless\HAL\include" -I"C:\Project\CH573F\Quark_Link_Wireless\LIB" -I"C:\Project\CH573F\Quark_Link_Wireless\Startup" -I"C:\Project\CH573F\Quark_Link_Wireless\APP" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\Storage" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\uart_task" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\usb_task" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\ble_task" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\boardbase" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\app_drv_fifo" -I"C:\Project\CH573F\Quark_Link_Wireless\Main" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\SYS_Config" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

