################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../QLW_lib/ble_task/ble_task.c 

OBJS += \
./QLW_lib/ble_task/ble_task.o 

C_DEPS += \
./QLW_lib/ble_task/ble_task.d 


# Each subdirectory must supply rules for building sources it contributes
QLW_lib/ble_task/%.o: ../QLW_lib/ble_task/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -I"C:\Project\CH573F\Quark_Link_Wireless\StdPeriphDriver\inc" -I"C:\Project\CH573F\Quark_Link_Wireless\RVMSIS" -I"C:\Project\CH573F\Quark_Link_Wireless\HAL\include" -I"C:\Project\CH573F\Quark_Link_Wireless\LIB" -I"C:\Project\CH573F\Quark_Link_Wireless\Startup" -I"C:\Project\CH573F\Quark_Link_Wireless\APP" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\Storage" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\uart_task" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\usb_task" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\ble_task" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\boardbase" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\Blinker" -I"C:\Project\CH573F\Quark_Link_Wireless\QLW_lib\app_drv_fifo" -I"C:\Project\CH573F\Quark_Link_Wireless\Main" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

