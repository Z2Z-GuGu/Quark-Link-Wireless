################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/ble_task/ble_task.c 

OBJS += \
./APP/ble_task/ble_task.o 

C_DEPS += \
./APP/ble_task/ble_task.d 


# Each subdirectory must supply rules for building sources it contributes
APP/ble_task/%.o: ../APP/ble_task/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -I"C:\Project\CH573F\Quark_Link_Wireless\StdPeriphDriver\inc" -I"C:\Project\CH573F\Quark_Link_Wireless\RVMSIS" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\app_drv_fifo" -I"C:\Project\CH573F\Quark_Link_Wireless\HAL\include" -I"C:\Project\CH573F\Quark_Link_Wireless\LIB" -I"C:\Project\CH573F\Quark_Link_Wireless\Startup" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\Blinker" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\boardbase" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\ble_task" -I"C:\Project\CH573F\Quark_Link_Wireless\APP" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\usb_task" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\uart_task" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\Application" -I"C:\Project\CH573F\Quark_Link_Wireless\APP\Storage" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

