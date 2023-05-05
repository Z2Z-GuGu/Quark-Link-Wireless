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
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\StdPeriphDriver\inc" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\RVMSIS" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\app_drv_fifo" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\HAL\include" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\LIB" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\Startup" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\Blinker" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\boardbase" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\ble_task" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\usb_task" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\uart_task" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\Application" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\Storage" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

