################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/peripheral.c \
../APP/peripheral_main.c 

OBJS += \
./APP/peripheral.o \
./APP/peripheral_main.o 

C_DEPS += \
./APP/peripheral.d \
./APP/peripheral_main.d 


# Each subdirectory must supply rules for building sources it contributes
APP/%.o: ../APP/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -DDEBUG=1 -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\StdPeriphDriver\inc" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\RVMSIS" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\app_drv_fifo" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\ble_uart_service" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\include" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\HAL\include" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\LIB" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\Profile\include" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\Startup" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\Blinker" -I"C:\Project\CH573F\11-CH573F-BLE-UART-USB\APP\boardbase" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

