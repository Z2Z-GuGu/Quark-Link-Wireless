################################################################################
# MRS Version: {"version":"1.8.5","date":"2023/05/22"}
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../StdPeriphDriver/CH57x_adc.c \
../StdPeriphDriver/CH57x_clk.c \
../StdPeriphDriver/CH57x_gpio.c \
../StdPeriphDriver/CH57x_pwm.c \
../StdPeriphDriver/CH57x_pwr.c \
../StdPeriphDriver/CH57x_spi0.c \
../StdPeriphDriver/CH57x_sys.c \
../StdPeriphDriver/CH57x_timer0.c \
../StdPeriphDriver/CH57x_timer1.c \
../StdPeriphDriver/CH57x_timer2.c \
../StdPeriphDriver/CH57x_timer3.c \
../StdPeriphDriver/CH57x_uart0.c \
../StdPeriphDriver/CH57x_uart1.c \
../StdPeriphDriver/CH57x_uart2.c \
../StdPeriphDriver/CH57x_uart3.c \
../StdPeriphDriver/CH57x_usbdev.c \
../StdPeriphDriver/CH57x_usbhostBase.c \
../StdPeriphDriver/CH57x_usbhostClass.c 

OBJS += \
./StdPeriphDriver/CH57x_adc.o \
./StdPeriphDriver/CH57x_clk.o \
./StdPeriphDriver/CH57x_gpio.o \
./StdPeriphDriver/CH57x_pwm.o \
./StdPeriphDriver/CH57x_pwr.o \
./StdPeriphDriver/CH57x_spi0.o \
./StdPeriphDriver/CH57x_sys.o \
./StdPeriphDriver/CH57x_timer0.o \
./StdPeriphDriver/CH57x_timer1.o \
./StdPeriphDriver/CH57x_timer2.o \
./StdPeriphDriver/CH57x_timer3.o \
./StdPeriphDriver/CH57x_uart0.o \
./StdPeriphDriver/CH57x_uart1.o \
./StdPeriphDriver/CH57x_uart2.o \
./StdPeriphDriver/CH57x_uart3.o \
./StdPeriphDriver/CH57x_usbdev.o \
./StdPeriphDriver/CH57x_usbhostBase.o \
./StdPeriphDriver/CH57x_usbhostClass.o 

C_DEPS += \
./StdPeriphDriver/CH57x_adc.d \
./StdPeriphDriver/CH57x_clk.d \
./StdPeriphDriver/CH57x_gpio.d \
./StdPeriphDriver/CH57x_pwm.d \
./StdPeriphDriver/CH57x_pwr.d \
./StdPeriphDriver/CH57x_spi0.d \
./StdPeriphDriver/CH57x_sys.d \
./StdPeriphDriver/CH57x_timer0.d \
./StdPeriphDriver/CH57x_timer1.d \
./StdPeriphDriver/CH57x_timer2.d \
./StdPeriphDriver/CH57x_timer3.d \
./StdPeriphDriver/CH57x_uart0.d \
./StdPeriphDriver/CH57x_uart1.d \
./StdPeriphDriver/CH57x_uart2.d \
./StdPeriphDriver/CH57x_uart3.d \
./StdPeriphDriver/CH57x_usbdev.d \
./StdPeriphDriver/CH57x_usbhostBase.d \
./StdPeriphDriver/CH57x_usbhostClass.d 


# Each subdirectory must supply rules for building sources it contributes
StdPeriphDriver/%.o: ../StdPeriphDriver/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused  -g -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\StdPeriphDriver\inc" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\RVMSIS" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\HAL\include" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\LIB" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\Startup" -I"C:\Users\BuGu\Documents\GitHub\Quark-Link-Wireless\code\APP" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

