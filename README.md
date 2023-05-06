# Quark-Link-Wireless
开源的多连接开发板芯片管理单元 Open source multi connection development board chip management unit

## Detailed introduction
Quark Link Wireless项目，简称QLW，是一个自我大二就开始的想法，受制于时间与能力，我直到2023年初才开始此项目的开发。QLW是一个开源的UART与多种通信方式转接的项目，用于Quark系列开发板主芯片与电脑/手机的连接，可以替代CP210x/CH340等USB转串口芯片。除基本的USB转UART串口外，QLW还提供USB转BLE、芯片BOOT、RESET控制、板载器件代理、主芯片外置上下拉电阻配置等功能，目前正在开发UART的非连接态数据暂存，所有功能可以通过UART（AT方式）、USB（配置模式）、BLE（0xFFE1服务）进行自由配置。
QLW项目以稳定性为核心，以极低的成本实现功能的拓展，同时缩减PCB版面空间，这对于Quark-mini系列开发板极为重要。我自己对稳定性要求较高，近几月一直在针对数据收发丢帧率进行优化，目前最佳成绩是1.3亿byte不丢一帧，后续将编写测试软件持续测试和优化QLW。

Quark Link The Wireless project, also known as QLW, was an idea that I started in my second year of university. Due to time and ability constraints, I did not start developing this project until early 2023. QLW is an open-source UART and multiple communication methods conversion project used to connect the main chip of the Quark series development board to computers/phones, which can replace USB to serial port chips such as CP210x/CH340. In addition to the basic USB to UART serial port, QLW also provides functions such as USB to BLE, chip BOOT, RESET control, onboard device proxy, and configuration of external pull-down resistors on the main chip. Currently, non connected data storage for UART is being developed, and all functions can be freely configured through UART (AT mode), USB (configuration mode), and BLE (0xFFE1 service).
The QLW project focuses on stability and achieves functional expansion at an extremely low cost, while reducing PCB layout space, which is extremely important for the development of the Quark mini series of boards. I have high requirements for stability myself and have been optimizing the frame loss rate for data transmission and reception in recent months. Currently, my best performance is 130 million bytes without losing a single frame. In the future, I will write testing software to continuously test and optimize QLW.

 + 基础功能：

    + USB转串口（最多可配置为四串口，目前开放两个通道）
    + BLE转串口（可配置多个BLE Service UUID）

  + 附加功能：

    + ESP32 精确 BOOT 控制，包含下载信号识别、下载完成复位信号识别、串口监视器信号识别
    + STM32 UART下载 精确 BOOT 控制，包含下载信号识别、下载完成复位信号识别、串口监视器信号识别（基于SSCOM）
    + Arduino（AVR）精确 BOOT 控制，包含下载信号识别、下载完成复位信号识别
    + STC 精确 BOOT 控制，包含下载信号识别、下载完成复位信号识别、串口监视器信号识别（基于SSCOM，STC ISP下载信号特征不明显，未做兼容设计）
    + BOOT引脚释放，使BOOT GPIO输入输出电阻保持正常
    + 单片机端可配置串口波特率、BLE端可介入配置串口波特率
    + 引脚失能模式：对单片机所有引脚（包括UART RX TX）配置释放，方便单片机执行特殊功能，如单片机UART0与外部串口设备交互
    + 配置信息掉电储存，上电加载，长按 SYS KEY 将复原QLW系统配置。

  + S-Quark mini-QLW 开发板特有功能

    + 使用 SYS LED 代替 PWR LED

      默认状态下 SYS LED 执行以下功能

      1. 正常状态 SYS LED 常亮表示电源正常，即：PWR LED 常亮
      2. 下载状态 SYS LED 闪烁
      3. USB断开/连接  SYS LED 闪烁两次
      4. USB - UART数据传输时快速闪烁
      5. QLW配置为低功耗模式下 SYS LED 常灭，每秒短暂闪烁一次
      6. QLW系统异常时  SYS LED 常灭

      除此之外 SYS LED 状态可由GPIO 0/1/2/3/14/15、QL引脚控制，可自由配置其高电平还是低电平点亮，模拟硬件上拉或下拉式LED

      SYS LED 也可使用单片机AT指令、USB json指令、BLE json指令控制

    + 使用 USER LED 代替 GPIO LED

      默认状态下USER LED 同步GPIO2电平，即GPIO2高电平点亮LED

      USER LED 状态可由GPIO 0/1/2/3/14/15、QL引脚控制，可自由配置其高电平还是低电平点亮，模拟硬件上拉或下拉式LED

      USER LED 也可使用单片机AT指令、USB json指令、BLE json指令控制

    + 使用 SYS KEY 代替 RST KEY

      默认状态下 SYS KEY  执行以下功能

      1. 短按 SYS KEY（10s内）作用同 RST KEY ，控制EN引脚，复位ESP32
      2. 长按 SYS KEY（超过10s）复原QLW系统配置，并复位ESP32

      除此之外 SYS KEY 可配置为向GPIO 0/1/2/3/14/15、QL引脚输出开关信号，可自由配置按下后输出高低电平，模拟硬件上拉或下拉式按键

    + 使用 USER KEY 代替 GPIO KEY

      默认状态下 USER KEY  向GPIO0输出无抖动的开关信号，即按下后GPIO0检测到低电平，抬起为高电平

      除此之外 USER KEY 可配置为向GPIO 0/1/2/3/14/15、QL引脚输出开关信号，可自由配置按下后输出高低电平，模拟硬件上拉或下拉式按键

      USER KEY 可配置为模拟按键抖动

    + 接管GPIO2/14/15（单线SDIO）的上拉电阻，在不需要连接SD卡时配置为对GPIO浮空，保证相关引脚阻抗正常

    + QL功能引脚，替换S-Quark mini2的GND脚位，提供调试等特殊作用，默认QL引脚将浮空

  + 关于QLW系统配置：所有配置信息均可通过USB（配置用COM）、BLE（Service UUID：0xFFE0）、UART（AT方式）修改。

    + 注：Windows下配置用COM一般为数字较小的COM口，macOS下配置用COM名称为“usbmodemQuark_Linker_3”,若难以分辨，可向任意串口发送“Who are you”，回复“I am config serial port”的是配置串口
    + 后期将提供QLW桌面配置软件，将提供Windows、macOS和Linux三个版本
    + 蓝牙方式可借助“Blinker”软件实现图形化配置

