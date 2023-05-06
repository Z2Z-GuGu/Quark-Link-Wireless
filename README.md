# Quark-Link-Wireless
开源的多连接开发板芯片管理单元 Open source multi connection development board chip management unit

## Detailed introduction
Quark Link Wireless项目，简称QLW，是一个自我大学二年级就开始的想法，受制于时间与能力，我直到2023年初才开始此项目的开发。QLW是一个开源的UART与多种通信方式转接的项目，用于Quark系列开发板主芯片与电脑/手机的连接，可以替代CP210x/CH340等USB转串口芯片。除基本的USB转UART串口外，QLW还提供USB转BLE、芯片BOOT、RESET控制、板载器件代理、主芯片外置上下拉电阻配置等功能，目前正在开发UART的非连接态数据暂存，所有功能可以通过UART（AT方式）、USB（配置模式）、BLE（0xFFE1服务）进行自由配置。
QLW项目以稳定性为核心，以极低的成本实现功能的拓展，同时缩减PCB版面空间，这对于Quark-mini系列开发板极为重要。我自己对稳定性要求较高，近几月一直在针对数据收发丢帧率进行优化，目前最佳成绩是1.3亿byte不丢一帧，后续将编写测试软件持续测试和优化QLW。

Quark Link The Wireless project, also known as QLW, was an idea that I started in my second year of university. Due to time and ability constraints, I did not start developing this project until early 2023. QLW is an open-source UART and multiple communication methods conversion project used to connect the main chip of the Quark series development board to computers/phones, which can replace USB to serial port chips such as CP210x/CH340. In addition to the basic USB to UART serial port, QLW also provides functions such as USB to BLE, chip BOOT, RESET control, onboard device proxy, and configuration of external pull-down resistors on the main chip. Currently, non connected data storage for UART is being developed, and all functions can be freely configured through UART (AT mode), USB (configuration mode), and BLE (0xFFE1 service).
The QLW project focuses on stability and achieves functional expansion at an extremely low cost, while reducing PCB layout space, which is extremely important for the development of the Quark mini series of boards. I have high requirements for stability myself and have been optimizing the frame loss rate for data transmission and reception in recent months. Currently, my best performance is 130 million bytes without losing a single frame. In the future, I will write testing software to continuously test and optimize QLW.

