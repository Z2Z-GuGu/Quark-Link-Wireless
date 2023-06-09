# QLW开发笔记

+ 说实话不太想记开发笔记，前一段时间在练科二，抽空终于把UART、USB、BLE三块代码进行封装，使用TMOS伪操作系统将三者组织起来（USB任务处理太多了，暂时没写进去），构建这块代码本来问题蛮多的，但都解决下来了，不想记录笔记（不过BLE那部分还是有记录的价值的，后面有时间总结一下）。今天上午发现三者在一些情况不能很好的融合，出现交火失败的问题，晚上着重测试了一下。手头WCH-Link刚到，还不太会用，就先拿逻辑分析仪顶上了，没想到这玩意儿分析BUG还有各任务占用CPU时间还是蛮顺手的，为此在bb库中特地添加了GPIO-DEBUG功能，以下是部分分析结果：

+ USB事务处理部分太垃圾了，一直在占用CPU时间，后面必须给他搞到TMOS里边儿

+ 综合多种触发BUG的模式：USB RX时UART RX、UART RX时USB RX、BLE连接后 UART RX、UART RX时BLE连接，代码都会卡到UART INT--线路错误中，若不读取线路错误码线路错误问题将持续导致UART中断，即使读取错误代码（0x23： b0010 0011 -- -- 发送 FIFO 空、接收 FIFO 缓冲区溢出、接收 FIFO 中有接收到的数据（读取 FIFO 中所有数据后，该位自动清 0）），UART RX FIFO满7bit的问题又会出现，导致重复进入中断

+ 首先说解决思路：

  1. 出现问题后先丢弃数据：如果软件FIFO满就丢弃数据
  2. 看一下UART、BLE、USB中断优先级的问题，BUG应该是由于UART持续发送消息帧，而某一时刻因USB/BLE中断触发导致UART中断不能及时接受RX FIFO信息，导致RX FIFO溢出，UART持续中断。

+ 牛逼，做完第一套后程序已经不卡了，三路同时交火也没问题，就看第二条效果如何，我估计把UART优先级提上来后UART这边至少不会出现严重的丢帧，目前UART丢帧率（17588/339521 = 5.18%）还是挺高的，USB丢帧率（453125/453190 = 0.0143%）还将就，目标是0%的丢帧率。

+ 刚上了个撤所这点子就来了，可以把UART RX FIFO触发从7bit改成4bit，这样即使有问题也多出来4倍的时间填补fifo而不至于立马溢出，更近一步，当BLE没连接时，7bit触发，当BLE连接后改成4bit触发，这样就兼顾程序鲁棒性和中断的高效性了，先处理中断优先级的问题吧，BLE改不了中断优先级再说

+ 看了一下手册应该都能设置的，初步设置如下：

  | 中断  | 优先级 | 是否抢占 |
  | ----- | ------ | -------- |
  | 保留  | 1      | 0        |
  | 保留  | 2      | 0        |
  | UART0 | 3      | 1        |
  | BLEL  | 4      | 1        |
  | BLEB  | 5      | 1        |
  | USB   | 6      | 1        |

+ 麻了，没找到直接修改中断优先级的库

+ 找到了，void PFIC_SetPriority(IRQn_Type IRQn, UINT8 priority); 在core_riscv.h里

+ 优先级改好了，测试环境为单独通道发送

+ 修改为4bit触发后UART丢帧率已接近0%（2165973/2165973），USB丢帧率略高（900/2165973 = 0.04155%），试一下把USB优先级调整为3，与UART相同

+ 麻了，并没有提高多少丢帧率（576/2165973 = 0.0266%），明天再吧USB 事务处理那玩意儿写进TMOS里吧，还得避开中断不能调用TMOS启动函数的问题，外面挂一个函数吧

+ 同时接受数据时：UART丢包率（2061/339520 = 0.6%（无参考意义））USB丢包率：（256/1018560 = 0.0251%）。麻，同时发送的话USB数据几乎不受影响，UART丢包率急升。

+ 把USB中断处理写到了TMOS里，目前单发USB，丢帧率稳定在（512/3055680 = 0.017%上）差不多，看一下UART单发，还是稳的一批，无限接近0%（0/1697600）；看一下USB，UART同时收发，还是不太理想，USB丢帧率（576/1358080 =  0.0424%），UART丢帧率（4019 / 339520 = 1.18%（有参考意义））。

+ 存在问题：电脑串口助手已经打开串口，复位芯片，重新点击打开串口，将出现设备读取失败的问题，估计和TMOS消息启动event机制中出现消息并发现象，TMOS不能及时触发event导致。

+ 备份为“11-CH573F-BLE-UART-USB-6”

+ 删除旧有代码USB_IRQProcessHandler();

+ 想到一个优化方案，即中断中使用DMA传出数据，在task_event中载入到软件fifo中，之后新版本修改吧

+ 新建一个Application库，用于处理消息流，和日常事务

+ 将全部FIFO初始化在初始化APP时完成

+ 不知道为什么UART、USB并收发时UART丢包率急剧下降！，估计是新加入的APP task修改了TMOS各task的优先级，目前UART丢帧率（399/339520 = 0.12%），USB丢帧率（320/1018560 = 0.0314%）

+ 下午链接了UART->USB,但连接USB->UART时出现了问题，丢包严重，经大量测试，发现丢的都是大包，55-64byte，花了大量时间定为bug，发现，写入UARTFIFO时len = 64经常被修改，应该是FIFO空间不足的问题，那就调大阈值，128吧

+ 解决不确定丢包问题后依然存在确定性（丢64bit）的问题，又定位了一晚上终于发现是“DATA1/0”不同步导致的，我也不知道芯片怎么就判断的不同步了，逻辑分析仪上看到的就是DATA0/1交替，芯片就是判断错误，麻了，要不直接不判断DATA0/1同步问题了，别，要从根源上解决问题。

+ 目前发现这种情况大部分出现在FIFO满之后，突然空出来

+ 明天直接搜R8_UEP1_CTRL

+ 又找了一上午问题，查寄存器值，出错的时候确实存在R8_UEP1_CTRL的RB_UEP_R_TOG位被篡改的情况，但全局搜索并一一排查程序内的R8_UEP1_CTRL寄存器操作情况，没有任何一处存在逻辑漏洞，考虑一下两种情况：

  1. R8_UEP1_CTRL的RB_UEP_R_TOG位自动修改，即RB_UEP_AUTO_TOG位存在篡改情况（经排查，无此问题）；
  2. R8_UEP1_CTRL的RB_UEP_R_TOG位在操作其他寄存器时被连带修改了，考虑它的上一个寄存器：R8_UEP1_T_LEN,虽然感觉可能性不大（强制转换过保证R8_UEP1_T_LEN没问题，此选项排除）

+ 算了，不考虑EP1 OUT端点的PID令牌DATA0/1问题了，直接不判断，走流程吧。

+ 4/4 经历几天的测试与颓废，终于在昨天缓过来，为彻底解决USB- UART通信丢帧的问题，决定使用USB DMA的功能对FIFO软件缓冲区进行彻底优化，重构一个全新的交替式DMA双缓体系。注：此缓冲区仅针对USB- UART的通信。

+ 4/6 终于在晚上11点之前完成了CDC向FTDI的迁移（初期工作：在电脑上可以识别到并正确驱动）

+ 4/8 移植USB传输部分：

  + ~~待解决变量：VendorControl~~
  + UIS_TOKEN_IN-USB_GET_DESCRIPTOR未更改
  + ~~待解决变量：Latency_Timer~~1/0
  + MODEM这块属实没看懂，为什么DTR2/RTS2要受制于DTR1/RTS1
  + 标准请求：USB_SET_FEATURE存在差异，暂不予理会
  + 549 SetupReqCode = 0xFF; ////

+ 4/13 抓包发现MODEM Control使用的是wValueH不是wValueL

+ 4/14 抓了很多包，终于发现为什么我写的CH573模拟FTDI设备打开串口那么久了，是因为FTDI超时上报问题，比如超过规定时间（16ms）无新数据，EP3上报一次01 06

+ 现在在考虑是否用回CDC协议，查询并实验两方面：

  1. CDC复合设备（虚拟多串口）是否可以在不修改电脑CDC驱动前提下实现，已测试，可以（macOS11、Windows10/11测试通过）
  2. CDC是否存在扩展协议，即使存在也很难在电脑API端调用这些，最终决定采用特定的DEF_SET_LINE_CODING设置来启动串口设置功能
  3. 网上存在“CDC-ACM”不能控制RTS的说法，经逻辑分析仪测试，Windows10可以控制RTS信号；macOS缺少相关软件，后期使用python搭配pyserial库写一个测试程序。

+ 今天构建的usb_task.c Rev3.0为双串口实例，EP0为串口1/2的控制端口，EP1/EP2为串口1/2的数据端口，控制信号中的EP0_BUF wIndexL位0x01/0x02可以筛出对应的串口

+ 测试USB传输极限速度为5.333Mbps

+ 经测试，复制64byte数据（32bit方式）需6us

+ 问题：发送数据至计算机时，单包只能发63byte，发64byte：

  + Windows串口助手直到发送不满64时才显示之前收到的满帧数据

  + macOS串口助手直接丢弃满帧64byte数据

  + 为保证良好兼容性，之后USB IN（TX）统一设置单包大小为63

+ 4/15 做了USB to UART的单独优化，目前USB收UART发，单独测试可达到千万byte不丢帧，消除多数组传递复制机制，代码效率高，可靠性好，但UART发送端代码可以继续优化，优化路径依赖于APP.c新建一个FIFO待启动应用序列（保证中断内可以调用TMOS）另外所有进入中断的变量做一下保护

+ 4/17 上午做了多种测试（send data往中断里写，往TMOS里写）效果都极差（主要是卡帧），考虑到代码效率和中断冲突等一系列问题，决定恢复原来的方案，auto send就行了，倒是把FIFO待启动应用序列测试完成了，效果拔群，之后把USB EP0部分写到TMOS里处理吧，下午攻克一下UART收，USB发。

+ 其实无中间缓冲的设计对于UART RX，USB IN还是挺复杂的，写了一下午的代码，问题其实有很多，首先是尾帧的输出问题，包含加上尾帧未超帧包大小（最简单的情况）、加上尾帧正好等于帧包大小（也简单）、不加尾帧正好等于帧包大小（复杂）、加上尾帧正好超出帧包大小（复杂）；另外无中间缓冲的设计确实存在往Flash内部存储和取出机制复杂的问题，并且当短暂的UART速度快于USB速度时，容易丢帧，所以考虑先把Flash存取问题解决后再搞UART收，USB发（IN端口），其实主要是测一下Flash的速度然后看一下未使用的Code Flash部分能不能作为Data Flash用。

+ 4/21 这两天碰不到电脑，做了些非代码的工作，主要是针对UART RX数据缓存的代码逻辑和数据结构问题进行设计，目前第二版结构如下：

  + （二版图）
  + 其中有一个问题挺困扰我的，就是data flash里边的index数据或者说config数据存到哪的问题，你说直接存到data flash吧，时间太长，如果算上擦除的时间的话更长，如果平时不保存直接留在RAM里吧是TMD一掉电就没了，憨的一批，目前想法是两种，一种额外增加掉电检测电路，掉电一瞬间保存数据（或者是先断掉外设电源后再保存数据），另外一种方案就是图里边儿的随时（一定条件下）保存一次数据。
  + 刚才想了一种结构，AB区保存法：两个256Byte的存储空间，交替存储，CPU空闲时间去擦除一个，保证要存储的时候随时能存

+ 先写吧，反正要用到DMA的两个64Byte EP1data，在目前的基础上，把USB发送的任务先改到USB task库里，现在像什么样子。

+ 麻的一批，不行啊，数据是乱的，而且老是给USB HUB干无语了，要不数据还是在UART这边法吧，障碍少点。

+ 改了半天，目前遇到的问题

  + 发63byte没问题

  + 发少于63byte像是不能触发USB发送一样，不死机，不发送

  + 发64byte，第一次可以触发并发送63byte内容，之后不发送，不触发

+ 4/26 终于把科三科四考完了，从科一到科四一遍过，也不枉费我那么下心研究那么多技巧，终于可以好好做这块东西的开发了。

+ 刚才没记录，补一下调试记录：通过逐级缩小范围的DEBUG_PIN方式找到问题，关键在于USB发送速度极快，且USB中断可以打破UART中断，导致尾帧发送之前USB已经发送完之前的数据，尾帧数据不能正常触发USB IN端口发送，添加发送代码后可以正常执行；同时还顺带修改了几个小BUG，目前是可以保证单独收/发的长期稳定运行。

+ 出现的新问题：收发同时进行时突然无法发送->推断程序在某处卡死->推断数据未及时接受导致UART线路错误，DEBUGE PIN证实此猜想->推断中断优先级的问题导致数据未及时接受->修改优先级发现无改善，说明关键问题不在此处，先改回去试试DEBUG PIN定位吧。

+ 4/27 今天的目标，完美解决上面的问题，然后测丢帧率，测全双工收发测大文件的收发。

+ 经测试，与抢占优先级无关

+ 测了一上午，发现当下代码对于USB的异常事件容忍度较低，只要出现USB异常，UART数据必“开始出错”，这个问题等到写UART数据缓存的时候再处理吧，上午看一下能不能把停止为、奇偶校验、波特率处理一下吧。

+ 没写新功能，又继续测了几下，发现发送7byte数据时超时触发功能不能正常启动，导致尾帧数据可能发不出来，与之前测的相违背，看手册吧。手册p58 9.3.3写明白了，这种情况是正常的

+ 看了之前写的代码，是通过TIMER3来辅助判断尾帧发送的，下午过来CTRL CV吧

+ 用TIMER3解决了该问题，旧问题：在同收发下可能出现卡死在UART_II_RECV_RDY

+ 确定由283行：if(!(EP1_Buf_State & T_Data0_State_Bit))不能进入导致卡死

+ 调到晚上终于差不多了，两个CH573F（其实是一个573一个571）对发333K Byte，不丢帧了

+ 4/28 重新把双串口调出来，然后重点搞DTR、RTS信号、波特率、停止位、校验位，做了双COM隔离，用bitmap做了flag，基本没难度，一路分析USB信号，对应写代码就行

+ 差15min回家吃饭，看看能不能把COM口名称搞出来

+ 搞了半小时，COM口名称无法做到分离，但Mac这边会在末尾添加数字（如1/3）用以区分

+ 试了一下CH573和571，实验证明代码有良好的兼容性

+ 花了一下午时间把手头所有的串口监视器和芯片烧录软件的特征波形记录了一遍，放到文件夹“串口助手/芯片烧录特征波形”内，全部保存为 .sr 文件，其中让我感到意外的是STC-ISP中芯片烧录没有对DTR/RTS产生任何影响，试了两个版本都没反应，之前设计的STC自动下载电路完全就是摆设，没用了，还是得检测串口的0x7F数据才靠谱

+ 刚才驾照邮寄来了，居然花了我20块运费，能买6-7块CH573F了，麻的很。

+ 接着刚才的话题，有了这些数据我就可以做一个自动检测芯片进行烧录的烧录器了，不需要提前设置。

+ 如何识别BOOT信号：

  + 首先检测跳变沿事件（包含RTS/DTR上下边沿）时间线，存储：

  ```C
  // 事件对应：
  /* 1 = DTR Rising
   * 2 = DTR Falling
   * 3 = RTS Rising
   * 4 = RTS Falling */
  uint8_t ESP32_Signal[] = {1, 3, 2, 4};
  uint16_t ESP32_Signal_time[] = {100, 3000, 200, 40000};
  uint8_t ESP32_Serial_Signal[] = {0x8A, 0xAB, 0x0C, 0x00};
  
  
  ```

+ 今天应该是写不完了，说一下目前的情况吧，放弃上面那种说法，换做跳变沿特征检测+UART数据检测

  + 目前已经写了一半逻辑，数据保存一下，晚上回去补吧

+ 这两天非常激动哇核心功能搞定，都忘了写开发笔记了。现在是 5/2，下面补一下笔记

+ 4/28 晚：写了一下BOOT逻辑，是这样的：从DTR/RTS跳边沿信号开始BOOT STATE信号第一次改变为ESP32_UART_Check，有此标识后，在接下来的每一帧USB OUT信号中插入ESP32烧录串口链接信号的检测，思路就是测一下发来多少byte，然后第一个byte匹配就说明处于BOOT内，在while（1）BOOT检测信号中复位，然后依次激活BOOT前准备，BOOT，BOOT引脚复位

+ 4/29 测试昨晚的信号，发现不能正常执行，通过DEBUG PIN方式找到错误为COM_CTRL_LINE_STATE中表示边沿新数据的bit位没有复位，修复后正常触发BOOT。然后出现了奇怪的烧录过程无法进行的BUG，继续套上逻辑分析仪看信号，由于S-Quark mini - QLW Bate1版本PCB未引出USB DPDN废了很大周折接出来线，等到测试的时候又TM莫名其妙好了，百试百灵，问题都揪不出来，注：ESP32烧录数据时实际使用460800作为其波特率，连接的时候用115200

+ 4/30 出去玩了

+ 5/1 玩了半天回来继续把KEY和LED逻辑写好，测试通过

+ 5/2 重现4/29的问题，在windows上下了ESP的flash tool下载出现问题，修复后正常触发芯片的BOOT，但电脑不识别，换一下吧，windows上记录波形，Mac上烧录吧

+ 干到下午了，查出来原因是：

  + 有两个/一个byte错了（读取成了后面第56个byte）/位置：倒数第二帧，第3/4位置
  + 丢了一帧64byte数据

+ 丢64byte数据原因查清：电脑并非DATA0-DATA1的发，可能出现DATA1-DATA1-DATA0的发，要打补丁

+ 有两个/一个/三个byte错了原因：大概率还是因为上面的DATA0/1的问题，看波形数据像是每七帧USB OUT数据就会有一个 DATA搞错的

+ 解决思路，首先看一下丢64byte是不是被当作不同步的信号丢掉了（再写一个else PIN ctrl），如果不同步就先把自动改为手动，如果同步就考虑修改uart auto send函数使其适应非正常的循环DATA0/1

+ 保存工程为11-CH573F-BLE-UART-USB-13

+ 凌晨一点继续看波形，发现其实电脑是发了DATA0/1（按交替次序来的）但每隔7帧必存在一个空帧，正常的程序应该流到这个空帧时迅速切换，但存在检测不到的可能，所以一跳就是跳俩（一个空帧和之后一帧）

+ 丢64byte根本原因：DATA0送出数据后触发接受新DATA0（空帧），紧接着发DATA1；DATA1的64Byte最后几帧刚送入UART FIFO，然后就立马触发了DATA1接受新数据，由于DATA0空，中断紧跟触发接受DATA0，DATA1的64Byte最后几帧送入后，自动切换为DATA0，结果刚开始读入（本应是空帧迅速切换到DATA1）DATA0 LEN和BUF被篡改，则此时直接读出新来的DATA0，前一帧DATA1被忽略了

  + 解决思路，中断中不允许触发下一帧接受-->首帧可以，在结束（EN = 0）之前不允许连续读帧-->但是表现正常的时候确实存在连续读帧的情况，怎么办呀；正常与否，关键在于有没有进入空帧的判断与迅速跳出，当出现空帧时切忌连续读帧，一定要有跳出动作（SEQ-2-3-QL.sr 中C点DATA0空帧，要么阻塞读取下一帧Data0（中断内），要么在uart auto send内在data0时强制跳出到data1）

+ 快两点了，我感觉不行了，得睡觉，晚安

+ 5/3 打了一个Frame_loss_mark的补丁，果不其然出现新问题了，BOOT无法执行，DEBUGPIN吧

+ 一看代码，Frame_loss_mark忘复位了，尬

+ 跑了一下，出现了一个错帧问题，很奇怪呀，每次位置都一样，不带变的，seq1倒数第二帧的第0、1被改为00x00，第2、3byte被篡改为之后56的数据，后面又是正常的，先多跑几组数据吧，刚才跑的结果命名为seq1-new-w

+ 经历了几次md5错误后终于跑出来一次success了，不知道丢帧问题解决没

+ 跑了好几次没发现丢64byte的问题了，应该解决了，然后看错帧问题，先用EP2端口输出数据，看一下接受的原始数据对不对

+ 又有了，不知道到底检测到len = 0没

+ 目前seq1/3是错帧，2是丢帧

+ 实验证明，丢帧的时机确实会给一个0x00的

+ 试一下直接在中断内阻塞下一帧，试之前留了个APP在桌面

+ 出现了新的问题，改回去吧

+ 好事多磨，经测试，发现接收的信号是没问题的，接下来看uart端接收数据到底长这么样子

+ 又是好事多磨系列，经测试，数据UART发送前就已经被篡改，接下来测一下index到底长什么样子

+ 这次还算挺快的，现在确认是index的问题：

  + ```
    DATA1 [ 40 41 42 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F ]
    ```

  + 

  + 正常的应该是

    + ```
      DATA0 [ 00 01 02 03 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F 60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F 70 71 72 73 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F ]
      ```

    + ```
      DATA1 [ 40 41 42 43 44 45 46 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F ]
      ```

  + 这也不正常呀，怪了，先回去吃饭了

+ 晚上逐个排查问题，逐个打补丁，结果问题还没解决，错帧确定由于index内存错误引起，丢帧暂未查清

+ 使用CH573F开发板测试index内存错误问题：使用串口助手测试，没有发现问题，初步怀疑两个问题：QLW板载CH573硬件RAM错误（可能性小）、index操作被中断打断产生错误；明天做一下几个测试：index操作中切掉所有中断、CH573开发板连接QWL开发板调试、所有中断设置中断DEBUGPIN标识

+ 5/4 实验一证明：与中断无关，都已经禁止中断了还是不行

+ 最新实验使用UART1代替EP2输出，发现index没毛病，可能是取值那边出了问题，先把实验用的count删了吧

+ 打算直接在中断时将数据读出，保存一下当前的版本吧存为xxx-14

+ 确认了一下，文件“seq1-w6.sr”证明出错的点与index无关，确定是取值或buf内存错误

+ 文件“seq1-w7.sr”证明出错的点与取值无关，确定是buf内存错误

+ 记录一下，2023/5/4 15:52 我用USB硬件单缓存重构了底层代码，相当简洁，时间上看效率确实稍微低了一点，但不出错呀，一遍编译就通过，直接收发调试没问题，然后QLW上传调试，四遍了，没一点毛病，哈哈哈哈哈哈哈哈，果然大道至简吗？之前费尽心机做的硬件双缓为了提高那么一点点效率，换来的就是百般出错，连续调了几天都调不好，今天重构底层，花了不到40分钟就一点问题没有了。

+ 晚上重构了BOOT控制那块的代码，然后完善了APP那边很多细节,比如500ms内未检测到UART烧录信号则退出BOOT状态，写了一个小demo：上传时SYS小灯闪烁，也是比较完美

+ 下面总结一下QLW目前可以实现的功能：

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

+ 5/5 用一个uint16_t BitMap表示系统状态吧，表示以下几种状态

  1. 系统运行异常
  2. USB连接状态更新
  3. USB连接状态
  4. UART TX Action
  5. UART RX Action
  6. 程序烧录中
  7. QLW系统低功耗
  8. QLW系统配置
  9. BLE连接状态更新
  10. BLE连接状态
  11. SYS KEY STATE
  12. BLE_to_UART_EN
  13. UART_to_BLE_EN

+ GPIO状态配置

  1. 对GPIO0 输入/输出 （0：浮空输入；1：输出）
  2. 对GPIO0 上下拉50K电阻（0：下拉50K；1：上拉50K）
  3. 对GPIO1 输入/输出 （0：浮空输入；1：输出）
  4. 对GPIO1 上下拉50K电阻（0：下拉50K；1：上拉50K）
  5. 对GPIO2 输入/输出 （0：浮空输入；1：输出）
  6. 对GPIO2 上下拉50K电阻（0：下拉50K；1：上拉50K）
  7. 对GPIO3 输入/输出 （0：浮空输入；1：输出）
  8. 对GPIO3 上下拉50K电阻（0：下拉50K；1：上拉50K）
  9. 对GPIO14 输入/输出 （0：浮空输入；1：输出）
  10. 对GPIO14 上下拉50K电阻（0：下拉50K；1：上拉50K）
  11. 对GPIO15 输入/输出 （0：浮空输入；1：输出）
  12. 对GPIO15 上下拉50K电阻（0：下拉50K；1：上拉50K）
  13. QL 输入/输出 （0：浮空输入；1：输出）
  14. QL 上下拉50K电阻（0：下拉50K；1：上拉50K）
  15. 新数据

  注：输入输出信号优先于上下拉信号

+ 使用一个uint16_t BitMap表示SYS LED状态

  1. SYS LED模式（0：系统状态指示，1：由用户控制）
  2. 由串口/USB/BLE控制
  3. GPIO点亮方式（0：即高电平点亮，低电平熄灭；1：相反）
  4. 由QL控制
  5. 由GPIO0控制
  6. 由GPIO1控制
  7. 由GPIO2控制
  8. 由GPIO3控制
  9. 由GPIO14控制
  10. 由GPIO15控制

  注：配置为多GPIO控制输入时，将GPIO的输入视为与门输入

+ 使用一个uint16_t BitMap表示USER LED状态

  1. USER LED模式（0：默认用GPIO2正向控制，1：由用户控制）
  2. 由串口/USB/BLE控制
  3. GPIO点亮方式（0：即高电平点亮，低电平熄灭；1：相反）
  4. 由QL控制
  5. 由GPIO0控制
  6. 由GPIO1控制
  7. 由GPIO2控制
  8. 由GPIO3控制
  9. 由GPIO14控制
  10. 由GPIO15控制

  注：配置为多GPIO控制输入时，将GPIO的输入视为与门输入

+ 使用一个uint16_t BitMap表示SYS KEY状态

  1. SYS KEY 模式（0：默认控制单片机复位，1：由用户选择数据输出对象）
  2. 对串口/USB/BLE输出
  3. 按键锁定（0：不锁定，模拟轻触按键；1：锁定，模拟自锁按键）
  4. 按键抖动模拟（0: 不抖动； 1: 模拟抖动）
  5. 按键极性（0：按下为低抬起为高；1：按下为高抬起为低）
  6. 向GPIO0输出
  7. 向GPIO1输出
  8. 向GPIO2输出
  9. 向GPIO3输出
  10. 向GPIO14输出
  11. 向GPIO15输出
  12. 向QL输出

+ 使用一个uint16_t BitMap表示USER KEY状态

  1. USER KEY 模式（0：默认按下对GPIO0输出低电平，1：由用户选择数据输出对象）
  2. 对串口/USB/BLE输出
  3. 按键锁定（0：不锁定，模拟轻触按键；1：锁定，模拟自锁按键）
  4. 按键极性（0：按下为低抬起为高；1：按下为高抬起为低）
  5. 向GPIO0输出
  6. 向GPIO1输出
  7. 向GPIO2输出
  8. 向GPIO3输出
  9. 向GPIO14输出
  10. 向GPIO15输出
  11. 向QL输出

  注：请尽量避免 SYS KEY 与 USER KEY 的输出冲突，如两个按键同时配置给GPIO0，冲突产生时，冲突引脚优先同步 USER KEY状态

+ IO事件处理架构：经典的输入-处理-输出工作流

+ json指令设计
  + 功能分类
    0. 使能类指令
       + BLE -> UART 开/关  (EN-B)
       + UART -> BLE 开/关  (EN-U)
       + 是否启用SDIO模式，上拉电阻接管  (EN-S)
       + 是否启用GPIO FREE模式，浮空全部QLW与ESP32连接的GPIO (EN-F)
    1. SYS LED控制指令
       + SYS LED 是否 = 默认模式  (SL-D)
       + SYS LED 是否 = 控制反转  (SL-I)
       + SYS LED 是否由（UART/USB/BLE控制）  (SL-C)
       + SYS LED = 1（UART/USB/BLE控制）  (SL-O)
       + SYS LED 是否由GPIO0控制  (SL-0)
       + SYS LED 是否由GPIO1控制  (SL-1)
       + SYS LED 是否由GPIO2控制  (SL-2)
       + SYS LED 是否由GPIO3控制  (SL-3)
       + SYS LED 是否由GPIO14控制  (SL-4)
       + SYS LED 是否由GPIO15控制  (SL-5)
       + SYS LED 是否由QL Pin控制  (SL-Q)
    2. USER LED控制指令
       + USER LED 是否 = 默认模式  (UL-D)
       + USER LED 是否 = 控制反转  (UL-I)
       + USER LED 是否由（UART/USB/BLE控制）  (UL-C)
       + USER LED = 1（UART/USB/BLE控制）  (UL-O)
       + USER LED 是否由GPIO0控制  (UL-0)
       + USER LED 是否由GPIO1控制  (UL-1)
       + USER LED 是否由GPIO2控制  (UL-2)
       + USER LED 是否由GPIO3控制  (UL-3)
       + USER LED 是否由GPIO14控制  (UL-4)
       + USER LED 是否由GPIO15控制  (UL-5)
       + USER LED 是否由QL Pin控制  (UL-Q)
    3. SYS KEY输出模式指令
       + SYS KEY 是否 = 默认模式  (SK-D)
       + SYS KEY 是否 = 按键自锁  (SK-L)
       + SYS KEY 是否 = 极性反转  (SK-I)
       + SYS KEY 是否 = 抖动模拟  (SK-S)
       + SYS KEY 是否对UART输出  (SK-U)
       + SYS KEY 是否对GPIO0输出  (SK-0)
       + SYS KEY 是否对GPIO1输出  (SK-1)
       + SYS KEY 是否对GPIO2输出  (SK-2)
       + SYS KEY 是否对GPIO3输出  (SK-3)
       + SYS KEY 是否对GPIO14输出  (SK-4)
       + SYS KEY 是否对GPIO15输出  (SK-5)
       + SYS KEY 是否对QL Pin输出  (SK-Q)
    4. USER KEY输出模式指令
       + USER KEY 是否 = 默认模式  (UK-D)
       + USER KEY 是否 = 按键自锁  (UK-L)
       + USER KEY 是否 = 极性反转  (UK-I)
       + USER KEY 是否 = 抖动模拟  (UK-S)
       + USER KEY 是否对UART输出  (UK-U)
       + USER KEY 是否对GPIO0输出  (UK-0)
       + USER KEY 是否对GPIO1输出  (UK-1)
       + USER KEY 是否对GPIO2输出  (UK-2)
       + USER KEY 是否对GPIO3输出  (UK-3)
       + USER KEY 是否对GPIO14输出  (UK-4)
       + USER KEY 是否对GPIO15输出  (UK-5)
       + USER KEY 是否对QL Pin输出  (UK-Q)
    5. BOOT控制
       + close BOOT Mode  (BO-C)
       + BOOT Auto Mode  (BO-A)
       + BOOT For ESP32  (BO-E)
       + BOOT For STM32  (BO-S)
       + BOOT For Arduino  (BO-O)
       + BOOT For STC  (BO-T)
    6. 获取系统状态/系统模式变量  (GT-S / GT-L / GT-K)
    7. 直接修改系统模式变量 (ST-L /  ST-K)
  + 设计模式开：{"KEY-NAME":"on"} / {"KEY-NAME":"press"}
  + 设计模式关：{"KEY-NAME":"off"} / {"KEY-NAME":"pressup"}
  + 获取系统状态：{"GT-SS":"tap"} -- {"SS":"uint16_h/uint16_l"}
  + 获取SYS LED模式：{"GT-SL":"tap"} -- {"SL":"uint16_h/uint16_l"}
  + 获取USR LED模式：{"GT-UL":"tap"} -- {"UL":"uint16_h/uint16_l"}
  + 获取SYS KEY模式：{"GT-SK":"tap"} -- {"SK":"uint16_h/uint16_l"}
  + 获取USR KEY模式：{"GT-UK":"tap"} -- {"UK":"uint16_h/uint16_l"}
  + 设置SYS LED模式：{"ST-SL":"uint16_h/uint16_l"} -- {"SL":"uint16_h/uint16_l"}
  + 默认回复{"KEY-NAME":"OK"}
  + 未定义命令回复{"Error":"Undefine"}
  
+ 重构一下GPIO部分，SYS、LED、KEY就不改了：
  + 全局变量：SYS_GPIO_BITMAP：
    1. GPIO0 DIR
    2. GPIO0 PU
    3. GPIO0 PD
    4. GPIO0 FUC
    5. GPIO1 DIR
    6. GPIO1 PU
    7. GPIO1 FUC
    8. GPIO1 DIR
    9. ...
  
+ 换个思路：GPIO作用选择：考虑GPIO功能间的互斥原理
  
  + uint8_t GPIO_FUNC[8];GPIO_FUNC[7] = FUNC change
  
  + 0 = default
  
  + 1 = connect to SYS LED
  
  + 2 = connect to USER LED
  
  + 3 = connect to SYS KEY
  
  + 4 = connect to USER KEY
  
  + 5 = unique Func
  
  + 6 = release
  
  + | GPIO_FUNC | GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO14 | GPIO15 | QL Pin |
    | ----: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
    |     0 | USR KEY | RX | USR LED | TX | Release | Release | DEBUG |
    |     1 | SYS LED | SYS LED | SYS LED | SYS LED | SYS LED | SYS LED | SYS LED |
    |     2 | USR LED | USR LED | USR LED | USR LED | USR LED | USR LED | USR LED |
    |     3 | SYS KEY | SYS KEY | SYS KEY | SYS KEY | SYS KEY | SYS KEY | SYS KEY |
    |     4 | USR KEY | USR KEY | USR KEY | USR KEY | USR KEY | USR KEY | USR KEY |
    |     5 | BOOT/AT | RX | BOOT/SD | TX | SDIO | SDIO | DEBUG |
    |     6 | Release | Release | Release | Release | Release | Release | Release |
  
+ LED STATE[2]重构

  + 0 = default
  + 1 = IO控制
  + 2 = 串口/USB/BLE控制为点亮
  + 3 = 串口/USB/BLE控制为熄灭
  + 0x80 = IO是否反转控制（1:低电平控制， 0:高电平控制）
  
+ IO to LED BitMap

+ KEY STATE[2]重构

  + 0 = default
  + 1 = 对IO输出
  + 2 = 对UART输出
  + 0x80 = IO是否反转输出（1:按下高电平， 0:按下低电平）
  + 0x40 = IO输出是否锁定（1:模拟自锁，0:模拟轻触）
  + 0x20 = IO输出是否抖动（1:抖动模拟，0:不抖动）
  
+ KEY to IO BitMap
