/**********************************************************************************
 * @name        Application.c
 * @version     R1.1
 * @brief       Application
 * @author      Z2Z GuGu
 * @date        2023/05/28
 * @code        GB2312
 **********************************************************************************/

#include "APP.h"

static uint8_t task_list[Task_List_LENGTH] = {0};

app_drv_fifo_t task_list_fifo;

uint8_t APP_TaskID = INVALID_TASK_ID;

uint8_t *APP_BOOT_State;

// 系统级状态BitMap变量，与Data Flash同步实现掉电保存
volatile uint16_t SYS_STATE_BITMAP = 0x00;

// KEY & LED 状态变量
volatile uint8_t LED_Bink_Mode[2] = {0};
volatile uint16_t LED_STATE_BITMAP[2] = {0};
volatile uint16_t KEY_STATE_BITMAP[2] = {0};

void TIM2_Init()
{
    TMR2_TimerInit(0x3FFFFFF);
}

/*******************************************************************************
 * @fn      Timer_Task_Start
 * @brief   time时间后执行 id号任务
 * @param   overtime: 时间ms max = 1000;
 * @param   id:   任务编号
 * @return  SUCCESS / ~SUCCESS
 *******************************************************************************/
uint8_t Timer_Task_ID = 0;
uint8_t Timer_Task_Start(uint32_t overtime, uint8_t id)
{
    if(Timer_Task_ID)
        return ~SUCCESS;

    Timer_Task_ID = id;
    TMR2_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志
    TMR2_TimerInit(overtime * 60000);     // 设置定时时间 ms
    PFIC_EnableIRQ(TMR2_IRQn);
    TMR2_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
    return SUCCESS;
}

/*******************************************************************************
 * @fn      Timer_Task_Stop
 * @brief   及时结束未来任务
 * @return  None
*******************************************************************************/
void Timer_Task_Stop()
{
    Timer_Task_ID = 0;
    TMR2_ITCfg(DISABLE, TMR0_3_IT_CYC_END); // 关闭中断
}

void BOOT_State_Init(uint8_t *pBOOT_State)
{
    APP_BOOT_State = pBOOT_State;
}

GPIOStateTpDef KEY_EVENT_to_GPIOState(uint8_t KEY_EVENT)
{
    if(KEY_EVENT == KEY_FALLING_EVENT)
        return GPIO_LOW;
    if(KEY_EVENT == KEY_RISING_EVENT)
        return GPIO_HIGH;
    return GPIO_INVER;
}

void Set_LED_Blink_Mode(uint8_t _LED_Index, uint8_t _Blink_Mode)
{
    LED_Bink_Mode[_LED_Index] = _Blink_Mode;
}

void LED_Ctrl(uint8_t LED_Index, uint8_t Ctrl_Mode)
{
    GPIOStateTpDef TEMP_LED_Ctrl;
    static uint8_t event_count[2] = {0};
    static uint8_t blink_count[2] = {0};
    uint8_t TEMP_LED_Mode = LED_Bink_Mode[LED_Index];
    // Counter
    event_count[LED_Index]++;
    if(event_count[LED_Index] >= 40)    // 500ms
        event_count[LED_Index] = 0;

    // Mode
    switch(TEMP_LED_Mode)
    {
        case LED_not_Blink_MODE:
            TEMP_LED_Ctrl = GPIO_HIGH;
            break;
        case LED_Blink_05tps_MODE:
            if(event_count[LED_Index] % 8 == 0)
                TEMP_LED_Ctrl = GPIO_INVER;
            break;
        case LED_Blink_10tps_MODE:
            if(event_count[LED_Index] % 4 == 0)
                TEMP_LED_Ctrl = GPIO_INVER;
            break;
        case LED_Blink_2t_MODE:
            if(event_count[LED_Index] % 16 == 0)
            {
                TEMP_LED_Ctrl = GPIO_INVER;
                blink_count[LED_Index]++;
                if(blink_count[LED_Index] >= 4)
                {
                    blink_count[LED_Index] = 0;
                    if(LED_Index == SYS_LED_INDEX)
                        LED_Bink_Mode[LED_Index] = LED_not_Blink_MODE;
                    else
                        LED_Bink_Mode[LED_Index] = LED_OFF_MODE;
                }
            }
            break;
        default:
            TEMP_LED_Ctrl = GPIO_LOW;
        break;
    }

    // OUTPUT
    if(LED_Index == SYS_LED_INDEX)
    {
        Pin_Ctrl(Sys_LED_Pin, TEMP_LED_Ctrl);
    }
    else
    {
        Pin_Ctrl(User_LED_Pin, TEMP_LED_Ctrl);
    }
}

void LED_Handle(uint8_t LED_Index)
{
    uint16_t TEMP_LED_STATE_BitMap = LED_STATE_BITMAP[LED_Index];
    uint8_t LED_IN_BitMap = 0x01;
    if(TEMP_LED_STATE_BitMap & DFT_LED_MODE_Bit)
    {   // 自定功能
        // 原数据获取
        LED_IN_BitMap |= (Pin_state(GPIO_G0_Pin)? GPIO0_DATA_Bit : 0x00);
        LED_IN_BitMap |= (Pin_state(GPIO_G1_Pin)? GPIO1_DATA_Bit : 0x00);
        LED_IN_BitMap |= (Pin_state(GPIO_G2_Pin)? GPIO2_DATA_Bit : 0x00);
        LED_IN_BitMap |= (Pin_state(GPIO_G3_Pin)? GPIO3_DATA_Bit : 0x00);
        LED_IN_BitMap |= (Pin_state(GPIO_G14_Pin)? GPIO14_DATA_Bit : 0x00);
        LED_IN_BitMap |= (Pin_state(GPIO_G15_Pin)? GPIO15_DATA_Bit : 0x00);
        LED_IN_BitMap |= (Pin_state(Board_QL_Pin)? QL_Pin_DATA_Bit : 0x00);
        // 数据滤除
        LED_IN_BitMap &= (uint8_t)((TEMP_LED_STATE_BitMap & LED_DATA_MASK) >> 2);
        // 数据与&
        if(LED_IN_BitMap != 0)
        {
            LED_IN_BitMap = 1;
        }
        if(TEMP_LED_STATE_BitMap & LED_POLARITY_Bit)
        {
            LED_IN_BitMap = !LED_IN_BitMap;
        }
        // OUT
        if(LED_IN_BitMap)
        {
            LED_Bink_Mode[LED_Index] |= LED_MODE_EN_Bit;
        }
        else
        {
            LED_Bink_Mode[LED_Index] &= ~LED_MODE_EN_Bit;
        }
    }
    else
    {   // 默认功能
        
        switch(LED_Index)
        {
            case SYS_LED_INDEX:
                if(SYS_STATE_BITMAP & SYS_RUNNING_BREAK_STATE_Bit)
                {   // 系统故障
                    LED_Bink_Mode[SYS_LED_INDEX] = LED_OFF_MODE;     // 常灭
                }
                else
                {   // 系统正常运行
                    LED_Bink_Mode[SYS_LED_INDEX] = LED_not_Blink_MODE;      // 常亮
                    if(SYS_STATE_BITMAP & SYS_USB_NEW_STATE_Bit)
                    {
                        SYS_STATE_BITMAP &= ~SYS_USB_NEW_STATE_Bit;
                        LED_Bink_Mode[SYS_LED_INDEX] = LED_Blink_2t_MODE;     // USB 连接/断开：闪两次
                    }
                    else if(SYS_STATE_BITMAP & SYS_BLE_NEW_STATE_Bit)
                    {
                        SYS_STATE_BITMAP &= ~SYS_BLE_NEW_STATE_Bit;
                        LED_Bink_Mode[SYS_LED_INDEX] = LED_Blink_2t_MODE;     // BLE 连接/断开：闪两次
                    }
                    else if(SYS_STATE_BITMAP & SYS_UART_FLASHING_STATE_Bit)
                        LED_Bink_Mode[SYS_LED_INDEX] = LED_Blink_05tps_MODE;   // 烧录中：每秒闪5次
                    else if((SYS_STATE_BITMAP & SYS_UART_TX_ACTION_Bit) || (SYS_STATE_BITMAP & SYS_UART_RX_ACTION_Bit))
                        LED_Bink_Mode[SYS_LED_INDEX] = LED_Blink_10tps_MODE;   // UART ACTION：每秒闪10次
                    else if(SYS_STATE_BITMAP & SYS_KEY_ACTION_Bit)
                        LED_Bink_Mode[SYS_LED_INDEX] = LED_OFF_MODE;     // 按下SYS按键：LED熄灭，ESP32 EN = 0
                }
                break;
            case USR_LED_INDEX:
                if(Pin_state(GPIO_G2_Pin))
                {
                    LED_Bink_Mode[USR_LED_INDEX] = LED_not_Blink_MODE;
                }
                else
                {
                    LED_Bink_Mode[USR_LED_INDEX] = LED_OFF_MODE;
                }
                break;
            default:
                break;
        }
    }
}

void KEY_Handle(uint8_t KEY_Index, uint8_t KEY_Event)
{
    uint16_t TEMP_KEY_STATE_BitMap = KEY_STATE_BITMAP[KEY_Index];
    uint8_t KEY_OUT = KEY_Event;
    if(TEMP_KEY_STATE_BitMap & DFT_KEY_MODE_Bit)
    {   // 自定功能
        // 模式处理
        if(TEMP_KEY_STATE_BitMap & KEY_POLARITY_Bit)
            KEY_OUT = ~KEY_Event;
        else
            KEY_OUT = KEY_Event;
        if(TEMP_KEY_STATE_BitMap & KEY_LOCK_Bit)
        {
            if(KEY_Event == KEY_FALLING_EVENT)
            {
                KEY_OUT = KEY_INVER_EVENT;
            }
            else
            {
                return;
            }
        }
        if(TEMP_KEY_STATE_BitMap & KEY_SHAKE_Bit)
        {   // 开延时task
            tmos_set_event(APP_TaskID, KEY_SHAKE_EVT);
        }

        // OUTPUT
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_GPIO0_Bit)
            Pin_Ctrl(GPIO_G0_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_GPIO1_Bit)
            Pin_Ctrl(GPIO_G1_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_GPIO2_Bit)
            Pin_Ctrl(GPIO_G2_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_GPIO3_Bit)
            Pin_Ctrl(GPIO_G3_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_GPIO14_Bit)
            Pin_Ctrl(GPIO_G14_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_GPIO15_Bit)
            Pin_Ctrl(GPIO_G15_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
        if(TEMP_KEY_STATE_BitMap & KEY_STATE_to_QL_Pin_Bit)
            Pin_Ctrl(Board_QL_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
    }
    else
    {   // 默认功能
        // if(KEY_Index == SYS_KEY_INDEX)
        switch(KEY_Index)
        {
            case SYS_KEY_INDEX:
                if(KEY_OUT == KEY_FALLING_EVENT)
                {
                    Pin_Ctrl(BOOT_EN_Pin, GPIO_LOW);
                    SYS_STATE_BITMAP |= SYS_KEY_ACTION_Bit;
                }
                else
                {
                    Pin_Ctrl(BOOT_EN_Pin, GPIO_HIGH);
                    SYS_STATE_BITMAP &= ~SYS_KEY_ACTION_Bit;
                }
                break;
            case USR_KEY_INDEX:
                Pin_Ctrl(GPIO_G0_Pin, KEY_EVENT_to_GPIOState(KEY_OUT));
                break;
            default:
                break;
        }
    }
}

uint16_t App_ProcessEvent(uint8 task_id, uint16 events)
{
    if(events & APP_Test_EVT)
    {
        DEBUG_PIN(A10_Pin, GPIO_INVER);
        return (events ^ APP_Test_EVT);
    }
    if(events & ESP_Start_From_SPI)
    {
        GPIOB_SetBits(BOOT_G0_Pin & chag_to_B);
        GPIOB_SetBits(BOOT_G2_Pin & chag_to_B);
        GPIOB_ResetBits(BOOT_EN_Pin & chag_to_B);
        GPIOB_ModeCfg(BOOT_G0_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        GPIOB_ModeCfg(BOOT_G2_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        GPIOB_ModeCfg(BOOT_EN_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        return (events ^ ESP_Start_From_SPI);
    }
    if(events & ESP_BOOT)
    {
        GPIOB_ResetBits(BOOT_G0_Pin & chag_to_B);
        GPIOB_ResetBits(BOOT_G2_Pin & chag_to_B);
        GPIOB_ResetBits(BOOT_EN_Pin & chag_to_B);
        GPIOB_ModeCfg(BOOT_G0_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        GPIOB_ModeCfg(BOOT_G2_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        GPIOB_ModeCfg(BOOT_EN_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        return (events ^ ESP_BOOT);
    }
    if(events & USR_RST_PREP_EVT)
    {
        GPIOB_ResetBits(BOOT_EN_Pin & chag_to_B);
        GPIOB_ModeCfg(BOOT_EN_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        return (events ^ USR_RST_PREP_EVT);
    }
    if(events & ESP_RST)
    {
        GPIOB_SetBits(BOOT_EN_Pin & chag_to_B);
        GPIOB_ModeCfg(BOOT_EN_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        return (events ^ ESP_RST);
    }
    if(events & ESP_G02_Release)
    {
        GPIOB_SetBits(BOOT_G0_Pin & chag_to_B);
        GPIOB_ModeCfg(BOOT_G0_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        // GPIOB_ModeCfg(BOOT_G0_Pin & chag_to_B, GPIO_ModeIN_Floating);
        GPIOB_ModeCfg(BOOT_G2_Pin & chag_to_B, GPIO_ModeIN_Floating);
        // GPIOB_SetBits(BOOT_G2_Pin & chag_to_B);
        // GPIOB_ModeCfg(BOOT_G2_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
        return (events ^ ESP_G02_Release);
    }
    if(events & SYS_KEY_EVT)
    {
        static GPIOStateTpDef SYS_KEY_old = GPIO_INVER;
        GPIOStateTpDef SYS_KEY_val;
        SYS_KEY_val = Pin_state(Sys_KEY_Pin);
        if(SYS_KEY_val != SYS_KEY_old)
        {
            SYS_KEY_old = SYS_KEY_val;
            KEY_Handle(SYS_KEY_INDEX, (SYS_KEY_val == GPIO_LOW) ? KEY_FALLING_EVENT : KEY_RISING_EVENT);
        }
        return (events ^ SYS_KEY_EVT);
    }
    if(events & USR_KEY_EVT)
    {
        static GPIOStateTpDef USR_KEY_old = GPIO_INVER;
        GPIOStateTpDef USR_KEY_val;
        USR_KEY_val = Pin_state(User_KEY_Pin);
        if(USR_KEY_val != USR_KEY_old)
        {
            USR_KEY_old = USR_KEY_val;
            KEY_Handle(USR_KEY_INDEX, (USR_KEY_val == GPIO_LOW) ? KEY_FALLING_EVENT : KEY_RISING_EVENT);
        }
        return (events ^ USR_KEY_EVT);
    }
    if(events & LED_UPDATE_EVT)
    {
        // 已弃用
        return (events ^ LED_UPDATE_EVT);
    }
    if(events & SYS_LED_EVT)
    {
        LED_Handle(SYS_LED_INDEX);
        LED_Ctrl(SYS_LED_INDEX, LED_Bink_Mode[SYS_LED_INDEX]);
        return (events ^ SYS_LED_EVT);
    }
    if(events & USR_LED_EVT)
    {
        LED_Handle(USR_LED_INDEX);
        LED_Ctrl(USR_LED_INDEX, LED_Bink_Mode[USR_LED_INDEX]);
        return (events ^ USR_LED_EVT);
    }
    if(events & KEY_SHAKE_EVT)
    {
        static uint8_t SHAKE_Count = 0;
        if(SHAKE_Count >= KEY_SHAKE_COUNT_MAX)
        {
            SHAKE_Count = 0;
            // tmos_clear_event(APP_TaskID, KEY_SHAKE_EVT);
        }
        else
        {
            SHAKE_Count++;

            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_GPIO0_Bit)
                Pin_Ctrl(GPIO_G0_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_GPIO1_Bit)
                Pin_Ctrl(GPIO_G1_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_GPIO2_Bit)
                Pin_Ctrl(GPIO_G2_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_GPIO3_Bit)
                Pin_Ctrl(GPIO_G3_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_GPIO14_Bit)
                Pin_Ctrl(GPIO_G14_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_GPIO15_Bit)
                Pin_Ctrl(GPIO_G15_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[0] & KEY_STATE_to_QL_Pin_Bit)
                Pin_Ctrl(Board_QL_Pin, GPIO_INVER);
            
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_GPIO0_Bit)
                Pin_Ctrl(GPIO_G0_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_GPIO1_Bit)
                Pin_Ctrl(GPIO_G1_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_GPIO2_Bit)
                Pin_Ctrl(GPIO_G2_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_GPIO3_Bit)
                Pin_Ctrl(GPIO_G3_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_GPIO14_Bit)
                Pin_Ctrl(GPIO_G14_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_GPIO15_Bit)
                Pin_Ctrl(GPIO_G15_Pin, GPIO_INVER);
            if(KEY_STATE_BITMAP[1] & KEY_STATE_to_QL_Pin_Bit)
                Pin_Ctrl(Board_QL_Pin, GPIO_INVER);

            tmos_start_task(APP_TaskID, KEY_SHAKE_EVT, 2);
        }
        return (events ^ KEY_SHAKE_EVT);
    }

    return 0;
}

void task_list_fifo_init()
{
    app_drv_fifo_init(&task_list_fifo, task_list, Task_List_LENGTH);
}

void All_FIFO_Init(void)
{
    task_list_fifo_init();
    ble_fifo_init();
//    usb_fifo_init();
    uart_fifo_init();
}

// #define UART0_Start_TX_from_USB_EVT     0x0008
// #define UART0_Start_TX_from_USB_SYB     1
// app_drv_fifo_write_from_same_addr(&task_list_fifo, &TEMP_Task_SYB, 1);  // 中断中启动TMOS task的特殊方式
void Task_Launcher(void)
{
    uint8_t Task_Symbol;
    while(!app_drv_fifo_is_empty(&task_list_fifo))
    {
        app_drv_fifo_read_to_same_addr(&task_list_fifo, &Task_Symbol, 1);
        switch(Task_Symbol)
        {
            // case UART0_Start_TX_from_USB_SYB: {
            //     tmos_set_event(UART_TaskID, UART0_Start_TX_from_USB_EVT);
            // }   break;
            default:
                break;
        }
    }
}

void APP_task_Init(void)
{
    APP_TaskID = TMOS_ProcessEventRegister(App_ProcessEvent);
    tmos_start_reload_task(APP_TaskID, SYS_KEY_EVT, 20);    // 12.5ms
    tmos_start_reload_task(APP_TaskID, USR_KEY_EVT, 20);    // 12.5ms
    tmos_start_reload_task(APP_TaskID, SYS_LED_EVT, 20);    // 50ms
    tmos_start_reload_task(APP_TaskID, USR_LED_EVT, 20);    // 12.5ms
    // tmos_start_reload_task(APP_TaskID, LED_UPDATE_EVT, 20);    // 12.5ms //已弃用
}

void BOOT_CTRL_LINE_Check(uint8_t COM_CTRL_LINE_STATE)
{
  if((COM_CTRL_LINE_STATE & COM_DTR_NEW_Edge) || (COM_CTRL_LINE_STATE & COM_RTS_NEW_Edge))
  { // 前提：出现跳边沿
    // UART1_SendByte(COM_CTRL_LINE_STATE);
    switch(COM_CTRL_LINE_STATE)
    {
      case ESP32_DLD_STATE:
        // 烧录信号，需进一步判断
        *APP_BOOT_State = ESP32_UART_Check;
        // 500ms内检测不到合适的串口数据则认为情报是假滴，退出UART_Check状态
        // 中断内不能调用tmos 任务调度函数
        Timer_Task_Start(500, Timer_Task_RST_BOOT_State);
        break;
      // case ESP_Tool_CTRL_STATE:
      //   *APP_BOOT_State = ESP32_UART_Check;
      //   break;
      case ESP32_DLDEND_STATE:
        // 烧录完成信号
        *APP_BOOT_State = ESP32_Reset_NOW;
        SYS_STATE_BITMAP &= ~SYS_UART_FLASHING_STATE_Bit;
        break;
      case ESP32_RST_STATE:
        // 串口软件打开信号
        *APP_BOOT_State = ESP32_Reset_NOW;
        break;
    //   case Arduino_DLD_STATE:
    //     // Arduino烧录信号 / 串口打开信号
    //     *APP_BOOT_State = Arduino_UART_Check;
    //     break;
      default:
        break;
    }
  }
  else
  {
    // if(!(COM_CTRL_LINE_STATE == ESP_Tool_CTRL_LINE))
    *APP_BOOT_State = IDLE_Mode;
  }
}

void BOOT_Serial_Chack(uint8_t First_bit)
{
    if(*APP_BOOT_State == ESP32_UART_Check)
    {
        if(R8_USB_RX_LEN == ESP32_BOOT_LEN)
        {
            if(First_bit == ESP32_BOOT_F_Bit) 
            {
                *APP_BOOT_State = ESP32_BOOT_NOW;
                SYS_STATE_BITMAP |= SYS_UART_FLASHING_STATE_Bit;
                Timer_Task_Stop();
            }
        }
    }
    // else if(*APP_BOOT_State == Arduino_UART_Check)
    // {
    //     if(R8_USB_RX_LEN == Arduino_BOOT_LEN)
    //     {
    //         if(First_bit == Arduino_BOOT_F_Bit) *APP_BOOT_State = Arduino_BOOT_NOW;
    //     }
    // }
    // else if(*APP_BOOT_State == SSCOM_UART_Check)
    // {
    //     if(R8_USB_RX_LEN == SSCOM_BOOT_LEN)
    //     {
    //         if(First_bit == SSCOM_BOOT_F_Bit) *APP_BOOT_State = STC_BOOT_NOW;
    //     }
    // }
}

void BOOT_Execute(void)
{
    if(*APP_BOOT_State & Execute_NOW_Bit)
    {
        switch (*APP_BOOT_State)
        {
        case ESP32_BOOT_NOW:
            tmos_start_task(APP_TaskID, ESP_BOOT, 0);
            tmos_start_task(APP_TaskID, ESP_RST, 10);
            tmos_start_task(APP_TaskID, ESP_G02_Release, 20);
            break;
        case ESP32_Reset_NOW:
            tmos_start_task(APP_TaskID, USR_RST_PREP_EVT, 0);
            tmos_start_task(APP_TaskID, ESP_RST, 10);
            break;
        
        default:
            break;
        }
        // 复位
        *APP_BOOT_State = IDLE_Mode;
    }
}

/*******************************************************************************
 * @fn      TMR2_IRQHandler
 * @brief   TMR2中断函数
 * @return  None
*******************************************************************************/
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR2_IRQHandler(void) // TMR2 定时中断
{
    TMR2_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志
    switch (Timer_Task_ID)
    {
    case Timer_Task_RST_BOOT_State:
        *APP_BOOT_State = IDLE_Mode;
        break;
    
    default:
        break;
    }
    Timer_Task_ID = 0;
    TMR2_ITCfg(DISABLE, TMR0_3_IT_CYC_END); // 关闭中断
}
