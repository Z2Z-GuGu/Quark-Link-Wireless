/**********************************************************************************
 * @name        uart_task.c
 * @version     R1.0
 * @brief       uart task
 * @author      Z2Z GuGu
 * @date        2023/03/07
 * @code        GB2312
 **********************************************************************************/

#include "uart_task.h"

static uint8_t uart_tx_buffer[UART_TX_BUFFER_LENGTH] = {0};
static uint8_t uart_rx_buffer[UART_RX_BUFFER_LENGTH] = {0};

app_drv_fifo_t BLE_to_UART_fifo;
app_drv_fifo_t UART_to_BLE_fifo;

// extern app_drv_fifo_t BLE_to_UART_fifo;
// extern app_drv_fifo_t ble_tx_fifo;


uint8_t UART_TaskID = INVALID_TASK_ID;

/*******************************************************************************
 * @fn      UART_RX_TimeOUT_Init
 * @brief   UART RX 超时发送，时间设置函数
 * @param   t：UART 通信速率
 * @return  None
*******************************************************************************/
void UART_RX_TimeOUT_Init(uint32_t t)
{
  TMR3_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志
  TMR3_TimerInit(FREQ_SYS / t * 80);     // 设置定时时间 1ms    //*80--TimeOut 8Byte  //*40--TimeOut 4Byte
  PFIC_EnableIRQ(TMR3_IRQn);
  // TMR3_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
}

/*******************************************************************************
 * @fn      UART_RX_TimeOUT_Start
 * @brief   UART RX 超时计时开始
 * @return  None
*******************************************************************************/
void UART_RX_TimeOUT_Start()
{
  R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;    //强制清零count
  R8_TMR3_CTRL_MOD = RB_TMR_COUNT_EN;
  TMR3_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
}

/*******************************************************************************
 * @fn      UART_RX_TimeOUT_Stop
 * @brief   UART RX 超时计时结束
 * @return  None
*******************************************************************************/
void UART_RX_TimeOUT_Stop()
{
  R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;    //强制清零count
  R8_TMR3_CTRL_MOD = RB_TMR_COUNT_EN;
  TMR3_ITCfg(DISABLE, TMR0_3_IT_CYC_END); // 开启中断
}

void uart_fifo_init()
{
    //tx fifo and tx fifo
    //The buffer length should be a power of 2
    app_drv_fifo_init(&BLE_to_UART_fifo, uart_tx_buffer, UART_TX_BUFFER_LENGTH);
    app_drv_fifo_init(&UART_to_BLE_fifo, uart_rx_buffer, UART_RX_BUFFER_LENGTH);
}

/*******************************************************************************
 * @fn      UART0_Init
 * @brief   UART初始化
 * @param   UART0_bps：UART 通信速率
 * @return  None
*******************************************************************************/
void UART0_Init(uint32_t UART0_bps)
{
    // already exit at bb GPIO_Init()
    // GPIOB_SetBits(GPIO_Pin_7);
    // GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    // GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART0_DefInit();
    UART0_BaudRateCfg(UART0_bps);
    UART0_ByteTrigCfg(UART_7BYTE_TRIG);
    UART0_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);       // RB_IER_THR_EMPTY | 
    PFIC_EnableIRQ(UART0_IRQn);
}

void uart_fifo_cheak_process()
{
    if(!app_drv_fifo_is_empty(&BLE_to_UART_fifo))
    {
        tmos_set_event(UART_TaskID, UART0_TX_NEW_FRAME_EVT);
    }
}

uint16 Uart_ProcessEvent(uint8 task_id, uint16 events)
{
    // DEBUG_PIN(UART_Pin, GPIO_HIGH);
    // uart_fifo_cheak_process();
    if(events & UART0_RX_NEW_FRAME_EVT)
    {
        // app_drv_fifo_write_from_same_addr(&BLE_to_UART_fifo, (uint8_t *)&R8_UART0_RBR, R8_UART0_RFC);
        // DEBUG_PIN(UART_Pin, GPIO_LOW);
        return (events ^ UART0_RX_NEW_FRAME_EVT);
    }
    if(events & UART0_TX_NEW_FRAME_EVT)
    {
        if ( (SYS_STATE_BITMAP & SYS_BLE_to_UART_EN_Bit) && \
            !(SYS_STATE_BITMAP & SYS_UART_FLASHING_STATE_Bit) && \
            !(SYS_STATE_BITMAP & SYS_UART_TX_ACTION_Bit))
        // 使能BLE to UART； 不在Flash状态； TX未处于发送状态
        {
            if(R8_UART0_TFC < UART_FIFO_SIZE)
            {
                app_drv_fifo_read_to_same_addr(&BLE_to_UART_fifo, (uint8_t *)&R8_UART0_THR, UART_FIFO_SIZE - R8_UART0_TFC);
            }
        }
        //    EP1_OUT_ACK_Condition();
        // DEBUG_PIN(UART_Pin, GPIO_LOW);
        return (events ^ UART0_TX_NEW_FRAME_EVT);
    }
    if(events & UART0_RX_TIMEOUT_EVT)
    {
        // DEBUG_PIN(UART_Pin, GPIO_LOW);
        return (events ^ UART0_RX_TIMEOUT_EVT);
    }
    if(events & UART0_SYS_STATE_EVT)
    {
        if(R8_UART0_TFC > 0)
            SYS_STATE_BITMAP |= SYS_UART_TX_ACTION_Bit;
        else
            SYS_STATE_BITMAP &= ~SYS_UART_TX_ACTION_Bit;

        if(R8_UART0_RFC > 0)
            SYS_STATE_BITMAP |= SYS_UART_RX_ACTION_Bit;
        else
            SYS_STATE_BITMAP &= ~SYS_UART_RX_ACTION_Bit;
        
        return (events ^ UART0_SYS_STATE_EVT);
    }
    // DEBUG_PIN(UART_Pin, GPIO_LOW);
    return 0;
}

void uart_task_Init()
{
    UART_RX_TimeOUT_Init(UART0_default_bps);
    UART0_Init(UART0_default_bps);
    UART_TaskID = TMOS_ProcessEventRegister(Uart_ProcessEvent);
    tmos_start_reload_task(UART_TaskID, UART0_SYS_STATE_EVT, 20);
    // tmos_set_event(UART_TaskID, UART0_RX_NEW_FRAME_EVT);
    // tmos_start_reload_task(UART_TaskID, UART0_RX_NEW_FRAME_EVT, 800);
}

uint8_t UART_TX_index = 0;
void uart_auto_send(void)
{
    // PFIC_DisableIRQ(UART0_IRQn);
    // PFIC_DisableIRQ(USB_IRQn);
    if(EP1_TEMP_Buf_Ready)
    {
        while(R8_UART0_TFC < 8)
        {
            if(UART_TX_index < EP1_R_Buf_Len)
            {
                // UART1_SendByte(Rev_Data1_Offset + UART_TX_index);
                R8_UART0_THR = pUSB_TEMP_BUF[UART_TX_index];
                UART_TX_index++;
            }
            else
            {
                UART_TX_index = 0;
                EP1_TEMP_Buf_Ready = 0; // 清除USB RX Ready标识
                R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
                break;
            }
        }
    }
    // PFIC_EnableIRQ(UART0_IRQn);
    // PFIC_EnableIRQ(USB_IRQn);
}

// 不考虑短时间快速数据的问题了，遇到这种问题直接往data flash里边存
void fill_usb_buffer_auto_send(uint8_t _send_now)
{
    static uint8_t EP1_IN_index = 0;
    uint8_t j;

    while (R8_UART0_RFC > 0)
    {
        pEP1_TEMP_TX_BUF[EP1_IN_index] = UART0_RecvByte();
        if(SYS_STATE_BITMAP & SYS_UART_to_BLE_EN_Bit)
            app_drv_fifo_write_from_same_addr(&UART_to_BLE_fifo, (uint8_t *)(pEP1_TEMP_TX_BUF + EP1_IN_index), 1);
        EP1_IN_index++;
        if(EP1_IN_index >= MAX_IN_PACKET_SIZE)
        {
            EP1_IN_index = 0;
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)            //保存首帧
              ((UINT32 *)(pEP1_RAM_Addr + 64))[j] = ((UINT32 *)(pEP1_TEMP_TX_BUF))[j];
            R8_UEP1_T_LEN = MAX_IN_PACKET_SIZE;
            R8_UEP1_CTRL = R8_UEP1_CTRL & (~MASK_UEP_T_RES);
            EP1_IN_Busy = 1;
        }
    }
    
    if(_send_now)
    {
        if(EP1_IN_Busy)
        {
            EP1_IN_Tail_LEN = EP1_IN_index;
            EP1_IN_Tail = 1;
        }
        else
        {
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)            //保存首帧
              ((UINT32 *)(pEP1_RAM_Addr + 64))[j] = ((UINT32 *)(pEP1_TEMP_TX_BUF))[j];
            R8_UEP1_T_LEN = EP1_IN_index;
            R8_UEP1_CTRL = R8_UEP1_CTRL & (~MASK_UEP_T_RES);
            EP1_IN_Busy = 1;
        }
        // 清空EP1_IN_index
        EP1_IN_index = 0;
    }
}

/*******************************************************************************
 * @fn      UART0_IRQHandler
 * @brief   UART0中断函数
 * @return  None
*******************************************************************************/
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART0_IRQHandler(void)
{
    DEBUG_PIN(UART_INT_Pin, GPIO_HIGH);
    switch(UART0_GetITFlag())
    {
        case UART_II_LINE_STAT: // 线路状态错误
        {
            uint8_t TEMP;
            // DEBUG_PIN(E_Pin, GPIO_INVER);
            // DEBUG_PIN(USB_INT_Pin, GPIO_INVER);
            TEMP = UART0_GetLinSTA();
            // UART0_SendByte(TEMP);
        } break;
        case UART_II_RECV_RDY: // 数据达到设置触发点，接受到7Byte数据从FIFO到软件FIFO
        {
            // DFT_SYS_LED_STATE = LED_10tps;  // 统一在APP.c中处理
            fill_usb_buffer_auto_send(0);
            DEBUG_PIN(USB_Pin, GPIO_INVER);
            UART_RX_TimeOUT_Start();
            // tmos_set_event(UART_TaskID, UART0_RX_NEW_FRAME_EVT);         // 中断内不能调用TMOS，忘了
        } break;
        case UART_II_RECV_TOUT: // 接收超时，暂时一帧数据(0-6 Byte)接收完成
        {
            // DFT_SYS_LED_STATE = LED_10tps;   // 过快（短）的数据不予处理
            // DFT_SYS_LED_STATE = LED_OFF;  // 统一在APP.c中处理
            UART_RX_TimeOUT_Stop();
            DEBUG_PIN(UART_Pin, GPIO_INVER);
            fill_usb_buffer_auto_send(1);
        } break;
        case UART_II_THR_EMPTY: // 发送缓存区空，可继续发送；
        {
            // DFT_SYS_LED_STATE = LED_ON;  // 统一在APP.c中处理
            // DEBUG_PIN(E_Pin, GPIO_INVER);
            //UART0_INTCfg(ENABLE, RB_IER_THR_EMPTY);或发送缓存区空将触发此函数
            /* 第一次进入该函数是UART0_INTCfg(ENABLE, RB_IER_THR_EMPTY)后
            * 此后FIFO空进该函数  */
        } break;

        case UART_II_MODEM_CHG: // 只支持串口0
          break;

        default:
          break;
    }
    DEBUG_PIN(UART_INT_Pin, GPIO_LOW);
}

/*******************************************************************************
 * @fn      TMR3_IRQHandler
 * @brief   TMR3中断函数
 * @return  None
*******************************************************************************/
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR3_IRQHandler(void) // TMR3 定时中断
{
  TMR3_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志
//   DFT_SYS_LED_STATE = LED_ON;  // 统一在APP.c中处理
  fill_usb_buffer_auto_send(1);
  TMR3_ITCfg(DISABLE, TMR0_3_IT_CYC_END); // 关闭中断
}
