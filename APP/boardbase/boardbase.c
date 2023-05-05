/**********************************
 *  @name       boardbase.c
 *  @brief      boardbase
 *  @author     Z2Z GuGu
 *  @date       2023/03/05
 *  @code       utf-8
 *********************************/

#include "boardbase.h"

err_code Pin_Ctrl(uint32_t Pin, GPIOStateTpDef State)
{
    if((Pin & is_Port_B) == 0)     // GPIOA
    {
        switch(State)
        {
            case GPIO_LOW:
                GPIOA_ResetBits(Pin);
                break;
            case GPIO_HIGH:
                GPIOA_SetBits(Pin);
                break;
            case GPIO_INVER:
                GPIOA_InverseBits(Pin);
                break;
            default:
                return INPUT_ERR;
        }
    }
    else        //GPIOB
    {
        Pin &= chag_to_B;
        switch(State)
        {
            case GPIO_LOW:
                GPIOB_ResetBits(Pin);
                break;
            case GPIO_HIGH:
                GPIOB_SetBits(Pin);
                break;
            case GPIO_INVER:
                GPIOB_InverseBits(Pin);
                break;
            default:
                return INPUT_ERR;
        }
    }
}

GPIOStateTpDef Pin_state(uint32_t Pin)
{
    if((Pin & is_Port_B) == 0)     // GPIOA
    {
        if(GPIOA_ReadPortPin(Pin) == 0)
        {
            return GPIO_LOW;
        }
        else
        {
            return GPIO_HIGH;
        }
    }
    else
    {
        Pin &= chag_to_B;
        if(GPIOB_ReadPortPin(Pin) == 0)
        {
            return GPIO_LOW;
        }
        else
        {
            return GPIO_HIGH;
        }
    }
}

void QWL_GPIO_Init(void)
{
    // Init BOOT Pin
    GPIOB_SetBits(BOOT_G0_Pin & chag_to_B);
    GPIOB_SetBits(BOOT_G2_Pin & chag_to_B);
    GPIOB_SetBits(BOOT_EN_Pin & chag_to_B);
    GPIOB_ModeCfg(BOOT_G0_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(BOOT_G2_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(BOOT_EN_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
    
    // Init UART Pin
    GPIOB_SetBits(UART_T0_Pin & chag_to_B);
    GPIOB_ModeCfg(UART_T0_Pin & chag_to_B, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(UART_R0_Pin & chag_to_B, GPIO_ModeIN_PU);

    // Init K&L Pin
    GPIOA_ResetBits(User_LED_Pin);
    GPIOA_ResetBits(Sys_LED_Pin);
    GPIOA_ModeCfg(User_LED_Pin, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(Sys_LED_Pin, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(Sys_KEY_Pin & chag_to_B, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(User_KEY_Pin, GPIO_ModeIN_PU);

    // Init Board QL Pin
    GPIOA_ResetBits(Board_QL_Pin);
    GPIOA_ModeCfg(Board_QL_Pin, GPIO_ModeOut_PP_5mA);
    // GPIOA_ModeCfg(Board_QL_Pin, GPIO_ModeIN_Floating);

#ifdef GPIO_DEBUG
    GPIOB_ResetBits(GPIO_Pin_15);
    GPIOB_ResetBits(GPIO_Pin_14);
    GPIOB_ResetBits(GPIO_Pin_13);
    GPIOB_ResetBits(GPIO_Pin_12);
    GPIOB_ResetBits(GPIO_Pin_22);
    GPIOA_ResetBits(GPIO_Pin_4);
    GPIOA_ResetBits(GPIO_Pin_5);
    GPIOA_ResetBits(GPIO_Pin_15);
    GPIOA_ResetBits(GPIO_Pin_14);
    GPIOA_ResetBits(GPIO_Pin_13);
    GPIOA_ResetBits(GPIO_Pin_12);
    GPIOA_ResetBits(GPIO_Pin_11);
    GPIOA_ResetBits(GPIO_Pin_10);
    GPIOB_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_5mA);        // ble
    GPIOB_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);        // usb
    GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);        // usb int
    GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);        // uart
    GPIOB_ModeCfg(GPIO_Pin_22, GPIO_ModeOut_PP_5mA);        // uart int
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_11, GPIO_ModeOut_PP_5mA);         // error
    GPIOA_ModeCfg(GPIO_Pin_10, GPIO_ModeOut_PP_5mA);         // error

    GPIOA_SetBits( GPIO_Pin_9 );
    GPIOA_ModeCfg( GPIO_Pin_9, GPIO_ModeOut_PP_5mA );    // TXD-配置推挽输出，注意先让IO口输出高电平
    UART1_DefInit();
    UART1_BaudRateCfg(460800);
#endif
}

void DEBUG_PIN(uint32_t Pin, GPIOStateTpDef State)
{
#ifdef GPIO_DEBUG
    Pin_Ctrl(Pin, State);
#endif
}

void IRQ_Prioritty_Setting(void)
{
    PFIC_SetPriority(UART0_IRQn, (UART0_Priority << 4) | Pre_Emption_bit);
    PFIC_SetPriority(BLEL_IRQn, (BLEL_Priority << 4) | Pre_Emption_bit);
    PFIC_SetPriority(BLEB_IRQn, (BLEB_Priority << 4) | Pre_Emption_bit);
    PFIC_SetPriority(USB_IRQn, (USB_Priority << 4) | Pre_Emption_bit);
    PFIC_SetPriority(TMR2_IRQn, (TMR2_Priority << 4));
    PFIC_SetPriority(TMR3_IRQn, (TMR3_Priority << 4));
}

/* test code
QWL_GPIO_Init();

Pin_Ctrl(User_LED_Pin, Pin_state(User_KEY_Pin));
Pin_Ctrl(Sys_LED_Pin, Pin_state(Sys_KEY_Pin));
*/
