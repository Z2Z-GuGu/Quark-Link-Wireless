/**********************************
 *  @name       boardbase.h
 *  @brief      boardbase
 *  @author     Z2Z GuGu
 *  @date       2023/03/05
 *  @code       utf-8
 *********************************/

#ifndef __BOARD_BASE_H__
#define __BOARD_BASE_H__

#include "projconfig.h"
#include "config.h"

// Pin define
#define GPIO_Port_A         (0x00000000)
#define GPIO_Port_B         (0x80000000)
#define is_Port_B           (0x80000000)
#define chag_to_B           (0x7fffffff)
// USB
#define USB_DP_Pin          GPIO_Port_B | GPIO_Pin_11
#define USB_DN_Pin          GPIO_Port_B | GPIO_Pin_10
// UART
#define UART_R0_Pin         GPIO_Port_B | GPIO_Pin_4
#define UART_T0_Pin         GPIO_Port_B | GPIO_Pin_7
// BOOT Control
#define BOOT_G0_Pin         GPIO_Port_B | GPIO_Pin_15
#define BOOT_G2_Pin         GPIO_Port_B | GPIO_Pin_13
#define BOOT_EN_Pin         GPIO_Port_B | GPIO_Pin_14
// GPIO
#define GPIO_G0_Pin         GPIO_Port_B | GPIO_Pin_15
#define GPIO_G1_Pin         GPIO_Port_B | GPIO_Pin_4
#define GPIO_G2_Pin         GPIO_Port_B | GPIO_Pin_13
#define GPIO_G3_Pin         GPIO_Port_B | GPIO_Pin_7
#define GPIO_G14_Pin        GPIO_Port_A | GPIO_Pin_4
#define GPIO_G15_Pin        GPIO_Port_A | GPIO_Pin_4
// KEY
#define User_KEY_Pin        GPIO_Port_A | GPIO_Pin_9
#define Sys_KEY_Pin         GPIO_Port_B | GPIO_Pin_22
// LED
#define User_LED_Pin        GPIO_Port_A | GPIO_Pin_13
#define Sys_LED_Pin         GPIO_Port_A | GPIO_Pin_12
// Board
#define Board_QL_Pin        GPIO_Port_A | GPIO_Pin_4

#define BLE_Pin             GPIO_Port_B | GPIO_Pin_15
#define USB_Pin             GPIO_Port_B | GPIO_Pin_14
#define USB_INT_Pin         GPIO_Port_B | GPIO_Pin_13
#define UART_Pin            GPIO_Port_B | GPIO_Pin_12
#define UART_INT_Pin        GPIO_Port_B | GPIO_Pin_22
#define A4_Pin              GPIO_Port_A | GPIO_Pin_4
#define A5_Pin              GPIO_Port_A | GPIO_Pin_5
#define A15_Pin             GPIO_Port_A | GPIO_Pin_15
#define A14_Pin             GPIO_Port_A | GPIO_Pin_14
#define A13_Pin             GPIO_Port_A | GPIO_Pin_13
#define A12_Pin             GPIO_Port_A | GPIO_Pin_12
#define A11_Pin             GPIO_Port_A | GPIO_Pin_11
#define A10_Pin             GPIO_Port_A | GPIO_Pin_10

// MAX = 7
#define Pre_Emption_bit     (0x80)
#define UART0_Priority      3
#define BLEL_Priority       4
#define BLEB_Priority       5
#define USB_Priority        3
#define TMR2_Priority       6
#define TMR3_Priority       6


//typedef enum
//{
//  SUCCESS0 = 0,
//  SUCCESS,
//  INPUT_ERR,
//  OUTPUT_ERR,
//  UNKNOWN_ERR,
//} err_code;

typedef enum
{
    SUC0 = 0,
    SUCCESS1,
    INPUT_ERR,
    OUTPUT_ERR,
    UNKNOWN_ERR,
}  err_code;

typedef enum
{
    GPIO_LOW =    0,      //
    GPIO_HIGH =   1,      //
    GPIO_INVER =  -1,     //
} GPIOStateTpDef;

err_code Pin_Ctrl(uint32_t Pin, GPIOStateTpDef State);
GPIOStateTpDef Pin_state(uint32_t Pin);
void QWL_GPIO_Init(void);
void DEBUG_PIN(uint32_t Pin, GPIOStateTpDef State);
void IRQ_Prioritty_Setting(void);
void BB_IO_MODE_SETTING(uint32_t pin, GPIOModeTypeDef mode);

#endif // __BOARD_BASE_H__
