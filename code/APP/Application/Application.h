/**********************************************************************************
 * @name        Application.h
 * @version     R1.0
 * @brief       Application
 * @author      Z2Z GuGu
 * @date        2023/03/30
 * @code        GB2312
 **********************************************************************************/

#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projconfig.h"
#include "config.h"
#include "HAL.h"

#include "blinker.h"
#include "boardbase.h"
#include "ble_task.h"
#include "usb_task.h"
#include "uart_task.h"

// UART Task Events
#define APP_Test_EVT        0x0001
#define ESP_Start_From_SPI  0x0002
#define ESP_BOOT            0x0004
#define ESP_RST             0x0008
#define ESP_G02_Release     0x0010
#define SYS_KEY_EVT         0x0020
#define USR_KEY_EVT         0x0040
#define SYS_LED_EVT         0x0080
#define USR_LED_EVT         0x0100
#define USR_RST_PREP_EVT    0x0200

// Timer Task IDs
#define Timer_Task_RST_BOOT_State   1

/* RTS(高四位)/DTR（低四位）状态
 *  H ｜ L ｜ Rising ｜ Falling ｜
 *  B ｜ 1 ｜   F    ｜    5    ｜
 *  usb_task.h:
 *  #define COM_NEW_DTR_Bit                 0x01
 *  #define COM_DTR_Bit                     0x02
 *  #define COM_DTR_NEW_Edge_Bit            0x04
 *  #define COM_DTR_Rising_Bit              0x08
 *  #define COM_NEW_RTS_Bit                 0x10
 *  #define COM_RTS_Bit                     0x20
 *  #define COM_RTS_NEW_Edge_Bit            0x40
 *  #define COM_RTS_Rising_Bit              0x80 */

#define ESP32_DLD_STATE                 0xBF
#define ESP32_DLDEND_STATE              0x51
#define ESP32_RST_STATE                 0xFF
#define ESP_Tool_CTRL_STATE             0x15  // 存在未知问题
#define Arduino_DLD_STATE              0xFF
#define SSCOM_CTRL_STATE                0xFB

#define ESP32_BOOT_F_Bit                0xC0
#define ESP32_BOOT_LEN                  46
#define ESP_Tool_BOOT_F_Bit             0xC0
#define ESP_Tool_BOOT_LEN               46
#define Arduino_BOOT_F_Bit              0x30
#define Arduino_BOOT_LEN                2
#define SSCOM_BOOT_F_Bit                0x7F
#define SSCOM_BOOT_LEN                  1

// BOOT_State
#define Execute_NOW_Bit                 0x80
// #define Only_UART_Check                 0x00
#define IDLE_Mode                       0x00
#define ESP32_UART_Check                0x01
#define Arduino_UART_Check              0x02
#define SSCOM_UART_Check                0x03
// #define Only_UART_S_Check               0x40
// #define ESP32_UART_S_Check              0x41
// #define Arduino_UART_S_Check            0x42
// #define SSCOM_UART_S_Check              0x43
#define ESP32_BOOT_NOW                  0x81
#define ESP32_Reset_NOW                 0x82
#define Arduino_BOOT_NOW                0x83
#define STC_BOOT_NOW                    0x84
#define STM32_BOOT_NOW                  0x85

#define TIM2_value          (uint16_t)(R32_TMR2_COUNT >>= 16)

extern app_drv_fifo_t task_list_fifo;
extern uint8_t APP_TaskID;
extern uint8_t SYS_STATE_Flashing;

void All_FIFO_Init(void);
void APP_task_Init(void);
void Task_Launcher(void);
void BOOT_Execute(void);
void BOOT_CTRL_LINE_Check(uint8_t COM_CTRL_LINE_STATE);
void BOOT_State_Init(uint8_t *pBOOT_State);
void BOOT_Serial_Chack(uint8_t First_bit);

#ifdef __cplusplus
}
#endif


#endif // __APPLICATION_H__
