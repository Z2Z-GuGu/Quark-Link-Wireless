/**********************************************************************************
 * @name        Application.h
 * @version     R1.1
 * @brief       Application
 * @author      Z2Z GuGu
 * @date        2023/05/28
 * @code        GB2312
 **********************************************************************************/

#ifndef __APP_H__
#define __APP_H__

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
#define KEY_SHAKE_EVT       0x0400
#define LED_UPDATE_EVT      0x0800

#define KEY_SHAKE_COUNT_MAX         3

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

// KEY & LED INDEX
#define SYS_KEY_INDEX                   0
#define USR_KEY_INDEX                   1
#define SYS_LED_INDEX                   0
#define USR_LED_INDEX                   1

// KEY EVENT
#define KEY_FALLING_EVENT               0x00
#define KEY_INVER_EVENT                 0x01
#define KEY_RISING_EVENT                0xFF

// KEY STATE BITMAP
#define DFT_KEY_MODE_Bit                0x0001      // 0: 默认功能，1: 由用户选择数据输出对象
#define KEY_STATE_to_UART_Bit           0x0002      // 1: 对串口输出
#define KEY_LOCK_Bit                    0x0004      // 1: 按键自锁
#define KEY_SHAKE_Bit                   0x0008      // 1: 按键模拟抖动
#define KEY_POLARITY_Bit                0x0010      // 1: 按键极性：按下为1
#define KEY_STATE_to_GPIO0_Bit          0x0020      // 1: 向GPIO0输出
#define KEY_STATE_to_GPIO1_Bit          0x0040      // 1: 向GPIO1输出
#define KEY_STATE_to_GPIO2_Bit          0x0080      // 1: 向GPIO2输出
#define KEY_STATE_to_GPIO3_Bit          0x0100      // 1: 向GPIO3输出
#define KEY_STATE_to_GPIO14_Bit         0x0200      // 1: 向GPIO14输出
#define KEY_STATE_to_GPIO15_Bit         0x0400      // 1: 向GPIO15输出
#define KEY_STATE_to_QL_Pin_Bit         0x0800      // 1: 向QL Pin输出

// LED STATE BITMAP
#define DFT_LED_MODE_Bit                0x0001      // 0: 默认功能，1: 由用户选择数据输入对象
#define LED_POLARITY_Bit                0x0002      // 1: 模拟上拉小灯，输入0 = 点亮
#define LED_STATE_from_UART_Bit         0x0004      // 1: 由串口、USB、BLE控制
#define LED_STATE_from_GPIO0_Bit        0x0008      // 1: 由GPIO0控制
#define LED_STATE_from_GPIO1_Bit        0x0010      // 1: 由GPIO1控制
#define LED_STATE_from_GPIO2_Bit        0x0020      // 1: 由GPIO2控制
#define LED_STATE_from_GPIO3_Bit        0x0040      // 1: 由GPIO3控制
#define LED_STATE_from_GPIO14_Bit       0x0080      // 1: 由GPIO14控制
#define LED_STATE_from_GPIO15_Bit       0x0100      // 1: 由GPIO15控制
#define LED_STATE_from_QL_Pin_Bit       0x0200      // 1: 由QL Pin控制

// LED IN DATA BitMap
#define GPIO0_DATA_Bit                  0x02
#define GPIO1_DATA_Bit                  0x04
#define GPIO2_DATA_Bit                  0x08
#define GPIO3_DATA_Bit                  0x10
#define GPIO14_DATA_Bit                 0x20
#define GPIO15_DATA_Bit                 0x40
#define QL_Pin_DATA_Bit                 0x80

// LED Blink Mode
#define LED_MODE_EN_Bit                 0x80
#define LED_not_Blink_MODE              0
#define LED_Blink_05tps_MODE            1
#define LED_Blink_10tps_MODE            2

// DFT_SYS_LED_STATE
#define LED_ON                          0x80
#define LED_OFF                         0x00
#define LED_05tps                       0x81
#define LED_10tps                       0x82

// LED STATE BITMAP MASK
#define LED_DATA_MASK                   0x03FC      //11 1111 1100


#define TIM2_value          (uint16_t)(R32_TMR2_COUNT >>= 16)

extern app_drv_fifo_t task_list_fifo;
extern uint8_t APP_TaskID;
extern uint8_t DFT_SYS_LED_STATE;   // SYS_STATE_Flashing
extern volatile uint16_t LED_STATE_BITMAP[2];
extern volatile uint16_t KEY_STATE_BITMAP[2];

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


#endif // __APP_H__
