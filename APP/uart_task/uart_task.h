/**********************************************************************************
 * @name        uart_task.h
 * @version     R1.0
 * @brief       uart task
 * @author      Z2Z GuGu
 * @date        2023/03/07
 * @code        GB2312
 **********************************************************************************/

#ifndef __UART_TASK_H__
#define __UART_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projconfig.h"
#include "config.h"
#include "HAL.h"
#include "app_drv_fifo.h"
#include "Application.h"
#include "Storage.h"
#include "boardbase.h"
#include "usb_task.h"

// UART Task Events
#define UART0_RX_NEW_FRAME_EVT      0x0001
#define UART0_TX_NEW_FRAME_EVT      0x0002
#define UART0_RX_TIMEOUT_EVT        0x0004

// UART LCR
#define UART_WORD_SZ_MASK           0x03
#define UART_WORD_SZ_5bit           0x00
#define UART_WORD_SZ_6bit           0x01
#define UART_WORD_SZ_7bit           0x02
#define UART_WORD_SZ_8bit           0x03
#define UART_STOP_BIT_MASK          0x04
#define UART_STOP_1bit              0x00
#define UART_STOP_2bit              0x04
#define UART_PAR_MOD_MASK           0x30
#define UART_PAR_MOD_MASK           0x30
#define UART_ODD_PARITY             0x00    //奇校验
#define UART_EVEN_PARITY            0x10    //偶校验
#define UART_MARK_PARITY            0x20    //置1 mark
#define UART_SPACE_PARITY           0x30    //空白位 space

extern uint8_t EP1_T_Buf0_Len;
extern uint8_t EP1_T_Buf1_Len;
extern uint8_t UART_TaskID;

void UART0_Init(uint32_t UART0_bps);
void UART_RX_TimeOUT_Init(uint32_t t);
void uart_task_Init();
void uart_fifo_init();
void uart_fifo_cheak_process();
void uart_auto_send(void);

#ifdef __cplusplus
}
#endif


#endif // __UART_TASK_H__
