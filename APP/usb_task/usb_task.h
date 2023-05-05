/**********************************************************************************
 * @name        usb_task.h
 * @version     R3.0
 * @brief       usb task
 * @author      Z2Z GuGu
 * @date        2023/03/06
 * @code        GB2312
 **********************************************************************************/

#ifndef __USB_TASK_H__
#define __USB_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "CONFIG.h"
#include "HAL.h"
#include "app_drv_fifo.h"
#include "Application.h"
#include "projconfig.h"
#include "boardbase.h"
#include "uart_task.h"

// UART Task Events
#define USB_RX_NEW_FRAME_EVT            0x0001
#define USB_TX_NEW_FRAME_EVT            0x0002
#define USB_RX_TIMEOUT_EVT              0x0004

#define THIS_ENDP0_SIZE                 64
#define MAX_PACKET_SIZE                 64
#define MAX_IN_PACKET_SIZE              63
#define UARTR_Ring_SIZE                 128

// EPn Buffer State BitMap:
#define R_Data0_State_Bit               0x01
#define R_Data1_State_Bit               0x02
#define R_Datax_Busy_Bit                0x04
#define R_Data_EN_Bit                   0x08
#define T_Data0_State_Bit               0x10
#define T_Data1_State_Bit               0x20
#define T_Datax_Busy_Bit                0x40
#define T_Data_EN_Bit                   0x80
// EPn Buffer State:
/* R_Ready means this DATA group can be sent to UART TX FIFO
 * R_Busy means this DATA group is being sent to UART TX
 * T_Ready means this DATA group can be sent by USB EPn
 * T_Busy means this DATA group is being sent by USB EPn  */
#define R_Data0_Ready                   0x01
#define R_Data1_Ready                   0x02
#define R_Data1_Busy                    0x04
#define R_Data_EN                       0x08
#define T_Data0_Ready                   0x10
#define T_Data1_Ready                   0x20
#define T_Data1_Busy                    0x40
#define T_Data_EN                       0x80

// COMx Index
#define COM0_indexL                     0x00
#define COM1_indexL                     0x02

// LINE_CODING_STATE
#define COM0_NEW_Data_Bit               0x01
#define COM1_NEW_Data_Bit               0x02

#define COM0_NEW_Data                   0x01
#define COM1_NEW_Data                   0x02

// CONTROL_LINE_STATE
#define EP0_DTR_Bit                     0x01
#define EP0_RTS_Bit                     0x02

#define COM_NEW_DTR_Bit                 0x01
#define COM_DTR_Bit                     0x02
#define COM_DTR_NEW_Edge_Bit            0x04
#define COM_DTR_Rising_Bit              0x08
#define COM_NEW_RTS_Bit                 0x10
#define COM_RTS_Bit                     0x20
#define COM_RTS_NEW_Edge_Bit            0x40
#define COM_RTS_Rising_Bit              0x80

#define COM_NEW_DTR                     0x01
#define COM_DTR_H                       0x02
#define COM_DTR_NEW_Edge                0x04
#define COM_DTR_Rising                  0x08
#define COM_NEW_RTS                     0x10
#define COM_RTS_H                       0x20
#define COM_RTS_NEW_Edge                0x40
#define COM_RTS_Rising                  0x80

/* endpoints enumeration 只用过ENDP0 */
#define ENDP0                           0x00
#define ENDP1                           0x01
#define ENDP2                           0x02
#define ENDP3                           0x03
#define ENDP4                           0x04

// EPnBUFFER 偏移量
#define Rev_Data0_Offset                0
#define Rev_Data1_Offset                64
#define Send_Data0_Offset               128
#define Send_Data1_Offset               192

/* ENDP x Type */
#define ENDP_TYPE_IN                    0x00    /* ENDP is IN Type */
#define ENDP_TYPE_OUT                   0x01    /* ENDP is OUT Type */

/* USB设备各种标志位定义 */
#define DEF_BIT_USB_RESET               0x01    /* 总线复位标志 */
#define DEF_BIT_USB_DEV_DESC            0x02    /* 获取过设备描述符标志 */
#define DEF_BIT_USB_ADDRESS             0x04    /* 设置过地址标志 */
#define DEF_BIT_USB_CFG_DESC            0x08    /* 获取过配置描述符标志 */
#define DEF_BIT_USB_SET_CFG             0x10    /* 设置过配置值标志 */
#define DEF_BIT_USB_WAKE                0x20    /* USB唤醒标志 */
#define DEF_BIT_USB_SUPD                0x40    /* USB总线挂起标志 */
#define DEF_BIT_USB_HS                  0x80    /* USB高速、全速标志 */

/* Stop Bits */
#define HAL_UART_ONE_STOP_BIT           0
#define HAL_UART_1_5_STOP_BIT           1
#define HAL_UART_TWO_STOP_BITS          2

/* Parity settings */
#define HAL_UART_NO_PARITY              0x00    //无校验
#define HAL_UART_ODD_PARITY             0x01    //奇校验
#define HAL_UART_EVEN_PARITY            0x02    //偶校验
#define HAL_UART_MARK_PARITY            0x03    //置1 mark
#define HAL_UART_SPACE_PARITY           0x04    //空白位 space

/* Character Size */
#define HAL_UART_5_BITS_PER_CHAR        5
#define HAL_UART_6_BITS_PER_CHAR        6
#define HAL_UART_7_BITS_PER_CHAR        7
#define HAL_UART_8_BITS_PER_CHAR        8

/* 类请求 */
//  3.1 Requests---Abstract Control Model
#define DEF_SEND_ENCAPSULATED_COMMAND   0x00
#define DEF_GET_ENCAPSULATED_RESPONSE   0x01
#define DEF_SET_COMM_FEATURE            0x02
#define DEF_GET_COMM_FEATURE            0x03
#define DEF_CLEAR_COMM_FEATURE          0x04
#define DEF_SET_LINE_CODING             0x20   // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING             0x21   // This request allows the host to find out the currently configured line coding.
#define DEF_SET_CONTROL_LINE_STATE      0x22
#define DEF_SEND_BREAK                  0x23

//  3.2 Notifications---Abstract Control Model
#define DEF_NETWORK_CONNECTION          0x00
#define DEF_RESPONSE_AVAILABLE          0x01
#define DEF_SERIAL_STATE                0x20

/* 保存USB中断的状态 ->改成几组的操作方式 */
#define USB_IRQ_FLAG_NUM                1

// __attribute__((aligned(4))) extern uint8_t EP1_Databuf;
extern volatile uint8_t EP1_Buf_State;
extern uint8_t EP1_R_Buf0_Len;
extern uint8_t EP1_R_Buf1_Len;
extern uint8_t Frame_loss_mark;
extern uint8_t EP1_R_Buf_Len;
extern uint8_t EP1_TEMP_Buf_Ready;
extern uint8_t EP1_IN_Busy;
extern uint8_t EP1_IN_Tail;
extern uint8_t EP1_IN_Tail_LEN;
extern uint8_t *pUSB_TEMP_BUF;
extern uint8_t *pEP1_TEMP_TX_BUF;

//USB_SETUP_REQ_t
#define UsbSetupBuf                     ((USB_SETUP_REQ_t *)EP0_Databuf)

typedef struct _USB_SETUP_REQ_
{
    uint8_t bRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint8_t wIndexL;
    uint8_t wIndexH;
    uint8_t wLengthL;
    uint8_t wLengthH;
} USB_SETUP_REQ_t;

typedef struct DevInfo
{
  uint8_t UsbConfig;      // USB配置标志
  uint8_t UsbAddress;     // USB设备地址
  uint8_t gSetupReq;      // USB控制传输命令码
  uint8_t gSetupLen;      // USB控制传输传输长度
  uint8_t gUsbInterCfg;   // USB设备接口配置
  uint8_t gUsbFlag;       // USB设备各种操作标志,位0=总线复位,位1=获取设备描述符,位2=设置地址,位3=获取配置描述符,位4=设置配置
}DevInfo_Parm;


void InitCDCDevice(void);
void usb_auto_send(void);

#ifdef __cplusplus
}
#endif


#endif // __USB_TASK_H__
