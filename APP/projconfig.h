/**********************************
 *  @name       projconfig.h
 *  @version    R1.0
 *  @brief      project config
 *  @author     Z2Z GuGu
 *  @date       2023/03/05
 *  @code       GB2312
 *********************************/

#ifndef __PROJ_CONFIG_H__
#define __PROJ_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_MAC                     FALSE
#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
#define MY_BLE_MAC {0x10, 0x00, 0x00, 0x00, 0x00, 0x00}
#endif

// #define GPIO_DEBUG

#define UART0_default_bps           115200

#define USB_Pipeline_Enable         1
#define BLE_Pipeline_Enable         1
#define UART_Pipeline_Enable        1

#define USB_Setting_Enable          1
#define BLE_Setting_Enable          1
#define UART_Setting_Enable         1

#define Task_List_LENGTH            8U

#define BLE_TX_BUFFER_LENGTH        256U
#define BLE_RX_BUFFER_LENGTH        256U
#define BLE_E_TX_BUFFER_LENGTH      128U
#define BLE_E_RX_BUFFER_LENGTH      128U

#define USB_TX_BUFFER_LENGTH        512U
#define USB_RX_BUFFER_LENGTH        256U

#define UART_TX_BUFFER_LENGTH       512U
#define UART_RX_BUFFER_LENGTH       256U


#ifdef __cplusplus
}
#endif


#endif // __PROJ_CONFIG_H__
