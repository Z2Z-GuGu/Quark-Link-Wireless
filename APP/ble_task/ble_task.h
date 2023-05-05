/**********************************************************************************
 * This code is modified from WCH CH573F_EVT/EXAM/BLE/BLE_UART/APP/include/peripheral.h
 * 
 * @name        ble_task.h
 * @version     R1.0
 * @brief       ble task
 * @author      Z2Z GuGu
 * @date        2023/03/05
 * @code        GB2312
 **********************************************************************************/

#ifndef __BLE_TASK_H__
#define __BLE_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stdint.h"
#include "HAL.h"
#include "projconfig.h"
#include "config.h"
#include "app_drv_fifo.h"
#include "uart_task.h"
#include "Application.h"


// ble_uart_service
#define SERVAPP_NUM_ATTR_SUPPORTED          7
#define SETTING_TX_VALUE_HANDLE             2
#define SETTING_RX_VALUE_HANDLE             2
#define UART_TX_VALUE_HANDLE                5
#define UART_RX_VALUE_HANDLE                5
#define BLE_UART_RX_BUFF_SIZE               1

// BLE Task Events
#define SBP_START_DEVICE_EVT                0x0001

#define SBP_READ_RSSI_EVT                   0x0004
#define SBP_PARAM_UPDATE_EVT                0x0008
#define UART_TO_BLE_SEND_EVT                0x0010

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD             1600

// How often to perform read rssi event
#define SBP_READ_RSSI_EVT_PERIOD            3200

// Parameter update delay
#define SBP_PARAM_UPDATE_DELAY              6400

// What is the advertising interval when device is discoverable (units of 625us, 80=50ms)
#define DEFAULT_ADVERTISING_INTERVAL        160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE           GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 10=12.5ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL   8

// Maximum connection interval (units of 1.25ms, 100=125ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL   20

// Slave latency to use parameter update
#define DEFAULT_DESIRED_SLAVE_LATENCY       0

// Supervision timeout value (units of 10ms, 100=1s)
#define DEFAULT_DESIRED_CONN_TIMEOUT        100

// Company Identifier: WCH
#define WCH_COMPANY_ID                      0x07D7

// GATTPROFILE_H:
#define SIMPLEPROFILE_SERV_UUID             0xFFE0

extern app_drv_fifo_t ble_tx_fifo;

// 158
typedef enum
{
    SEND_TO_BLE_TO_SEND = 1,
    SEND_TO_BLE_ALLOC_FAILED,
    SEND_TO_BLE_SEND_FAILED,
} send_to_ble_state_t;

typedef enum
{
    BLE_UART_EVT_TX_NOTI_DISABLED = 1,
    BLE_UART_EVT_TX_NOTI_ENABLED,
    BLE_UART_EVT_BLE_DATA_RECIEVED,
} ble_uart_evt_type_t;

typedef enum
{
    BLE_SETTING_CHANNEL = 1,
    BLE_UART_CHANNEL,
} ble_channel_type_t;

typedef struct
{
    uint16 connHandle; // Connection handle of current connection
    uint16 connInterval;
    uint16 connSlaveLatency;
    uint16 connTimeout;
} peripheralConnItem_t;

typedef struct
{
    uint8_t const *p_data; /**< A pointer to the buffer with received data. */
    uint16_t       length; /**< Length of received data. */
} ble_uart_evt_rx_data_t;

typedef struct
{
    ble_uart_evt_type_t    type;
    ble_uart_evt_rx_data_t data;
} ble_uart_evt_t;

typedef void (*ble_uart_ProfileChangeCB_t)(uint16_t connection_handle, ble_uart_evt_t *p_evt, ble_channel_type_t BLE_Channel);

extern uint8 BLE_TaskID;
void ble_fifo_init();
void ble_fifo_cheak_process(void);
void ble_task_Init(void);

#ifdef __cplusplus
}
#endif


#endif // __BLE_TASK_H__
