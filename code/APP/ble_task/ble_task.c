/**********************************************************************************
 * This code is modified from WCH CH573F_EVT/EXAM/BLE/BLE_UART/APP/peripheral.c
 * 
 * @name        ble_task.c
 * @version     R1.0
 * @brief       ble task
 * @author      Z2Z GuGu
 * @date        2023/03/05
 * @code        GB2312
 **********************************************************************************/

#include "ble_task.h"

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
u8C MacAddr[6] = MY_BLE_MAC;
#endif

// BLE LIB RAM
__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

// Task ID for internal task/event processing
uint8 BLE_TaskID = INVALID_TASK_ID;

static uint8_t to_be_send_buf[BLE_BUFF_MAX_LEN - 4 - 3];

static uint8_t ble_tx_buffer[BLE_TX_BUFFER_LENGTH] = {0};
static uint8_t ble_rx_buffer[BLE_RX_BUFFER_LENGTH] = {0};
static uint8_t ble_e_tx_buffer[BLE_E_TX_BUFFER_LENGTH] = {0};
static uint8_t ble_e_rx_buffer[BLE_E_RX_BUFFER_LENGTH] = {0};

app_drv_fifo_t ble_tx_fifo;
app_drv_fifo_t ble_rx_fifo;
app_drv_fifo_t ble_e_tx_fifo;
app_drv_fifo_t ble_e_rx_fifo;

extern app_drv_fifo_t uart_tx_fifo;
extern app_drv_fifo_t ble_tx_fifo;
extern app_drv_fifo_t usb_tx_fifo;

//fifo length less that MTU-3, retry times
uint32_t uart_to_ble_send_evt_cnt = 0;

send_to_ble_state_t send_to_ble_state = SEND_TO_BLE_TO_SEND;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] = {
    // complete name
    0x07, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'Q', 'L', 'W', '-', 'R', '1',
    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is best kept short to conserve power while advertisting)
static uint8 advertData[] = {
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)

    11,
    GAP_ADTYPE_MANUFACTURER_SPECIFIC,
    
    LO_UINT16(WCH_COMPANY_ID),
    HI_UINT16(WCH_COMPANY_ID),
    0x88, 0xa0,
    // mac addr
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,                  // length of this data
    GAP_ADTYPE_16BIT_MORE, // some of the UUID's, but not all
    LO_UINT16(SIMPLEPROFILE_SERV_UUID),
    HI_UINT16(SIMPLEPROFILE_SERV_UUID)};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "S-Quark mini2 R3";

// Connection item list
static peripheralConnItem_t BLE_ConnList;

// ble_uart GATT Profile Service UUID
const uint8_t ble_uart_ServiceUUID[ATT_BT_UUID_SIZE] =
    {0xe0, 0xff};

// Characteristic uuid
const uint8_t ble_setting_CharUUID[ATT_BT_UUID_SIZE] =
    {0xe1, 0xff};

const uint8_t ble_uart_CharUUID[ATT_BT_UUID_SIZE] =
    {0xe2, 0xff};

// Profile Service attribute
static const gattAttrType_t ble_uart_Service = {ATT_BT_UUID_SIZE, ble_uart_ServiceUUID};

// Profile Characteristic 2 Properties
static uint8 ble_setting_TxCharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE;
static uint8 ble_uart_TxCharProps    = GATT_PROP_NOTIFY | GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE;

// Characteristic 2 Value
static uint8 ble_setting_TxCharValue = 0;
static uint8 ble_uart_TxCharValue    = 0;

// Simple Profile Characteristic 2 User Description
static gattCharCfg_t BLE_TxCCCD[4];

static gattAttribute_t ble_uart_ProfileAttrTbl[] = {
    // Simple Profile Service
    {
        {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
        GATT_PERMIT_READ,                       /* permissions */
        0,                                      /* handle */
        (uint8 *)&ble_uart_Service              /* pValue */
    },

    // Characteristic 1 Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &ble_setting_TxCharProps},

    // Characteristic Value 1
    {
        {ATT_BT_UUID_SIZE, ble_setting_CharUUID},
        GATT_PERMIT_WRITE,
        0,
        (uint8 *)&ble_setting_TxCharValue},
        
    // Characteristic 1 User Description
    {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)BLE_TxCCCD},

    // Characteristic 2 Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &ble_uart_TxCharProps},

    // Characteristic Value 2
    {
        {ATT_BT_UUID_SIZE, ble_uart_CharUUID},
        GATT_PERMIT_WRITE,
        0,
        (uint8 *)&ble_uart_TxCharValue},

    // Characteristic User Description
    {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)BLE_TxCCCD},
};

ble_channel_type_t BLE_TX_Channel;

// LOCAL FUNCTIONS
static void ProcessTMOSMsg(tmos_event_hdr_t *pMsg);

static void BLE_StateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);
static void BLE_RssiCB(uint16 connHandle, int8 rssi);
static void BLE_ParamUpdateCB(uint16 connHandle, uint16 connInterval, uint16 connSlaveLatency, uint16 connTimeout);

static bStatus_t ble_uart_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset, uint16 maxLen, uint8 method);
static bStatus_t ble_uart_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                      uint8 *pValue, uint16 len, uint16 offset, uint8 method);
static void ble_uart_HandleConnStatusCB(uint16 connHandle, uint8 changeType);


// PROFILE CALLBACKS
// GAP Role Callbacks
static gapRolesCBs_t GAP_Role_CBs = {
    BLE_StateNotificationCB, // Profile State Change Callbacks
    BLE_RssiCB,              // When a valid RSSI is read from controller (not used by application)
    BLE_ParamUpdateCB};

// Broadcast Callbacks
static gapRolesBroadcasterCBs_t BroadcasterCBs = {
    NULL, // Not used in peripheral role
    NULL  // Receive scan request callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t GAP_BondMgrCBs = {
    NULL, // Passcode callback (not used by application)
    NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple Profile Service Callbacks
gattServiceCBs_t ble_uart_ProfileCBs = {
    ble_uart_ReadAttrCB,  // Read callback function pointer
    ble_uart_WriteAttrCB, // Write callback function pointer
    NULL                  // Authorization callback function pointer
};

//ble uart service callback handler
void BLE_Setting_RX_CB(uint16_t connection_handle, ble_uart_evt_t *p_evt, ble_channel_type_t BLE_Channel)
{
    // uint8_t TEMP;
    // char JSON_STR[128];
    switch(BLE_Channel)
    {
        case BLE_SETTING_CHANNEL:
            switch(p_evt->type)
            {
                case BLE_UART_EVT_TX_NOTI_DISABLED:
                    PRINT("%02x:bleuart_EVT_TX_NOTI_DISABLED\r\n", connection_handle);
                    break;
                case BLE_UART_EVT_TX_NOTI_ENABLED:
                    PRINT("%02x:bleuart_EVT_TX_NOTI_ENABLED\r\n", connection_handle);
                    break;
                case BLE_UART_EVT_BLE_DATA_RECIEVED:
                    PRINT("BLE RX DATA len:%d\r\n", p_evt->data.length);
                    uint16_t to_write_length = p_evt->data.length;
                    // app_drv_fifo_write(&ble_e_rx_fifo, (uint8_t *)p_evt->data.p_data, &to_write_length);
                    app_drv_fifo_write(&ble_e_tx_fifo, (uint8_t *)p_evt->data.p_data, &to_write_length);
                    // BLE_TX_Channel = SETTING_TX_VALUE_HANDLE;
                    // tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                    // uint16_t write_length = p_evt->data.length;
                    // app_drv_fifo_write(&ble_e_tx_fifo, (uint8_t *)p_evt->data.p_data, &write_length);
                    break;
                default:
                    break;
            }
        break;
        case BLE_UART_CHANNEL:
            switch(p_evt->type)
            {
                case BLE_UART_EVT_TX_NOTI_DISABLED:
                    PRINT("%02x:bleuart_EVT_TX_NOTI_DISABLED\r\n", connection_handle);
                    break;
                case BLE_UART_EVT_TX_NOTI_ENABLED:
                    PRINT("%02x:bleuart_EVT_TX_NOTI_ENABLED\r\n", connection_handle);
                    break;
                case BLE_UART_EVT_BLE_DATA_RECIEVED:
                    PRINT("BLE RX DATA len:%d\r\n", p_evt->data.length);
                    uint16_t to_write_length = p_evt->data.length;
                    // app_drv_fifo_write(&ble_rx_fifo, (uint8_t *)p_evt->data.p_data, &to_write_length);
                    app_drv_fifo_write(&ble_tx_fifo, (uint8_t *)p_evt->data.p_data, &to_write_length);
                    break;
                default:
                    break;
            }
        break;
    }
}

/*********************************************************************
 * @fn          ble_uart_ReadAttrCB
 *
 * @brief       Read an attribute.
 *              UART RX DATA
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static bStatus_t ble_uart_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset, uint16 maxLen, uint8 method)
{
    bStatus_t status = SUCCESS;
    PRINT("ReadAttrCB\n");
    // If attribute permissions require authorization to read, return error
    if(gattPermitAuthorRead(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if(pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        if(uuid == GATT_CLIENT_CHAR_CFG_UUID)
        // if(pAttr->type.uuid[0] == ble_setting_CharUUID[0] && pAttr->type.uuid[1] == ble_setting_CharUUID[1])
        {
            *pLen = 2;
            tmos_memcpy(pValue, pAttr->pValue, 2);
        }
    }
    return (status);
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *          UART TX DATA
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t ble_uart_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                      uint8 *pValue, uint16 len, uint16 offset, uint8 method)
{
    bStatus_t status = SUCCESS;
    //uint8 notifyApp = 0xFF;
    // If attribute permissions require authorization to write, return error
    if(gattPermitAuthorWrite(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    if(pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        if(uuid == GATT_CLIENT_CHAR_CFG_UUID)
        {
            status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                    offset, GATT_CLIENT_CFG_NOTIFY);
            if(status == SUCCESS)
            {
                uint16         charCfg = BUILD_UINT16(pValue[0], pValue[1]);
                ble_uart_evt_t evt;

                //PRINT("CCCD set: [%d]\n", charCfg);
                evt.type = (charCfg == GATT_CFG_NO_OPERATION) ? BLE_UART_EVT_TX_NOTI_DISABLED : BLE_UART_EVT_TX_NOTI_ENABLED;
                BLE_Setting_RX_CB(connHandle, &evt, BLE_SETTING_CHANNEL);
            }
        }

        // Setting Characteristic
        if(pAttr->handle == ble_uart_ProfileAttrTbl[SETTING_RX_VALUE_HANDLE].handle)
        {
            ble_uart_evt_t evt;
            evt.type = BLE_UART_EVT_BLE_DATA_RECIEVED;
            evt.data.length = (uint16_t)len;
            evt.data.p_data = pValue;
            BLE_Setting_RX_CB(connHandle, &evt, BLE_SETTING_CHANNEL);
        }
        // UART Characteristic
        if(pAttr->handle == ble_uart_ProfileAttrTbl[UART_RX_VALUE_HANDLE].handle)
        {
            ble_uart_evt_t evt;
            evt.type = BLE_UART_EVT_BLE_DATA_RECIEVED;
            evt.data.length = (uint16_t)len;
            evt.data.p_data = pValue;
            BLE_Setting_RX_CB(connHandle, &evt, BLE_UART_CHANNEL);
        }
    }
    return (status);
}

uint8_t ble_uart_notify_is_ready(uint16 connHandle)
{
    return (GATT_CLIENT_CFG_NOTIFY == GATTServApp_ReadCharCfg(connHandle, BLE_TxCCCD));
}

/*********************************************************************
 * @fn          BloodPressure_IMeasNotify
 *
 * @brief       Send a notification containing a bloodPressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t ble_uart_notify(uint16 connHandle, attHandleValueNoti_t *pNoti, ble_channel_type_t BLE_Channel)
{
    //uint16 value = BLE_TxCCCD[0].value;
    uint16 value = GATTServApp_ReadCharCfg(connHandle, BLE_TxCCCD);

    // If notifications enabled
    if(value & GATT_CLIENT_CFG_NOTIFY)
    {
        // Set the handle
        switch(BLE_Channel)
        {
            case BLE_SETTING_CHANNEL:
                pNoti->handle = ble_uart_ProfileAttrTbl[SETTING_TX_VALUE_HANDLE].handle;
            break;
            case BLE_UART_CHANNEL:
                pNoti->handle = ble_uart_ProfileAttrTbl[UART_TX_VALUE_HANDLE].handle;
            break;
        }
        

        // Send the Indication
        return GATT_Notification(connHandle, pNoti, FALSE);
    }
    return bleIncorrectMode;
}

/*********************************************************************
 * @fn          simpleProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void ble_uart_HandleConnStatusCB(uint16 connHandle, uint8 changeType)
{
    // Make sure this is not loopback connection
    if(connHandle != LOOPBACK_CONNHANDLE)
    {
        // Reset Client Char Config if connection has dropped
        if((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
           ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) &&
            (!linkDB_Up(connHandle))))
        {
            //BLE_TxCCCD[0].value = 0;
            GATTServApp_InitCharCfg(connHandle, BLE_TxCCCD);

            // switch(BLE_Channel)
            // {
            //     case BLE_SETTING_CHANNEL:
            //         GATTServApp_InitCharCfg(connHandle, BLE_SETTING_TxCCCD);
            //     break;
            //     case BLE_UART_CHANNEL:
            //         GATTServApp_InitCharCfg(connHandle, BLE_TxCCCD);
            //     break;
            // }
            //PRINT("clear client configuration\n");
        }
    }
}

/*********************************************************************
 * @fn      ble_uart_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t ble_uart_add_service()
{
    uint8 status = SUCCESS;

    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, BLE_TxCCCD);
    // Register with Link DB to receive link status change callback
    linkDB_Register(ble_uart_HandleConnStatusCB);

    //    BLE_TxCCCD.connHandle = INVALID_CONNHANDLE;
    //    BLE_TxCCCD.value = 0;
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(ble_uart_ProfileAttrTbl,
                                         GATT_NUM_ATTRS(ble_uart_ProfileAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &ble_uart_ProfileCBs);
    if(status != SUCCESS)
        PRINT("Add ble uart service failed!\n");

    return (status);
}

/*********************************************************************
 * @fn      LinkEstablished
 *
 * @brief   Process link established.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void LinkEstablished(gapRoleEvent_t *pEvent)
{
    gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

    // See if already connected
    if(BLE_ConnList.connHandle != GAP_CONNHANDLE_INIT)
    {
        GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
        PRINT("Connection max...\n");
    }
    else
    {
        BLE_ConnList.connHandle = event->connectionHandle;
        BLE_ConnList.connInterval = event->connInterval;
        BLE_ConnList.connSlaveLatency = event->connLatency;
        BLE_ConnList.connTimeout = event->connTimeout;

        // Set timer for param update event
        tmos_start_task(BLE_TaskID, SBP_PARAM_UPDATE_EVT, SBP_PARAM_UPDATE_DELAY);

        PRINT("Conn %x - Int %x \n", event->connectionHandle, event->connInterval);
    }
}

/*********************************************************************
 * @fn      BLE_LinkTerminated
 *
 * @brief   Process link terminated.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void BLE_LinkTerminated(gapRoleEvent_t *pEvent)
{
    gapTerminateLinkEvent_t *event = (gapTerminateLinkEvent_t *)pEvent;

    if(event->connectionHandle == BLE_ConnList.connHandle)
    {
        BLE_ConnList.connHandle = GAP_CONNHANDLE_INIT;
        BLE_ConnList.connInterval = 0;
        BLE_ConnList.connSlaveLatency = 0;
        BLE_ConnList.connTimeout = 0;

        // Restart advertising
        {
            uint8 advertising_enable = TRUE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertising_enable);
        }
    }
    else
    {
        PRINT("ERR..\n");
    }
}

/*********************************************************************
 * @fn      BLE_StateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void BLE_StateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState)
    {
        case GAPROLE_STARTED:
            PRINT("Initialized..\n");
            break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                BLE_LinkTerminated(pEvent);
            }
            PRINT("Advertising..\n");
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                LinkEstablished(pEvent);
                PRINT("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            PRINT("Connected Advertising..\n");
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                BLE_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                if(pEvent->gap.hdr.status != SUCCESS)
                {
                    PRINT("Waiting for advertising..\n");
                }
                else
                {
                    PRINT("Error..\n");
                }
            }
            else
            {
                PRINT("Error..%x\n", pEvent->gap.opcode);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error..\n");
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      BLE_RssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void BLE_RssiCB(uint16 connHandle, int8 rssi)
{
    PRINT("RSSI -%d dB Conn  %x \n", -rssi, connHandle);
}

/*********************************************************************
 * @fn      BLE_ParamUpdateCB
 *
 * @brief   Parameter update complete callback
 *
 * @param   connHandle - connect handle
 *          connInterval - connect interval
 *          connSlaveLatency - connect slave latency
 *          connTimeout - connect timeout
 *
 * @return  none
 */
static void BLE_ParamUpdateCB(uint16 connHandle, uint16 connInterval,
                                    uint16 connSlaveLatency, uint16 connTimeout)
{
    if(connHandle == BLE_ConnList.connHandle)
    {
        BLE_ConnList.connInterval = connInterval;
        BLE_ConnList.connSlaveLatency = connSlaveLatency;
        BLE_ConnList.connTimeout = connTimeout;

        PRINT("Update %x - Int %x \n", connHandle, connInterval);
    }
    else
    {
        PRINT("BLE_ParamUpdateCB err..\n");
    }
}

/*********************************************************************
 * @fn      BLE_InitConnItem
 *
 * @brief   Init Connection Item
 *
 * @param   BLE_ConnList -
 *
 * @return  NULL
 */
static void BLE_InitConnItem(peripheralConnItem_t *BLE_ConnList)
{
    BLE_ConnList->connHandle = GAP_CONNHANDLE_INIT;
    BLE_ConnList->connInterval = 0;
    BLE_ConnList->connSlaveLatency = 0;
    BLE_ConnList->connTimeout = 0;
}

/*********************************************************************
 * @fn      ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        default:
            break;
    }
}

/*********************************************************************
 * @fn      BLE_ProcessEvent
 *
 * @brief   Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id - The TMOS assigned task ID.
 * @param   events  - events to process.  This is a bit map and can
 *                    contain more than one event.
 *
 * @return  events not processed
 */
uint16 BLE_ProcessEvent(uint8 task_id, uint16 events)
{
    static attHandleValueNoti_t noti;
    // DEBUG_PIN(BLE_Pin, GPIO_HIGH);
    //  VOID task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;

        if((pMsg = tmos_msg_receive(BLE_TaskID)) != NULL)
        {
            ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        // DEBUG_PIN(BLE_Pin, GPIO_LOW);
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & SBP_START_DEVICE_EVT)
    {
        // Start the Device
        GAPRole_PeripheralStartDevice(BLE_TaskID, &GAP_BondMgrCBs, &GAP_Role_CBs);
        // DEBUG_PIN(BLE_Pin, GPIO_LOW);
        return (events ^ SBP_START_DEVICE_EVT);
    }
    if(events & SBP_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        GAPRole_PeripheralConnParamUpdateReq(BLE_ConnList.connHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             BLE_TaskID);

        //        GAPRole_PeripheralConnParamUpdateReq( BLE_ConnList.connHandle,
        //                                              10,
        //                                              20,
        //                                              0,
        //                                              400,
        //                                              BLE_TaskID);

        // DEBUG_PIN(BLE_Pin, GPIO_LOW);
        return (events ^ SBP_PARAM_UPDATE_EVT);
    }

    if(events & UART_TO_BLE_SEND_EVT)
    {
        static uint16_t read_length = 0;
        uint8_t result = 0xff;
        switch(send_to_ble_state)
        {
            case SEND_TO_BLE_TO_SEND:
            {
                switch(BLE_TX_Channel)
                {
                    case BLE_UART_CHANNEL:
                    {
                        //notify is not enabled
                        if(!ble_uart_notify_is_ready(BLE_ConnList.connHandle))
                        {
                            if(BLE_ConnList.connHandle == GAP_CONNHANDLE_INIT)
                            {
                                //connection lost, flush rx fifo here
                                app_drv_fifo_flush(&ble_tx_fifo);
                            }
                            break;
                        }
                        read_length = ATT_GetMTU(BLE_ConnList.connHandle) - 3;

                        if(app_drv_fifo_length(&ble_tx_fifo) >= read_length)
                        {
                            PRINT("FIFO_LEN:%d\r\n", app_drv_fifo_length(&ble_tx_fifo));
                            result = app_drv_fifo_read(&ble_tx_fifo, to_be_send_buf, &read_length);
                            uart_to_ble_send_evt_cnt = 0;
                        }
                        else
                        {
                            if(uart_to_ble_send_evt_cnt > 10)
                            {
                                result = app_drv_fifo_read(&ble_tx_fifo, to_be_send_buf, &read_length);
                                uart_to_ble_send_evt_cnt = 0;
                            }
                            else
                            {
                                tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 4);
                                uart_to_ble_send_evt_cnt++;
                                PRINT("NO TIME OUT\r\n");
                            }
                        }
                    }
                    break;
                    case BLE_SETTING_CHANNEL:
                    {
                        //notify is not enabled
                        if(!ble_uart_notify_is_ready(BLE_ConnList.connHandle))
                        {
                            if(BLE_ConnList.connHandle == GAP_CONNHANDLE_INIT)
                            {
                                //connection lost, flush rx fifo here
                                app_drv_fifo_flush(&ble_e_tx_fifo);
                            }
                            break;
                        }
                        read_length = ATT_GetMTU(BLE_ConnList.connHandle) - 3;

                        if(app_drv_fifo_length(&ble_e_tx_fifo) >= read_length)
                        {
                            PRINT("FIFO_LEN:%d\r\n", app_drv_fifo_length(&ble_e_tx_fifo));
                            result = app_drv_fifo_read(&ble_e_tx_fifo, to_be_send_buf, &read_length);
                            uart_to_ble_send_evt_cnt = 0;
                        }
                        else
                        {
                            if(uart_to_ble_send_evt_cnt > 10)
                            {
                                result = app_drv_fifo_read(&ble_e_tx_fifo, to_be_send_buf, &read_length);
                                uart_to_ble_send_evt_cnt = 0;
                            }
                            else
                            {
                                tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 4);
                                uart_to_ble_send_evt_cnt++;
                                PRINT("NO TIME OUT\r\n");
                            }
                        }
                    }
                    break;
                }

                if(APP_DRV_FIFO_RESULT_SUCCESS == result)
                {
                    noti.len = read_length;
                    noti.pValue = GATT_bm_alloc(BLE_ConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);
                    if(noti.pValue != NULL)
                    {
                        tmos_memcpy(noti.pValue, to_be_send_buf, noti.len);
                        result = ble_uart_notify(BLE_ConnList.connHandle, &noti, BLE_TX_Channel);
                        if(result != SUCCESS)
                        {
                            PRINT("R1:%02x\r\n", result);
                            send_to_ble_state = SEND_TO_BLE_SEND_FAILED;
                            GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
                            tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                        }
                        else
                        {
                            send_to_ble_state = SEND_TO_BLE_TO_SEND;
                            read_length = 0;
                            tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                        }
                    }
                    else
                    {
                        send_to_ble_state = SEND_TO_BLE_ALLOC_FAILED;
                        tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                    }
                }
                else
                {
                    //send_to_ble_state = SEND_TO_BLE_FIFO_EMPTY;
                }
            }    break;
            case SEND_TO_BLE_ALLOC_FAILED:
            case SEND_TO_BLE_SEND_FAILED:
            {
                noti.len = read_length;
                noti.pValue = GATT_bm_alloc(BLE_ConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);
                if(noti.pValue != NULL)
                {
                    tmos_memcpy(noti.pValue, to_be_send_buf, noti.len);
                    result = ble_uart_notify(BLE_ConnList.connHandle, &noti, BLE_TX_Channel);
                    if(result != SUCCESS)
                    {
                        PRINT("R2:%02x\r\n", result);
                        send_to_ble_state = SEND_TO_BLE_SEND_FAILED;
                        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
                        tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                    }
                    else
                    {
                        send_to_ble_state = SEND_TO_BLE_TO_SEND;
                        read_length = 0;
                        tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                    }
                }
                else
                {
                    send_to_ble_state = SEND_TO_BLE_ALLOC_FAILED;
                    tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
                }
            }    break;
            default:
                break;
        }
        
        // DEBUG_PIN(BLE_Pin, GPIO_LOW);
        return (events ^ UART_TO_BLE_SEND_EVT);
    }
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      BLE_Init
 *
 * @brief   Initialization function for the Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void BLE_Init()
{
    uint8_t i;
    BLE_TaskID = TMOS_ProcessEventRegister(BLE_ProcessEvent);

    // Setup the GAP Peripheral Role Profile
    {
        uint8  initial_advertising_enable = TRUE;
        uint16 desired_min_interval = 6;
        uint16 desired_max_interval = 1000;

        
        #if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
            for(i = 0; i < 6; i++)
            {
                advertData[6+i] = MacAddr[i];
            }
        #else
            {
                uint8_t MacAddr[6];
                GetMACAddress(MacAddr);
                for(i = 0; i < 6; i++)
                {
                    advertData[6+i] = MacAddr[i]; // ä½¿ç”¨èŠ?‰‡macåœ°å€
                }
            }
        #endif

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &desired_min_interval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &desired_max_interval);
    }

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Set advertising interval
    {
        uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);
    }

    // Setup the GAP Bond Manager
    {
        uint32 passkey = 0; // passkey "000000"
        uint8  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8  mitm = TRUE;
        uint8  bonding = TRUE;
        uint8  ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8), &bonding);
    }

    // Initialize GATT attributes
    ble_uart_add_service();                     // BLE_Setting_RX_CB
    GGS_AddService(GATT_ALL_SERVICES);          // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);  // GATT attributes

    // Init Connection Item
    BLE_InitConnItem(&BLE_ConnList);

    // Register receive scan request callback
    GAPRole_BroadcasterSetCB(&BroadcasterCBs);

    // Setup a delayed profile startup
    tmos_set_event(BLE_TaskID, SBP_START_DEVICE_EVT);
}

/*********************************************************************
*********************************************************************/
void ble_fifo_cheak_process(void)
{
    if(!app_drv_fifo_is_empty(&ble_tx_fifo))
    {
        BLE_TX_Channel = BLE_UART_CHANNEL;
        tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
    }
    if(!app_drv_fifo_is_empty(&ble_e_tx_fifo))
    {
        BLE_TX_Channel = BLE_SETTING_CHANNEL;
        tmos_start_task(BLE_TaskID, UART_TO_BLE_SEND_EVT, 2);
    }
}

void ble_fifo_init()
{
    //tx fifo and tx fifo
    //The buffer length should be a power of 2
    app_drv_fifo_init(&ble_tx_fifo, ble_tx_buffer, BLE_TX_BUFFER_LENGTH);
    app_drv_fifo_init(&ble_rx_fifo, ble_rx_buffer, BLE_RX_BUFFER_LENGTH);
    app_drv_fifo_init(&ble_e_tx_fifo, ble_e_tx_buffer, BLE_E_TX_BUFFER_LENGTH);
    app_drv_fifo_init(&ble_e_rx_fifo, ble_e_rx_buffer, BLE_E_RX_BUFFER_LENGTH);
}

void ble_task_Init(void)
{
    CH57X_BLEInit();
    HAL_Init();
    GAPRole_PeripheralInit();
    BLE_Init();
}

