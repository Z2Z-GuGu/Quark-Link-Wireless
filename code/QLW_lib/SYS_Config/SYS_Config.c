/***
 *  @name       SYS_Config.c
 *  @brief      ʹ��Json�ַ�����ϵͳ���ܽ������ã��������Blinker
 *  @author     Z2Z-GuGu
 *  @date       2023/2/10
 *  @code       GB2312
 */

#include "SYS_Config.h"

// token�ַ���
jsmntok_t token_pool[16];

const char* blinker_object[] = {
  "get", "btn-123", "btn-7ty", "btn-yoe", "btn-hn0", "joy-3ck", "joy-93l", "ran-e3v", "ran-t5r", "col-5gs"};
const char* blinker_keyword[] = {
  "state", "val", "tap", "on", "off", "press", "pressup"};

const uint8_t SYS_Order_Group_NUM = 9;
const uint8_t Order_Keyword_NUM = 5;
const char* SYS_Order_Group[] = {
  "EN", "SL", "UL", "SK", "UK", "BO", "GT", "ST", "ge"};
const char* Order_Keyword[] = {
  "on", "off", "press", "pressup", "tap", "state"};
const char *SYS_Order_Member[] = {
  "BU", "UB", "SD", "D", "I", "C", "L", "S", "U", "Q", "A", "E", "O", "T", "SS", "SL", "UL", "SK", "UK"};

/****************************************************************************
 * @fn      jsoneq
 * @brief   Json tok �ַ����ȶ�
 * @param   json  json �ַ���
 * @param   tok   jsmn token�� ���ȶ�tokrnλ�� 
 * @param   s     �ȶ� token ֵ
 * @return  0     token ��ͬ
 *          1     token ��ͬ
 ****************************************************************************/
static int jsoneq(char *json, jsmntok_t *tok, const char *s) 
{
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) 
  {
    return 1;
  }
  return 0;
}

/****************************************************************************
 * @fn      jsoneq_l2
 * @brief   Json tok �ַ���ǰ��λ�ȶ�
 * @param   json  json �ַ���
 * @param   tok   jsmn token�� ���ȶ�tokrnλ��
 * @param   s     �ȶ� token ֵ
 * @return  0     token ��ͬ
 *          1     token ��ͬ
 ****************************************************************************/
static int jsoneq_l2(char *json, jsmntok_t *tok, const char *s) 
{
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, 2) == 0) 
  {
    return 1;
  }
  return 0;
}

/****************************************************************************
 * @fn      string_to_uint16
 * @date    2023/2/11
 * @author  Z2Z-gugu
 * @brief   �ַ���ת���֣����ƻ�ĩβԭ����
 * @param   string_start  �ַ�����λ��
 * @param   string_len    �ַ�������
 * @return  ת���������
 ***************************************************************************/
uint16_t string_to_uint16(char *string_start, size_t string_len)
{
  string_start[string_len] = '\0';
  return atoi(string_start);
}

uint8_t SYS_Config_by_Json(char *JSON_string, size_t JSON_len, char *OUTPUT_string, uint8_t *OUTPUT_len)
{
  int token_num;          //r
  int8_t Ctrl_State = -1;
  uint16_t TEMP_Param[2] = {0};
  uint8_t count;          //i
  uint8_t SYS_Order_Group_Index = 0xff;
  jsmn_parser jsmn_p;     //jsmn������

  jsmn_init(&jsmn_p);     // ��ʼ��json������
  token_num = jsmn_parse(&jsmn_p, JSON_string, JSON_len, \
                          token_pool, sizeof(token_pool) / sizeof(token_pool[0]));

  if (token_num < 0) 
  {
    PRINT("Failed to parse JSON: %d\n", token_num);
    return -1;
  }  
  /* Assume the top-level element is an object */
  if (token_num < 1 || token_pool[object_token].type != JSMN_OBJECT) 
  {
    PRINT("Object unexpected\n");
    return -1;
  }

  // token�صڶ�Ԫ������SYS Config Keyword������ȶ�ǰ��λ�ַ���ȷ��SYS Config��
  for (count = 0; count < SYS_Order_Group_NUM; count++)      // object_count
  {
    if (jsoneq_l2(JSON_string, &token_pool[assembly_token], SYS_Order_Group[count]) == 1)
    {
      // ����index
      SYS_Order_Group_Index = count;
      if(count == 8) // {"get":"state"} ����Blinker
      {
        *OUTPUT_len = sprintf(OUTPUT_len, "{\"state\":\"connected\"}\n");
        return 0;
      }
      break;
    }
  }
  if(SYS_Order_Group_Index == 0xff)
  {
    PRINT("Get unknown assembly: %.*s\n", token_pool[assembly_token].end - token_pool[assembly_token].start, JSON_string + token_pool[assembly_token].start);
    return -1;
  }
  // Ctrl_State = ?
  if(token_pool[value_token].type == JSMN_STRING)
  {
    for (count = 0; count < keyword_num; count++)   // keyword_count
    {
      if (jsoneq(JSON_string, &token_pool[value_token], blinker_keyword[count]) == 1)
      {
        // ����keyword index
        blinker_universal_data->blinker_keyword_index = count;
        if(count == 0 || count == 2)  // on/press
          Ctrl_State = 1;
        else if(count == 1 || count == 3)
          Ctrl_State = 0;
        else if(count == 1)
        {
          Ctrl_State = 2; // 
          TEMP_Param[0] = (uint16_t)((JSON_string[10] << 8) | JSON_string[11]);
          TEMP_Param[1] = (uint16_t)((JSON_string[12] << 8) | JSON_string[13]);
        }
        break;
      }
    }
  }
  // ��ʼ�ж�����
  // "EN", "SL", "UL", "SK", "UK", "BO", "GT", "ST"};
  switch(SYS_Order_Group_Index)
  {
    case 0:   //EN
      switch(JSON_string[5])
      {
        case 'B':
          SYS_Param_Config(SYS_STATE_Inex, SYS_BLE_to_UART_EN_Bit, Ctrl_State);
          break;
        case 'U':
          SYS_Param_Config(SYS_STATE_Inex, SYS_UART_to_BLE_EN_Bit, Ctrl_State);
          break;
        case 'F':
          SYS_Param_Config(SYS_STATE_Inex, SYS_GPIO_FREE_MODE_Bit, Ctrl_State);
          break;
        case 'S':
          SYS_Param_Config(SYS_STATE_Inex, SYS_SDIO_MODE_EN_Bit, Ctrl_State);
          break;
      }
      break;
    case 1:   //SL
      switch(JSON_string[5])
      {
        case 'D':
          SYS_Param_Config(SYS_LED_STATE_Inex, DFT_LED_MODE_Bit, Ctrl_State);
          break;
        case 'I':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_POLARITY_Bit, Ctrl_State);
          break;
        case 'C':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_UART_Bit, Ctrl_State);
          break;
        case 'O':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_UART_CTRL_Bit, Ctrl_State);
          break;
        case '0':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_GPIO0_Bit, Ctrl_State);
          break;
        case '1':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_GPIO1_Bit, Ctrl_State);
          break;
        case '2':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_GPIO2_Bit, Ctrl_State);
          break;
        case '3':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_GPIO3_Bit, Ctrl_State);
          break;
        case '4':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_GPIO14_Bit, Ctrl_State);
          break;
        case '5':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_GPIO15_Bit, Ctrl_State);
          break;
        case 'Q':
          SYS_Param_Config(SYS_LED_STATE_Inex, LED_STATE_from_QL_Pin_Bit, Ctrl_State);
          break;
      }
      break;
    case 2:   //UL
      switch(JSON_string[5])
      {
        case 'D':
          SYS_Param_Config(USR_LED_STATE_Inex, DFT_LED_MODE_Bit, Ctrl_State);
          break;
        case 'I':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_POLARITY_Bit, Ctrl_State);
          break;
        case 'C':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_UART_Bit, Ctrl_State);
          break;
        case 'O':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_UART_CTRL_Bit, Ctrl_State);
          break;
        case '0':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_GPIO0_Bit, Ctrl_State);
          break;
        case '1':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_GPIO1_Bit, Ctrl_State);
          break;
        case '2':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_GPIO2_Bit, Ctrl_State);
          break;
        case '3':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_GPIO3_Bit, Ctrl_State);
          break;
        case '4':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_GPIO14_Bit, Ctrl_State);
          break;
        case '5':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_GPIO15_Bit, Ctrl_State);
          break;
        case 'Q':
          SYS_Param_Config(USR_LED_STATE_Inex, LED_STATE_from_QL_Pin_Bit, Ctrl_State);
          break;
      }
      break;
    case 3:   //SK
      switch(JSON_string[5])
      {
        case 'D':
          SYS_Param_Config(SYS_KEY_STATE_Inex, DFT_LED_MODE_Bit, Ctrl_State);
          break;
        case 'L':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_LOCK_Bit, Ctrl_State);
          break;
        case 'I':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_POLARITY_Bit, Ctrl_State);
          break;
        case 'S':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_SHAKE_Bit, Ctrl_State);
          break;
        case 'U':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_UART_Bit, Ctrl_State);
          break;
        case '0':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_GPIO0_Bit, Ctrl_State);
          break;
        case '1':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_GPIO1_Bit, Ctrl_State);
          break;
        case '2':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_GPIO2_Bit, Ctrl_State);
          break;
        case '3':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_GPIO3_Bit, Ctrl_State);
          break;
        case '4':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_GPIO14_Bit, Ctrl_State);
          break;
        case '5':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_GPIO15_Bit, Ctrl_State);
          break;
        case 'Q':
          SYS_Param_Config(SYS_KEY_STATE_Inex, KEY_STATE_to_QL_Pin_Bit, Ctrl_State);
          break;
      }
      break;
    case 4:   //UK
      switch(JSON_string[5])
      {
        case 'D':
          SYS_Param_Config(USR_KEY_STATE_Inex, DFT_LED_MODE_Bit, Ctrl_State);
          break;
        case 'L':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_LOCK_Bit, Ctrl_State);
          break;
        case 'I':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_POLARITY_Bit, Ctrl_State);
          break;
        case 'S':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_SHAKE_Bit, Ctrl_State);
          break;
        case 'U':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_UART_Bit, Ctrl_State);
          break;
        case '0':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_GPIO0_Bit, Ctrl_State);
          break;
        case '1':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_GPIO1_Bit, Ctrl_State);
          break;
        case '2':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_GPIO2_Bit, Ctrl_State);
          break;
        case '3':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_GPIO3_Bit, Ctrl_State);
          break;
        case '4':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_GPIO14_Bit, Ctrl_State);
          break;
        case '5':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_GPIO15_Bit, Ctrl_State);
          break;
        case 'Q':
          SYS_Param_Config(USR_KEY_STATE_Inex, KEY_STATE_to_QL_Pin_Bit, Ctrl_State);
          break;
      }
      break
    case 5:   //BO
      // QLW for ESP32
      break;
    case 6:   //GT
      switch(JSON_string[5])
      {
        case 'S':
          *OUTPUT_len = sprintf(OUTPUT_len, "{\"S\":\"%d\"}\n", SYS_STATE_BITMAP);
          return 0;
        case 'L':
          *OUTPUT_len = sprintf(OUTPUT_len, "{\"L\":\"%d%d\"}\n", LED_STATE_BITMAP[0], LED_STATE_BITMAP[1]);
          return 0;
        case 'K':
          *OUTPUT_len = sprintf(OUTPUT_len, "{\"K\":\"%d%d\"}\n", KEY_STATE_BITMAP[0], KEY_STATE_BITMAP[1]);
          return 0;
      }
      break;
    case 7:   //ST
      switch(JSON_string[5])
      {
        case 'L':
          LED_STATE_BITMAP[0] = TEMP_Param[0];
          LED_STATE_BITMAP[1] = TEMP_Param[1];
          *OUTPUT_len = sprintf(OUTPUT_len, "{\"L\":\"%d%d\"}\n", LED_STATE_BITMAP[0], LED_STATE_BITMAP[1]);
          return 0;
        case 'K':
          KEY_STATE_BITMAP[0] = TEMP_Param[0];
          KEY_STATE_BITMAP[1] = TEMP_Param[1];
          *OUTPUT_len = sprintf(OUTPUT_len, "{\"K\":\"%d%d\"}\n", KEY_STATE_BITMAP[0], KEY_STATE_BITMAP[1]);
          return 0;
      }
      break;
    default: 
      break;
  }
}

/****************************************************************************
 * @fn      blinker_data_analaysis
 * @date    2023/2/11
 * @author  Z2Z-gugu
 * @brief   ��������ַ�������Ϊ��׼blinker���ݽṹ��
 * @param   JSON_string             �ַ�����λ��
 * @param   JSON_len                �ַ�������
 * @param   blinker_universal_data  ��׼blinker���ݽṹ��
 * @return  0   ����
 *          -1  ��������
 ***************************************************************************/
int8_t blinker_data_analaysis(char *JSON_string, size_t JSON_len, blinker_universal_data_t *blinker_universal_data)
{
  int token_num;          //r
  uint8_t count;          //i
  jsmn_parser jsmn_p;     //jsmn������

  blinker_universal_data->blinker_assembly_index = 0xff;
  blinker_universal_data->blinker_keyword_index = 0xff;
  blinker_universal_data->blinker_key_value[0] = 0xffff;
  blinker_universal_data->blinker_key_value[1] = 0xffff;
  blinker_universal_data->blinker_key_value[2] = 0xffff;
  blinker_universal_data->blinker_key_value[3] = 0xffff;

  jsmn_init(&jsmn_p);     // ��ʼ��json������
  token_num = jsmn_parse(&jsmn_p, JSON_string, JSON_len, \
                          token_pool, sizeof(token_pool) / sizeof(token_pool[0]));

  if (token_num < 0) 
  {
    PRINT("Failed to parse JSON: %d\n", token_num);
    return -1;
  }
  
  /* Assume the top-level element is an object */
  if (token_num < 1 || token_pool[object_token].type != JSMN_OBJECT) 
  {
    PRINT("Object unexpected\n");
    return -1;
  }

/*
PRINT("[type][start][end][size]\n");
for(count = 0;count < token_num; count++)
{
  PRINT("[%4d][%5d][%3d][%4d]\n", token_pool[count].type, token_pool[count].start, token_pool[count].end, token_pool[count].size);
}
*/
/*
"{\"JOYKey\":[128,128]}";
[type][start][end][size]
[   1][    0][ 20][   1]
[   3][    2][  8][   1]
[   2][   10][ 19][   2]
[   4][   11][ 14][   0]
[   4][   15][ 18][   0]

"{\"JOYKey\":111}";
[type][start][end][size]
[   1][    0][ 14][   1]
[   3][    2][  8][   1]
[   4][   10][ 13][   0]
*/
  // token�صڶ�Ԫ������blinker_assembly������ȶ�
  if (token_pool[assembly_token].type == JSMN_STRING) 
  {
    for (count = 0; count < assembly_num; count++)      // object_count
    {
      if (jsoneq(JSON_string, &token_pool[assembly_token], blinker_object[count]) == 1)
      {
        // ����index
        blinker_universal_data->blinker_assembly_index = count;
        break;
      }
    }
    if(blinker_universal_data->blinker_assembly_index == 0xff)
    {
      PRINT("Get unknown assembly: %.*s\n", token_pool[assembly_token].end - token_pool[assembly_token].start, JSON_string + token_pool[assembly_token].start);
    }
  }
  else
  {
    PRINT("Assembly unexpected\n");
    return -1;
  }

  switch(token_pool[value_token].type)
  {
    case JSMN_STRING:   //keyword
    {
      for (count = 0; count < keyword_num; count++)   // keyword_count
      {
        if (jsoneq(JSON_string, &token_pool[value_token], blinker_keyword[count]) == 1)
        {
          // ����keyword index
          blinker_universal_data->blinker_keyword_index = count;
          break;
        }
      }
    }
    break;
    case JSMN_ARRAY:    //array
    {
      int j;
      for (j = 0; j < token_pool[value_token].size; j++)
      {
        blinker_universal_data->blinker_key_value[j] =
          string_to_uint16(JSON_string + token_pool[value_token + 1 + j].start,
                          token_pool[value_token + 1 + j].end - token_pool[value_token + 1 + j].start);
      }
    }
    break;
    case JSMN_PRIMITIVE:  //number
    {
      blinker_universal_data->blinker_key_value[0] =
        string_to_uint16(JSON_string + token_pool[value_token].start,
                        token_pool[value_token].end - token_pool[value_token].start);
    }
    break;
    default:
    PRINT("Key unexpected\n");
    return -1;
  }
  return 0;
}
/*
// example
static const char *JSON_STRING1 =
//    "{\"btn_123\":[251,225,2355,245]}";
    "{\"btn_abc\":\"pressup\"}";

blinker_universal_data_t blinker_universal_data1;

    uint8_t TEMP;
    TEMP = blinker_data_analaysis(JSON_STRING1, strlen(JSON_STRING1), &blinker_universal_data1);
    if(TEMP == 0)
    {
        PRINT("object_index = %d\r\n", blinker_universal_data1.blinker_assembly_index);
        PRINT("keyword_index = %d\r\n", blinker_universal_data1.blinker_keyword_index);
        PRINT("data = %d, %d, %d, %d\r\n",  blinker_universal_data1.blinker_key_value[0], 
                                            blinker_universal_data1.blinker_key_value[1], 
                                            blinker_universal_data1.blinker_key_value[2], 
                                            blinker_universal_data1.blinker_key_value[3]);
    }
    else
    {
        PRINT("get none\r\n");
    }
*/
/*
object_index = 2
keyword_index = 6
data = 65535, 65535, 65535, 65535
*/

/********************************************
 * @fn      Blinker_send
 * @author  Z2Z-gugu
 * @date    2023/2/12
 * @brief   ����Blinker������Ϣ
 * @note    �Ͻ����ж���ʹ�øú���
 * @param   fifo    ��д���fifo����app_uart_rx_fifo
 * @param   data    Ҫд����ַ����׵�ַ����TEMP_STR
 * @param   write_length    ���ݳ��ȣ���strlen(TEMP_STR)
 * @param   taskID  ��������ID����Peripheral_TaskID
 * @param   event   ��������������룬��UART_TO_BLE_SEND_EVT
 * @return  0       ����ʧ��
 *          1       ���ͳɹ�
 ********************************************/
uint8_t Blinker_send(app_drv_fifo_t *fifo, uint8_t *data, uint16_t write_length, tmosTaskID taskID, tmosEvents event)
{
    uint32_t irq_status;
    uint16_t error;
    error = app_drv_fifo_write(fifo, data, &write_length);
    if(error != APP_DRV_FIFO_RESULT_SUCCESS)
    {
        PRINT("app_drv_fifo_write ERROR!\r\n");
        return 0;
    }
    SYS_DisableAllIrq(&irq_status);
    tmos_start_task(taskID, event, 2);
    SYS_RecoverIrq(irq_status);
    return 1;
}
