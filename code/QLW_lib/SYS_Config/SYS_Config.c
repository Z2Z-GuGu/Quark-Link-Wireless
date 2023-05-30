/***
 *  @name       SYS_Config.c
 *  @brief      ʹ��Json�ַ�����ϵͳ���ܽ�������
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
