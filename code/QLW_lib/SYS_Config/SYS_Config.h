/*  
 *  @name       SYS_Config.h
 *  @brief      使用Json字符串对系统功能进行配置
 *  @author     Z2Z-GuGu
 *  @date       2023/2/10
 *  @code       GB2312
 */

#ifndef SYS_CONFIG__H_
#define SYS_CONFIG__H_

#define JSMN_HEADER

#include <string.h>
#include <stdlib.h>
#include "jsmn.h"
#include "projconfig.h"
#include "config.h"
#include "app_drv_fifo.h"
#include "APP.h"

#define object_token        0
#define assembly_token      1
#define value_token         2

#define assembly_num        10
#define keyword_num         7

// #define assembly_num        10

// #define BKW_get blinker_keyword[0]

typedef struct
{
    uint8_t blinker_assembly_index;       // blinker assembly index
    uint8_t blinker_keyword_index;        // blinker keyword index
    uint16_t blinker_key_value[4];        // blinker key_value
} blinker_universal_data_t;

typedef struct
{
    char *JSON_string;
    size_t JSON_len;
    char *OUTPUT_string;
    uint8_t OUTPUT_len;
} SYS_Config_t;

int8_t blinker_data_analaysis(char *JSON_string, size_t JSON_len, blinker_universal_data_t *blinker_universal_data);

uint8_t Blinker_send(app_drv_fifo_t *fifo, uint8_t *data, uint16_t write_length, tmosTaskID taskID, tmosEvents event);

#endif  // SYS_CONFIG__H_
