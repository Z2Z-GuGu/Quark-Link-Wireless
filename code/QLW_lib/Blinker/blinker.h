/*  @name       Blinker.h
 *  @brief      兼容Blinker的CH57xBLE工程
 *  @author     Z2Z
 *  @date       2023/2/10
 */

#ifndef BLINKER_H
#define BLINKER_H

#define JSMN_HEADER

#include <string.h>
#include <stdlib.h>
#include "jsmn.h"
#include "projconfig.h"
#include "config.h"
#include "app_drv_fifo.h"

#define object_token        0
#define assembly_token      1
#define value_token         2

#define assembly_num        10
#define keyword_num         7

// #define BKW_get blinker_keyword[0]


// char blinker_keyword[8] = {
//   "get", "state", "val", "tap", "on", "off", "press", "pressup"
// };

typedef struct
{
    uint8_t blinker_assembly_index;       // blinker assembly index
    uint8_t blinker_keyword_index;        // blinker keyword index
    uint16_t blinker_key_value[4];        // blinker key_value
} blinker_universal_data_t;

int8_t blinker_data_analaysis(char *JSON_string, size_t JSON_len, blinker_universal_data_t *blinker_universal_data);

uint8_t Blinker_send(app_drv_fifo_t *fifo, uint8_t *data, uint16_t write_length, tmosTaskID taskID, tmosEvents event);

#endif
