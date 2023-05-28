/**********************************************************************************
 * @name        Storage.h
 * @version     R1.0
 * @brief       Storage API
 * @author      Z2Z GuGu
 * @date        2023/04/20
 * @code        GB2312
 **********************************************************************************/

#ifndef __STORAGE_H__
#define __STORAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projconfig.h"
#include "config.h"
#include "HAL.h"
#include "boardbase.h"

// EEPROM (Data Flash) Address
#define Config_Info_Start_ADDR      0x000000L
// F-Level ROM : 30K + 1K
#define F_level_ROM_Start_ADDR      0x000400L
#define F_level_ROM_Warning_ADDR    0x077BFFL
#define F_level_ROM_End_ADDR        0x077FFFL

// Code Flash Address (Real code : 128K / S-Level ROM : 320K)
#define S_level_ROM_Start_ADDR      0x020000L
#define S_level_ROM_End_ADDR        0x06FFFFL

#ifdef __cplusplus
}
#endif


#endif // __STORAGE_H__
