/**********************************************************************************
 * @name        main.c
 * @version     R1.0
 * @brief       main
 * @author      Z2Z GuGu
 * @date        2023/03/05
 * @code        GB2312
 * @License     GPLv3
 **********************************************************************************/

#include "projconfig.h"
#include "config.h"
#include "HAL.h"

#include "Storage.h"
#include "SYS_Config.h"
#include "boardbase.h"
#include "APP.h"
#include "ble_task.h"
#include "usb_task.h"
#include "uart_task.h"

int main(void)
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);
    QWL_GPIO_Init();
    IRQ_Prioritty_Setting();
    DelayMs(50);
    Pin_Ctrl(Sys_LED_Pin, GPIO_HIGH);
    All_FIFO_Init();

    ble_task_Init();
    InitCDCDevice();
    uart_task_Init();
    APP_task_Init();

    while(1)
    {
        Task_Launcher();
        TMOS_SystemProcess();

        uart_fifo_cheak_process();
        ble_fifo_cheak_process();
        uart_auto_send();
        BOOT_Execute();
    }
}


//=================================================================================================================

