# CH582M移植QLW记录

+ 创建工程
+ 删除src，StdPeriphDriver，LIB文件夹
+ QLW-573中复制APP、Main、QLW_lib三个文件夹
+ 从最新WCH给出的EVB中复制StdPeriphDriver、HAL、LIB三个文件夹
+ 打开项目管理-C/C++ General- Paths and Symbols：
  + Include - GNU C add：（“is a workspace path”）
    + /${ProjName}/LIB
    + /${ProjName}/RVMSIS
    + /${ProjName}/APP
    + /${ProjName}/Main
    + /${ProjName}/StdPeriphDriver/inc
    + /${ProjName}/HAL/include
    + /${ProjName}/QLW_lib/app_drv_fifo
    + /${ProjName}/QLW_lib/ble_task
    + /${ProjName}/QLW_lib/boardbase
    + /${ProjName}/QLW_lib/Storage
    + /${ProjName}/QLW_lib/SYS_Config
    + /${ProjName}/QLW_lib/uart_task
    + /${ProjName}/QLW_lib/usb_task
  + Symbols - GNU C add:
    + DEBUG 0
  + Libraries add:(没有.、没有路径、没有“is a workspace path”)
    + CH58xBLE
    + ISP583
  + Libraries Paths add:（“is a workspace path”）
    + /${ProjName}/StdPeriphDriver
    + /${ProjName}/LIB
  + Source Location add:（NULL）
+ 排除部分不需要编译的C文件

+ ble_task.c函数（CH57X_BLEInit）-（CH58X_BLEInit）
+ 编译：可以无错通过
+ 修剪不需要的部分：LED、BOOT、BLINKER、JSON、BLE（关闭使能即可）