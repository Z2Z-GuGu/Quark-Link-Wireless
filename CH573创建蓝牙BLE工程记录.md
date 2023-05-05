# CH573创建蓝牙BLE工程记录

+ 创建工程
+ 删除src，StdPeriphDriver文件夹
+ 粘贴APP、HAL、LIB、Profile、StdPeriphDriver五个文件夹
+ 打开项目管理-C/C++ General- Paths and Symbols：
  + Include - GNU C add：
    + APP
    + HAL
    + LIB
    + Profile
    + “is a workspace path”
  + Symbols - GNU C add:
    + -
  + Libraries add:
    + CH57xBLE
  + Libraries Paths add:
    + /${ProjName}/LIB
  + Source Location add:
+ 排除部分不需要编译的C文件