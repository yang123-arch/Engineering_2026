//=====================================================================================
//                                   Motor_Driver[兼容层]
//
//      电机框架已迁移到 motor_core.h / motor_core.c（ops 表 + device + driver + group）。
//      本文件仅保留兼容入口，方便旧代码 #include "Motor_Driver.h" 继续编译，
//      新代码请直接使用 motor_core.h 的 API。
//
//      迁移对照：
//        旧 Motor_Data_StructTypeDef  ->  motor_data_t（motor_core.h 已提供同名别名）
//        旧 Get_TotalAngle_Speed()    ->  motor_core.c 中的同名函数
//=====================================================================================

#ifndef __Motor_Driver__
#define __Motor_Driver__

#include "main.h"
#include "motor_core.h"
#include <stdint.h>

/* 旧的集中式电机数量宏 Motor_Num 已废弃：电机数量现在由 motor_init.c 的设备表决定 */

#endif /* __Motor_Driver__ */
