//=====================================================================================
//                         Motor_DMAIAO_Type[达妙各型号量程]
//
//      达妙单机 MIT 三段量程：位置 q [rad]、速度 v [rad/s]、力矩 t [N·m]。
//      驱动 probe 时按 type_id 写进 ctx，编码 / 解码 / 输出限幅共用同一份。
//
//      q/v/t 必须等于电机寄存器 PMAX / VMAX /
//      TMAX，否则编码、解码都按错的比例； 改了电机寄存器，同步改这里。 3507 /
//      8009 / 8009P / 8006 / 4340 / 4310 取自 data/dm_data.csv（Motor_DM_read
//      读出， 2026-10-29），π 写成 3.141593f，和读出的 float 逐位相同。 t
//      是编码量程不是额定值（TMAX 一般大于峰值力矩），额定 / 峰值限幅由上层做。
//      未确认的型号先沿用达妙手册默认（P=±π、V=±30、T=±20），后续按型号补。
//=====================================================================================

#ifndef __MOTOR_DMAIAO_TYPE_H__
#define __MOTOR_DMAIAO_TYPE_H__

/* 4005（手册参数未确认，先按达妙默认填） */
#define MOTOR_DM_4005_Q_LIMIT 3.1415926f
#define MOTOR_DM_4005_V_LIMIT 30.0f
#define MOTOR_DM_4005_T_LIMIT 3.0f
/* 3507（gimbal_clap 读出） */
#define MOTOR_DM_3507_Q_LIMIT 3.141593f
#define MOTOR_DM_3507_V_LIMIT 30.0f
#define MOTOR_DM_3507_T_LIMIT 5.0f
/* 8009（gimbal_pitch1 / pitch2 读出） */
#define MOTOR_DM_8009_Q_LIMIT 3.141593f
#define MOTOR_DM_8009_V_LIMIT 45.0f
#define MOTOR_DM_8009_T_LIMIT 54.0f
/* 8009P（chassis_leg1 / leg2 读出） */
#define MOTOR_DM_8009P_Q_LIMIT 3.141593f
#define MOTOR_DM_8009P_V_LIMIT 30.0f
#define MOTOR_DM_8009P_T_LIMIT 54.0f
/* 8006（gimbal_yaw 读出） */
#define MOTOR_DM_8006_Q_LIMIT 3.141593f
#define MOTOR_DM_8006_V_LIMIT 30.0f
#define MOTOR_DM_8006_T_LIMIT 20.0f
/* 4340（gimbal_j5 读出）。反馈角度范围 ±716°，Motor_DM_Storage_Data 的过圈
   判断按 360° 一圈写的，反馈不越过 ±12.5 rad 时 Total_Angle 才对 */
#define MOTOR_DM_4340_Q_LIMIT 12.5f
#define MOTOR_DM_4340_V_LIMIT 10.0f
#define MOTOR_DM_4340_T_LIMIT 28.0f
/* 4310（gimbal_j4 / j6）。读出时 J6 的 TMAX 是 10，J4 是 5；
   统一定为 10，J4 的寄存器要同步改成 10 */
#define MOTOR_DM_4310_Q_LIMIT 3.141593f
#define MOTOR_DM_4310_V_LIMIT 30.0f
#define MOTOR_DM_4310_T_LIMIT 10.0f
/* 兼容型号 DM_DJI：沿用旧工程语义，力矩铺满 16384 原始值区间 */
#define MOTOR_DM_DJI_Q_LIMIT 3.1415926f
#define MOTOR_DM_DJI_V_LIMIT 30.0f
#define MOTOR_DM_DJI_T_LIMIT 16384.0f

/*===| 旧工程参数换算：DM_DJI → 具体型号 |===
 *      DM_DJI 按 ±π、±30 解码，力矩指令 ±16384 原始值对应电机里的 ±TMAX。
 *      换成具体型号后：
 *        新角度读数        = 旧读数   × MOTOR_DM_DJI_K_Q(PMAX)
 *        新速度读数        = 旧读数   × MOTOR_DM_DJI_K_V(VMAX)
 *        新力矩指令 [N·m]  = 旧原始值 × MOTOR_DM_DJI_K_T(整定旧参数时的 TMAX)
 * |===*/
#define MOTOR_DM_DJI_K_Q(pmax) ((pmax) / MOTOR_DM_DJI_Q_LIMIT)
#define MOTOR_DM_DJI_K_V(vmax) ((vmax) / MOTOR_DM_DJI_V_LIMIT)
#define MOTOR_DM_DJI_K_T(tmax) ((tmax) / MOTOR_DM_DJI_T_LIMIT)

#endif /* __MOTOR_DMAIAO_TYPE_H__ */
