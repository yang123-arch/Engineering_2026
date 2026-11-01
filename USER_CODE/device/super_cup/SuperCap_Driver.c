//============================================================================================================================================================================
//                                              Super_Driver[超电驱动]
//      包含了一些超电驱动相关的封装好的函数
//      队内超电定义控制CANID为0x30,无反馈
//      通过双板通信得到超电开关
//      在底盘communicate_task以100Hz循环发送控制信息[开关，功率限制，缓冲区能量]
//      未获取裁判系统数据时默认关断超电
//
//============================================================================================================================================================================
#include "SuperCap_Driver.h"
#include "CAN_Driver.h"
#include "Define.h"
#include <string.h>

/*===| 超电控制数据结构体 |===*/
SuperCap_Control_Data_StructTypedef SuperCap_Control_Data_Struct;
/*===| 超电反馈数据结构体 |===*/
SuperCap_FeedBack_Data_StructTypedef SuperCap_FeedBack_Data;

/**
 * @brief 循环发送超电控制帧
 */
void SuperCap_Send_Control_Data(uint8_t enable, uint8_t max_referee_power,
                                uint8_t power_buffer_energy)
{
    SuperCap_Control_Data_Struct.Enable_Flag = enable;
    SuperCap_Control_Data_Struct.Max_Referee_P = max_referee_power;
    SuperCap_Control_Data_Struct.Power_Buff_Energy = power_buffer_energy;
    
    CAN_Send_Data(&SuperCap_CAN, SuperCap_Control_CAN_ID, (uint8_t *)&SuperCap_Control_Data_Struct);
}

/**
 * @brief 解析超电反馈帧
 */
void SuperCap_Unpack_Data(uint8_t *Data)
{
    if (Data == NULL) return;
    memcpy((uint8_t *)&SuperCap_FeedBack_Data, Data, 8);
}

