//=====================================================================================
//                    Remote_Control[遥控器数据解包]
//      解析遥控器的数据，得到遥控器各个通道状态和键鼠信息，供RoboControl使用
//=====================================================================================

#include "Remote_Control.h"

/*===| 遥控器数据结构体 |===*/
remote_rawdata_t remote_rawdata;
Remote_StructTypeDef Remote;
Remote_StructTypeDef Remote_Last;

/**
 * @brief 遥控串口接收中断回调函数
 *
 * @param 串口收到的数组
 */
void Remote_Rx_CallBack(uint8_t *DataBuff) {
  // CRC16校验
  if (verify_CRC16_check_sum(DataBuff, 21) == TRUE) {
    //更新连接状态
    Remote.If_Remote_Connect = 1;
    Remote.Remote_GetData_Ticker = 0;
    Remote.If_Remote_Data_New = 1;

    //数据解包
    memcpy(&remote_rawdata, DataBuff, sizeof(remote_rawdata));
    //存储上一次数据(建议注释此处，在其他地方>50ms触发一次该函数用于消抖)
    //        Remote_Save_Last_Data();
    //存储数据
    Remote.Right_X = (remote_rawdata.ch_0 - Remote_CH_VALUE_OFFSET) / 660.0f;
    Remote.Right_Y = (remote_rawdata.ch_1 - Remote_CH_VALUE_OFFSET) / 660.0f;
    Remote.Left_Y = (remote_rawdata.ch_2 - Remote_CH_VALUE_OFFSET) / 660.0f;
    Remote.Left_X = (remote_rawdata.ch_3 - Remote_CH_VALUE_OFFSET) / 660.0f;
    Remote.Mode = remote_rawdata.mode_sw;
    Remote.Pause = remote_rawdata.pause;
    Remote.Custom_L = remote_rawdata.fn_1;
    Remote.Custom_R = remote_rawdata.fn_2;
    Remote.Wheel = (remote_rawdata.wheel - Remote_CH_VALUE_OFFSET) / 660.0f;
    Remote.Trigger = remote_rawdata.trigger;

    Remote.Mouse_Vx = (remote_rawdata.mouse_x) / 32.0f;
    Remote.Mouse_Vy = (remote_rawdata.mouse_y) / 32.0f;
    Remote.Mouse_Vz = (remote_rawdata.mouse_z) / 32.0f;

    Remote.Mouse_L = remote_rawdata.mouse_left;
    Remote.Mouse_M = remote_rawdata.mouse_middle;
    Remote.Mouse_R = remote_rawdata.mouse_right;

    Remote.Keyboard_W = !!(remote_rawdata.key1 & 0x01);
    Remote.Keyboard_S = !!(remote_rawdata.key1 & 0x02);
    Remote.Keyboard_A = !!(remote_rawdata.key1 & 0x04);
    Remote.Keyboard_D = !!(remote_rawdata.key1 & 0x08);
    Remote.Keyboard_Shift = !!(remote_rawdata.key1 & 0x10);
    Remote.Keyboard_Ctrl = !!(remote_rawdata.key1 & 0x20);
    Remote.Keyboard_Q = !!(remote_rawdata.key1 & 0x40);
    Remote.Keyboard_E = !!(remote_rawdata.key1 & 0x80);
    Remote.Keyboard_R = !!(remote_rawdata.key2 & 0x01);
    Remote.Keyboard_F = !!(remote_rawdata.key2 & 0x02);
    Remote.Keyboard_G = !!(remote_rawdata.key2 & 0x04);
    Remote.Keyboard_Z = !!(remote_rawdata.key2 & 0x08);
    Remote.Keyboard_X = !!(remote_rawdata.key2 & 0x10);
    Remote.Keyboard_C = !!(remote_rawdata.key2 & 0x20);
    Remote.Keyboard_V = !!(remote_rawdata.key2 & 0x40);
    Remote.Keyboard_B = !!(remote_rawdata.key2 & 0x80);
  }
}

/**
 * @brief 断链清理：把遥控数据清成"未连接、C 档（停机）、无任何输入"
 * @note  由断链检测（Error_Check_Task）在超时后调用；停机由 Remote_Rx_Task
 *        读到 If_Remote_Connect == 0 后自己执行，机器的使能状态只有它在改。
 *        接收回调在中断里写同一个结构体，所以整段关中断做，不会清到一半
 *        被新帧插进来。进来时 Remote_GetData_Ticker 已被回调清零，说明调用
 *        前刚收到一帧、遥控已经回来了，这次不清。
 *        超时计数保留原值，由调用方继续累加；重复调用无副作用
 */
void Remote_Clear(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint16_t ticker = Remote.Remote_GetData_Ticker;
  if (ticker != 0U) {
    memset(&Remote, 0, sizeof(Remote));
    Remote.Remote_GetData_Ticker = ticker;
    Remote.If_Remote_Connect = 0U;
    Remote.Mode = Remote_Mode_C;
  }

  if (primask == 0U)
    __enable_irq();
}

//记录上一次值，用于消抖(建议50ms以上触发一次)
void Remote_Save_Last_Data(void) {
  Remote_Last.If_Remote_Data_New = 1;
  while (Remote_Last.If_Remote_Data_New == 1)
    osDelay(1);
  memcpy(&Remote_Last, &Remote, sizeof(Remote_StructTypeDef));
}

/**
 * @brief DT7/DR16 通道位域解包，结果写入 Remote（结构体不变）
 * @param DataBuff 指向 18 字节通道数据（不含 0xA9 0x53 帧头）
 * @note  逻辑来自 robo_control_dt7.c 的 Remote_Control_GetData
 */
void Remote_GetData_SBUS(uint8_t *DataBuff) {
  float right_x, right_y, left_x, left_y, wheel;
  uint8_t s1, s2;
  int16_t mouse_x, mouse_y, mouse_z;

  right_x =
      (((DataBuff[0] | (DataBuff[1] << 8)) & 0x07ff) - Remote_CH_VALUE_OFFSET) /
      660.0f;
  right_y = ((((DataBuff[1] >> 3) | (DataBuff[2] << 5)) & 0x07ff) -
             Remote_CH_VALUE_OFFSET) /
            660.0f;
  left_x = ((((DataBuff[2] >> 6) | (DataBuff[3] << 2) | (DataBuff[4] << 10)) &
             0x07ff) -
            Remote_CH_VALUE_OFFSET) /
           660.0f;
  left_y = ((((DataBuff[4] >> 1) | (DataBuff[5] << 7)) & 0x07ff) -
            Remote_CH_VALUE_OFFSET) /
           660.0f;
  wheel =
      -((DataBuff[16] | (DataBuff[17] << 8)) - Remote_CH_VALUE_OFFSET) / 660.0f;

  s1 = ((DataBuff[5] >> 4) & 0x000C) >> 2;
  s2 = ((DataBuff[5] >> 4) & 0x0003);

  mouse_x = (int16_t)(DataBuff[6] | (DataBuff[7] << 8));
  mouse_y = (int16_t)(DataBuff[8] | (DataBuff[9] << 8));
  mouse_z = (int16_t)(DataBuff[10] | (DataBuff[11] << 8));

  /* 摇杆死区与线性重映射（与 dt7 一致） */
  if (right_x > 0.1f)
    right_x = (right_x - 0.1f) / 0.9f;
  else if (right_x < -0.1f)
    right_x = (right_x + 0.1f) / 0.9f;
  else
    right_x = 0;

  if (right_y > 0.1f)
    right_y = (right_y - 0.1f) / 0.9f;
  else if (right_y < -0.1f)
    right_y = (right_y + 0.1f) / 0.9f;
  else
    right_y = 0;

  if (left_x > 0.1f)
    left_x = (left_x - 0.1f) / 0.9f;
  else if (left_x < -0.1f)
    left_x = (left_x + 0.1f) / 0.9f;
  else
    left_x = 0;

  if (left_y > 0.1f)
    left_y = (left_y - 0.1f) / 0.9f;
  else if (left_y < -0.1f)
    left_y = (left_y + 0.1f) / 0.9f;
  else
    left_y = 0;

  Remote.Right_X = right_x;
  Remote.Right_Y = right_y;
  Remote.Left_X = left_x;
  Remote.Left_Y = left_y;
  Remote.Wheel = wheel;

  /* S1: 1=上 3=中 2=下 → Mode C/N/S */
  if (s1 == 1)
    Remote.Mode = Remote_Mode_C;
  else if (s1 == 3)
    Remote.Mode = Remote_Mode_N;
  else if (s1 == 2)
    Remote.Mode = Remote_Mode_S;

  /* S2 下档作 Pause；其余拨杆位暂无独立字段，保持 0 */
  Remote.Pause = 0;
  Remote.Custom_L = (s2 == 2) ? 1 : 0;
  Remote.Custom_R = 0;
  Remote.Trigger = (s2 == 3) ? 1 : 0;

  Remote.Mouse_Vx = mouse_x / 32.0f;
  Remote.Mouse_Vy = mouse_y / 32.0f;
  Remote.Mouse_Vz = mouse_z / 32.0f;
  Remote.Mouse_L = DataBuff[12];
  Remote.Mouse_M = 0;
  Remote.Mouse_R = DataBuff[13];

  Remote.Keyboard_W = !!(DataBuff[14] & 0x01);
  Remote.Keyboard_S = !!(DataBuff[14] & 0x02);
  Remote.Keyboard_A = !!(DataBuff[14] & 0x04);
  Remote.Keyboard_D = !!(DataBuff[14] & 0x08);
  Remote.Keyboard_Shift = !!(DataBuff[14] & 0x10);
  Remote.Keyboard_Ctrl = !!(DataBuff[14] & 0x20);
  Remote.Keyboard_Q = !!(DataBuff[14] & 0x40);
  Remote.Keyboard_E = !!(DataBuff[14] & 0x80);
  Remote.Keyboard_R = !!(DataBuff[15] & 0x01);
  Remote.Keyboard_F = !!(DataBuff[15] & 0x02);
  Remote.Keyboard_G = !!(DataBuff[15] & 0x04);
  Remote.Keyboard_Z = !!(DataBuff[15] & 0x08);
  Remote.Keyboard_X = !!(DataBuff[15] & 0x10);
  Remote.Keyboard_C = !!(DataBuff[15] & 0x20);
  Remote.Keyboard_V = !!(DataBuff[15] & 0x40);
  Remote.Keyboard_B = !!(DataBuff[15] & 0x80);

  Remote.If_Remote_Data_New = 1;
}

/**
 * @brief DT7/DR16(SBUS口/图传)接收回调
 */
void Remote_Rx_CallBack_SBUS(uint8_t *DataBuff) {
  Remote.If_Remote_Connect = 1;
  Remote.Remote_GetData_Ticker = 0;
  Remote_GetData_SBUS(DataBuff);
}
