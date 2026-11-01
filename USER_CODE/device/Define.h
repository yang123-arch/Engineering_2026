//======================================================================
//                    Define[全局定义]
//      这里定义了一些可能需要经常修改或者引用的宏定义
//======================================================================

#ifndef __Define__
#define __Define__

/*===| 外设接口定义 |===*/
#define IMU_SPI &hspi4
#define IMU_Heat_TIM &htim15
#define IMU_Heat_TIM_CHANNAL TIM_CHANNEL_1
#define IMU_Heat_TIM_ARR 20000

#define LED_TIM &htim1
#define LED_R_TIM_CHANNAL TIM_CHANNEL_2
#define LED_G_TIM_CHANNAL TIM_CHANNEL_3
#define LED_B_TIM_CHANNAL TIM_CHANNEL_4
#define LED_TIM_ARR 1000

// 485 4Mbits/s
#define RS485_1_UART &huart3
#define RS485_2_UART &huart2
#define SBUS_UART &huart1

#define SBUS_UART_DMA &hdma_usart1_rx
#define RS485_2_UART_DMA &hdma_usart2_rx
#define RS485_1_UART_DMA &hdma_usart3_rx
#define UART4_DMA &hdma_uart4_rx
#define UART5_DMA &hdma_uart5_rx

#define PWM_TIM &htim3
#define PWM_1_TIM_CHANNAL TIM_CHANNEL_1
#define PWM_2_TIM_CHANNAL TIM_CHANNEL_2
#define PWM_3_TIM_CHANNAL TIM_CHANNEL_3
#define PWM_4_TIM_CHANNAL TIM_CHANNEL_4

#define Buzzer_TIM &htim5
#define Buzzer_TIM_CHANNAL TIM_CHANNEL_1

#define USER_SPI &hspi2
#define USER_I2C &hi2c2

////////////////////////////////////////////////////////////////////
//                                                               //
// 修改了CANID后接收数据不用动，需要在发送函数处根据ID重新排序 //
//                                                               //
////////////////////////////////////////////////////////////////////
/*===| 底盘电机通信相关定义 |===*/
#define Chassis_Motor_CAN hfdcan1            //底盘CAN
#define Chassis_Motor_Send_CAN_ID 0x200      //底盘电机CAN发送ID
#define Chassis_Motor1_Feedback_CAN_ID 0x201 //底盘电机1CAN反馈ID
#define Chassis_Motor2_Feedback_CAN_ID 0x202 //底盘电机2CAN反馈ID
#define Chassis_Motor3_Feedback_CAN_ID 0x203 //底盘电机3CAN反馈ID
#define Chassis_Motor4_Feedback_CAN_ID 0x204 //底盘电机4CAN反馈ID

#define Chassis_Leg1_CAN hfdcan1          //底盘CAN
#define Chassis_Leg1_Send_CAN_ID 0x0A     //底盘电机CAN发送ID
#define Chassis_Leg1_Feedback_CAN_ID 0x0B //底盘电机1CAN反馈ID
#define Chassis_Leg2_CAN hfdcan1          //底盘CAN
#define Chassis_Leg2_Send_CAN_ID 0x0C     //底盘电机CAN发送ID
#define Chassis_Leg2_Feedback_CAN_ID 0x0D //底盘电机1CAN反馈ID

/*===| 云台电机通信相关定义 |===*/
#define Gimbal_Yaw_CAN hfdcan2           // Yaw_使用的CAN
#define Gimbal_Yaw_Send_CAN_ID 0x01      // Yaw_CAN发送ID
#define Gimbal_Yaw_Feedback_CAN_ID 0x101 // Yaw_CAN反馈ID

#define Gimbal_Pitch1_CAN hfdcan2           // Pitch_使用的CAN
#define Gimbal_Pitch1_Send_CAN_ID 0x02      // Pitch_CAN发送ID
#define Gimbal_Pitch1_Feedback_CAN_ID 0x102 // Pitch_CAN反馈ID

#define Gimbal_Pitch2_CAN hfdcan2           // Pitch2_使用的CAN
#define Gimbal_Pitch2_Send_CAN_ID 0x03      // Pitch2_CAN发送ID
#define Gimbal_Pitch2_Feedback_CAN_ID 0x103 // Pitch2_CAN反馈ID

#define Camera_Y_CAN hfdcan2                  // Camera_Y使用的CAN
#define Gimbal_Camera_Y_Send_CAN_ID 0x200     // Pitch2_CAN发送ID
#define Gimbal_Camera_Y_Feedback_CAN_ID 0x201 // Pitch2_CAN反馈ID

#define Camera_X_CAN hfdcan2                  // Camera_X使用的CAN
#define Gimbal_Camera_X_Send_CAN_ID 0x1FE     // Pitch2_CAN发送ID
#define Gimbal_Camera_X_Feedback_CAN_ID 0x206 // Pitch2_CAN反馈ID

#define Gimbal_J4_CAN hfdcan3           // J4_使用的CAN
#define Gimbal_J4_Send_CAN_ID 0x04      // J4_CAN发送ID
#define Gimbal_J4_Feedback_CAN_ID 0x104 // J4_CAN反馈ID

#define Gimbal_J5_CAN hfdcan3           // J5_使用的CAN
#define Gimbal_J5_Send_CAN_ID 0x05      // J5_CAN发送ID
#define Gimbal_J5_Feedback_CAN_ID 0x105 // J5_CAN反馈ID

#define Gimbal_J6_CAN hfdcan3           // J6_使用的CAN
#define Gimbal_J6_Send_CAN_ID 0x06      // J6_CAN发送ID
#define Gimbal_J6_Feedback_CAN_ID 0x106 // J6_CAN反馈ID

#define Gimbal_J7_CAN hfdcan3                    // J7_使用的CAN
#define Gimbal_J7_Send_CAN_ID (0x140 + 0x07)     // J7_CAN发送ID
#define Gimbal_J7_Feedback_CAN_ID (0x140 + 0x07) // J7_CAN反馈ID

#define Gimbal_Clap_CAN hfdcan3           // Clap_使用的CAN
#define Gimbal_Clap_Send_CAN_ID 0x08      // Clap_CAN发送ID
#define Gimbal_Clap_Feedback_CAN_ID 0x108 // Clap_CAN反馈ID
/*===| 发射电机通信相关定义 |===*/
#define Shoot_Fric_First_CAN hfdcan3                  // Fric_使用的CAN
#define Shoot_Fric_First_Send_CAN_ID 0x200            // Fric_CAN发送ID
#define Shoot_Fric_First_Right_Feedback_CAN_ID 0x201  //前右Fric_CAN反馈ID
#define Shoot_Fric_First_Left_Feedback_CAN_ID 0x202   //前左Fric_CAN反馈ID
#define Shoot_Fric_First_Middle_Feedback_CAN_ID 0x203 //前中Fric_CAN反馈ID

#define Shoot_Fric_Second_CAN hfdcan3                  // Fric_使用的CAN
#define Shoot_Fric_Second_Send_CAN_ID 0x1FF            // Fric_CAN发送ID
#define Shoot_Fric_Second_Right_Feedback_CAN_ID 0x206  //前右Fric_CAN反馈ID
#define Shoot_Fric_Second_Left_Feedback_CAN_ID 0x207   //前左Fric_CAN反馈ID
#define Shoot_Fric_Second_Middle_Feedback_CAN_ID 0x208 //前中Fric_CAN反馈ID

#define Shoot_Trigger_CAN hfdcan2           // Trigger_使用的CAN
#define Shoot_Trigger_Send_CAN_ID 0x200     // Trigger_CAN发送ID
#define Shoot_Trigger_Feedback_CAN_ID 0x204 // Trigger_CAN反馈ID
/*===| 超电通信相关定义 |==*/
#define SuperCap_CAN hfdcan1           // hcan1
#define SuperCap_Control_CAN_ID 0x030  //超电控制CANID
#define SuperCap_FeedBack_CAN_ID 0x031 //超电反馈CANID

/*===| 陀螺仪通信相关定义 |==*/
#define IMU_CAN hfdcan3           // hcan2
#define IMU_Control_CAN_ID 0x001  // IMU控制CANID
#define IMU_FeedBack_CAN_ID 0x002 // IMU反馈CANID

/*===| 电机定义 |===*/
#define Motor_3508_Reduction 19.2032f
#define Motor_2006_Reduction 36.0f
#define Motor_S3508_Reduction 15.7647f

/*===| 底盘相关参数定义 |===*/

#define Chassis_A_Distance 0.2268f      //轮子到底盘中心水平距离
#define Chassis_B_Distance 0.210745f    //轮子到底盘中心垂直距离
#define Chassis_R_Distance 0.309599249f //轮子到底盘中心距离 根号(a2+b2)
//    #define Chassis_R_Distance                  0.196f //轮子到底盘中心距离
//    根号(a2+b2)
#define Chassis_Wheel_Radius 0.053 //轮半径
#define Chassis_RPM_to_m_s                                                     \
  (2 * PI * Chassis_Wheel_Radius / 60.0f /                                     \
   Motor_S3508_Reduction)     //底盘RPM转换成m/s的系数
#define Robo_Vx_MaxSpeed 2.0f //机器最大X速度
#define Robo_Vy_MaxSpeed 2.0f //机器最大Y速度
#define Robo_Wz_MaxSpeed 6.0f //机器最大Z角速度

#define Chassis_Acc 2.0f //底盘加速度

#define Chassis_Dash_Speed_Multiple 1.5f //冲刺模式后轮速度倍率

/*===| 云台相关参数定义 |===*/
#define Gimbal_Median_Yaw 0.0f //云台朝前时Yaw电机的编码值
#define Gimbal_Median_Pitch 0  //云台水平时Pitch电机的角度

#define Gimbal_Pitch_Limit_Up 90.0f
#define Gimbal_Pitch_Limit_Down -90.0f

#define PI_to_Degree (180.0f / PI)
#define Degree_to_PI (PI / 180.0f)
/*===| 发射相关参数定义 |===*/
#define Fric_Acc 0.1f
#define Fric_Radius 0.03f
#define Fric_RPM_to_m_s (2 * PI * Fric_Radius / 60.0f)
#define Triger_Reduction (Motor_2006_Reduction * 55 / 17)

#endif
