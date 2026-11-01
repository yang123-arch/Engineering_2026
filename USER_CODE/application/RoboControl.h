#ifndef __RoboControl__
#define __RoboControl__

#include "main.h"
#include "Chassis.h"
#include "Shoot.h"
#include "Gimbal.h"

typedef enum
{
    none = 0,
    Joystick,
    KeyboardMouse,
    Customer
} Robo_Controler_EnumTypedef;

/*===| 机器人整体状态数据结构体定义 |===*/
typedef struct
{
    uint8_t Robo_Enable;                    //是否启动机器人
    float Smooth_Start_K;                   //平滑启动比例
    Robo_Controler_EnumTypedef Controler;   //控制方式
    uint8_t IF_Seting_Zero;
    
    float Robo_Target_Vx;                   //相对云台的前后速度
    float Robo_Target_Vy;                   //相对云台的左右速度
    float Robo_Target_Wz;                   //相对云台的旋转速度
    float Robo_Target_Yaw;                  //云台目标Yaw值
    float Robo_Target_Pitch1;                //云台目标Pitch值
	float Robo_Target_Pitch2; 				//云台目标Pitch2值
	float Robo_Target_J4; 				//云台目标J4值
	float Robo_Target_J5; 				//云台目标J5值
	float Robo_Target_J6; 				//云台目标J6值
	float Robo_Target_J7; 				//云台目标J7值
	float Robo_Target_Clap; 				//云台目标Clap值
    float Robo_Target_Leg_Angle;
	float Robo_Target_Camera_Y;
	float Robo_Target_Camera_X;
    
    float Robo_Target_Vx_Last;              //上一次相对云台的前后速度
    float Robo_Target_Vy_Last;              //上一次相对云台的左右速度
    float Robo_Target_Wz_Last;              //上一次相对云台的旋转速度
    float Robo_Target_Yaw_Last;             //上一次云台目标Yaw值
    float Robo_Target_Pitch1_Last;           //上一次云台目标Pitch值
	float Robo_Target_Pitch2_Last;			//上一次云台目标Pitch2值
	float Robo_Target_J4_Last;			//上一次云台目标J4值
	float Robo_Target_J5_Last;			//上一次云台目标J5值
	float Robo_Target_J6_Last;			//上一次云台目标J6值
	float Robo_Target_J7_Last;			//上一次云台目标J6值
	float Robo_Target_Clap_Last;			//上一次云台目标J6值
    float Robo_Target_Leg_Angle_Last;
	float Robo_Target_Camera_Y_Last;
	float Robo_Target_Camera_X_Last;
		
	float CustomerData_Last_Angle[7];   //自定义控制器上一次数据
		
	
    float Yaw_Err;                          //云台相对底盘的角度差值[-180 ~ +180]
    
    Gimbal_State_EnumTypedef Gimbal_State;  //云台状态
    
    Shoot_State_EnumTypedef Shoot_State;    //发射机构状态
    
    uint8_t Chassis_Speed_Level;            //底盘速度等级1~5
    Chassis_State_EnumTypedef Chassis_State;//底盘运动状态
    uint8_t SPIN_Direction_Flag;            //小陀螺旋转方向标志位
    uint8_t Chassis_Follow_45_Direction_Flag;//底盘跟随云台偏移方向标志位
    uint8_t If_Enable_Leg;
	uint8_t reset_flag;
		
		uint8_t auto_up_flag;										//自动上岛的标志位		
		uint8_t number;								//自动存取矿的标志位
//		uint8_t auto_get_R_flag;		
//		uint8_t auto_put_L_flag;
//		uint8_t auto_put_R_flag;
    
    uint8_t SuperCap_State;                 //超电放电开关
    float SuperCap_V;                       //超电电压
    float Referee_P;                        //裁判系统Chassis功率
    float Chassis_P;                        //底盘功率
	 
    uint8_t Refresh_UI_Flag;                //刷新UI标志位    
	
	uint8_t En_MODE_Flag;                   //旋转编码器模式标志位
	uint8_t Joy_MODE_Flag;					//力摇杆模式标志位
	uint8_t auto_up_flag_2;					//自动上岛的标志位2
	
	int8_t En_MODE_Flag_2;
	int8_t En_MODE_Flag_3;
} RoboControl_StructTypeDef;

/*===| 机器人整体状态数据结构体 |===*/
extern RoboControl_StructTypeDef RoboControl_Struct;
void RoboControl_UpdateSuperCapFeedback(uint8_t *data);


/*===| 输入参数滤波 |===*/
void Control_Filter(void);

/*===| 根据底盘运动状态得到Wz |===*/
void Get_Chassis_Wz(void);

/*===| 遥控器控制 |===*/
void RemoteControl_Float(void);
void RemoteControl_Bool(void);

/*===| 键盘控制 |===*/
void KeyControl_Float(void);
void KeyControl_Bool(void);

/*===| 停止机器人 |===*/
void Robo_Stop(void);

/*===| 重启机器人 |===*/
void Robo_Restart(void);


#endif
