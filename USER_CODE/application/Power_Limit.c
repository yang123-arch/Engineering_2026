#include "Power_Limit.h"
#include "Referee_Unpack.h"
#include "PID.h"

float Motor_Power_Pre[6];
float Motor_Power_Zoom[6];
float Power_K;
double Inverse_Torque[2];
float Torque_Current[12];
float Power_Pre_Sum;
double a;
double b;
double c;


/**
 * @brief 功率限制模型
 *
 * @param 底盘数据结构体
 * @param 电机数据结构体
 * @param 底盘最大功率
 */
void Power_Limit(Chassis_StructTypeDef *Chassis_Struct,Motor_Data_StructTypeDef *Motor_Data,float P_max)
{
        /*===| 根据目标力矩和当前转速预测下一时刻的功率 |===*/
        Motor_Power_Pre[0] = Power_Predict_Calculate(&Chassis_Struct->Wheel[0].dev->data,Chassis_Struct->Wheel[0].output);
        Motor_Power_Pre[1] = Power_Predict_Calculate(&Chassis_Struct->Wheel[1].dev->data,Chassis_Struct->Wheel[1].output);
        Motor_Power_Pre[2] = Power_Predict_Calculate(&Chassis_Struct->Wheel[2].dev->data,Chassis_Struct->Wheel[2].output);
        Motor_Power_Pre[3] = Power_Predict_Calculate(&Chassis_Struct->Wheel[3].dev->data,Chassis_Struct->Wheel[3].output);
//        Motor_Power_Pre[4] = Power_Predict_Calculate(&Motor.Chassis_Track1,&Chassis_Struct->Track_Speed_PID[0]);
//        Motor_Power_Pre[5] = Power_Predict_Calculate(&Motor.Chassis_Track2,&Chassis_Struct->Track_Speed_PID[1]);
        
        /*===| 计算缩放系数[P_max / Power_Pre_Sum] |===*/
        Power_K = Power_Proportional_scaling(Motor_Power_Pre,P_max);
        for(uint8_t i = 0  ;i < 4 ; i++)
        {
                Motor_Power_Zoom[i] = Power_K * Motor_Power_Pre[i];
        }
        
        /*===| 根据缩放后的功率，逆结算出输出的力矩 |===*/
        for(uint8_t i = 0  ;i < 4 ; i++)
        {
                Torque_Inverse_Solution(&Torque_Current[2*i],Motor_Power_Zoom[i],&Motor_Data[i]);
        }
        
        for(uint8_t i = 0 ; i<4 ; i++)
        {
                Torque_decision(Chassis_Struct,i);
        }
}        

/**
 * @brief 功率预测模型
 *
 * @param 电机数据结构体
 * @param 将要下发的输出（joint_t.output）
 */
float Power_Predict_Calculate(Motor_Data_StructTypeDef *Motor_Data,float Output)
{
        float Power_Per;
        Power_Per = (Output * Current2Torque) * Motor_Data->Speed_RPM / 9.55f + K1 * Output * Output + K2 * Motor_Data->Speed_RPM * Motor_Data->Speed_RPM + Static_Power;
        
        return Power_Per;
}


/**
 * @brief 功率缩放计算
 *
 * @param 预测后四个电机的功率
 * @param 最大底盘总功率
 */
float Power_Proportional_scaling(float *Power_Pre,float P_max)
{
        float Proportional_Constant;
        
        Power_Pre_Sum = Power_Pre[0] + Power_Pre[1] + Power_Pre[2] + Power_Pre[3];
        if (Power_Pre_Sum <= 0.0f || P_max <= 0.0f)
        {
                return 1.0f;
        }
        Proportional_Constant = P_max / Power_Pre_Sum;
        
        if(Proportional_Constant > 1.0f || Proportional_Constant < 0.0f) Proportional_Constant = 1.0f;
        return Proportional_Constant;
}




/**
 * @brief 逆结算出功率缩放后的力矩
 *
 * @param 力矩结构体[Pos,Neg]
 * @param 单电机功率
 * @param 电机PID结构体
 * @param 电机数据结构体
 */
//void Torque_Inverse_Solution(float * Torque_Current,float P_max_Single,Motor_Data_StructTypeDef *Motor_Data)
//{
//        a = K1;
//        b = Torque2Current * Motor_Data->Speed_RPM / 9.55f;
//        c = K2 * (Motor_Data->Speed_RPM) * (Motor_Data->Speed_RPM) + Static_Power - P_max_Single;
//        
//        Inverse_Torque[0] = (-b + sqrt(b * b - 4.0f * a *c)) / (2.0f * a);
//        Inverse_Torque[1] = (-b - sqrt(b * b - 4.0f * a *c)) / (2.0f * a);
//        
//        if(Inverse_Torque[0] >= 0) 
//        {
//                Torque_Current[0] = Inverse_Torque[0];
//                Torque_Current[1] = Inverse_Torque[1];
//        }
//        else 
//        {
//                Torque_Current[0] = Inverse_Torque[1];
//                Torque_Current[1] = Inverse_Torque[0];
//        }
//        
//        Limit_float(&Torque_Current[0],16000,-16000);
//        Limit_float(&Torque_Current[1],16000,-16000);
//}

void Torque_Inverse_Solution(float * Torque_Current,float P_max_Single,Motor_Data_StructTypeDef *Motor_Data)
{
        a = K1;
        b = Current2Torque * Motor_Data->Speed_RPM / 9.55f;
        c = K2 * (Motor_Data->Speed_RPM) * (Motor_Data->Speed_RPM) + Static_Power - P_max_Single;
        
        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0)
        {
                discriminant = 0.0;
        }
        Inverse_Torque[0] = (-b + sqrt(discriminant)) / (2.0 * a);
        Inverse_Torque[1] = (-b - sqrt(discriminant)) / (2.0 * a);
        
        Torque_Current[0] = Inverse_Torque[0];
        Torque_Current[1] = Inverse_Torque[1];
        
        Limit_float(&Torque_Current[0],16000,-16000) ;
        Limit_float(&Torque_Current[1],16000,-16000) ;
}

/**
 * @brief 力矩决策
 */
void Torque_decision(Chassis_StructTypeDef *Chassis_Struct,uint8_t Order)
{
        if(Order < 4)
        {
                if(Chassis_Struct->Wheel[Order].output >0)
                {
                        Chassis_Struct->Wheel[Order].output = Torque_Current[2*Order];
                }
                else
                {
                        Chassis_Struct->Wheel[Order].output = Torque_Current[2*Order+1];
                }
        }
//        else
//        {
//                if(Chassis_Struct->Track_Speed_PID[Order-4].Output >0)
//                {
//                        Chassis_Struct->Track_Speed_PID[Order-4].Output = Torque_Current[2*Order];
//                }
//                else 
//                {
//                        Chassis_Struct->Track_Speed_PID[Order-4].Output = Torque_Current[2*Order+1];
//                }
//        }
        
}
