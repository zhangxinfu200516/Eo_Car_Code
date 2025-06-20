#include "dvc_encodermotor.h"

void Class_EncoderMotor::Init(TIM_HandleTypeDef *__TIM_EncoderHandle,uint16_t __encoder_resolution, uint32_t __max_encoder_sum,float __GearRation,float __WheelDiameter)
{
    //定时器编码器外设初始化
    HAL_TIM_Encoder_Start(__TIM_EncoderHandle,TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(__TIM_EncoderHandle,TIM_CHANNEL_2);

    TIM_EncoderHandle = __TIM_EncoderHandle;
    Max_Encoder_Sum = __max_encoder_sum;
    Encoder_Resolution = __encoder_resolution;
    GearRation = __GearRation;
    WheelDiameter = __WheelDiameter;

    Now_Encoder = 0;
    //上次脉冲值
    Last_Encoder = 0;
    //脉冲差值
    Encoder_Diff = 0;
    //脉冲累加和 ±
    Encoder_Diff_Sum = 0;
    //当前圈数
    Total_Round = 0;    
    //当前角度
    Now_Angle = 0;
}

void Class_EncoderMotor::TIM_Encoder_Calculate()
{
    //获取当前脉冲值
    Now_Encoder = TIM_EncoderHandle->Instance->CNT;
    //计算脉冲差值
    Encoder_Diff = (int32_t)(Now_Encoder - Last_Encoder);
    //更新上次编码器值
    Last_Encoder = Now_Encoder;

    if(Encoder_Diff > Max_Encoder_Sum/2)
    {
        Encoder_Diff = Encoder_Diff - Max_Encoder_Sum;
    }
    else if(Encoder_Diff < -(int32_t)Max_Encoder_Sum/2)
    {
        Encoder_Diff = Encoder_Diff + Max_Encoder_Sum;
    }
    //计算角速度 1ms
    Now_Omega_Angle = Filter((float)Encoder_Diff / (float)Encoder_Resolution * 360.0f * 1000.0f,Record_Omega_Angle);
    Now_Velocity = Now_Omega_Angle * DEG_TO_RAD * WheelDiameter / 2.0f;
    
    //模拟单圈编码器 0-Encoder_Resolution 计算圈数
    Encoder_Diff_Sum += Encoder_Diff;
    if(Encoder_Diff_Sum > (int32_t)Encoder_Resolution)
    {
        Encoder_Diff_Sum -= (int32_t)Encoder_Resolution;
        Total_Round++;
    }
    else if(Encoder_Diff_Sum < 0)
    {
        Encoder_Diff_Sum += (int32_t)Encoder_Resolution;
        Total_Round--;
    }
    //计算角度
    Now_Angle = (float)Encoder_Diff_Sum / ((float)Encoder_Resolution) * 360.0f;

    // // 计算归一化的角度差 (处理360°跳变)
    // float angle_diff = Now_Angle - Last_Angle;

    // // 关键修正：将角度差约束到[-180, 180]区间
    // if (angle_diff > 180.0f)
    // {
    //     angle_diff -= 360.0f;
    // }
    // else if (angle_diff < -180.0f)
    // {
    //     angle_diff += 360.0f;
    // }

    // // 计算角速度 (度/秒)
    // Now_Omega_Angle = angle_diff / 0.001f; // 假设采样周期1ms

    // // 更新历史角度
    // Last_Angle = Now_Angle;

    // // 计算线速度
    // Now_Velocity = Now_Omega_Angle * DEG_TO_RAD * WheelDiameter / 2.0f;
    
}
void Class_EncoderMotor::OutPut_PWM()
{
    // 电机A
}
void Class_EncoderMotor::TIM_Calculate_PeriodElapsedCallback()
{
    //计算轮实际速度
    TIM_Encoder_Calculate();
    //PID
    PID_Omage.Set_Target(Target_Velocity);
    PID_Omage.Set_Now(Now_Velocity);
    PID_Omage.TIM_Adjust_PeriodElapsedCallback();
    PWM_Out = PID_Omage.Get_Out();
    //OutPut_PWM();
}
