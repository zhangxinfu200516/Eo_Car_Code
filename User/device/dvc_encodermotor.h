#ifndef DVC_ENCODERMOTOR_H
#define DVC_ENCODERMOTOR_H

#include "drv_tim.h"
#include "drv_math.h"
#include "alg_pid.h"
#define RECORD_NUM 20
class Class_EncoderMotor
{
public:

    Class_PID PID_Omage;
    
    void Init(TIM_HandleTypeDef *__TIM_EncoderHandle,uint16_t __encoder_resolution = 4096, uint32_t __max_encoder_sum = 65535 , float __GearRation = 1.0f,float __WheelDiameter = 0.065f); 
    void TIM_Encoder_Calculate();
    void TIM_Calculate_PeriodElapsedCallback();
    inline void  Set_EncoderMotor_Target_Velocity(float __Target_Velocity);
    inline int16_t Get_EncoderMotor_PWMOutput();
    inline float Filter(float new_data, float *data_record);
    inline void Set_EncoderMotor_PWMOutput(int16_t __PWM_Out);
    inline float Get_EncoderMotor_Now_Velocity();
protected:
  //编码器外设句柄
    TIM_HandleTypeDef * TIM_EncoderHandle;
    //编码器最大值 65535
    uint16_t Max_Encoder_Sum;
    //电机输出轴转一圈所输出的脉冲值
    uint32_t Encoder_Resolution = 4096;
    //齿轮比例
    float GearRation;
    //电机直径
    float WheelDiameter;
    //当前脉冲值
    int32_t Now_Encoder;
    //上次脉冲值
    int32_t Last_Encoder;
		//脉冲累加和 ±
    int32_t Encoder_Diff_Sum;
    //脉冲差值
    int32_t Encoder_Diff;
    //当前圈数
    int16_t Total_Round;
    //当前角度
    float Now_Angle;
    float Last_Angle;
    //当前角速度
    float Now_Omega_Angle;//度每秒
    float Now_Velocity;//m/s
    //滤波缓冲区
    float Record_Omega_Angle[RECORD_NUM];

    float Target_Velocity;
    //输出PWM值
    int16_t PWM_Out;
    void OutPut_PWM();
};
void Class_EncoderMotor::Set_EncoderMotor_Target_Velocity(float __Target_Velocity)
{
  Target_Velocity = __Target_Velocity;
}
int16_t Class_EncoderMotor::Get_EncoderMotor_PWMOutput()
{
  return PWM_Out;
}
void Class_EncoderMotor::Set_EncoderMotor_PWMOutput(int16_t __PWM_Out)
{
  PWM_Out = __PWM_Out;
}

float Class_EncoderMotor::Filter(float new_data, float *data_record)
{
    float sum = 0.0;

    for (uint8_t i = RECORD_NUM - 1; i > 0; i--) // 将现有数据后移一位
    {
        data_record[i] = data_record[i - 1];
        sum += data_record[i - 1];
    }
    data_record[0] = new_data; // 第一位是新的数据
    sum += new_data;

    return sum / (RECORD_NUM * 1.0); // 返回均值
}
float Class_EncoderMotor::Get_EncoderMotor_Now_Velocity()
{
  return Now_Velocity;
}
#endif // !DVC_ENCODERMOTOR_H
