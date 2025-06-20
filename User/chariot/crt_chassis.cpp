#include "crt_chassis.h"

void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    //轮向电机初始化
    E_Motor[0].Init(&htim2,encoder_resolution,65535,GearRation,WheelDiameter);
    E_Motor[1].Init(&htim3,encoder_resolution,65535,GearRation,WheelDiameter);   
    E_Motor[2].Init(&htim4,encoder_resolution,65535,GearRation,WheelDiameter);
    E_Motor[3].Init(&htim5,encoder_resolution,65535,GearRation,WheelDiameter);
    //电机PID初始化
    E_Motor[0].PID_Omage.Init(100000.0f,100.0f,5.0f,0.0f,2000.0f,20000.0f);
    E_Motor[1].PID_Omage.Init(100000.0f,100.0f,5.0f,0.0f,2000.0f,20000.0f);
    E_Motor[2].PID_Omage.Init(100000.0f,100.0f,5.0f,0.0f,2000.0f,20000.0f);
    E_Motor[3].PID_Omage.Init(100000.0f,100.0f,5.0f,0.0f,2000.0f,20000.0f);
}   
void Class_Tricycle_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 四轮子自锁
            for (int i = 0; i < 4; i++)
            { 
                E_Motor[i].Set_EncoderMotor_PWMOutput(0);
            }            
            
        }
        break;
		case (Chassis_Control_Type_SPIN_Positive) :
        case (Chassis_Control_Type_SPIN_Negative):
        case (Chassis_Control_Type_FLLOW):
        {
            //底盘四电机模式配置
            //底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
            }
            #ifdef MAI_WHEEL_CHASSIS
            //速度换算，正运动学分解 1:左上角 2:左下角 3:右下角 4:右上角
            float motor1_temp_linear_vel = -Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor3_temp_linear_vel = -Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor4_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            //角速度*减速比  设定目标 直接给到电机输出轴
            E_Motor[0].Set_EncoderMotor_Target_Velocity(-motor1_temp_linear_vel);
            E_Motor[1].Set_EncoderMotor_Target_Velocity(-motor2_temp_linear_vel);
            E_Motor[2].Set_EncoderMotor_Target_Velocity(motor3_temp_linear_vel);
            E_Motor[3].Set_EncoderMotor_Target_Velocity(motor4_temp_linear_vel);
            #elif defined(TRICYCLE_CHASSIS)
            static float K = sqrt(2.0f)/2.0f; 
            //速度正解算，正运动学分解 1:左上角 2:左下角 3:右下角 4:右上角
            float motor1_temp_linear_vel =  K * Target_Velocity_Y - K * Target_Velocity_X + Target_Omega * (Chassis_radius);
            float motor2_temp_linear_vel = -K * Target_Velocity_Y - K * Target_Velocity_X + Target_Omega * (Chassis_radius);
            float motor3_temp_linear_vel = -K * Target_Velocity_Y + K * Target_Velocity_X + Target_Omega * (Chassis_radius);
            float motor4_temp_linear_vel =  K * Target_Velocity_Y + K * Target_Velocity_X + Target_Omega * (Chassis_radius);
            //角速度*减速比  设定目标 直接给到电机输出轴
            E_Motor[0].Set_EncoderMotor_Target_Velocity(-motor1_temp_linear_vel);
            E_Motor[1].Set_EncoderMotor_Target_Velocity(-motor2_temp_linear_vel);
            E_Motor[2].Set_EncoderMotor_Target_Velocity(-motor3_temp_linear_vel);
            E_Motor[3].Set_EncoderMotor_Target_Velocity(-motor4_temp_linear_vel);
            //速度反解算
            float m1 = -E_Motor[0].Get_EncoderMotor_Now_Velocity();
            float m2 = -E_Motor[1].Get_EncoderMotor_Now_Velocity();
            float m3 = -E_Motor[2].Get_EncoderMotor_Now_Velocity();
            float m4 = -E_Motor[3].Get_EncoderMotor_Now_Velocity();
            // 逆运动学解算
            Now_Velocity_X = ((m4 - m1) + (m3 - m2)) / (4.0f * K);
            Now_Velocity_Y = ((m1 - m2) + (m4 - m3)) / (4.0f * K);
            Now_Omega = (m1 + m2 + m3 + m4) / (4.0f * Chassis_radius);

            // 速度积分得到位置
            //梯形积分更精准
            const float Dt = 0.001f;//1ms
            // 添加速度死区，避免微小速度漂移
            const float VEL_DEADZONE = 0.01f; // 根据实际情况调整
            if (fabs(Now_Velocity_X) < VEL_DEADZONE)
                Now_Velocity_X = 0;
            if (fabs(Now_Velocity_Y) < VEL_DEADZONE)
                Now_Velocity_Y = 0;
            Now_Position_X += (Pre_Velocity_X + Now_Velocity_X) * 0.5f * Dt;
            Now_Position_Y += (Pre_Velocity_Y + Now_Velocity_Y) * 0.5f * Dt;

            // 前一项赋值
            Pre_Velocity_X = Now_Velocity_X;
            Pre_Velocity_Y = Now_Velocity_Y;
            #endif // DEBUG
            //各个电机具体PID
            for (int i = 0; i < 4; i++)
            {
                E_Motor[i].TIM_Calculate_PeriodElapsedCallback();
            }
        }
        break;
    }   
}
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
    Speed_Resolution();
    //电机输出力矩纠正
    //偶数为左侧轮子 奇数为右侧轮子
    for (auto i = 0; i < 4; i++)
    {
        int16_t PWM_Out = E_Motor[i].Get_EncoderMotor_PWMOutput();
        //int16_t PWM_Out = 0;
        #ifdef MAI_WHEEL_CHASSIS
        if (i == 0)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, PWM_Out);
            }
        }
        else if (i == 1)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, -PWM_Out);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
            }
        }
        else if (i == 2)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_Out);
            }
        }
        else if (i == 3)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -PWM_Out);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            }
        }
        #elif defined(TRICYCLE_CHASSIS)
                if (i == 0)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, PWM_Out);
            }
        }
        else if (i == 1)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -PWM_Out);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            }
        }
        else if (i == 2)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Out);
            }
        }
        else if (i == 3)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -PWM_Out);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            }
        }
        #endif
    }
}