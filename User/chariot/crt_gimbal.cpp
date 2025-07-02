#include "crt_gimbal.h"

void Class_Gimbal::Init()
{

    //lk初始化
    LK_Motor.Init(&hcan1,LK_Motor_ID_0x142,0.0f);
    LK_Motor.PID_Angle.Init(10.0f,1.0f,0.0f,0.0f,20.0f,100.0f);
    LK_Motor.PID_Omega.Init(5.0f,1.0f,0.0f,0.0f,50.0f,200.0f);
    //总线舵机初始化
    Yaw.Init(&huart5,0x00,20.0f,20.0f,0.0f);
    Pitch.Init(&huart5,0x01,20.0f,20.0f,0.0f);

    Yaw_PID.Init(0.0018f,0.0f,0.00015f,0.0f,0.0f,60.0f,0.0f,0.0f,0.0f,0.001f);//15
    Pitch_PID.Init(0.001f,0.0001f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,0.001f);//10

    Yaw_Slope.Init(0.08f,0.08f);
    Pitch_Slope.Init(0.04f,0.04f);
}
void Class_Gimbal::Output()
{   
    LK_Motor.Set_LK_Motor_Control_ID(LK_Motor_Control_Torque);

    switch (Gimbal_Control_Type)
    {
    case Gimbal_Control_Type_Disable:
    {
        LK_Motor.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        LK_Motor.Set_Target_Torque(0.0f);
        
        Pitch.Set_Target_Angle(Pitch_Zero_Pos);
        Yaw.Set_Target_Angle(Yaw_Zero_Pos);

    }
    break;
    case Gimbal_Control_Type_Enable:
    {
        LK_Motor.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
        LK_Motor.Set_Target_Angle(Yaw_Lk_Angle);
        //
       
        Pitch_Slope.Set_Target(Target_Pitch);
        Pitch_Slope.TIM_Calculate_PeriodElapsedCallback();

        Yaw_Slope.Set_Target(Target_Yaw);
        Yaw_Slope.TIM_Calculate_PeriodElapsedCallback();

        float tmp_pitch_target_out = Pitch_Slope.Get_Out();
        float tmp_yaw_target_out = Yaw_Slope.Get_Out();
        
        Math_Constrain(&tmp_pitch_target_out, Shoot_Pitch_Min, Shoot_Pitch_Max);
        Math_Constrain(&tmp_yaw_target_out, Shoot_Yaw_Min, Shoot_Yaw_Max);
        Pitch.Set_Target_Angle(tmp_pitch_target_out);
        Yaw.Set_Target_Angle(tmp_yaw_target_out);

        Pitch.Set_Target_Omega_Deg(Target_Pitch_Speed);
        Yaw.Set_Target_Omega_Deg(Target_Yaw_Speed);

    }
    break;
    }
}
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    #ifdef __LK_Motor_Control_OneRound_Position_PIDControler
    LK_Motor.Set_LK_Motor_Control_ID(LK_Motor_Control_OneRound_Position_PIDControler);
    LK_Motor.Set_Target_Angle(Yaw_Lk_Angle);
    LK_Motor.Output();
    #endif
    Output();
    //LK_Motor.TIM_Process_PeriodElapsedCallback();



    static uint8_t count20 = 0;
    count20++;
    if(count20 > 10)
    {   
        static uint8_t count2 = 0;
        count2++;
        
        if(count2 % 3 == 0)
            Yaw.TIM_Process_PeriodElapsedCallback();
        else if(count2 % 3 == 1)
            Pitch.TIM_Process_PeriodElapsedCallback();
        else if(count2 % 3 == 2)
            Yaw.Request_Uart_Servo_Multilaps_Angle();
        count20 = 0;
    }
    
}