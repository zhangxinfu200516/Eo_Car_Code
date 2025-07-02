#include "ita_car.h"
#include "config.h"

#define encoder_resolution 400
int64_t Time_count = 0;
void Class_EoCar::Init()
{
    //底盘初始化
    Chassis.Init();
    Chassis_PID_Omage.Init(0.4f,0.0f,0.01f,0.0f,0.0f,4.0f);
    Chassis_Slope.Init(0.25f,0.25f);

    Gimbal.Init();

    N100.Init(&huart6);

    Remote.Init(&huart2);

    MiniPC.Init_UART(&huart4);
    MiniPC.Chassis = &Chassis;

    OLED.Init();

    FSM_MINIPC_Alive_Protect.Init(2,0);
    FSM_N100_Alive_Protect.Init(2,0);
    FSM_Bluetooth_Alive_Protect.Init(2,0);

    FSM_Control_Gimbal.Init(4,0);
}
void Class_EoCar::Control_Chassis()
{
    float tmp_chassis_vx,tmp_chassis_vy,tmp_chassis_w;
    float Tmp_Target_Chassis_Yaw;
    #ifdef MINIPC_ENABLE
    if(MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && (MiniPC.Get_Minipc_flags() & 0x02) == 1)
    {
        tmp_chassis_vx = MiniPC.Get_Minipc_Linear_X();
        tmp_chassis_vy = MiniPC.Get_Minipc_Linear_Y();
    }
    //tmp_chassis_w = MiniPC.Get_Minipc_Angular_Z();
    #elif defined(REMOTE_ENABLE)
    tmp_chassis_vx = Remote.Get_Remote_Left_Y() * Chassis.Get_Velocity_X_Max();
    tmp_chassis_vy = Remote.Get_Remote_Left_X() * -Chassis.Get_Velocity_Y_Max();
    #endif
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
    {
        if ((Remote.Get_Button_One() >> 2) & 0x01)
        {
            tmp_chassis_vx = Remote.Get_Remote_Left_Y() * Chassis.Get_Velocity_X_Max();
            tmp_chassis_vy = Remote.Get_Remote_Left_X() * -Chassis.Get_Velocity_Y_Max();
            tmp_chassis_w = Remote.Get_Remote_Right_X() * Chassis.Get_Omega_Max() * 0.05f;

            Tmp_Target_Chassis_Yaw = Get_Target_Chassis_Yaw();
            Tmp_Target_Chassis_Yaw += tmp_chassis_w;
            while (Target_Chassis_Yaw > 180.0f)
            {
                Target_Chassis_Yaw -= 360.0f;
            }
            while (Target_Chassis_Yaw < -180.0f)
            {
                Target_Chassis_Yaw += 360.0f;
            }
            Set_Target_Chassis_Yaw(Tmp_Target_Chassis_Yaw);
        }
        else
        {
            if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
            {
                tmp_chassis_vx = MiniPC.Get_Minipc_Linear_X();
                tmp_chassis_vy = MiniPC.Get_Minipc_Linear_Y();
            }
            Tmp_Target_Chassis_Yaw = Get_Target_Chassis_Yaw();
            Set_Target_Chassis_Yaw(Tmp_Target_Chassis_Yaw);

        }
    }
    else
    {
        if ((Remote.Get_Button_One() >> 3) & 0x01)
        {
            tmp_chassis_vx = Remote.Get_Remote_Left_Y() * Chassis.Get_Velocity_X_Max();
            tmp_chassis_vy = Remote.Get_Remote_Left_X() * -Chassis.Get_Velocity_Y_Max();
            tmp_chassis_w = Remote.Get_Remote_Right_X() * Chassis.Get_Omega_Max() * 0.05f;

            Tmp_Target_Chassis_Yaw = Get_Target_Chassis_Yaw();
            Tmp_Target_Chassis_Yaw += tmp_chassis_w;
            while (Target_Chassis_Yaw > 180.0f)
            {
                Target_Chassis_Yaw -= 360.0f;
            }
            while (Target_Chassis_Yaw < -180.0f)
            {
                Target_Chassis_Yaw += 360.0f;
            }
            Set_Target_Chassis_Yaw(Tmp_Target_Chassis_Yaw);
        }
    }
    // float Tmp_Target_Chassis_Yaw = (Gimbal.Yaw.Get_Now_Angle() - Yaw_Zero_Pos);
    Tmp_Target_Chassis_Yaw = Get_Target_Chassis_Yaw();
    Chassis_Slope.Set_Target(Tmp_Target_Chassis_Yaw);
    Chassis_Slope.TIM_Calculate_PeriodElapsedCallback();
    Set_Target_Chassis_Yaw(Chassis_Slope.Get_Out());
    
    //N100航向角度处理
    if(N100.Get_N100_Heading_Angle() > 180.0f)
    {
        Set_Actual_Chassis_Yaw(N100.Get_N100_Heading_Angle() - 360.0f);
    }
    else
    {
        Set_Actual_Chassis_Yaw(N100.Get_N100_Heading_Angle());
    }
    //底盘航向正方向矫正
    Chassis_PID_Omage.Set_Target(Target_Chassis_Yaw);
    Chassis_PID_Omage.Set_Now(Actual_Chassis_Yaw);
    Chassis_PID_Omage.TIM_Adjust_PeriodElapsedCallback();
    tmp_chassis_w = -Chassis_PID_Omage.Get_Out();

    Chassis.Set_Velocity_X(tmp_chassis_vx);
    Chassis.Set_Velocity_Y(tmp_chassis_vy);
    Chassis.Set_Omega(tmp_chassis_w);
}
float K = 0.1;
uint8_t flag_3 ;
void Class_EoCar::Control_Gimbal()
{   
    flag_3 = ((MiniPC.Get_Minipc_flags() >> 2) & 0x01);

    
    static float yunhang_K = K * 1.25f / 2.0f;
    static float yunhang_K_pitch = K / 2.0f;
    //if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && ((MiniPC.Get_Minipc_flags() >> 1) & 0x01) == 0)
#ifdef DEBUG
    if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
    {
        if ((MiniPC.Get_Minipc_flags() & 0x01) == 1)
        {
            // 设置minipc发送的像素差
            Gimbal.Set_pixel_dx(MiniPC.Get_process_pixel_dx());
            Gimbal.Set_pixel_dy(MiniPC.Get_process_pixel_dy());
            // yaw
            Gimbal.Yaw_PID.Set_Target(0.0f);
            Gimbal.Yaw_PID.Set_Now(Gimbal.Get_pixel_dx());
            Gimbal.Yaw_PID.TIM_Adjust_PeriodElapsedCallback();
            float Tmp_Yaw = Gimbal.Get_Target_Yaw_Angle();
            Tmp_Yaw += -Gimbal.Yaw_PID.Get_Out();
            Math_Constrain(&Tmp_Yaw, Shoot_Yaw_Min, Shoot_Yaw_Max);
            Gimbal.Set_Target_Yaw_Angle(Tmp_Yaw);
            // //pitch
            Gimbal.Pitch_PID.Set_Target(0.0f);
            Gimbal.Pitch_PID.Set_Now(Gimbal.Get_pixel_dy());
            Gimbal.Pitch_PID.TIM_Adjust_PeriodElapsedCallback();
            float Tmp_Pitch = Gimbal.Get_Target_Pitch_Angle();
            Tmp_Pitch += -Gimbal.Pitch_PID.Get_Out();
            Math_Constrain(&Tmp_Pitch,Shoot_Pitch_Min,Shoot_Pitch_Max);
            Gimbal.Set_Target_Pitch_Angle(Tmp_Pitch);
        }
        else
        {
            // 设置minipc发送的像素差
            Gimbal.Set_pixel_dx(0.0f);
            Gimbal.Set_pixel_dy(0.0f);
            // yaw
            Gimbal.Yaw_PID.Set_Target(0.0f);
            Gimbal.Yaw_PID.Set_Now(Gimbal.Get_pixel_dx());
            Gimbal.Yaw_PID.TIM_Adjust_PeriodElapsedCallback();
            float Tmp_Yaw = Gimbal.Get_Target_Yaw_Angle();
            Tmp_Yaw += -Gimbal.Yaw_PID.Get_Out();
            Math_Constrain(&Tmp_Yaw, Shoot_Yaw_Min, Shoot_Yaw_Max);
            Gimbal.Set_Target_Yaw_Angle(Tmp_Yaw);
            // //pitch
            Gimbal.Pitch_PID.Set_Target(0.0f);
            Gimbal.Pitch_PID.Set_Now(Gimbal.Get_pixel_dy());
            Gimbal.Pitch_PID.TIM_Adjust_PeriodElapsedCallback();
            float Tmp_Pitch = Gimbal.Get_Target_Pitch_Angle();
            Tmp_Pitch += -Gimbal.Pitch_PID.Get_Out();
            Math_Constrain(&Tmp_Pitch,Shoot_Pitch_Min,Shoot_Pitch_Max);
            Gimbal.Set_Target_Pitch_Angle(Tmp_Pitch);


            // if (((MiniPC.Get_Minipc_flags() >> 2) & 0x01) == 1)
            // {
            //     // 定在现在位置
            //     float now_tmp_yaw = Gimbal.Get_Target_Yaw_Angle();
            //     Math_Constrain(&now_tmp_yaw, Shoot_Yaw_Min,Shoot_Yaw_Max);
            //     float now_tmp_pitch = Gimbal.Get_Target_Pitch_Angle();
            //     Math_Constrain(&now_tmp_pitch, Shoot_Pitch_Min, Shoot_Pitch_Max);
            //     Gimbal.Set_Target_Yaw_Angle(now_tmp_yaw);
            //     Gimbal.Set_Target_Pitch_Angle(now_tmp_pitch);
            // }
            // else
            // {
            //     float yaw_xunhang = Gimbal.Get_Target_Yaw_Angle();
            //     yaw_xunhang += yunhang_K;
            //     if (yaw_xunhang <= Yaw_Min)
            //         yunhang_K = fabsf(yunhang_K);
            //     else if (yaw_xunhang >= Yaw_Max)
            //         yunhang_K = -fabsf(yunhang_K);

            //     float pitch_xunhang = Gimbal.Get_Target_Pitch_Angle();
            //     pitch_xunhang += yunhang_K_pitch;
            //     if (pitch_xunhang <= Pitch_Min)
            //         yunhang_K_pitch = fabsf(yunhang_K_pitch);
            //     else if (pitch_xunhang >=  Pitch_Max)
            //         yunhang_K_pitch = -fabsf(yunhang_K_pitch);

            //     // 云台控制
            //     Math_Constrain(&yaw_xunhang, Shoot_Yaw_Min, Shoot_Yaw_Max);
            //     Gimbal.Set_Target_Yaw_Angle(yaw_xunhang);
            //     Math_Constrain(&pitch_xunhang,Pitch_Min,Pitch_Max);
            //     Gimbal.Set_Target_Pitch_Angle(pitch_xunhang);
            // }
        }
    }
    else
    {
        float yaw_xunhang_1 = Gimbal.Get_Target_Yaw_Angle();
        yaw_xunhang_1 += yunhang_K;
        if (yaw_xunhang_1 <= Yaw_Min)
            yunhang_K = fabsf(yunhang_K);
        else if (yaw_xunhang_1 >= Yaw_Max)
            yunhang_K = -fabsf(yunhang_K);

        float pitch_xunhang_1 = Gimbal.Get_Target_Pitch_Angle();
        pitch_xunhang_1 += yunhang_K_pitch;
        if (pitch_xunhang_1 <= Pitch_Min)
            yunhang_K_pitch = fabsf(yunhang_K_pitch);
        else if (pitch_xunhang_1 >= Pitch_Max )
            yunhang_K_pitch = -fabsf(yunhang_K_pitch);
        // 云台控制
        Math_Constrain(&yaw_xunhang_1, Shoot_Yaw_Min , Shoot_Yaw_Max);
        Gimbal.Set_Target_Yaw_Angle(yaw_xunhang_1);
        Math_Constrain(&pitch_xunhang_1,Pitch_Min,Pitch_Max);
        Gimbal.Set_Target_Pitch_Angle(pitch_xunhang_1);
    }
#endif
    FSM_Control_Gimbal.Status[FSM_Control_Gimbal.Get_Now_Status_Serial()].Time++;
    switch (FSM_Control_Gimbal.Get_Now_Status_Serial())
    {
    case 0:
    {
        Gimbal.Set_Target_Pitch_Angle(Pitch_Zero_Pos);
        Gimbal.Set_Target_Yaw_Angle(Yaw_Zero_Pos);

        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && (MiniPC.Get_Minipc_flags() & 0x01) == 1)
        {
            FSM_Control_Gimbal.Set_Status(1);//自瞄
        }

        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
        {
            FSM_Control_Gimbal.Set_Status(3); // 巡航
        }

    }
    break;
    case 1://自瞄
    {
        // 设置minipc发送的像素差
        Gimbal.Set_pixel_dx(MiniPC.Get_process_pixel_dx());
        Gimbal.Set_pixel_dy(MiniPC.Get_process_pixel_dy());
        // yaw
        Gimbal.Yaw_PID.Set_Target(0.0f);
        Gimbal.Yaw_PID.Set_Now(Gimbal.Get_pixel_dx());
        Gimbal.Yaw_PID.TIM_Adjust_PeriodElapsedCallback();
        float Tmp_Yaw = Gimbal.Get_Target_Yaw_Angle();
        Tmp_Yaw += -Gimbal.Yaw_PID.Get_Out();
        Math_Constrain(&Tmp_Yaw, Shoot_Yaw_Min, Shoot_Yaw_Max);
        Gimbal.Set_Target_Yaw_Angle(Tmp_Yaw);
        // //pitch
        Gimbal.Pitch_PID.Set_Target(0.0f);
        Gimbal.Pitch_PID.Set_Now(Gimbal.Get_pixel_dy());
        Gimbal.Pitch_PID.TIM_Adjust_PeriodElapsedCallback();
        float Tmp_Pitch = Gimbal.Get_Target_Pitch_Angle();
        Tmp_Pitch += -Gimbal.Pitch_PID.Get_Out();
        Math_Constrain(&Tmp_Pitch, Shoot_Pitch_Min, Shoot_Pitch_Max);
        Gimbal.Set_Target_Pitch_Angle(Tmp_Pitch);

        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && (MiniPC.Get_Minipc_flags() & 0x01) == 0)
        {
            FSM_Control_Gimbal.Set_Status(2); // 保持
        }

        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
        {
            FSM_Control_Gimbal.Set_Status(3); // 巡航
        }
    }
    break;
    case (2): // 保持
    {
        if (FSM_Control_Gimbal.Status[FSM_Control_Gimbal.Get_Now_Status_Serial()].Time > 300)
        {
            FSM_Control_Gimbal.Set_Status(3);//巡航
        }
        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && (MiniPC.Get_Minipc_flags() & 0x01) == 1)
        {
            FSM_Control_Gimbal.Set_Status(1); // 自瞄
        }

        // 设置minipc发送的像素差
        Gimbal.Set_pixel_dx(0.0f);
        Gimbal.Set_pixel_dy(0.0f);
        // yaw
        Gimbal.Yaw_PID.Set_Target(0.0f);
        Gimbal.Yaw_PID.Set_Now(Gimbal.Get_pixel_dx());
        Gimbal.Yaw_PID.TIM_Adjust_PeriodElapsedCallback();
        float Tmp_Yaw = Gimbal.Get_Target_Yaw_Angle();
        Tmp_Yaw += -Gimbal.Yaw_PID.Get_Out();
        Math_Constrain(&Tmp_Yaw, Shoot_Yaw_Min, Shoot_Yaw_Max);
        Gimbal.Set_Target_Yaw_Angle(Tmp_Yaw);
        // //pitch
        Gimbal.Pitch_PID.Set_Target(0.0f);
        Gimbal.Pitch_PID.Set_Now(Gimbal.Get_pixel_dy());
        Gimbal.Pitch_PID.TIM_Adjust_PeriodElapsedCallback();
        float Tmp_Pitch = Gimbal.Get_Target_Pitch_Angle();
        Tmp_Pitch += -Gimbal.Pitch_PID.Get_Out();
        Math_Constrain(&Tmp_Pitch, Shoot_Pitch_Min, Shoot_Pitch_Max);
        Gimbal.Set_Target_Pitch_Angle(Tmp_Pitch);
    }
    break;
    case (3): // 巡航
    {
        float yaw_xunhang_1 = Gimbal.Get_Target_Yaw_Angle();
        yaw_xunhang_1 += yunhang_K;
        if (yaw_xunhang_1 <= Yaw_Min)
            yunhang_K = fabsf(yunhang_K);
        else if (yaw_xunhang_1 >= Yaw_Max)
            yunhang_K = -fabsf(yunhang_K);

        float pitch_xunhang_1 = Gimbal.Get_Target_Pitch_Angle();
        pitch_xunhang_1 += yunhang_K_pitch;
        if (pitch_xunhang_1 <= Pitch_Min)
            yunhang_K_pitch = fabsf(yunhang_K_pitch);
        else if (pitch_xunhang_1 >= Pitch_Max)
            yunhang_K_pitch = -fabsf(yunhang_K_pitch);
        // 云台控制
        Math_Constrain(&yaw_xunhang_1, Shoot_Yaw_Min, Shoot_Yaw_Max);
        Gimbal.Set_Target_Yaw_Angle(yaw_xunhang_1);
        Math_Constrain(&pitch_xunhang_1, Shoot_Pitch_Min, Shoot_Pitch_Max);
        Gimbal.Set_Target_Pitch_Angle(pitch_xunhang_1);

        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && (MiniPC.Get_Minipc_flags() & 0x01) == 1)
        {
            FSM_Control_Gimbal.Set_Status(1); // 自瞄
        }
    }
    break;
    }
}

void Class_EoCar::TIM_Calculate_PeriodElapsedCallback()
{
    //1ms
    Time_count++;
    //N100处理数据
    N100.Data_Process();

    Control_Chassis();

    // static uint8_t Gimbal_cnt = 0;
    // Gimbal_cnt ++;
    // if(Gimbal_cnt > 2)
    // {
    //控制云台
    Control_Gimbal();
    //     Gimbal_cnt = 0;
    // }
    //MINIPC控制
    static uint8_t Minipc_cnt = 0,Minipc_cnt_live = 0;
    Minipc_cnt ++;
    if(Minipc_cnt > 50)
    {   
        Minipc_cnt_live++;
        if(Minipc_cnt_live > 2)
        {
            //10Hz存活检测
            MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
            //N100存活检测
            N100.TIM1msMod50_Alive_PeriodElapsedCallback();
            
            Minipc_cnt_live = 0;
        }
        
        //离线状态机 MINIPC N100
        TIM_Unline_Protect_PeriodElapsedCallback();
        TIM_N100_Unline_Protect_PeriodElapsedCallback();
        TIM_Bluetooth_Unline_Protect_PeriodElapsedCallback();
        //MINIPC_TX数据
        MiniPC.TIM_Write_PeriodElapsedCallback();
        Minipc_cnt = 0;
    }
    //MINIPC离线检测


    //善后函数
    Chassis.TIM_Calculate_PeriodElapsedCallback();
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    
    //OLED显示
    static uint8_t count100 = 0;
    count100 ++;
    if(count100 > 100)
    {
        int32_t tmp_voltage = (int32_t)(Get_Voltage() * 100.0f);
        OLED.OLED_ShowString(0, 0, (const uint8_t*)"Power: ");
        OLED.OLED_ShowNumber(50, 0, tmp_voltage / 100,2,12);
        OLED.OLED_ShowString(63, 0, (const uint8_t*)".");
        OLED.OLED_ShowNumber(73, 0, tmp_voltage % 100,2,12);

        //imu
        if(N100.Get_N100_Live_Status() == N100_Live_Status_ENABLE)
        {
            OLED.OLED_ShowString(0, 10, (const uint8_t*)"IMU_Yaw: OK___ ");
        }
        else
        {
            OLED.OLED_ShowString(0, 10, (const uint8_t*)"IMU_Yaw: ERROR ");
        }

        int32_t tmp_Odom_X = (int32_t)(Chassis.Get_Now_Position_X() * 100.0f);
        if(tmp_Odom_X >= 0)
        {
            OLED.OLED_ShowString(0,  20, (const uint8_t*)"Odom_X: +");
            OLED.OLED_ShowNumber(64+10, 20, tmp_Odom_X / 100,2,12);
            OLED.OLED_ShowString(64+13+10, 20, (const uint8_t*)".");
            OLED.OLED_ShowNumber(64+23+10, 20, tmp_Odom_X % 100,2,12);
        }
        else
        {
            OLED.OLED_ShowString(0,  20, (const uint8_t*)"Odom_X: -");
            OLED.OLED_ShowNumber(64+10, 20, -tmp_Odom_X / 100,2,12);
            OLED.OLED_ShowString(64+13+10, 20, (const uint8_t*)".");
            OLED.OLED_ShowNumber(64+23+10, 20, -tmp_Odom_X % 100,2,12);
        }

        int32_t tmp_Odom_Y = (int32_t)(Chassis.Get_Now_Position_Y() * 100.0f);
        if(tmp_Odom_Y >= 0)
        {
            OLED.OLED_ShowString(0,  30, (const uint8_t*)"Odom_Y: +");
            OLED.OLED_ShowNumber(64+10, 30, tmp_Odom_Y / 100,2,12);
            OLED.OLED_ShowString(64+13+10, 30, (const uint8_t*)".");
            OLED.OLED_ShowNumber(64+23+10, 30, tmp_Odom_Y % 100,2,12);
        }
        else
        {
            OLED.OLED_ShowString(0,  30, (const uint8_t*)"Odom_Y: -");
            OLED.OLED_ShowNumber(64+10, 30, -tmp_Odom_Y / 100,2,12);
            OLED.OLED_ShowString(64+13+10, 30, (const uint8_t*)".");
            OLED.OLED_ShowNumber(64+23+10, 30, -tmp_Odom_Y % 100,2,12);
        }

        OLED.OLED_Refresh_Gram();

        count100 = 0;
    }
    // //蜂鸣器
    // static uint8_t Buzzer_Flag = 0;
    // if (Time_count < 50 && Buzzer_Flag == 0)
    // {
    //     //蜂鸣
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    // }
    // else if (Time_count >= 51 && Time_count < 100 && Buzzer_Flag == 0)
    // {
    //     //停止蜂鸣
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    //     Buzzer_Flag = 1;
    // }

    if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
    {
        static int16_t Buzzer_timcnt = 0; // 1ms
        Buzzer_timcnt++;
        if (Buzzer_timcnt > 400)
            Buzzer_timcnt = 0;
    
        if (Buzzer_timcnt <= 50)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    
    // if(MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE && ((MiniPC.Get_Minipc_flags() >> 3 ) && 0x01) == 1)
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    // }
    
#ifdef BUZZER_ENABLE
    static uint16_t Buzzer_timcnt = 0; // 1ms
    Buzzer_timcnt++;
    if (Buzzer_timcnt > 400)
        Buzzer_timcnt = 0;
    if (Buzzer_timcnt <= 50)
        // 蜂鸣
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    #endif
}

void Class_EoCar::TIM_Unline_Protect_PeriodElapsedCallback()
{
    //FSM_VT13_Alive_Protect.Status[FSM_VT13_Alive_Protect.Get_Now_Status_Serial()].Time++;
    switch (FSM_MINIPC_Alive_Protect.Get_Now_Status_Serial())
    {
    case 0:
    {
        if(huart4.ErrorCode)
        {
            FSM_MINIPC_Alive_Protect.Set_Status(1);
        }
    }
    break;
    case 1:
    {
        HAL_UART_DMAStop(&huart4); // 停止以重启
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
        
        FSM_MINIPC_Alive_Protect.Set_Status(0);
    }
    break;
    }
}

void Class_EoCar::TIM_N100_Unline_Protect_PeriodElapsedCallback()
{
     switch (FSM_N100_Alive_Protect.Get_Now_Status_Serial())
    {
    case 0:
    {
        if(huart6.ErrorCode)
        {
            FSM_N100_Alive_Protect.Set_Status(1);
        }
    }
    break;
    case 1:
    {
        HAL_UART_DMAStop(&huart6); // 停止以重启
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
        
        FSM_N100_Alive_Protect.Set_Status(0);
    }
    break;
    }
}

void Class_EoCar::TIM_Bluetooth_Unline_Protect_PeriodElapsedCallback()
{
    switch (FSM_Bluetooth_Alive_Protect.Get_Now_Status_Serial())
    {
    case 0:
    {
        if (huart2.ErrorCode)
        {
            FSM_Bluetooth_Alive_Protect.Set_Status(1);
        }
    }
    break;
    case 1:
    {
        HAL_UART_DMAStop(&huart2); // 停止以重启

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);

        FSM_Bluetooth_Alive_Protect.Set_Status(0);
    }
    break;
    }
}
