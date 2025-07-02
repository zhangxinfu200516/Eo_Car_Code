#include "task.h"
#include "ita_car.h"  
#include "adc.h"
#include "dvc_dwt.h"

uint8_t CanRxFifoBuffer[7][8];
Class_EoCar EoCar;
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x142):
    {
        EoCar.Gimbal.LK_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    default:
    {
        EoCar.N100.CAN_RxCpltCallback(CAN_RxMessage->Data);
        // static uint8_t acc_flag = 0;
        // static uint8_t data_num = 0;
        // static uint8_t last_count = 0;
        // if (CAN_RxMessage->Data[0] == 0xFC && last_count == 0xFD)
        // {
        //     data_num = 1;
        //     if (CAN_RxMessage->Data[1] == 0x41 && CAN_RxMessage->Data[2] == 0x30)
        //         acc_flag = 1;
        // }
        // last_count = CAN_RxMessage->Data[7];

        // if (data_num)
        // {
        //     for (auto i = 0; i < 8; i++)
        //     {
        //         CanRxFifoBuffer[data_num - 1][i] = CAN_RxMessage->Data[i];
        //     }
        //     data_num++;
        // }

        // if (acc_flag == 1 && data_num == 8)
        // {
        //     data_num = 0;
        //     acc_flag = 0;

        //     for (auto i = 0; i < 7; i++)
        //     {
        //         for (auto j = 0; j < 8; j++)
        //         {
        //             reinterpret_cast<uint8_t *>(&EoCar.N100.Frame_Format)[i * 8 + j] = CanRxFifoBuffer[i][j];
        //         }
        //     }
        // }
    }
    break;
    }
}
float FPS;
uint32_t last_cnt;
void Task_UART4_Callback(uint8_t *Buffer, uint16_t Length)
{
    FPS = DWT_GetDeltaT(&last_cnt);
    EoCar.MiniPC.UART_RxCpltCallback(Buffer);

}

void Task_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    //总线舵机
    if(Buffer[0] == 0x05 && Buffer[1] == 0x1c)
    {
        EoCar.Gimbal.Yaw.CAN_RxCpltCallback(Buffer);
    }

}

void Task_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    EoCar.N100.UART_RxCpltCallback(Buffer);
}

void Task_UART2_Callback(uint8_t *Buffer, uint16_t Length)
{

    //蓝牙遥控
    if(Buffer[0] == 0xAA && Buffer[1] == 0x55)
    {
        // if(Buffer[10] == EoCar.Remote.calculate_checksum(Buffer,10))
        // {
        //     EoCar.Remote.CAN_RxCpltCallback(Buffer);
        // }
        if(Buffer[9] == EoCar.Remote.calculate_checksum(Buffer,9))
        {
            EoCar.Remote.CAN_RxCpltCallback(Buffer);
        }
    }
}

int16_t PWM = 500;
float V = 5.0f;
void Task1ms_TIM14_Callback()
{
    EoCar.TIM_Calculate_PeriodElapsedCallback();

    if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3) == GPIO_PIN_SET)
    {
        EoCar.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_Enable);
        EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
    }
    else
    {
        EoCar.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_Enable);
        EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

        if((EoCar.Remote.Get_Button_One() >> 3) & 0x01)
        {
            EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
    }

    // 3.3V输出
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, (int16_t)(1000.0f * V / 5.0f)); // 相当于一个周期内（20ms）有1ms高脉冲

}
void Task1ms_TIM12_Callback()
{
    
}
extern "C" void Task_Init(void)
{
    DWT_Init(168);
    //集中总线can1/can2
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    //定时器循环任务
    TIM_Init(&htim14, Task1ms_TIM14_Callback);
    TIM_Init(&htim12, Task1ms_TIM12_Callback);
    //串口初始化
     UART_Init(&huart4, Task_UART4_Callback, Rx_Package_Length); 
     UART_Init(&huart5, Task_UART5_Callback, 20); 
     UART_Init(&huart6, Task_UART6_Callback, 64); 
     UART_Init(&huart2, Task_UART2_Callback, 20); 
     
    /********************************* 交互层初始化 *********************************/
    EoCar.Init();
    /********************************* 使能调度时钟 *********************************/
    HAL_TIM_Base_Start_IT(&htim14);
    //HAL_TIM_Base_Start_IT(&htim12);
    //PWM初始化
    //编码器电机
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    //舵机
    
    //3.3v供电
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    
    //adc初始化
    HAL_ADC_Start_IT(&hadc1);
}
int16_t Power_Data;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 电阻分压，具体根据原理图简单分析可以得到
    const float Revise = 0.99f;
    Power_Data = HAL_ADC_GetValue(&hadc1);
    EoCar.Set_Voltage((float)Power_Data * 3.3f * 11.0f * Revise / 1.0f / 4096.0f);

    HAL_ADC_Start_IT(&hadc1); // 中断模式重启
}
void Task_Loop(void)
{
}