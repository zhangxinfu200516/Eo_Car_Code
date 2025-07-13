#include "task.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "drv_math.h"
#include "dvc_imu.h"
class Class_Gimbal
{
public:
    Class_ATK_IMU imu;
    void Init(void);
    void TIM_Calculate_PeriodElapsedCallback(void);
protected:

};

//声明类变量
Class_Gimbal Gimbal;

void Class_Gimbal::Init(void)
{
    //IMU初始化
    imu.Init(&huart3);
    //舵机pwm初始化
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}
uint16_t Get_Servo_Pwm_Form_Angle(float set_angle);
float angle = 0;
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback(void)
{
//    imu.TIM_Unline_Protect_PeriodElapsedCallback();

    // 舵机控制
    static uint8_t tim10ms = 0;
    tim10ms++;
    if (tim10ms > 10)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Get_Servo_Pwm_Form_Angle(angle));
        tim10ms = 0;
    }

    if(huart1.ErrorCode)//蓝牙模块
    {
        HAL_UART_DMAStop(&huart1); // 停止以重启
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if(huart2.ErrorCode)//遥控器模块
    {
        HAL_UART_DMAStop(&huart2); // 停止以重启
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
		    else if(huart3.ErrorCode)//遥控器模块
    {
        HAL_UART_DMAStop(&huart3); // 停止以重启
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }

}
//void Task_UART1_Callback(uint8_t *Buffer, uint16_t Length)
//{
//    
//}
extern "C" void Task_UART2_Callback(uint8_t *Buffer, uint16_t Length)
{
    
}
extern "C" void Task_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    //Gimbal.imu.UART_RxCpltCallback(Buffer);
	uint8_t Gyro_Data[40],j = 0;
			
        if((Buffer[j] == 0x55 && Buffer[j+1] == 0x55) && Buffer[j+2] == 0x01)
        {
            for(j = 0; j < 40; j++)
            {
                Gyro_Data[j] = Buffer[j];
            }
				}
    Gimbal.imu.Data.roll = (float)((int16_t)(Gyro_Data[5] << 8) | Gyro_Data[4]) / 32768.0f * 180.0f;
    Gimbal.imu.Data.pitch = (float)((int16_t)(Gyro_Data[7] << 8) | Gyro_Data[6]) / 32768.0f * 180.0f;
    Gimbal.imu.Data.yaw = (float)((int16_t)(Gyro_Data[9] << 8) | Gyro_Data[8]) / 32768.0f * 180.0f;
    Gimbal.imu.Data.omega_roll = (float)((int16_t)(Gyro_Data[35] << 8) | Gyro_Data[34]) / 32768.0f * 2000.0f;
    Gimbal.imu.Data.omega_pitch = (float)((int16_t)(Gyro_Data[37] << 8) | Gyro_Data[36]) / 32768.0f * 2000.0f;
    Gimbal.imu.Data.omega_yaw = (float)((int16_t)(Gyro_Data[39] << 8) | Gyro_Data[38]) / 32768.0f * 2000.0f;
}

uint16_t Get_Servo_Pwm_Form_Angle(float set_angle)
{   
    Math_Constrain(&set_angle,0.0f,270.0f);
    return (uint16_t)(500.0f + set_angle/270.0f*2000.0f);
}
void Task1ms_TIM1_Callback()
{
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
}

extern "C" void Task_Init(void)
{
    //定时器初始化
    TIM_Init(&htim1,Task1ms_TIM1_Callback);
    
    //串口初始化
    UART_Init(&huart1,Task_UART1_Callback,56);
    UART_Init(&huart2,Task_UART2_Callback,19);
    
    //开启定时器
    HAL_TIM_Base_Start_IT(&htim1);

    Gimbal.Init();

}
void Task_Loop(void)
{
}