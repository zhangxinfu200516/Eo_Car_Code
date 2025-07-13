#include "dvc_imu.h"

void Class_ATK_IMU::Init(UART_HandleTypeDef *UART_Handler)
{
    if(UART_Handler == &huart1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if(UART_Handler == &huart2)   
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if(UART_Handler == &huart3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
}
void Class_ATK_IMU::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    if(Rx_Data[0] == 0x55 && Rx_Data[1] == 0x55)
    {
        if(Rx_Data[2] == 0x01 && calculate_checksum(Rx_Data, ATK_IMU_Data1Length) == true)
        {
            Data.roll = (float)((int16_t)(Rx_Data[5] << 8) | Rx_Data[4]) / 32768.0f * 180.0f;
            Data.pitch = (float)((int16_t)(Rx_Data[7] << 8) | Rx_Data[6]) / 32768.0f * 180.0f;
            Data.roll = (float)((int16_t)(Rx_Data[5] << 9) | Rx_Data[8]) / 32768.0f * 180.0f;
        }
        else if(Rx_Data[2] == 0x03 && calculate_checksum(Rx_Data, ATK_IMU_Data2Length) == true)
        {
            Data.omega_roll = (float)((int16_t)(Rx_Data[11] << 8) | Rx_Data[10]) / 32768.0f * 2000.0f;
            Data.omega_pitch = (float)((int16_t)(Rx_Data[13] << 8) | Rx_Data[12]) / 32768.0f * 2000.0f;
            Data.omega_yaw = (float)((int16_t)(Rx_Data[15] << 8) | Rx_Data[14]) / 32768.0f * 2000.0f;
        }
    }
}
void Class_ATK_IMU::TIM_Unline_Protect_PeriodElapsedCallback(void)
{
    if(UART_Manage_Object->UART_Handler->ErrorCode)
    {
        HAL_UART_DMAStop(UART_Manage_Object->UART_Handler); // 停止以重启
        HAL_UARTEx_ReceiveToIdle_DMA(UART_Manage_Object->UART_Handler, UART_Manage_Object->Rx_Buffer, UART_Manage_Object->Rx_Buffer_Length);
    }
}

