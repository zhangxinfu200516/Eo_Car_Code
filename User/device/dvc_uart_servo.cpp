#include "dvc_uart_servo.h"
#include <string.h>
void Class_Uart_Servo::Init(UART_HandleTypeDef *huart,uint8_t __Servo_ID,float __AccInterval_ms,float __DecInterval_ms,float __Power_mW)
{
    if(huart == &huart4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if(huart == &huart5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    Servo_ID = __Servo_ID;
    AccInterval_ms = __AccInterval_ms;
    DecInterval_ms = __DecInterval_ms;
    Power_mW = __Power_mW;
    
}
void Class_Uart_Servo::Data_Process()
{
    Now_Angle = (float)Rx_Servo_Data.angle / 10.0f;
}
void Class_Uart_Servo::CAN_RxCpltCallback(uint8_t *Rx_Data)
{

    if (Rx_Data[2] == FSUS_CMD_QUERY_SERVO_ANGLE_MTURN && (Rx_Data[11] == calculate_checksum(Rx_Data, 11)))
    {
        if (Rx_Data[4] == Servo_ID)
        {
            memcpy(&Rx_Servo_Data, &Rx_Data[4], sizeof(Rx_Servo_Data));

            Data_Process();
        }
    }
}
void Class_Uart_Servo::Request_Uart_Servo_Multilaps_Angle()
{
    Tx_Pack_Data.header = Tx_Frame;
    Tx_Pack_Data.cmdId = FSUS_CMD_QUERY_SERVO_ANGLE_MTURN;
    Tx_Pack_Data.Data_Length = 0x01;
    Tx_Pack_Data.Data_Content[0] = Servo_ID;

    Tx_Pack_Data.checksum = calculate_checksum(reinterpret_cast<uint8_t *>(&Tx_Pack_Data),5);

    memcpy(&UART_Manage_Object->Tx_Buffer[0], &Tx_Pack_Data.header, 2);
    memcpy(&UART_Manage_Object->Tx_Buffer[2], &Tx_Pack_Data.cmdId, 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[3], &Tx_Pack_Data.Data_Length, 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[4], &Tx_Pack_Data.Data_Content[0], 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[5], &Tx_Pack_Data.checksum, 1);
    UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, 6);
}
void Class_Uart_Servo::Request_Control_Uart_Servo_Multilaps_Angle()
{

    Tx_Pack_Data.header = Tx_Frame;
    Tx_Pack_Data.cmdId = MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_VELOCITY;
    Tx_Pack_Data.Data_Length = sizeof(Tx_Servo_Data);
    memcpy(&Tx_Pack_Data.Data_Content[0],&Tx_Servo_Data,sizeof(Tx_Servo_Data));

    Tx_Pack_Data.checksum = calculate_checksum(reinterpret_cast<uint8_t *>(&Tx_Pack_Data),4+sizeof(Tx_Servo_Data));
   
    memcpy(&UART_Manage_Object->Tx_Buffer[0], &Tx_Pack_Data.header, 2);
    memcpy(&UART_Manage_Object->Tx_Buffer[2], &Tx_Pack_Data.cmdId, 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[3], &Tx_Pack_Data.Data_Length, 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[4], &Tx_Pack_Data.Data_Content[0], sizeof(Tx_Servo_Data));
    memcpy(&UART_Manage_Object->Tx_Buffer[4+sizeof(Tx_Servo_Data)], &Tx_Pack_Data.checksum, 1);
    UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, 5+sizeof(Tx_Servo_Data));
}
void Class_Uart_Servo::Request2_Control_Uart_Servo_Multilaps_Angle()
{
    Tx_Pack_Data.header = Tx_Frame;
    Tx_Pack_Data.cmdId = MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_INTERVAL;
    Tx_Pack_Data.Data_Length = sizeof(Tx_Servo_Data2);
    memcpy(&Tx_Pack_Data.Data_Content[0],&Tx_Servo_Data2,sizeof(Tx_Servo_Data2));

    Tx_Pack_Data.checksum = calculate_checksum(reinterpret_cast<uint8_t *>(&Tx_Pack_Data),4+sizeof(Tx_Servo_Data2));

    memcpy(&UART_Manage_Object->Tx_Buffer[0], &Tx_Pack_Data.header, 2);
    memcpy(&UART_Manage_Object->Tx_Buffer[2], &Tx_Pack_Data.cmdId, 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[3], &Tx_Pack_Data.Data_Length, 1);
    memcpy(&UART_Manage_Object->Tx_Buffer[4], &Tx_Pack_Data.Data_Content[0], sizeof(Tx_Servo_Data2));
    memcpy(&UART_Manage_Object->Tx_Buffer[4+sizeof(Tx_Servo_Data2)], &Tx_Pack_Data.checksum, 1);
    UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, 5+sizeof(Tx_Servo_Data2));
}

void Class_Uart_Servo::Output()
{
    //Requset_CMD_ID = MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_VELOCITY;
    Requset_CMD_ID = MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_INTERVAL;
    switch (Requset_CMD_ID)
    {
    case MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_VELOCITY:
    {
        Tx_Servo_Data.servo_id = Servo_ID;
        Tx_Servo_Data.angle  = (int32_t)(Target_Angle * 10.0f);
        Tx_Servo_Data.targteVelocity = (uint16_t)(Target_Omega_Deg * 10.0f);
        Tx_Servo_Data.accelerationTime = (uint16_t)AccInterval_ms;
        Tx_Servo_Data.decelerationTime = (uint16_t)DecInterval_ms;
        Tx_Servo_Data.power = (uint16_t)Power_mW;
        Request_Control_Uart_Servo_Multilaps_Angle();
    }
    break;
    case MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_INTERVAL:
    {
        Tx_Servo_Data2.servo_id = Servo_ID;
        Tx_Servo_Data2.angle = (int32_t)(Target_Angle * 10.0f);
        Tx_Servo_Data2.interval = Interval;
        Tx_Servo_Data2.power = (uint16_t)Power_mW;
        Request2_Control_Uart_Servo_Multilaps_Angle();
    }
    break;
    }

}
void Class_Uart_Servo::TIM_Process_PeriodElapsedCallback()
{
    Output();

    //Request_Uart_Servo_Multilaps_Angle();
}
