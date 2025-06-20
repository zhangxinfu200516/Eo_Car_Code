#include "dvc_XTARK_BlueTooth.h"
#include <string.h>
void Class_DVC_Bluetooth::Init(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
}    


void Class_DVC_Bluetooth::Judge_Key(Enum_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    //带触发的判断
    switch (Pre_Status)
    {
    case (KEY_FREE):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = Key_Status_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = Key_Status_TRIG_FREE_PRESSED;
        }
        break;
        }
    }
    break;
    case (KEY_PRESSED):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = Key_Status_TRIG_PRESSED_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = Key_Status_PRESSED;
        }
        break;
        }
    }
    break;
    }
}


void Class_DVC_Bluetooth::Data_Process()
{
    // Data.Remote_Left_X = (float)Now_UART_Rx_Data.Remote_Left_X / 100.0f;
    // Data.Remote_Left_Y = (float)Now_UART_Rx_Data.Remote_Left_Y / 100.0f;
    // Data.Remote_Right_X = (float)Now_UART_Rx_Data.Remote_Right_X / 100.0f;
    // Data.Remote_Right_Y = (float)Now_UART_Rx_Data.Remote_Right_Y / 100.0f;
    // for (auto i = 0; i < 8; i++)
    // {
    //     Judge_Key(&Data.Key[i], (Now_UART_Rx_Data.Button_One >> i) & (0x1), (Pre_UART_Rx_Data.Button_One >> i) & (0x1));
    // }
    // for (auto i = 0; i < 8; i++)
    // {
    //     Judge_Key(&Data.Key[i], (Now_UART_Rx_Data.Button_Two >> i) & (0x1), (Pre_UART_Rx_Data.Button_Two >> i) & (0x1));
    // }
    Data.Remote_Left_X = (float)Now_UART_Rx_Data2.Remote_Left_X / 100.0f;
    Data.Remote_Left_Y = (float)Now_UART_Rx_Data2.Remote_Left_Y / 100.0f;
    Data.Remote_Right_X = (float)Now_UART_Rx_Data2.Remote_Right_X / 100.0f;
    Data.Remote_Right_Y = (float)Now_UART_Rx_Data2.Remote_Right_Y / 100.0f;
    Data.Button_One = Now_UART_Rx_Data2.Button_One;
}
void Class_DVC_Bluetooth::CAN_RxCpltCallback(uint8_t *Rx_Data)
{

    Flag++;

    // for (auto i = 0; i < 11; i++)
    // {
    //     reinterpret_cast<uint8_t *>(&Now_UART_Rx_Data)[i] = Rx_Data[i];
    // }

    // Data_Process();

    // memcpy(&Pre_UART_Rx_Data, &Now_UART_Rx_Data, sizeof(Struct_Bluetooth_RxData));

    for (auto i = 0; i < 10; i++)
    {
        reinterpret_cast<uint8_t *>(&Now_UART_Rx_Data2)[i] = Rx_Data[i];
    }

    Data_Process();
}
