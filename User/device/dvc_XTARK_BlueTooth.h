#ifndef DVC_BLUETOOTH_H
#define DVC_BLUETOOTH_H
#include "drv_uart.h"

//按键开关位置
#define KEY_FREE (0)
#define KEY_PRESSED (1)
/**
 * @brief 按键状态
 *
 */
enum Enum_Key_Status 
{
    Key_Status_FREE = 0,           //松开状态
    Key_Status_PRESSED,            //按下状态
    Key_Status_TRIG_FREE_PRESSED,  //松开到按下的突变状态
    Key_Status_TRIG_PRESSED_FREE,  //按下到松开的突变状态
};

struct Struct_Bluetooth_RxData
{
    uint16_t Frame_header;
    uint8_t Frame_Length;
    uint8_t Frame_number;
    int8_t Remote_Left_X;
    int8_t Remote_Left_Y;
    int8_t Remote_Right_X;
    int8_t Remote_Right_Y;
    uint8_t Button_One;
    uint8_t Button_Two;
    uint8_t _CRC;//累加和
}__attribute__((packed));

struct Struct_Bluetooth_RxData2
{
    uint16_t Frame_header;
    uint8_t Frame_Length;
    uint8_t Frame_number;
    int8_t Remote_Left_X;
    int8_t Remote_Left_Y;
    int8_t Remote_Right_X;
    int8_t Remote_Right_Y;
    uint8_t Button_One;
    uint8_t _CRC;//累加和
}__attribute__((packed));

struct Struct_Bluetooth_PorcessData
{
    float Remote_Left_X;
    float Remote_Left_Y;
    float Remote_Right_X;
    float Remote_Right_Y;
    uint8_t Button_One;
    Enum_Key_Status Key[16];
};
class Class_DVC_Bluetooth
{
public:
    //处理后数据
    Struct_Bluetooth_PorcessData Data;
    
    void Init(UART_HandleTypeDef *huart);
    inline uint8_t calculate_checksum(uint8_t *data, int len);
    inline float Get_Remote_Left_X();
    inline float Get_Remote_Left_Y();
    inline float Get_Remote_Right_X();
    inline float Get_Remote_Right_Y();
    inline uint8_t Get_Button_One();
    void CAN_RxCpltCallback(uint8_t *Rx_Data);
protected:
    Struct_UART_Manage_Object *UART_Manage_Object;
    //原始数据
    //现在时刻的遥控器状态信息
    Struct_Bluetooth_RxData Now_UART_Rx_Data;
    //前一时刻的遥控器状态信息
    Struct_Bluetooth_RxData Pre_UART_Rx_Data;

    //原始数据2
    Struct_Bluetooth_RxData2 Now_UART_Rx_Data2;

    //存活标志位
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    void Judge_Key(Enum_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
    void Data_Process();
};

// 计算数据累加和函数
// 参数：data - 指向数据数组的指针
//        len - 数据数组长度
// 返回值：累加和（单字节）
uint8_t Class_DVC_Bluetooth::calculate_checksum(uint8_t *data, int len)
{
    uint8_t sum = 0; // 初始化累加和为0

    // 遍历数据数组并累加每个字节
    for (int i = 0; i < len; i++)
    {
        sum += data[i]; // 累加每个字节（自动丢弃进位）
    }

    return sum; // 返回累加结果
}
float Class_DVC_Bluetooth::Get_Remote_Left_X()
{
    return Data.Remote_Left_X;
}
float Class_DVC_Bluetooth::Get_Remote_Left_Y()
{
    return Data.Remote_Left_Y;
}
float Class_DVC_Bluetooth::Get_Remote_Right_X()
{
    return Data.Remote_Right_X;
}
float Class_DVC_Bluetooth::Get_Remote_Right_Y()
{
    return Data.Remote_Right_Y;
}
uint8_t Class_DVC_Bluetooth::Get_Button_One()
{
    return Data.Button_One;
}
#endif // !DVC_BLUETOOTH_H