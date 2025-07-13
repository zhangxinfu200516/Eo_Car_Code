#ifndef __DVC_IMU_H__
#define __DVC_IMU_H__
#include "drv_uart.h"
#include "stdbool.h"
#include "dma.h"
//欧拉角
#define ATK_IMU_Data1Length (0x06+0x05)
//三轴角速度
#define ATK_IMU_Data2Length (0x0C+0x05)

struct Struct_ATK_IMU_Data
{
    float roll;//°
    float pitch;
    float yaw;
    float omega_roll;//°/s
    float omega_pitch;
    float omega_yaw;
};
struct Struct_ATK_Data_Frame
{
    uint16_t Frame_header;
    uint8_t ID;
    uint8_t Data_Length;
    uint8_t Data[20];
    uint32_t Check_Sum;
};
class Class_ATK_IMU
{
public:
	Struct_ATK_IMU_Data Data;	
    void Init(UART_HandleTypeDef *UART_Handler);
    void UART_RxCpltCallback(uint8_t *Rx_Data);
    inline bool calculate_checksum(uint8_t *data, int len);
    void TIM_Unline_Protect_PeriodElapsedCallback(void);
protected:
    Struct_UART_Manage_Object *UART_Manage_Object;

    

};
// 计算数据累加和函数
// 参数：data - 指向数据数组的指针
//        len - 数据数组长度
// 返回值：累加和（单字节）
bool Class_ATK_IMU::calculate_checksum(uint8_t *data, int len)
{
    uint8_t sum = 0; // 初始化累加和为0

    // 遍历数据数组并累加每个字节
    for (int i = 0; i < len-1; i++)
    {
        sum += data[i]; // 累加每个字节（自动丢弃进位）
    }

    if(sum == data[len-1])
    {
        return true; // 校验和正确
    }
    else
    {
        return false; // 校验和错误
    }
    
}
#endif // !__DVC_IMU_H__