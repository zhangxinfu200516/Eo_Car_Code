#ifndef DVC_UART_SERVO_H
#define DVC_UART_SERVO_H
#include "drv_uart.h"
// 返回的响应数据包最长的长度
#define FSUS_PACK_RESPONSE_MAX_SIZE 350


//请求读取舵机多圈角度指令
#define FSUS_CMD_QUERY_SERVO_ANGLE_MTURN 0x10           // 查询舵机角度(多圈)
//请求控制舵机多圈角度指令-指定速度
#define MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_VELOCITY 0x0F // 控制舵机角度(多圈)指定速度
//请求控制舵机多圈角度指令
#define MOVE_ON_MULTI_TURN_ANGLE_MODE_EX_BY_INTERVAL    0x0D
//// 数据帧结构体（统一结构）
struct Struct_Data_Frame_Package
{
    uint16_t header;
    uint8_t cmdId;
    uint8_t Data_Length;  // 统一使用8位长度
    uint8_t Data_Content[FSUS_PACK_RESPONSE_MAX_SIZE];
    uint8_t checksum;
}__attribute__((packed));

//读取舵机多圈角度响应数据
struct Struct_Uart_Servo_Multilaps_Data
{
    uint8_t servo_id;
    int16_t angle;//舵机角度，数值正负代表方向( 单位0.1度，举例 angle=900 就等于90度)
    uint16_t turn;//转动圈数
}__attribute__((packed));
//
struct Struct_Uart_Servo_Multilaps_Control_Data
{
    uint8_t servo_id;
    int32_t angle;//舵机角度，数值范围：-3686400～3686400
    uint16_t targteVelocity;//目标速度
    uint16_t accelerationTime;//加速度时间ms
    uint16_t decelerationTime;//减速度时间ms
    uint16_t power;//执行功率mW
}__attribute__((packed));

struct Struct_Uart_Servo_Multilaps_Control_Data2
{
    uint8_t servo_id;
    int32_t angle;//舵机角度，数值范围：-3686400～3686400
    uint32_t interval;//到达目标角度的时间 (单位ms)，最大值为 4096000
    uint16_t power;//执行功率mW
}__attribute__((packed));

class Class_Uart_Servo
{
public:

    void Init(UART_HandleTypeDef *huart,uint8_t __Servo_ID,float __AccInterval_ms,float __DecInterval_ms,float __Power_mW);
    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    inline uint8_t calculate_checksum(uint8_t *data, int len);
    inline float Get_Now_Angle();
    inline void Set_Target_Angle(float angle);
    inline void Set_Target_Omega_Deg(float omega_deg);
    void TIM_Process_PeriodElapsedCallback();
    void Request_Uart_Servo_Multilaps_Angle();
protected:
    Struct_UART_Manage_Object *UART_Manage_Object;

    Struct_Data_Frame_Package Rx_Pack_Data;
    Struct_Data_Frame_Package Tx_Pack_Data;

    Struct_Uart_Servo_Multilaps_Data Rx_Servo_Data;
    Struct_Uart_Servo_Multilaps_Control_Data Tx_Servo_Data;
    Struct_Uart_Servo_Multilaps_Control_Data2 Tx_Servo_Data2;

    const uint16_t Tx_Frame = 0x4C12;
    const uint16_t Rx_Frame = 0x1C05;

    //
    uint8_t Servo_ID;
    uint8_t Requset_CMD_ID;
    //多圈定速控制的读写数据
    float Target_Angle;
    float Target_Omega_Deg;
    float AccInterval_ms;
    float DecInterval_ms;
    float Power_mW;
    //多圈不定速读写数据
    uint32_t Interval = 100;//ms
    //多圈角度
    float Now_Angle;

    void Data_Process();
    void Output();
    // void Request_Uart_Servo_Multilaps_Angle();
    void Request_Control_Uart_Servo_Multilaps_Angle();
    void Request2_Control_Uart_Servo_Multilaps_Angle();

};

uint8_t Class_Uart_Servo::calculate_checksum(uint8_t *data, int len)
{
    uint8_t sum = 0; // 初始化累加和为0

    // 遍历数据数组并累加每个字节
    for (int i = 0; i < len; i++)
    {
        sum += data[i]; // 累加每个字节（自动丢弃进位）
    }

    return (sum % 256); // 返回累加结果
}
void Class_Uart_Servo::Set_Target_Angle(float angle)
{
    Target_Angle = angle;
}
void Class_Uart_Servo::Set_Target_Omega_Deg(float omega_deg)
{
    Target_Omega_Deg = omega_deg;   
}
float Class_Uart_Servo::Get_Now_Angle()
{
    return Now_Angle;
}
#endif