#ifndef _N100_H_
#define _N100_H_
#include "drv_math.h"
#include "drv_can.h"
#include "drv_uart.h"

enum Enum_N100_Live_Status
{
    N100_Live_Status_DISABLE = 0,
    N100_Live_Status_ENABLE,
};
// 单位说明：

// 角速度：弧度/秒 (rad/s)

// 加速度：米/秒² (m/s²)，含重力分量

// 磁力计：毫高斯 (mG)

// 气压：帕斯卡 (Pa)

// 温度：摄氏度 (°C)

// 时间戳：微秒 (µs)
struct Struct_IMUData_Packet_t
{
    float gyroscope_x;          // unit: rad/s
    float gyroscope_y;          // unit: rad/s
    float gyroscope_z;          // unit: rad/s
    float accelerometer_x;      // m/s^2
    float accelerometer_y;      // m/s^2
    float accelerometer_z;      // m/s^2
    float magnetometer_x;       // mG
    float magnetometer_y;       // mG
    float magnetometer_z;       // mG
    float imu_temperature;      // C
    float Pressure;             // Pa
    float pressure_temperature; // C
    uint32_t Timestamp;         // us
} __attribute__((packed));

struct Struct_AHRSData_Packet_t
{
    float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float HeadingSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
	float Heading;     //unit: rad
	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	long long Timestamp; //unit: us
}__attribute__((packed));

struct Struct_Frame_Format_t
{
    uint8_t Frame_header;
    uint8_t Command_Type;
    uint8_t Data_Length;
    uint8_t Serial_Number;
    uint8_t Frame_header_CRC8;
    uint16_t Data_CRC16;
    uint8_t Data[48];
    uint8_t Frame_Tail;
}__attribute__((packed));

struct Struct_IMU_Frame_Format_t
{
    uint8_t Frame_header;
    uint8_t Command_Type;
    uint8_t Data_Length;
    uint8_t Serial_Number;
    uint8_t Frame_header_CRC8;
    uint16_t Data_CRC16;
    uint8_t Data[56];
    uint8_t Frame_Tail;
}__attribute__((packed));

class Class_N100
{
public:
    Struct_Frame_Format_t  Frame_Format;
    Struct_IMU_Frame_Format_t IMU_Frame_Format;

    inline float Get_N100_Heading_Angle();
    inline Enum_N100_Live_Status Get_N100_Live_Status();

    void Init(CAN_HandleTypeDef *hcan);
    void Init(UART_HandleTypeDef *huart);
    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void UART_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Process_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void Data_Process();    
protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Struct_UART_Manage_Object *UART_Manage_Object;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Struct_AHRSData_Packet_t AHRSData;
    Struct_IMUData_Packet_t IMUData;

    Enum_N100_Live_Status N100_Live_Status = N100_Live_Status_DISABLE;
};
float Class_N100::Get_N100_Heading_Angle()
{
    return AHRSData.Heading;
}
Enum_N100_Live_Status Class_N100::Get_N100_Live_Status()
{
    return N100_Live_Status;
}
#endif /* _N100_H_ */