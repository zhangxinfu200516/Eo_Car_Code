/**
 * @file dvc_minipc.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_MINIPC_H
#define DVC_MINIPC_H

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "main.h"
#include "drv_math.h"
#include "drv_uart.h"
#include "crt_chassis.h"
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

static const uint16_t MINPC_CRC16_INIT = 0xFFFF;

static const uint16_t W_CRC_TABLE[256] = 
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief 迷你主机状态
 *
 */
enum Enum_MiniPC_Status :uint8_t
{
    MiniPC_Status_DISABLE = 0,
    MiniPC_Status_ENABLE,
};

struct Struct_Rx_Package
{
    uint8_t header;
    float linear_x;          // x方向线速度 m/s
    float linear_y;          // y方向线速度 m/s
    float angular_yaw;       // yaw角速度 rad/s
    int pixel_dx;            // 目标横向像素差值，右正左负
    int pixel_dy;            // 目标竖直方向像素差值，下正上负
    uint8_t flags;           // 标志位
    uint16_t crc16; // crc16校验
} __attribute__((packed));

struct Struct_Tx_Package
{
    uint8_t header;
    float encoder_speed_LF;  // 左前轮速度 m/s
    float encoder_speed_RF;  // 右前轮速度
    float encoder_speed_LR;  // 左后轮速度
    float encoder_speed_RR;  // 右后轮速度
    uint16_t crc16; // crc16校验
} __attribute__((packed));

#define Rx_Package_Length sizeof(Struct_Rx_Package)
#define Tx_Package_Length sizeof(Struct_Tx_Package)
#define FITER_NUM 5
/**
 * @brief Specialized, 迷你主机类
 *
 */
// 前置声明（必须）
class Class_Tricycle_Chassis;
class Class_MiniPC
{
public:
    Class_Tricycle_Chassis *Chassis;
    void Init_UART(UART_HandleTypeDef *huart, uint8_t __frame_header = 0x5A);

    inline Enum_MiniPC_Status Get_MiniPC_Status();
    inline float Get_Minipc_Linear_X();
    inline float Get_Minipc_Linear_Y();
    inline float Get_Minipc_Angular_Yaw();
    inline int Get_MInipc_Pixel_Dx();
    inline int Get_MInipc_Pixel_Dy();
    inline float Get_Yaw_Incremental_Angles();
    inline float Get_Pitch_Integral_Angles();
    inline float Filter(float new_data, float *data_record);
    inline float Get_process_pixel_dx();
    inline float Get_process_pixel_dy();
    inline uint8_t Get_Minipc_flags();
    inline uint8_t Get_Zimiao_flag();

    inline void Set_Pixel_To_Angle(int32_t __pixel_dx,int32_t __pixel_dy);

    void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
    bool Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
    uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);
    void UART_RxCpltCallback(uint8_t *Rx_Data);

    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void TIM_Write_PeriodElapsedCallback();
protected:
    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;

    Struct_Tx_Package Tx_Package;
    Struct_Rx_Package Rx_Package;

    //数据包头标
    uint8_t Frame_Header;
    //数据包尾标
    uint8_t Frame_Rear; 

    float fx;
    float fy;
    //处理后的像素差值
    float process_pixel_dx;
    float process_pixel_dy;

    float Record_Pixel_Dx[FITER_NUM];
    //迷你主机接收flag
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    //迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    uint8_t Zimiao_flag;
    //迷你主机对外接口信息
    float Yaw_Incremental_Angles;
    float Pitch_Integral_Angles;
    //内部函数
    void Data_Process_UART(uint8_t *Rx_Data);
    void Output_UART();

};
/* Exported variables --------------------------------------------------------*/
Enum_MiniPC_Status Class_MiniPC::Get_MiniPC_Status()
{
    return MiniPC_Status;
}
float Class_MiniPC::Get_Minipc_Linear_X()
{
    return Rx_Package.linear_x;
}
float Class_MiniPC::Get_Minipc_Linear_Y()
{
    return Rx_Package.linear_y;
}
float Class_MiniPC::Get_Minipc_Angular_Yaw()
{
    return Rx_Package.angular_yaw;
}
int Class_MiniPC::Get_MInipc_Pixel_Dx()
{
    return Rx_Package.pixel_dx;
}
int Class_MiniPC::Get_MInipc_Pixel_Dy()
{
    return Rx_Package.pixel_dy;
}
void Class_MiniPC::Set_Pixel_To_Angle(int32_t __pixel_dx,int32_t __pixel_dy)
{
    Yaw_Incremental_Angles = My_atan(__pixel_dx,fx);
    Pitch_Integral_Angles = My_atan(__pixel_dy,fy);
}
float Class_MiniPC::Get_Yaw_Incremental_Angles()
{
    return Yaw_Incremental_Angles;
}
float Class_MiniPC::Get_Pitch_Integral_Angles()
{
    return Pitch_Integral_Angles;
}
float Class_MiniPC::Filter(float new_data, float *data_record)
{
    float sum = 0.0;

    for (uint8_t i = FITER_NUM - 1; i > 0; i--) // 将现有数据后移一位
    {
        data_record[i] = data_record[i - 1];
        sum += data_record[i - 1];
    }
    data_record[0] = new_data; // 第一位是新的数据
    sum += new_data;

    return sum / (FITER_NUM * 1.0); // 返回均值
}
float Class_MiniPC::Get_process_pixel_dx()
{
    return process_pixel_dx;
}
float Class_MiniPC::Get_process_pixel_dy()
{
    return process_pixel_dy;
}
uint8_t Class_MiniPC::Get_Minipc_flags()
{
    return Rx_Package.flags;
}
uint8_t Class_MiniPC::Get_Zimiao_flag()
{
    return Zimiao_flag;
}
/* Exported function declarations --------------------------------------------*/
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
