#include "dvc_N100.h"

void Class_N100::Init(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
}
void Class_N100::Init(UART_HandleTypeDef *huart)
{
    if(huart == &huart6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
}
/*************
实现16进制的can数据转换成浮点型数据
****************/
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    uint32_t transition_32;
    float tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;
    transition_32 = 0;
    transition_32 |= Data_4 << 24;
    transition_32 |= Data_3 << 16;
    transition_32 |= Data_2 << 8;
    transition_32 |= Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1; // 符号位
    // 先右移操作，再按位与计算，出来结果是30到23位对应的e
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    // 将22~0转化为10进制，得到对应的x系数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);
    return tmp;
}
long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4, uint8_t Data_5, uint8_t Data_6, uint8_t Data_7, uint8_t Data_8)
{
    long long transition_64;
    transition_64 = 0;
    transition_64 |= Data_8 << 56;
    transition_64 |= Data_7 << 48;
    transition_64 |= Data_6 << 40;
    transition_64 |= Data_5 << 32;
    transition_64 |= Data_4 << 24;
    transition_64 |= Data_3 << 16;
    transition_64 |= Data_2 << 8;
    transition_64 |= Data_1;
    return transition_64;
}
void Class_N100::Data_Process()
{
    AHRSData.RollSpeed = DATA_Trans(Frame_Format.Data[0], Frame_Format.Data[1], Frame_Format.Data[2], Frame_Format.Data[3]) * RAD_TO_DEG;
    AHRSData.PitchSpeed = DATA_Trans(Frame_Format.Data[4], Frame_Format.Data[5], Frame_Format.Data[6], Frame_Format.Data[7]) * RAD_TO_DEG;
    AHRSData.HeadingSpeed = DATA_Trans(Frame_Format.Data[8], Frame_Format.Data[9], Frame_Format.Data[10], Frame_Format.Data[11]) * RAD_TO_DEG;
    AHRSData.Roll = DATA_Trans(Frame_Format.Data[12], Frame_Format.Data[13], Frame_Format.Data[14], Frame_Format.Data[15]) * RAD_TO_DEG;
    AHRSData.Pitch = DATA_Trans(Frame_Format.Data[16], Frame_Format.Data[17], Frame_Format.Data[18], Frame_Format.Data[19]) * RAD_TO_DEG;
    AHRSData.Heading = DATA_Trans(Frame_Format.Data[20], Frame_Format.Data[21], Frame_Format.Data[22], Frame_Format.Data[23]) * RAD_TO_DEG;
    AHRSData.Qw = DATA_Trans(Frame_Format.Data[24], Frame_Format.Data[25], Frame_Format.Data[26], Frame_Format.Data[27]);
    AHRSData.Qx = DATA_Trans(Frame_Format.Data[28], Frame_Format.Data[29], Frame_Format.Data[30], Frame_Format.Data[31]);
    AHRSData.Qy = DATA_Trans(Frame_Format.Data[32], Frame_Format.Data[33], Frame_Format.Data[34], Frame_Format.Data[35]);
    AHRSData.Qz = DATA_Trans(Frame_Format.Data[36], Frame_Format.Data[37], Frame_Format.Data[38], Frame_Format.Data[39]);
    AHRSData.Timestamp = timestamp(Frame_Format.Data[40], Frame_Format.Data[41], Frame_Format.Data[42], Frame_Format.Data[43],
                                   Frame_Format.Data[44], Frame_Format.Data[45], Frame_Format.Data[46], Frame_Format.Data[47]);
    
    IMUData.gyroscope_x = DATA_Trans(IMU_Frame_Format.Data[0],IMU_Frame_Format.Data[1],IMU_Frame_Format.Data[2],IMU_Frame_Format.Data[3]);
    IMUData.gyroscope_y = DATA_Trans(IMU_Frame_Format.Data[4],IMU_Frame_Format.Data[5],IMU_Frame_Format.Data[6],IMU_Frame_Format.Data[7]);
    IMUData.gyroscope_z = DATA_Trans(IMU_Frame_Format.Data[8],IMU_Frame_Format.Data[9],IMU_Frame_Format.Data[10],IMU_Frame_Format.Data[11]);
    IMUData.accelerometer_x = DATA_Trans(IMU_Frame_Format.Data[12],IMU_Frame_Format.Data[13],IMU_Frame_Format.Data[14],IMU_Frame_Format.Data[15]);
    IMUData.accelerometer_y = DATA_Trans(IMU_Frame_Format.Data[16],IMU_Frame_Format.Data[17],IMU_Frame_Format.Data[18],IMU_Frame_Format.Data[19]);
    IMUData.accelerometer_z = DATA_Trans(IMU_Frame_Format.Data[20],IMU_Frame_Format.Data[21],IMU_Frame_Format.Data[22],IMU_Frame_Format.Data[23]);
    IMUData.magnetometer_x = DATA_Trans(IMU_Frame_Format.Data[24],IMU_Frame_Format.Data[25],IMU_Frame_Format.Data[26],IMU_Frame_Format.Data[27]);
    IMUData.magnetometer_y = DATA_Trans(IMU_Frame_Format.Data[28],IMU_Frame_Format.Data[29],IMU_Frame_Format.Data[30],IMU_Frame_Format.Data[31]);
    IMUData.magnetometer_z = DATA_Trans(IMU_Frame_Format.Data[32],IMU_Frame_Format.Data[33],IMU_Frame_Format.Data[34],IMU_Frame_Format.Data[35]);
    IMUData.imu_temperature = DATA_Trans(IMU_Frame_Format.Data[36],IMU_Frame_Format.Data[37],IMU_Frame_Format.Data[38],IMU_Frame_Format.Data[39]);
    IMUData.Pressure = DATA_Trans(IMU_Frame_Format.Data[40],IMU_Frame_Format.Data[41],IMU_Frame_Format.Data[42],IMU_Frame_Format.Data[43]);
    IMUData.pressure_temperature = DATA_Trans(IMU_Frame_Format.Data[44],IMU_Frame_Format.Data[45],IMU_Frame_Format.Data[46],IMU_Frame_Format.Data[47]);
    IMUData.Timestamp = timestamp(IMU_Frame_Format.Data[48],IMU_Frame_Format.Data[49],IMU_Frame_Format.Data[50],IMU_Frame_Format.Data[51],
                                   IMU_Frame_Format.Data[52],IMU_Frame_Format.Data[53],IMU_Frame_Format.Data[54],IMU_Frame_Format.Data[55]);
}

uint8_t CanRxFifo[7][8];
     uint8_t acc_flag = 0;
     uint8_t data_num = 0;
     uint8_t last_count = 0;
void Class_N100::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag ++;

    if (Rx_Data[0] == 0xFC && last_count == 0xFD)
    {
        data_num = 1;
        if (Rx_Data[1] == 0x41 && Rx_Data[2] == 0x30)
            acc_flag = 1;
    }
    last_count = Rx_Data[7];

    if (data_num)
    {
        for (auto i = 0; i < 8; i++)
        {
            CanRxFifo[data_num - 1][i] = Rx_Data[i];
        }
        data_num++;
    }

    if (acc_flag == 1 && data_num == 8)
    {
        data_num = 0;
        acc_flag = 0;

        for (auto i = 0; i < 7; i++)
        {
            for (auto j = 0; j < 8; j++)
            {
                reinterpret_cast<uint8_t *>(&Frame_Format)[i*8+j] = CanRxFifo[i][j];
            }
        }
    }
    //数据解包
    //Data_Process();
}
void Class_N100::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    if(Rx_Data[0] == 0xFC)
    {
        //AHRS数据解包
        if(Rx_Data[1] == 0x41 && Rx_Data[2] == 0x30 && Rx_Data[55] == 0xFD)
        {
            memcpy(&Frame_Format,Rx_Data,56);
        }
        //IMU数据解包
        else if(Rx_Data[1] == 0x40 && Rx_Data[2] == 0x38 && Rx_Data[63] == 0xFD)
        {
            Flag ++;

            memcpy(&IMU_Frame_Format,Rx_Data,64);
        }
    }
}
void Class_N100::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if(Pre_Flag == Flag)
    {
        N100_Live_Status = N100_Live_Status_DISABLE;
    }
    else
    {
        N100_Live_Status = N100_Live_Status_ENABLE;
    }

    Pre_Flag = Flag;
}