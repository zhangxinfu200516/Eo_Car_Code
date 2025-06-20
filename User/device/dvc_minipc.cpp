/**
 * @file dvc_minipc.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright ustc-robowalker (c) 2023
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"
#include <string.h>
/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/


/**
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 */
void Class_MiniPC::Init_UART(UART_HandleTypeDef *huart, uint8_t __frame_header)
{
    if(huart == &huart4)
    {
      UART_Manage_Object = &UART4_Manage_Object;
    }
    // else if(huart == &huart2)
    // {
    //   UART_Manage_Object = &UART2_Manage_Object;
    // }
    Frame_Header = __frame_header;
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
    Tx_Package.header = Frame_Header;
    Tx_Package.encoder_speed_LF = Chassis->E_Motor[0].Get_EncoderMotor_Now_Velocity();
    Tx_Package.encoder_speed_LR = Chassis->E_Motor[1].Get_EncoderMotor_Now_Velocity();
    Tx_Package.encoder_speed_RR = -Chassis->E_Motor[2].Get_EncoderMotor_Now_Velocity();
    Tx_Package.encoder_speed_RF = -Chassis->E_Motor[3].Get_EncoderMotor_Now_Velocity();
    memcpy(UART_Manage_Object->Tx_Buffer, &Tx_Package, Tx_Package_Length);
    Append_CRC16_Check_Sum(UART_Manage_Object->Tx_Buffer,Tx_Package_Length);
    UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, Tx_Package_Length);
}
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
  if(Pre_Flag == Flag)
  {
    MiniPC_Status = MiniPC_Status_DISABLE;
  }
  else
  {
    MiniPC_Status = MiniPC_Status_ENABLE;
  }

  Pre_Flag = Flag;
}

/**
 * @brief uart通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::UART_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  if(rx_data[0] == 0xA5 && Verify_CRC16_Check_Sum(rx_data,Rx_Package_Length) == true)
  {
    Flag++;
    memcpy(&Rx_Package, rx_data, Rx_Package_Length);

    process_pixel_dx = (float)Rx_Package.pixel_dx;//Filter((float)Rx_Package.pixel_dx,Record_Pixel_Dx);
    process_pixel_dy = (float)Rx_Package.pixel_dy;//Filter((float)Rx_Package.pixel_dx,Record_Pixel_Dx);

    //将yaw轴像素差转换为角度差 -- 纠正正方向
    //Set_Pixel_To_Angle(Rx_Package.pixel_dx,Rx_Package.pixel_dy);
  }
}


/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, MINPC_CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, MINPC_CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

