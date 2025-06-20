#include "dvc_oled.h"

void Class_OLED::Init(void)
{
    OLED_RST_Clr();
    HAL_Delay(100);
    OLED_RST_Set();

    OLED_WR_Byte(0xAE, OLED_CMD); // Close display //关闭显示
    OLED_WR_Byte(0xD5, OLED_CMD); // The frequency frequency factor, the frequency of the shock //设置时钟分频因子,震荡频率
    OLED_WR_Byte(80, OLED_CMD);   //[3:0], the frequency dividing factor;[7:4], oscillation frequency //[3:0],分频因子;[7:4],震荡频率
    OLED_WR_Byte(0xA8, OLED_CMD); // Set the number of driver paths //设置驱动路数
    OLED_WR_Byte(0X3F, OLED_CMD); // Default 0x3f(1/64) //默认0X3F(1/64)
    OLED_WR_Byte(0xD3, OLED_CMD); // Setting display deviation //设置显示偏移
    OLED_WR_Byte(0X00, OLED_CMD); // Default is 0//默认为0

    OLED_WR_Byte(0x40, OLED_CMD); // Sets the number of rows to display starting line [5:0] //设置显示开始行 [5:0],行数

    OLED_WR_Byte(0x8D, OLED_CMD); // Charge pump setup //电荷泵设置
    OLED_WR_Byte(0x14, OLED_CMD); // Bit2, on/off //bit2，开启/关闭
    OLED_WR_Byte(0x20, OLED_CMD); // Set up the memory address mode //设置内存地址模式
    OLED_WR_Byte(0x02, OLED_CMD); //[1:0],00, column address mode;01, line address mode;10. Page address mode;The default 10; //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
    OLED_WR_Byte(0xA1, OLED_CMD); // Segment redefine setting,bit0:0,0- >;0;1, 0 - & gt;127; //段重定义设置,bit0:0,0->0;1,0->127;
    OLED_WR_Byte(0xC0, OLED_CMD); // Set the COM scan direction;Bit3:0, normal mode;1, Re-define schema COM[n-1]- >;COM0;N: Number of driving paths//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
    OLED_WR_Byte(0xDA, OLED_CMD); // Set the COM hardware pin configuration //设置COM硬件引脚配置
    OLED_WR_Byte(0x12, OLED_CMD); //[5:4]configuration //[5:4]配置

    OLED_WR_Byte(0x81, OLED_CMD); // Contrast Settings //对比度设置
    OLED_WR_Byte(0xEF, OLED_CMD); // 1~ 255; Default 0x7f (brightness Settings, the bigger the brighter) //1~255;默认0X7F (亮度设置,越大越亮)
    OLED_WR_Byte(0xD9, OLED_CMD); // Set the pre-charging cycle //设置预充电周期
    OLED_WR_Byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
    OLED_WR_Byte(0xDB, OLED_CMD); // Setting vcomh voltage multiplier//设置VCOMH 电压倍率
    OLED_WR_Byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    OLED_WR_Byte(0xA4, OLED_CMD); // Global display; Bit0:1, open; 0, close; (white screen/black screen)//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
    OLED_WR_Byte(0xA6, OLED_CMD); // Settings display mode; Bit0:1, anti-phase display; 0, normal display//设置显示方式;bit0:1,反相显示;0,正常显示
    OLED_WR_Byte(0xAF, OLED_CMD); // Open display //开启显示

    OLED_Clear();
}
void Class_OLED::TIM_Process_PeriodElapsedCallback()
{
    static uint8_t count100 = 0;
    count100 ++;
    if(count100 > 100)
    {
        OLED_ShowString(0, 0, (const uint8_t*)"Power: ");
        
        OLED_ShowString(0, 10, (const uint8_t*)"IMU_Yaw: ");


        OLED_Refresh_Gram();

        count100 = 0;
    }

    // int16_t Voltage_Show = 1234;

    // OLED_ShowNumber(75, 50, Voltage_Show / 100, 2, 12);
    // OLED_ShowString(88, 50, (const uint8_t*)".");
    // OLED_ShowNumber(98, 50, Voltage_Show % 100, 2, 12);
    // OLED_ShowString(110, 50, (const uint8_t*)"V");
}