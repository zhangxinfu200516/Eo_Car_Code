#ifndef __RGB_H__
#define __RGB_H__
 
#include "main.h"
 

/*这里是上文计算所得CCR的宏定义*/
#define CODE_1       (58)       //1码定时器计数次数58
#define CODE_0       (25)       //0码定时器计数次数25
 
/*建立一个定义单个LED三原色值大小的结构体*/
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color_TypeDef;
 
#define Pixel_NUM 376  //LED数量宏定义，这里我使用一个LED，（单词pixel为像素的意思）
extern int target_time_flag;
extern int Arrow_flag;
void RGB_target_ready(void);
void RGB_SetColor(uint16_t LedId,RGB_Color_TypeDef Color);//给一个LED装载24个颜色数据码（0码和1码）
//void RGB_SetColor2(uint16_t LedId,RGB_Color_TypeDef Color);
void Reset_Load(void); //该函数用于将数组最后24个数据变为0，代表RESET_code
void RGB_SendArray(void);          //发送最终数组
void RGB_SendArray2(void);
void RGB_SendArray3(void);
void RGB_RED(uint16_t Pixel_Len);  //显示红灯
void RGB_RED2(uint16_t Pixel_Len);
void RGB_BLACK2(uint16_t Pixel_Len);
void RGB_RED3(uint16_t Pixel_Len);
void RGB_BLACK3(uint16_t Pixel_Len);
void RGB_BLUE(uint16_t Pixel_Len); //显示蓝灯
void RGB_1_ring(void);//零环
void RGB_target(void);//待机打状态具体点灯函数，在RGB_target_ready函数中调用
void RGB_all_black(void);//全灭
void RGB_Arrow(int16_t m);//箭头流水灯
void RGB_Arrow_ALL_LIGHT(void);//流水灯条全亮
void RGB_Arrow_ALL_BLACK(void);//流水灯条全灭

extern RGB_Color_TypeDef COLOR1;
 
 
#endif
