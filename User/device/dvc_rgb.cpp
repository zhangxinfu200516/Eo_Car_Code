#include "dvc_rgb.h"
#include "tim.h"
#include "dvc_dwt.h"
 
/*Some Static Colors------------------------------*/
const RGB_Color_TypeDef RED      = {220,0,0};   //显示红色RGB数据
const RGB_Color_TypeDef BLUE     = {0,0,255};
const RGB_Color_TypeDef BLACK    = {0,0,0};
const RGB_Color_TypeDef WHITE    = {255,255,255};

RGB_Color_TypeDef COLOR1={220,0,0};//红色

/*二维数组存放最终PWM输出数组，每一行24个
数据代表一个LED，最后一行24个0代表RESET码*/
uint32_t Pixel_Buf[Pixel_NUM+1][24];
int Arrow_flag=3;
int target_time_flag=1;
int now_light=0;
/*
功能：设定单个RGB LED的颜色，把结构体中RGB的24BIT转换为0码和1码
参数：LedId为LED序号，Color：定义的颜色结构体
*/
void RGB_SetColor(uint16_t LedId,RGB_Color_TypeDef Color)
{
	uint16_t i; 
	if(LedId > Pixel_NUM)return; //avoid overflow 防止写入ID大于LED总数
	
	for(i=0;i<8;i++) Pixel_Buf[LedId][i]   = ( (Color.G & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
	for(i=8;i<16;i++) Pixel_Buf[LedId][i]  = ( (Color.R & (1 << (15-i)))? (CODE_1):CODE_0 );//数组某一行8~15转化存放R
	for(i=16;i<24;i++) Pixel_Buf[LedId][i] = ( (Color.B & (1 << (23-i)))? (CODE_1):CODE_0 );//数组某一行16~23转化存放B
}
 

/*
功能：最后一行装在24个0，输出24个周期占空比为0的PWM波，作为最后reset延时，这里总时长为24*1.2=30us > 24us(要求大于24us)
*/
void Reset_Load(void)
{
	uint16_t i;
	for(i=0;i<24;i++)
	{
		Pixel_Buf[Pixel_NUM][i] = 0;
	}
}//注释掉后点灯代码便可以在while循环中使用了

/*
功能：发送数组
参数：(&htim1)定时器1，(TIM_CHANNEL_1)通道1，((uint32_t *)Pixel_Buf)待发送数组，
			(Pixel_NUM+1)*24)发送个数，数组行列相乘
*/
void RGB_target_ready(void)//待击打状态函数
{	
	target_time_flag=0;
	Arrow_flag=0;
	if(target_time_flag==0)
	{
		RGB_target();
		target_time_flag=1;
	}
}
/*
功能：待机打图案点亮函数
参数：直接调用即可，若放在主函数中则务必确保该函数不会短时间反复循环执行
*/

//PA0
void RGB_SendArray(void)
{
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)Pixel_Buf,(Pixel_NUM+1)*24);
}
 
//PA1
void RGB_SendArray2(void)
{
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)Pixel_Buf,(Pixel_NUM+1)*24);
}

void RGB_SendArray3(void)
{
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t *)Pixel_Buf,(Pixel_NUM+1)*24);
}

/*
功能：显示红色
参数：Pixel_Len为显示LED个数
*/
void RGB_RED(uint16_t Pixel_Len)
{
	uint16_t i;
	
		for(i=0;i<376;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=0;i<Pixel_Len;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	Reset_Load();
	RGB_SendArray();
}
 
void RGB_RED2(uint16_t Pixel_Len)
{
	uint16_t i;

		for(i=0;i<376;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=0;i<Pixel_Len;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}

	Reset_Load();
	DWT_Delay(5);
	RGB_SendArray2();
}


void RGB_BLACK2(uint16_t Pixel_Len)
{
	uint16_t i;

	for(i=0;i<Pixel_Len;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}

	Reset_Load();
	RGB_SendArray2();
}

void RGB_RED3(uint16_t Pixel_Len)
{
	uint16_t i;
//	for(i=0;i<Pixel_Len;i++)//全灭
//	{
//		RGB_SetColor(i,BLACK);
//	}
		for(i=0;i<376;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=0;i<Pixel_Len;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=Pixel_Len;i<376;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	Reset_Load();
	RGB_SendArray3();
}



void RGB_BLACK3(uint16_t Pixel_Len)
{
	uint16_t i;
	for(i=0;i<Pixel_Len;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
	Reset_Load();
	RGB_SendArray3();
}

/*
功能：显示蓝色
参数：Pixel_Len为显示LED个数
*/
void RGB_BLUE(uint16_t Pixel_Len)
{
	uint16_t i;
	for(i=0;i<Pixel_Len;i++)//给对应个数LED写入蓝色
	{
		RGB_SetColor(i,BLUE);
	}
	Reset_Load();
	RGB_SendArray();
}

void RGB_1_ring(void)
{

	uint16_t i;
	
	for(i=0;i<376;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
//	RGB_SendArray();
//	HAL_Delay(10);
	for(i=0;i<2;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=2;i<4;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=4;i<16;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=16;i<18;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=18;i<30;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=30;i<32;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=32;i<44;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=44;i<46;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=46;i<56;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=56;i<376;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	Reset_Load();
	DWT_Delay(5);
	RGB_SendArray();

}


void RGB_all_black(void)//全灭函数
{
	uint16_t i;
	for(i=0;i<376;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
	Reset_Load();
	RGB_SendArray();
	DWT_Delay(5);
	RGB_BLACK2(260);
	DWT_Delay(5);
	Arrow_flag=2;
}


void RGB_target(void)//待机打状态具体点灯函数，在RGB_target_ready函数中调用
{

uint16_t i;
	
	for(i=0;i<376;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=0;i<1;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=1;i<5;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=5;i<15;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=15;i<19;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=19;i<29;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=29;i<33;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=33;i<43;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=43;i<47;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=47;i<56;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=56;i<152;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=152;i<153;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=153;i<157;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=157;i<165;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=165;i<169;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=169;i<177;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=177;i<181;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=181;i<189;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=189;i<193;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=193;i<201;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=201;i<205;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=205;i<211;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=211;i<215;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=215;i<221;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=221;i<225;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=225;i<231;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=231;i<235;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=235;i<241;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=241;i<245;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=245;i<250;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=250;i<254;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=254;i<259;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=259;i<263;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=263;i<268;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=268;i<272;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=272;i<276;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=276;i<324;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=324;i<325;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=325;i<327;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=327;i<330;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=330;i<332;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=332;i<335;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=335;i<337;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=337;i<340;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=340;i<342;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=342;i<344;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=344;i<372;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	
	Reset_Load();
	DWT_Delay(5);
	RGB_SendArray();
	//DWT_Delay(50);
	//RGB_RED2(118);
	//RGB_RED3(90);
	

}


/*****************/
void RGB_Arrow_ALL_LIGHT(void)
{
	RGB_RED3(300);
}
/*****************/
void RGB_Arrow_ALL_BLACK(void)
{
	RGB_BLACK3(300);
}
/*****************/
void RGB_Arrow(int16_t m)
{
	int16_t i,n;
	n=8*m;
	if(n==256)
	{
		n=0;
	}
	for(i=0;i<256;i++)//全灭
	{
		RGB_SetColor(i,BLACK);
	}
		RGB_SetColor(0+n,COLOR1);
	for(i=1+n;i<7+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=7+n;i<10+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=10+n;i<14+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=14+n;i<19+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=19+n;i<21+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=21+n;i<24+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=24+n;i<25+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=25+n;i<31+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=31+n;i<34+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=34+n;i<38+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=38+n;i<43+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=43+n;i<45+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	//第一个箭头
	
	RGB_SetColor(48+n,COLOR1);
	for(i=49+n;i<55+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=55+n;i<58+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=58+n;i<62+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=62+n;i<67+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=67+n;i<69+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=69+n;i<72+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=72+n;i<73+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=73+n;i<79+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=79+n;i<82+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=82+n;i<86+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=86+n;i<91+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=91+n;i<93+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	//第二个箭头
	RGB_SetColor(96+n,COLOR1);
	for(i=96+n;i<103+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=103+n;i<106+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=106+n;i<110+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=110+n;i<115+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=115+n;i<117+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=117+n;i<120+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=120+n;i<121+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=121+n;i<127+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=127+n;i<130+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=130+n;i<134+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=134+n;i<139+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=139+n;i<141+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	//第三个箭头
	RGB_SetColor(96+48+n,COLOR1);
	for(i=96+48+n;i<103+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=103+48+n;i<106+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=106+48+n;i<110+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=110+48+n;i<115+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=115+48+n;i<117+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=117+48+n;i<120+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=120+48+n;i<121+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=121+48+n;i<127+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=127+48+n;i<130+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=130+48+n;i<134+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=134+48+n;i<139+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=139+48+n;i<141+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	//第四个箭头
	RGB_SetColor(96+48+48+n,COLOR1);
	for(i=96+48+48+n;i<103+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=103+48+48+n;i<106+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=106+48+48+n;i<110+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=110+48+48+n;i<115+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=115+48+48+n;i<117+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=117+48+48+n;i<120+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=120+48+48+n;i<121+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=121+48+48+n;i<127+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=127+48+48+n;i<130+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=130+48+48+n;i<134+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=134+48+48+n;i<139+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=139+48+48+n;i<141+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	//第五个箭头
	RGB_SetColor(96+48+48+48+n,COLOR1);
	for(i=96+48+48+48+n;i<103+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=103+48+48+48+n;i<106+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=106+48+48+48+n;i<110+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=110+48+48+48+n;i<115+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=115+48+48+48+n;i<117+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=117+48+48+48+n;i<120+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=120+48+48+48+n;i<121+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=121+48+48+48+n;i<127+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=127+48+48+48+n;i<130+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=130+48+48+48+n;i<134+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=134+48+48+48+n;i<139+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=139+48+48+48+n;i<141+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}//第六个箭头
	RGB_SetColor(96+48+48+48+48+n,COLOR1);
	for(i=96+48+48+48+48+n;i<103+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=103+48+48+48+48+n;i<106+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=106+48+48+48+48+n;i<110+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=110+48+48+48+48+n;i<115+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=115+48+48+48+48+n;i<117+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=117+48+48+48+48+n;i<120+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=120+48+48+48+48+n;i<121+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	
	for(i=121+48+48+48+48+n;i<127+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=127+48+48+48+48+n;i<130+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=130+48+48+48+48+n;i<134+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	for(i=134+48+48+48+48+n;i<139+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,BLACK);
	}
	for(i=139+48+48+48+48+n;i<141+48+48+48+48+n;i++)//给对应个数LED写入红色
	{
		RGB_SetColor(i,COLOR1);
	}
	//为了代码可读性，所以选择了比较蠢的实现方式
	//Reset_Load();
	RGB_SendArray3();
}


