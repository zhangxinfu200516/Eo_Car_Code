#include "dvc_robot_arm.h"
void Servo_Pwm_Init(void)
{
    //pwm初始化
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
    //HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
   // HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}
void Class_Robot_Arm::Init(float *Set_InitAngle)
{
    //初始化舵机PWM输出
    Servo_Pwm_Init();

    //初始化上电后关节位置
    OutPut(Set_InitAngle[0],Set_InitAngle[1],Set_InitAngle[2],Set_InitAngle[3],Set_InitAngle[4],Set_InitAngle[5]);
    
    //初始化各个关节实际角度
    for (auto i = 0; i < 6; i++)
    {
        Joint[i].Now_Angle = Set_InitAngle[i];
    }
}
/**************************************************************************
函数功能：将角度转换为脉宽，进而改变PWM输出占空比
入口参数：角度
返回  值：无
备注    ：角度范围为-90度到90度，当输入角度为-90度时，脉宽为0.5ms；当输入角度为0度时，脉宽为1.5ms；当输入角度为90度时，脉宽为2.5ms；
**************************************************************************/
void Class_Robot_Arm::translate_angle_to_pulse(float angle_1,float angle_2,float angle_3,float angle_4,float angle_5,float angle_6)
{
    Joint[Joint_1].PWM_Out = (((Joint[Joint_1].Target_Angle + 70) / 90 ) + 0.5)*(20000/20);
	Joint[Joint_2].PWM_Out = (((Joint[Joint_2].Target_Angle +90) / 90 ) + 0.5)*(20000/20);
	Joint[Joint_3].PWM_Out = (((Joint[Joint_3].Target_Angle) / 90 ) + 0.5)*(20000/20);
	Joint[Joint_4].PWM_Out = ((((Joint[Joint_4].Target_Angle + 90) / 270 * 180) / 90) + 0.5)*(20000/20);
	Joint[Joint_5].PWM_Out = ((((Joint[Joint_5].Target_Angle / 90 * 65) + 90) / 90 ) + 0.5)*(20000/20);
	Joint[Joint_6].PWM_Out = (((Joint[Joint_6].Target_Angle ) / 90 ) + 0.5)*(20000/20);

    for (auto i = 0; i < 6; i++)
    {
        Math_Constrain(&Joint[i].PWM_Out,500,2500);
    }
    
}
void Class_Robot_Arm::OutPut(float angle_1, float angle_2, float angle_3, float angle_4, float angle_5,  float angle_6)
{
	translate_angle_to_pulse(angle_1,angle_2,angle_3,angle_4,angle_5,angle_6);
	
	if(Joint[Joint_1].PWM_Out != NULL)
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, Joint[0].PWM_Out);
	}
	if(Joint[Joint_2].PWM_Out != NULL)
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, Joint[1].PWM_Out);
	}
	if(Joint[Joint_3].PWM_Out != NULL)
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, Joint[2].PWM_Out);
	}
	if(Joint[Joint_4].PWM_Out != NULL)
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, Joint[3].PWM_Out);
	}
//	if(Joint[Joint_5].PWM_Out != NULL)
//	{
//		__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, Joint[4].PWM_Out);
//	}
//	if(Joint[Joint_6].PWM_Out != NULL)
//	{
//		__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, Joint[5].PWM_Out);
//	}
}

/**************************************************************************
函数功能：根据目标位置，计算目标角度
入口参数：末端执行器姿态
返回  值：无
备注    ：target_x：目标距离原点距离；target_y：目标高度；
          alpha：末端控制器角度；target_angle：目标角度

侧视图                         俯视图

target_y                        90度
|                                |
|                                |
|______target_x      180度_______|________0度

**************************************************************************/
void Class_Robot_Arm::JointAngle_Resolution(float target_x, float target_y, float target_z)
{       
	float len_1, len_2, len_3, len_4;   //a1为底部圆台高度 剩下三个为三个机械臂长度 
	float j1,j2,j3,j4 ;   //四个姿态角
	float L, H, bottom_r;					//	L =	a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); P为底部圆盘半径R
	float j_sum;			//j2,j3,j4之和
	float len, high;   //总长度,总高度
	float cos_j3, sin_j3; //用来存储cosj3,sinj3数值
	float cos_j2, sin_j2;
	float k1, k2;
	int i;
	float n, m;
	n = 0;
	m = 0;

	//输入初始值
	bottom_r = BOTTOM_R; 		//底部圆盘半径
	len_1 = BOTTOM_H; 	//底部圆盘高度
	//机械臂长度
	len_2 = Joint_2_LENGTH;
	len_3 = Joint_3_LENGTH;
	len_4 = Joint_4_LENGTH;

    if (target_y >= Target_POS_MAX_Y)
        target_y = Target_POS_MAX_Y;
    else if (target_y <= Target_POS_MIN_Y)
        target_y = Target_POS_MIN_Y;

    if (target_x == 0)
		j1 = 90;
	else
		j1 = 90 - atan(target_x / (target_y + bottom_r)) * (57.3);


	for (i = 0; i <= 180; i ++)
	{
		j_sum = 3.1415927 * i / 180;

		len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
		high = target_z;

		L = len - len_4 * sin(j_sum);
		H = high - len_4 * cos(j_sum) - len_1;

		cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
		sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

		j3 = atan((sin_j3) / (cos_j3)) * (57.3);

		k2 = len_3 * sin(j3 / 57.3);
		k1 = len_2 + len_3 * cos(j3 / 57.3);

		cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
		sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

		j2 = atan((sin_j2) / (cos_j2)) * 57.3;
		j4 = j_sum * 57.3 - j2 - j3;

		if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
		{
			n ++;
		}
	}


	for (i = 0; i <= 180; i ++)
	{
		j_sum = 3.1415927 * i / 180;

		len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
		high = target_z;

		L = len - len_4 * sin(j_sum);
		H = high - len_4 * cos(j_sum) - len_1;

		cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
		sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

		j3 = atan((sin_j3) / (cos_j3)) * (57.3);

		k2 = len_3 * sin(j3 / 57.3);
		k1 = len_2 + len_3 * cos(j3 / 57.3);

		cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
		sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

		j2 = atan((sin_j2) / (cos_j2)) * 57.3;
		j4 = j_sum * 57.3 - j2 - j3;

		if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
		{
			m ++;
			if (m == n / 2 || m == (n + 1) / 2)
				break;			
		}
	}
    //为四个关节角度赋值
	Joint[0].Target_Angle = j1;
	Joint[1].Target_Angle = j2;
	Joint[2].Target_Angle = j3;
	Joint[3].Target_Angle = j4;
}


float abs_angle_error[6]={0,0,0,0,0,0};
void Class_Robot_Arm::Joint_Move_Control(float *Joint_Target_Angle)
{

	static float K = 1.0f;

	for (auto  i = 0; i < 6; i++)
	{
		abs_angle_error[i] = fabs(Joint_Target_Angle[i] - Joint[i].Now_Angle);   //误差绝对值

		for (;abs_angle_error[i] >= 3;abs_angle_error[i] - K)
		{
			K = abs_angle_error[i] > 20.0f ? 1.5f : 1.0f;

			float pwm_angle = (Joint_Target_Angle[i] > Joint[i].Now_Angle ? Joint_Target_Angle[i] - abs_angle_error[i] : Joint_Target_Angle[i] + abs_angle_error[i]);

			switch (i)
			{
			case Joint_1:
			OutPut(pwm_angle,Joint[Joint_2].Now_Angle,Joint[Joint_3].Now_Angle,Joint[Joint_4].Now_Angle,Joint[Joint_5].Now_Angle,Joint[Joint_6].Now_Angle);
			break;
			case Joint_2:
			OutPut(Joint[Joint_1].Now_Angle,pwm_angle,Joint[Joint_3].Now_Angle,Joint[Joint_4].Now_Angle,Joint[Joint_5].Now_Angle,Joint[Joint_6].Now_Angle);
			break;
			case Joint_3:
			OutPut(Joint[Joint_1].Now_Angle,Joint[Joint_2].Now_Angle,pwm_angle,Joint[Joint_4].Now_Angle,Joint[Joint_5].Now_Angle,Joint[Joint_6].Now_Angle);
			break;
			case Joint_4:
			OutPut(Joint[Joint_1].Now_Angle,Joint[Joint_2].Now_Angle,Joint[Joint_3].Now_Angle,pwm_angle,Joint[Joint_5].Now_Angle,Joint[Joint_6].Now_Angle);
			break;
			case Joint_5:
			OutPut(Joint[Joint_1].Now_Angle,Joint[Joint_2].Now_Angle,Joint[Joint_3].Now_Angle,Joint[Joint_4].Now_Angle,pwm_angle,Joint[Joint_6].Now_Angle);
			break;
			case Joint_6:
			OutPut(Joint[Joint_1].Now_Angle,Joint[Joint_2].Now_Angle,Joint[Joint_3].Now_Angle,Joint[Joint_4].Now_Angle,Joint[Joint_5].Now_Angle,pwm_angle);
			break;
			}
			
			Joint[i].Now_Angle = pwm_angle;
		}
		
		Joint[i].Now_Angle = Joint_Target_Angle[i];
	}

}
void Class_Robot_Arm::TIM_Process_PeriodElapsedCallback()
{

}