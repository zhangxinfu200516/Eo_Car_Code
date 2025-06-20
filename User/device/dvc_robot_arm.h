#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H
#include "drv_tim.h"
#include "gpio.h"
#include "drv_math.h"

#define Target_POS_MAX_Y 18.0f //机械臂最大高度
#define Target_POS_MIN_Y 3.0f //机械臂最小高度
#define BOTTOM_R 14.0f //底部圆盘半径
#define BOTTOM_H 10.0f //底部圆盘高度
#define Joint_2_LENGTH 10.0f //第二个关节长度
#define Joint_3_LENGTH 10.0f //第二个关节长度
#define Joint_4_LENGTH 10.0f //第二个关节长度
enum Enum_Joint_ID
{
    Joint_1 = 0,
    Joint_2,
    Joint_3,
    Joint_4,
    Joint_5,
    Joint_6,
};
struct Struct_Robot_Arm_TargetPos
{
    float Target_X;
    float Target_Y;
    float Target_Z;
}__attribute__((packed));

struct Struct_Robot_Arm_Joint
{
    float Target_Angle;
    float Now_Angle;
    int32_t PWM_Out;
}__attribute__((packed));

//六轴舵机机械臂
class Class_Robot_Arm
{
public:
    void Init(float *Set_InitAngle);
    void TIM_Process_PeriodElapsedCallback();
protected:
//成员变量
    Struct_Robot_Arm_TargetPos TargetPos;
    Struct_Robot_Arm_Joint Joint[6];
//成员函数
    void translate_angle_to_pulse(float angle_1,float angle_2,float angle_3,float angle_4,float angle_5,float angle_6);
    void OutPut(float angle_1, float angle_2, float angle_3, float angle_4, float angle_5,  float angle_6);
    void Joint_Move_Control(float *Joint_Target_Angle);
    void JointAngle_Resolution(float target_x, float target_y, float target_z);
};

#endif // !ROBOT_ARM_H