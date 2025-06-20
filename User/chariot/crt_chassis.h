#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "dvc_encodermotor.h"
#include "dvc_minipc.h"
//#define MAI_WHEEL_CHASSIS
#define TRICYCLE_CHASSIS

#ifdef  MAI_WHEEL_CHASSIS
#define encoder_resolution (1300.0f) //电机输出轴转一圈的输出脉冲
#define HALF_WIDTH (0.15f)//半轮距 单位m
#define HALF_LENGTH (0.07f)//半轴距 单位m
#define GearRation (1.0f)
#define WheelDiameter (0.055f)
#elif defined (TRICYCLE_CHASSIS)
#define encoder_resolution (41000.0f) //电机输出轴转一圈的输出脉冲 
#define  Chassis_radius (0.137f)//底盘半径
#define GearRation (1.0f)
#define WheelDiameter (0.075f)
#endif
/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type :uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN_Positive,
    Chassis_Control_Type_SPIN_Negative,
};

//omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:
    friend class Class_MiniPC;  // 关键声明
    
    void Init(float __Velocity_X_Max = 2.0f, float __Velocity_Y_Max = 2.0f, float __Omega_Max = 8.0f);
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline void Set_Velocity_X(float __Velocity_X);
    inline void Set_Velocity_Y(float __Velocity_Y);
    inline void Set_Omega(float __Omega);

    inline float Get_Now_Position_X();
    inline float Get_Now_Position_Y();

    void TIM_Calculate_PeriodElapsedCallback();
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
protected:
    //初始化相关常量
    Class_EncoderMotor E_Motor[4];

    //速度X限制
    float Velocity_X_Max;
    //速度Y限制
    float Velocity_Y_Max;
    //角速度限制
    float Omega_Max;
    //底盘小陀螺模式角速度
    float Spin_Omega = 4.0f;
    //常量

    //内部变量
    //转动电机目标值
    float Target_Wheel_Omega[4];

    //读变量
    //写变量

     //当前位移X
    float Now_Position_X = 0.0f;
    //当前位移Y
    float Now_Position_Y = 0.0f;
    //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;
    //前一个线速度 x
    float Pre_Velocity_X = 0.0f;
    //前一个线速度 y
    float Pre_Velocity_Y = 0.0f;

    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_FLLOW;
    //目标速度X
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;

    //内部函数
    void Speed_Resolution();
};
void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}
float Class_Tricycle_Chassis::Get_Velocity_X_Max()
{
    return Velocity_X_Max;
}
float Class_Tricycle_Chassis::Get_Velocity_Y_Max()
{
    return Velocity_Y_Max;
}
float Class_Tricycle_Chassis::Get_Omega_Max()
{
    return Omega_Max;
}
void Class_Tricycle_Chassis::Set_Velocity_X(float __Velocity_X)
{
    Target_Velocity_X = __Velocity_X;
}
void Class_Tricycle_Chassis::Set_Velocity_Y(float __Velocity_Y)
{
    Target_Velocity_Y = __Velocity_Y;
}
void Class_Tricycle_Chassis::Set_Omega(float __Omega)
{
    Target_Omega = __Omega;
}
float Class_Tricycle_Chassis::Get_Now_Position_X()
{
    return Now_Position_X;
}
float Class_Tricycle_Chassis::Get_Now_Position_Y()
{
    return Now_Position_Y;
}
#endif
