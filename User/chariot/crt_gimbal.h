#ifndef CRT_GIMBAL
#define CRT_GIMBAL
#include "dvc_lkmotor.h"
#include "dvc_uart_servo.h"
#include "alg.slope.h"
#include "config.h"
enum Enum_Gimbal_Control_Type :uint8_t
{
    Gimbal_Control_Type_Disable = 0,
    Gimbal_Control_Type_Enable,
};
class Class_Gimbal
{
public:
    Class_LK_Motor LK_Motor;    

    Class_Uart_Servo Pitch;
    Class_Uart_Servo Yaw;
    Class_PID Yaw_PID;
    Class_PID Pitch_PID;
    Class_Slope Yaw_Slope;
    Class_Slope Pitch_Slope;
    
    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();
    inline float Get_pixel_dx();
    inline float Get_pixel_dy();

    inline void Set_pixel_dx(float __pixel_dx);
    inline void Set_pixel_dy(float __pixel_dy);
    
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);
    inline void Set_Yaw_Lk_Angle(float __Yaw_Lk_Angle);
    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
protected:
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_Disable; 
    float Yaw_Lk_Angle;
    //舵机坐标系下角度
    float Target_Pitch = 75.0f; 
    float Target_Yaw = Yaw_Zero_Pos;
    const float Target_Pitch_Speed = 250.0f;
    const float Target_Yaw_Speed = 250.0f;
    float Last_Pitch;
    float Last_Yaw;
    //获取舵机像素偏差
    float pixel_dx;
    float pixel_dy;

    void Output();
};

void Class_Gimbal::Set_Yaw_Lk_Angle(float __Yaw_Lk_Angle)
{
    Yaw_Lk_Angle = __Yaw_Lk_Angle;
}
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return Target_Yaw;
}
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw = __Target_Yaw_Angle;
}
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch = __Target_Pitch_Angle;
}
void Class_Gimbal::Set_pixel_dx(float __pixel_dx)
{
    pixel_dx = __pixel_dx;
}
void Class_Gimbal::Set_pixel_dy(float __pixel_dy)
{
    pixel_dy = __pixel_dy;
}
float Class_Gimbal::Get_pixel_dx()
{
    return pixel_dx;
}
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return Target_Pitch;
}
float Class_Gimbal::Get_pixel_dy()
{
    return pixel_dy;
}
#endif // !CRT_GIMBAL