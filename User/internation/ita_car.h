#ifndef ITA_CAR_H
#define ITA_CAR_H
/* USER CODE BEGIN Includes */
#include "drv_can.h"
#include "drv_uart.h"
#include "crt_chassis.h"
#include "crt_gimbal.h"
#include "dvc_N100.h"
#include "dvc_XTARK_BlueTooth.h"
#include "dvc_uart_servo.h"
#include "dvc_minipc.h"
#include "dvc_oled.h"
#include "alg_fsm.h"
#include "dvc_dwt.h"
/* USER CODE BEGIN  */
class Class_EoCar;
// extern Class_EoCar EoCar;

class Class_EoCar
{
public:
    Class_Tricycle_Chassis Chassis;
    Class_PID Chassis_PID_Omage;
    Class_Slope Chassis_Slope;

    Class_Gimbal Gimbal;

    Class_N100 N100;
    Class_DVC_Bluetooth Remote;
    Class_MiniPC MiniPC;
    Class_OLED OLED;
    
    Class_FSM FSM_MINIPC_Alive_Protect;
    Class_FSM FSM_N100_Alive_Protect;
    Class_FSM FSM_Bluetooth_Alive_Protect;
    Class_FSM FSM_Control_Gimbal;
    inline void Set_Actual_Chassis_Yaw(float yaw);
    inline void Set_Voltage(float __voltage);
    inline void Set_Target_Chassis_Yaw(float _Target_Chassis_Yaw);
    inline float Get_Target_Chassis_Yaw();
    inline float Get_Voltage();
    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM_N100_Unline_Protect_PeriodElapsedCallback();
    void TIM_Bluetooth_Unline_Protect_PeriodElapsedCallback();
    void Control_Chassis();
    void Control_Gimbal();
protected:
    float Target_Chassis_Yaw;
    float Actual_Chassis_Yaw;

    float Voltage;
    void TIM1msMod50_Alive_PeriodElapsedCallback();
};
void Class_EoCar::Set_Actual_Chassis_Yaw(float yaw)
{
    Actual_Chassis_Yaw = yaw;
}
void Class_EoCar::Set_Voltage(float __voltage)
{
    Voltage = __voltage;
}
float Class_EoCar::Get_Voltage()
{
    return Voltage;
}
void Class_EoCar::Set_Target_Chassis_Yaw(float _Target_Chassis_Yaw)
{
    Target_Chassis_Yaw = _Target_Chassis_Yaw;
}
float Class_EoCar::Get_Target_Chassis_Yaw()
{
    return Target_Chassis_Yaw;
}
#endif 
