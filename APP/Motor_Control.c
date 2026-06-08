#include "Motor_Control.h"
#include "bsp_ADC.h"
#include "usb.h"
#include "PID.h"

// 电流限流阈值 (单位：mA，1ADC单位≈0.806mA)
#define Motor_SPRAY_CURRENT_MAX_LIMIT     2000   // 喷涂电机最大工作电流
#define Motor_SPRAY_CURRENT_DANGER        2500   // 喷涂电机危险电流，触发快速降速
#define Motor_MIX_CURRENT_MAX_LIMIT       300    // 搅拌电机最大工作电流
#define Motor_MIX_CURRENT_DANGER          350    // 搅拌电机危险电流，触发快速降速

#define Motor_SPRAY_MIX_CURRENT_HYSTERESIS    100    // 电流回差，防止限流频繁抖动
#define Motor_SPRAY_MIX_DUTY_MIN          10     // 电机最小运行占空比
#define Motor_SPRAY_MIX_DUTY_MAX          100    // 电机最大运行占空比

// 限流控制参数
#define DEBOUNCE_CNT            5     // 防抖计数，连续5次超限才触发（每次调用间隔1ms）


// TIM2_CH1 驱动 搅拌电机 | mix_pwm_duty 对应定时器CCR比较值
// TIM3_CH1 驱动 喷涂电机 | spray_pwm_duty 对应定时器CCR比较值
uint16_t mix_pwm_duty = 0;
uint32_t spray_pwm_duty = 0;


void Motor_Spray_Current_Limit_Control(uint16_t spray_current, uint16_t read_sprary_current)
{
    float Spray_Current_ERROR = ((float)spray_current - (float)read_sprary_current)/1000.f;  // 电流误差，单位A
    
     Intergrator(&Spray_Current_Inter, Spray_Current_ERROR, &Spray_Current_Inter.output);

      // PI输出是占空比
    float pi_output = (float)(Spray_Current_Inter.kp * Spray_Current_ERROR + Spray_Current_Inter.ki * Spray_Current_Inter.output);

    uint32_t temp = (int32_t)pi_output;
    if(temp > 2399) temp = 2399;
    if(temp < 240)  temp = 240;
    spray_pwm_duty = temp;
   
}
void Motor_Mix_Current_Limit_Control(uint16_t mix_current, uint16_t read_mix_current)
{
    float Mix_Current_ERROR = ((float)mix_current - (float)read_mix_current) / 1000.0f;

     Intergrator(&Mix_Current_Inter, Mix_Current_ERROR, &Mix_Current_Inter.output);

    float pi_output = Mix_Current_Inter.kp * Mix_Current_ERROR + Mix_Current_Inter.ki * Mix_Current_Inter.output;

    // 先限幅float，再转整数
    if(pi_output > 2399.0f) pi_output = 2399.0f;
    if(pi_output < 240.0f)  pi_output = 240.0f;

    int32_t temp = (int32_t)pi_output;
    if(temp > 2399) temp = 2399;
    if(temp < 240)  temp = 240;

    mix_pwm_duty = (uint16_t)temp;
}