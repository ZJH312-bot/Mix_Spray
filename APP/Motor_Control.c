#include "Motor_Control.h"
#include "bsp_ADC.h"
#include "usb.h"
// 限流参数
#define Motor_SPRAY_CURRENT_MAX_LIMIT     2000   // 喷涂电机正常限流阈值 (mA)
#define Motor_SPRAY_CURRENT_DANGER        2500   // 喷涂电机危险阈值 (mA)，触发快速减额
#define Motor_MIX_CURRENT_MAX_LIMIT       2000   // 搅拌电机正常限流阈值 (mA)
#define Motor_MIX_CURRENT_DANGER          2500   // 搅拌电机危险阈值 (mA)，触发快速减额

#define Motor_SPRAY_MIX_CURRENT_HYSTERESIS    100    // 回差，防止震荡
#define Motor_SPRAY_MIX_DUTY_MIN          10     // 最小占空比百分比 (防止完全停转)
#define Motor_SPRAY_MIX_DUTY_MAX          100    // 最大占空比百分比

// 限流系数（整数百分比）
#define DUTY_REDUCE_PERCENT     95   // 降低到95%
#define DUTY_RECOVER_PERCENT    102  // 恢复到102%

// 状态变量
volatile uint8_t  History_SPRAY_duty_percent = 0;    // 衰减前目标占空比 0~100
volatile uint8_t  History_MIX_duty_percent = 0;    // 衰减前目标占空比 0~100

volatile uint16_t  last_current = 0.0;
// TIM2_CH1 → 搅拌电机 → mix_pwm_duty
// TIM3_CH1 → 喷涂电机 → spray_pwm_duty
uint16_t mix_pwm_duty = 0;
uint32_t spray_pwm_duty = 0;

/**
 * @brief  设置搅拌电机的速度
 */
void Motor_MIX_SPEED_Set(uint8_t MIX_SPEED)
{
    if (MIX_SPEED > 100) MIX_SPEED = 100;

    //  计算20kHz PWM对应的比较值
    // 公式：比较值 = 速度百分比 × ARR(2399) / 100
    mix_pwm_duty = (uint32_t)MIX_SPEED * 2399 /100;
}

/**
 * @brief  设置喷涂电机的速度
 */
void Motor_SPRAY_SPEED_Set(uint8_t SPRAY_SPEED)
{
    if (SPRAY_SPEED > 100) SPRAY_SPEED = 100;
    //  计算20kHz PWM对应的比较值
    // 公式：比较值 = 速度百分比 × ARR(2399) / 100
    spray_pwm_duty = (uint32_t)SPRAY_SPEED * 2399 / 100;
}

//喷涂电机速度限制
void Motor_SPRAY_Current_Limit(void) 
{
    uint16_t current = motor1_cur;  
    
    // 情况1：电流超过危险阈值 → 快速大幅度降低占空比
    if (current >= Motor_SPRAY_CURRENT_DANGER) 
    {
        system_state_data.spray_motor_speed = (uint8_t)(system_state_data.spray_motor_speed * DUTY_REDUCE_PERCENT / 100);
        if (system_state_data.spray_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) system_state_data.spray_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;

        return; 
    }
    // 情况2：电流超过正常限流阈值但未达危险 → 缓慢降低占空比
    else if (current >= Motor_SPRAY_CURRENT_MAX_LIMIT) 
    {
        // 使用更缓的降低系数或固定步长减小
        system_state_data.spray_motor_speed = (uint8_t)(system_state_data.spray_motor_speed  * 98 / 100);
        if (system_state_data.spray_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) system_state_data.spray_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;

        return;  
    }
    // 情况3：电流正常且低于（阈值 - 回差） → 尝试恢复占空比
    else if (current <= (Motor_SPRAY_CURRENT_MAX_LIMIT - Motor_SPRAY_MIX_CURRENT_HYSTERESIS)&& system_state_data.spray_motor_speed < History_SPRAY_duty_percent) 
    {
        system_state_data.spray_motor_speed = (uint8_t)(system_state_data.spray_motor_speed * DUTY_RECOVER_PERCENT / 100);
        if (system_state_data.spray_motor_speed > Motor_SPRAY_MIX_DUTY_MAX) system_state_data.spray_motor_speed = Motor_SPRAY_MIX_DUTY_MAX;

        return;
       
    } 
    
}
void Motor_MIX_Current_Limit(void) 
{
    uint16_t current = motor2_cur;  
    
    // 情况1：电流超过危险阈值 → 快速大幅度降低占空比
    if (current >= Motor_MIX_CURRENT_DANGER) 
    {
        system_state_data.mix_motor_speed = (uint8_t)(system_state_data.mix_motor_speed * DUTY_REDUCE_PERCENT / 100);
        if (system_state_data.mix_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) system_state_data.mix_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;
        return;
    }
    // 情况2：电流超过正常限流阈值但未达危险 → 缓慢降低占空比
    else if (current >= Motor_MIX_CURRENT_MAX_LIMIT) 
    {
        // 使用更缓的降低系数或固定步长减小
        system_state_data.mix_motor_speed = (uint8_t)(system_state_data.mix_motor_speed  * 98 / 100);
        if (system_state_data.mix_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) system_state_data.mix_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;
        return;
    }
    // 情况3：电流正常且低于（阈值 - 回差） → 尝试恢复占空比
    else if (current <= (Motor_MIX_CURRENT_MAX_LIMIT - Motor_SPRAY_MIX_CURRENT_HYSTERESIS)&& system_state_data.mix_motor_speed < History_MIX_duty_percent) 
    {
        system_state_data.mix_motor_speed = (uint8_t)(system_state_data.mix_motor_speed * DUTY_RECOVER_PERCENT / 100);
        if (system_state_data.mix_motor_speed > Motor_SPRAY_MIX_DUTY_MAX) system_state_data.mix_motor_speed = Motor_SPRAY_MIX_DUTY_MAX;
        
    } 
}
