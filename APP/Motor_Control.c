#include "Motor_Control.h"
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
    mix_pwm_duty = (uint32_t)MIX_SPEED * 2399 *0.01;
}

/**
 * @brief  设置喷涂电机的速度
 */
void Motor_SPRAY_SPEED_Set(uint8_t SPRAY_SPEED)
{
    if (SPRAY_SPEED > 100) SPRAY_SPEED = 100;
    //  计算20kHz PWM对应的比较值
    // 公式：比较值 = 速度百分比 × ARR(2399) / 100
    spray_pwm_duty = (uint32_t)SPRAY_SPEED * 2399 *0.01;
}
