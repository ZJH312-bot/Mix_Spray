#include "Motor_Control.h"
#include "bsp_ADC.h"
#include "usb.h"

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

// 全局变量
volatile uint8_t  start_flag = 0;    // 喷涂电机历史设定占空比 0~100

volatile uint16_t  last_current = 0;
// TIM2_CH1 驱动 搅拌电机 | mix_pwm_duty 对应定时器CCR比较值
// TIM3_CH1 驱动 喷涂电机 | spray_pwm_duty 对应定时器CCR比较值
uint16_t mix_pwm_duty = 0;
uint32_t spray_pwm_duty = 0;

/**
 * @brief  设置搅拌电机转速
 * @param  MIX_SPEED: 转速百分比 0~100
 */
void Motor_MIX_SPEED_Set(uint8_t MIX_SPEED)
{
    if (MIX_SPEED > 100) MIX_SPEED = 100;

    // 转换为20kHz PWM比较值
    // 比较值 = 转速百分比 × 自动重装载值(2399) / 100
    mix_pwm_duty = (uint32_t)MIX_SPEED * 2399 /100;
}

/**
 * @brief  设置喷涂电机转速
 * @param  SPRAY_SPEED: 转速百分比 0~100
 */
void Motor_SPRAY_SPEED_Set(uint8_t SPRAY_SPEED)
{
    if (SPRAY_SPEED > 100) SPRAY_SPEED = 100;
    // 转换为20kHz PWM比较值
    // 比较值 = 转速百分比 × 自动重装载值(2399) / 100
    spray_pwm_duty = (uint32_t)SPRAY_SPEED * 2399 / 100;
}

/**
 * @brief  喷涂电机电流限流保护
 * @note   三级限流逻辑：危险级快速降速 → 限流级缓慢降速 → 正常级缓慢恢复
 */
void Motor_SPRAY_Current_Limit(void) 
{
    uint16_t current = motor1_cur;  
    static uint8_t danger_cnt = 0;   // 危险电流防抖计数器
    static uint8_t limit_cnt = 0;    // 限流电流防抖计数器
    static uint8_t recover_cnt = 0;  // 电流恢复防抖计数器
    
    // 第一级：危险电流保护 → 快速降速（防止堵转烧管）
    if (current >= Motor_SPRAY_CURRENT_DANGER) 
    {
        danger_cnt++;
        limit_cnt = 0;
        recover_cnt = 0;
        if (danger_cnt >= DEBOUNCE_CNT) 
        {
            danger_cnt = 0;
            start_flag = 0; 
            system_state_data.spray_motor_speed = (uint8_t)(system_state_data.spray_motor_speed * 95 / 100);
            if (system_state_data.spray_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) 
                system_state_data.spray_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;
        }
        return; 
    }
    danger_cnt = 0;
    
    // 第二级：正常限流保护 → 缓慢降速（防止长时间过载）
    if (current >= Motor_SPRAY_CURRENT_MAX_LIMIT) 
    {
        limit_cnt++;
        recover_cnt = 0;
        if (limit_cnt >= DEBOUNCE_CNT)
        {
            limit_cnt = 0;
            system_state_data.spray_motor_speed = (uint8_t)(system_state_data.spray_motor_speed * 98 / 100);
            if (system_state_data.spray_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) 
                system_state_data.spray_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;
        }
        return;  
    }
    limit_cnt = 0;
    
    // 第三级：电流恢复 → 缓慢回升到历史设定值
    if (current <= (Motor_SPRAY_CURRENT_MAX_LIMIT - Motor_SPRAY_MIX_CURRENT_HYSTERESIS)
        && system_state_data.spray_motor_speed < system_state_data.History_SPRAY_duty_percent) 
    {
        recover_cnt++;
        if (recover_cnt >= DEBOUNCE_CNT) 
        {
            recover_cnt = 0;
            system_state_data.spray_motor_speed += 1;
            if (system_state_data.spray_motor_speed > Motor_SPRAY_MIX_DUTY_MAX) 
                system_state_data.spray_motor_speed = Motor_SPRAY_MIX_DUTY_MAX;
        }
        return;
    }
    recover_cnt = 0;
}

/**
 * @brief  搅拌电机电流限流保护
 * @note   三级限流逻辑：危险级快速降速 → 限流级缓慢降速 → 正常级缓慢恢复
 */
void Motor_MIX_Current_Limit(void) 
{
    uint16_t current = motor2_cur;  
    static uint8_t danger_cnt = 0;
    static uint8_t limit_cnt = 0;
    static uint8_t recover_cnt = 0;
    
    // 第一级：危险电流保护 → 快速降速（防止堵转烧管）
    if (current >= Motor_MIX_CURRENT_DANGER) 
    {
        danger_cnt++;
        limit_cnt = 0;
        recover_cnt = 0;
        if (danger_cnt >= DEBOUNCE_CNT) 
        {
            danger_cnt = 0;
            start_flag = 0;
            system_state_data.mix_motor_speed = (uint8_t)(system_state_data.mix_motor_speed * 95 / 100);
            if (system_state_data.mix_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) 
                system_state_data.mix_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;
        }
        return;
    }
    danger_cnt = 0;
    
    // 第二级：正常限流保护 → 缓慢降速（防止长时间过载）
    if (current >= Motor_MIX_CURRENT_MAX_LIMIT) 
    {
        limit_cnt++;
        recover_cnt = 0;
        if (limit_cnt >= DEBOUNCE_CNT) 
        {
            limit_cnt = 0;
            system_state_data.mix_motor_speed = (uint8_t)(system_state_data.mix_motor_speed * 98 / 100);
            if (system_state_data.mix_motor_speed < Motor_SPRAY_MIX_DUTY_MIN) 
                system_state_data.mix_motor_speed = Motor_SPRAY_MIX_DUTY_MIN;
        }
        return;
    }
    limit_cnt = 0;
    
    // 第三级：电流恢复 → 缓慢回升到历史设定值
    if (current <= (Motor_MIX_CURRENT_MAX_LIMIT - Motor_SPRAY_MIX_CURRENT_HYSTERESIS)
        && system_state_data.mix_motor_speed < system_state_data.History_MIX_duty_percent) 
    {
        recover_cnt++;
        if (recover_cnt >= DEBOUNCE_CNT) 
        {
            recover_cnt = 0;
            system_state_data.mix_motor_speed += 1;
            if (system_state_data.mix_motor_speed > Motor_SPRAY_MIX_DUTY_MAX) 
                system_state_data.mix_motor_speed = Motor_SPRAY_MIX_DUTY_MAX;
        }
        return;
    }
    recover_cnt = 0;
}