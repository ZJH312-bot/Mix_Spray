/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint16_t mix_pwm_duty ;
extern uint32_t spray_pwm_duty;
extern volatile uint8_t  History_SPRAY_duty_percent ;    // 衰减前目标占空比 0~100
extern volatile uint8_t  History_MIX_duty_percent ;    // 衰减前目标占空比 0~100
extern volatile uint8_t  Current_Time_flag ;

void Motor_MIX_SPEED_Set(uint8_t MIX_SPEED);
void Motor_SPRAY_SPEED_Set(uint8_t SPRAY_SPEED);
void Motor_SPRAY_Current_Limit(void);
void Motor_MIX_Current_Limit(void);

#ifdef __cplusplus
}
#endif
#endif 

