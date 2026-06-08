/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint16_t mix_pwm_duty ;
extern uint32_t spray_pwm_duty;

void Motor_Spray_Current_Limit_Control(uint16_t spray_current, uint16_t read_sprary_current);
void Motor_Mix_Current_Limit_Control(uint16_t mix_current, uint16_t read_mix_current);

#ifdef __cplusplus
}
#endif
#endif 

