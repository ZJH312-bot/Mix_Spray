/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint16_t mix_pwm_duty ;
extern uint32_t spray_pwm_duty;

void Motor_MIX_SPEED_Set(uint8_t MIX_SPEED);
void Motor_SPRAY_SPEED_Set(uint8_t SPRAY_SPEED);

#ifdef __cplusplus
}
#endif
#endif 

