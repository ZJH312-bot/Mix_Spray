#ifndef __GND_SCAN_H__
#define __GND_SCAN_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include "tim.h"

extern volatile uint8_t GND_State ;
extern volatile bool LED_flag;
extern volatile uint8_t TX_flag ;
extern volatile uint8_t Current_flag ;// 电流控制标志，0表示不控制，1表示控制
extern volatile uint8_t adc_ready_flag;  // ADC就绪标志



#ifdef __cplusplus
}
#endif
#endif 

