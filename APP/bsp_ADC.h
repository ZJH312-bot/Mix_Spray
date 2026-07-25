#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "adc.h"
#include <stdint.h>
// 
extern volatile uint16_t motor1_cur;    // 电机1电流 (mA)
extern volatile uint16_t motor2_cur;    // 电机2电流 (mA)

void ADC_Get_Current(void); // 从 adc_buf 计算电流值并存回 motor1_cur 和 motor2_cur，内含滑动平均滤波

void ADC_Start(void);

#ifdef __cplusplus
}
#endif
#endif 
