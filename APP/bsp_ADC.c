#include "bsp_ADC.h"


volatile uint16_t adc_buf[2] = {0};    // DMA 目标数组（硬件自动写）
volatile uint16_t motor1_cur = 0;
volatile uint16_t motor2_cur = 0;

#define ADC1_ZERO_OFFSET 0  // ADC 零点偏移
#define ADC2_ZERO_OFFSET 0  // ADC 零点偏移

// I(mA) = ADC × 3300 / (4095 × 0.01 × 100)
//      = ADC × 3300 / 4095
//      = ADC × 0.8058
// 在 1ms 中断里调用这个函数
void ADC_Get_Current(void)
{
    uint16_t adc1_raw = 0;
    if(adc_buf[0] > ADC1_ZERO_OFFSET)
    {
        adc1_raw = adc_buf[0] - ADC1_ZERO_OFFSET;
    }else
    {
        adc1_raw = 0;
    }
    
    // 通道 1 → mA
    uint16_t raw_current1  = (uint16_t)( (int32_t)adc1_raw * 3300 / 4095 ) ;

     // 一阶滤波
    static float filtered1 = 0.0f;
    static uint8_t init1 = 0;
    if (!init1) {
        filtered1 = (float)raw_current1;
        init1 = 1;
    } else {
        filtered1 = 0.386f * (float)raw_current1 + 0.614f * filtered1;
    }
    
    // 存回 motor1_cur（注意可能需四舍五入）
    motor1_cur = (uint16_t)(filtered1 + 0.5f);

    // 通道 2 → mA
    uint16_t adc2_raw = 0;
    if(adc_buf[1] > ADC2_ZERO_OFFSET)
    {
        adc2_raw = adc_buf[1] - ADC2_ZERO_OFFSET;
    }else
    {
        adc2_raw = 0;
    }
    uint16_t raw_current2 = (uint16_t)( (int32_t)adc2_raw * 3300 / 4095 ) ;
    static float filtered2 = 0.0f;
    static uint8_t init2 = 0;
    if (!init2) {
        filtered2 = (float)raw_current2;
        init2 = 1;
    } else {
        filtered2 = 0.386f * (float)raw_current2 + 0.614f * filtered2;
    }
    motor2_cur = (uint16_t)(filtered2 + 0.5f);
}

//  omega = 2.0f * 3.1415926f * fc;  // 角频率 (rad/s),fc为截止频率
//  alpha = omega * Ts / (1.0f + omega * Ts);
//  filtered_current = alpha * input + (1.0f - alpha) * filtered_current;