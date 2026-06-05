#include "bsp_ADC.h"
#include "usbd_cdc_if.h"

volatile uint16_t adc_buf[2] = {0};
volatile uint16_t motor1_cur = 0;
volatile uint16_t motor2_cur = 0;

#define ADC1_ZERO_OFFSET 0
#define ADC2_ZERO_OFFSET 0
#define MVA_WINDOW 16

static uint16_t buf1[MVA_WINDOW] = {0};
static uint16_t sum1 = 0;
static uint8_t idx1 = 0;
static uint8_t cnt1 = 0;

static uint16_t buf2[MVA_WINDOW] = {0};
static uint16_t sum2 = 0;
static uint8_t idx2 = 0;
static uint8_t cnt2 = 0;


char buf[32];

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
    
    uint16_t raw_current1 = (uint16_t)( (int32_t)adc1_raw * 3300 / 4095 );

    sum1 += raw_current1;
    sum1 -= buf1[idx1];
    buf1[idx1] = raw_current1;
    idx1++;
    if (idx1 >= MVA_WINDOW) idx1 = 0;
    if (cnt1 < MVA_WINDOW) cnt1++;

    if (cnt1 == MVA_WINDOW) {
        motor1_cur = (uint16_t)((sum1 + (MVA_WINDOW / 2)) / MVA_WINDOW);
    } else {
        motor1_cur = (uint16_t)((sum1 + (cnt1 / 2)) / cnt1);
    }
    
    uint16_t adc2_raw = 0;
    if(adc_buf[1] > ADC2_ZERO_OFFSET)
    {
        adc2_raw = adc_buf[1] - ADC2_ZERO_OFFSET;
    }else
    {
        adc2_raw = 0;
    }
    uint16_t raw_current2 = (uint16_t)( (int32_t)adc2_raw * 3300 / 4095 );


    sum2 += raw_current2;
    sum2 -= buf2[idx2];
    buf2[idx2] = raw_current2;
    idx2++;
    if (idx2 >= MVA_WINDOW) idx2 = 0;
    if (cnt2 < MVA_WINDOW) cnt2++;

    if (cnt2 == MVA_WINDOW) {
        motor2_cur = (uint16_t)((sum2 + (MVA_WINDOW / 2)) / MVA_WINDOW);
    } else {
        motor2_cur = (uint16_t)((sum2 + (cnt2 / 2)) / cnt2);
    }

    sprintf(buf, "%hu,%hu\n", motor1_cur, motor2_cur);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
