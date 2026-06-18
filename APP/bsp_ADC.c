#include "bsp_ADC.h"
#include "usbd_cdc_if.h"
#include "tim.h"

extern DMA_HandleTypeDef hdma_adc;

static  uint16_t adc_buf[2] = {0};
volatile uint16_t motor1_cur = 0;
volatile uint16_t motor2_cur = 0;

#define ADC1_ZERO_OFFSET 0
#define ADC2_ZERO_OFFSET 0
#define MVA_WINDOW 16
#define ADC_CNT 20
static uint16_t buf1[MVA_WINDOW] = {0};
static uint16_t sum1 = 0;
static uint8_t idx1 = 0;
static uint8_t cnt1 = 0;

static uint16_t buf2[MVA_WINDOW] = {0};
static uint16_t sum2 = 0;
static uint8_t idx2 = 0;
static uint8_t cnt2 = 0;
volatile uint16_t ch2[ADC_CNT] = {0};  // 通道2
volatile uint16_t ch3[ADC_CNT] = {0};  // 通道3
volatile uint32_t adc_cnt = 0;    // ADC就绪标志
uint64_t ADC1_Sum = 0;
uint64_t ADC2_Sum = 0;

void ADC_Start(void)
{
     // 启动 ADC DMA 双路采集（循环模式）
       HAL_ADC_Start_DMA(&hadc,(uint32_t*)adc_buf, 2);

}
char buf[32];
void ADC_Get_Current(void)
{   
		uint16_t adc1_raw = 0;
    uint16_t ADC1_Avage = 0;
    uint16_t ADC2_Avage = 0;
		
	
    ADC1_Avage = ADC1_Sum/adc_cnt;          // 取平均

    ADC2_Avage = ADC2_Sum/adc_cnt;          // 取平均
	
	ADC1_Sum = 0;
    ADC2_Sum = 0;
	adc_cnt = 0;
    if(ADC1_Avage > ADC1_ZERO_OFFSET)
    {
        adc1_raw = ADC1_Avage - ADC1_ZERO_OFFSET;
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
    if(ADC2_Avage > ADC2_ZERO_OFFSET)
    {
        adc2_raw = ADC2_Avage - ADC2_ZERO_OFFSET;
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

    // sprintf(buf, "%hu,%hu\n", motor1_cur, motor2_cur);
    // CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
		__disable_irq();
         ADC1_Sum += adc_buf[0];  // 通道2
         ADC2_Sum += adc_buf[1];  // 通道3
         adc_cnt++;
        //  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // ADC转换完成指示灯闪烁
		__enable_irq();
    }
}