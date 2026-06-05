#include "usb.h"
#include <string.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

System_State_data system_state_data = {0};


// 最小发送缓冲区 固定10字节
static uint8_t usb_tx_buf[10];
// 最小接收缓冲区 固定9字节
static uint8_t usb_rx_buf[9];
static uint8_t usb_rx_cnt = 0;

uint8_t usb_Receive_buf[9];
uint32_t usb_Receive_Len;
volatile uint8_t USB_Receive_flag = 0;

extern PCD_HandleTypeDef hpcd_USB_FS;  

void USB_SoftReset(void)
{

 // 先把 PA12 (D+) 变成普通推挽输出，拉低 → 模拟拔掉USB
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio_conf = {0};

  gpio_conf.Pin = GPIO_PIN_12;
  gpio_conf.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_conf.Pull = GPIO_NOPULL;
  gpio_conf.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_conf);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // 拉低 D+ → 电脑识别断开
  HAL_Delay(300);

  // 释放引脚，还给 USB 外设
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);

  HAL_Delay(300);
}

static uint8_t CheckSum_Calc(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0; 
    
    for(uint16_t i=0; i<len; i++)
    {
        sum += buf[i];
    }
    
    return (uint8_t)(sum & 0xFF); 
}
// 组帧发送
 uint8_t USB_Tx_SendFrame(uint8_t cmd, uint8_t Motor_Control,uint8_t spray_motor_speed, uint8_t mix_motor_speed, 
                        uint8_t Auto_continuous_time, uint8_t Auto_interval_time,uint8_t GROUND_STATE)
{
    // USB未连接直接退出
    if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 1;
    }

    // 组装下位机上报帧头 BB AA
    usb_tx_buf[0] = FRAME_HEAD;
    usb_tx_buf[1] = cmd;
    usb_tx_buf[2] = Motor_Control;
    usb_tx_buf[3] = spray_motor_speed;
    usb_tx_buf[4] = mix_motor_speed;
    usb_tx_buf[5] = Auto_continuous_time;
    usb_tx_buf[6] = Auto_interval_time;
    usb_tx_buf[7] = GROUND_STATE;

    // 计算前8字节CRC
    uint8_t sum8 = CheckSum_Calc(usb_tx_buf, 8);
    usb_tx_buf[8] = sum8 ;
   
    // USB CDC发送一帧9字节
    CDC_Transmit_FS(usb_tx_buf, 9);

    return 0;
}

static void USB_RunCmd(uint8_t Motor_Control, uint8_t spray_motor_speed, uint8_t mix_motor_speed, 
                        uint8_t Auto_continuous_time, uint8_t Auto_interval_time,System_State_data *state_data)
{
   state_data->Motor_Control = Motor_Control;
   state_data->History_SPRAY_duty_percent = spray_motor_speed;
   state_data->History_MIX_duty_percent = mix_motor_speed;
   state_data->Auto_continuous_time = Auto_continuous_time;
   state_data->Auto_interval_time = Auto_interval_time;
}
/**
 * @brief  USB接收入口，直接放在 CDC_Receive_FS 里调用
 */
void USB_Rx_Parse(uint8_t *buf, uint32_t *len)
{
    uint32_t i;
    // 填充缓冲区
    for(i = 0; i < *len && usb_rx_cnt < 8; i++)
    {
        usb_rx_buf[usb_rx_cnt++] = buf[i];
    }

    // 收满一帧6字节 开始解析
    if(usb_rx_cnt >= 8)
    {
        // 校验帧头
        if(usb_rx_buf[0] == FRAME_HEAD)
        {
           
            // 取出收到的sum8
            uint8_t recv_sum8 =  usb_rx_buf[7];
            uint8_t calc_sum8 = CheckSum_Calc(usb_rx_buf, 7);

            // CRC校验通过才执行
            if(recv_sum8 == calc_sum8)
            {
                USB_RunCmd(usb_rx_buf[2], usb_rx_buf[3], usb_rx_buf[4], usb_rx_buf[5], usb_rx_buf[6], &system_state_data);	
            }
        }

        // 清空计数 准备下一帧
        usb_rx_cnt = 0;
    }
}   