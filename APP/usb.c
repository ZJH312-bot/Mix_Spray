#include "usb.h"
#include <string.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "USB_Link.h"       // APP层：连接异常检测(接收上位机反馈帧当心跳)

System_State_data system_state_data = {0};


// 最小发送缓冲区 固定10字节
static uint8_t usb_tx_buf[10];
// 接收缓冲区：要放得下最长的帧(反馈帧 9 字节)
static uint8_t usb_rx_buf[USB_RX_BUF_SIZE];
static uint8_t usb_rx_cnt = 0;

uint8_t usb_Receive_buf[USB_RX_BUF_SIZE];
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
/**
 * @brief  清空接收缓冲残留（软重连前后调用，避免解析到半帧/旧帧）
 */
void USB_Rx_Flush(void)
{
    usb_rx_cnt       = 0;
    USB_Receive_flag = 0;
    usb_Receive_Len  = 0;
}

// 组帧发送
 uint8_t USB_Tx_SendFrame(uint8_t cmd, uint8_t Motor_Control,uint8_t spray_motor_speed, uint8_t mix_motor_speed, 
                        uint8_t Auto_continuous_time, uint8_t Auto_interval_time,uint8_t GROUND_STATE,uint8_t level_state)
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
    // bit0~bit1: 接地状态(GROUND_STATE, 取值0/1/2)
    // bit4(第4位): 液位状态(level_state, 1=有液位 0=无液位)
    usb_tx_buf[7] = GROUND_STATE | ((level_state & 0x01) << 4);

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
//    state_data->spray_motor_speed = spray_motor_speed;
   state_data->History_MIX_duty_percent = mix_motor_speed;
//    state_data->mix_motor_speed = mix_motor_speed;
   state_data->Auto_continuous_time = Auto_continuous_time;
   state_data->Auto_interval_time = Auto_interval_time;
}
/**
 * @brief  USB接收入口，直接放在 CDC_Receive_FS 里调用
 * @note   上位机有两种帧，长度不同，用 长度 + 命令码 双重区分：
 *           DATA[1]=0x00 命令帧，8 字节 → 写控制状态
 *           DATA[1]=0xFF 反馈帧，9 字节 → 只当链路心跳，不解析内容
 *                                        (上位机每收到一帧下位机上报就回一帧)
 */
void USB_Rx_Parse(uint8_t *buf, uint32_t *len)
{
    uint32_t i;
    uint8_t  need_len;

    // 填充缓冲区（最多一帧的字节数）
    for(i = 0; i < *len && usb_rx_cnt < USB_RX_BUF_SIZE; i++)
    {
        usb_rx_buf[usb_rx_cnt++] = buf[i];
    }

    if(usb_rx_cnt < 2) return;   // 还没拿到 DATA[1]，判断不了帧长

    // 按命令码确定整帧长度
    need_len = (usb_rx_buf[1] == CMD_HOST_FEEDBACK) ? USB_ACK_FRAME_LEN : USB_CMD_FRAME_LEN;

    if(usb_rx_cnt < need_len) return;   // 整帧还没收齐，等下一包

    if(usb_rx_buf[0] == FRAME_HEAD)
    {
        if(usb_rx_buf[1] == CMD_HOST_FEEDBACK)
        {
            // 上位机反馈帧：说明上位机收到并处理了下位机的上报 → 链路心跳
            // (这里不校验内容，只认帧头+命令码；命令帧只在操作时下发，不能当心跳)
             if(usb_rx_buf[8] == CheckSum_Calc(usb_rx_buf, 8))
            USB_Link_NotifyAckFrame(&usb_link);
        }
        else
        {
            // 上位机命令帧：校验通过才写控制状态
            if(usb_rx_buf[7] == CheckSum_Calc(usb_rx_buf, 7))
            {
                USB_Link_NotifyCmdFrame(&usb_link);   // 合法命令帧：参与"恢复电机"的连续帧计数
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // LED1指示灯亮，表示上下位机交互正常
                USB_RunCmd(usb_rx_buf[2], usb_rx_buf[3], usb_rx_buf[4], usb_rx_buf[5], usb_rx_buf[6], &system_state_data);
            }
        }
    }

    usb_rx_cnt = 0;   // 清空计数 准备下一帧
}   