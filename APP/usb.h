
#ifndef __USB_H__
#define __USB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 帧头
#define FRAME_HEAD 0xAC


typedef enum
{
    MIX_STATE          = 0x00,//手动模式下搅拌电机状态
    Auto_MIX_STATE     = 0x01,//自动模式下搅拌电机状态
    SPRAY_STATE        = 0x04,//喷涂电机状态
} System_State;
typedef struct
{
   uint8_t Motor_Control;       // 电机控制命令
   uint8_t mix_motor_speed;     // 搅拌电机速度
   uint8_t spray_motor_speed;   // 喷涂电机速度
   uint8_t Auto_continuous_time;// 自动连续搅拌时间
   uint8_t Auto_interval_time;  // 自动搅拌间隔时间
   uint8_t Current_Auto_continuous_time_Cnt;  // 当前自动连续搅拌时间计数
   uint8_t Current_Auto_interval_time_Cnt;    // 当前自动搅拌间隔时间计数
   uint8_t Auto_State;           // 自动模式状态,0表示当前在搅拌进行搅拌时间计数，1表示当前在搅拌进行间隔时间计数
} System_State_data;

/*****************  位操作  ************************/
#define	GET_BIT(x, bit)	    ((x & (1 << bit)) >> bit)
#define	SETT_BIT(x, bit)	(x |= (1 << bit))
#define	CLEA_BIT(x, bit)	(x &= ~(1 << bit))

void USB_Rx_Parse(uint8_t *buf, uint32_t *len);
void USB_SoftReset(void);
// 组帧发送
 uint8_t USB_Tx_SendFrame(uint8_t cmd, uint8_t Motor_Control,uint8_t spray_motor_speed, uint8_t mix_motor_speed, 
                        uint8_t Auto_continuous_time, uint8_t Auto_interval_time,uint8_t GROUND_STATE);

extern System_State_data system_state_data;
extern uint8_t usb_Receive_buf[9];
extern uint32_t usb_Receive_Len;
extern volatile uint8_t USB_Receive_flag ;

#ifdef __cplusplus
}
#endif
#endif 

