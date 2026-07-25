#include "GND_Scan.h"
#include "usb.h"
#include "Motor_Control.h"
volatile uint32_t sys_tick;
volatile uint8_t GND_State = 1;//接地状态，1表示未接地，0表示已接地，2表示接地不良
volatile bool LED_flag = 0;// LED状态标志，0表示LED灭，1表示LED亮
volatile uint8_t TX_flag = 0;// 发送状态标志，0表示不发送，1表示发送
volatile uint8_t Current_flag = 0;// 电流控制标志，0表示不控制，1表示控制

uint8_t Sec_Cnt = 0;// 秒计数器，每60秒更新一次自动模式计数
uint16_t MS_Cnt = 0;// 毫秒计数器，每1000ms更新一次接地状态
volatile uint16_t pulse_count;// 脉冲计数器，每秒更新一次，根据脉冲数量判断接地状态
/**
 * @brief  定时器更新中断回调函数（每1ms触发一次）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14)
  {
    Current_flag = 1; // 设置电流控制标志，主循环里会调用限流函数
    // if(start_flag==1&&system_state_data.History_MIX_duty_percent>system_state_data.mix_motor_speed) 
    // {
    //   system_state_data.mix_motor_speed++; // 逐步增加搅拌电机占空比，缓坡启动
    // }
    // if(start_flag==1&&system_state_data.History_SPRAY_duty_percent>system_state_data.spray_motor_speed) 
    // {
    //   system_state_data.spray_motor_speed++; // 逐步增加喷涂电机占空比，缓坡启动
    // }
    MS_Cnt++;
    if(MS_Cnt >= 1000) // 每1000ms更新一次
    {
      MS_Cnt = 0;
      LED_flag=!LED_flag;
		  TX_flag = 1;
		
      // 范围判断：正常市电频率48~52Hz → 1秒48~52个脉冲
      if (pulse_count >= 45)
      {
        GND_State = 0; // 已接地
      }
      else  if(pulse_count < 45&&pulse_count > 0) // 脉冲过少，可能未接地或接地不良
      {
        GND_State = 2; // 接地不良
      }else if(pulse_count == 0) // 无脉冲，未接地
      {
        GND_State = 1; // 未接地
      }
      // if(pulse_count >=2000)GND_State = 2;
       pulse_count = 0;
      
      if(GET_BIT(system_state_data.Motor_Control, Auto_MIX_STATE))// 搅拌电机自动模式
      {
        Sec_Cnt++;
        if(Sec_Cnt >= 60) // 每1分钟更新一次自动模式计数
        {
          Sec_Cnt = 0;
          if(system_state_data.Auto_State ==0) // 当前在搅拌时间计数
          { 
              if(system_state_data.Auto_continuous_time >0)
              {
                system_state_data.Current_Auto_continuous_time_Cnt++;
                if(system_state_data.Current_Auto_continuous_time_Cnt >= system_state_data.Auto_continuous_time)
                {
                    system_state_data.Current_Auto_continuous_time_Cnt = 0;
                    system_state_data.Auto_State = 1; // 切换到间隔时间计数
                }
              }
          }
          else // 当前在间隔时间计数
          {
            if(system_state_data.Auto_interval_time >0)
            {
              system_state_data.Current_Auto_interval_time_Cnt++;
              if(system_state_data.Current_Auto_interval_time_Cnt >= system_state_data.Auto_interval_time)
              {
                  system_state_data.Current_Auto_interval_time_Cnt = 0;
                  system_state_data.Auto_State = 0; // 切换到搅拌时间计数
              }
            }
          }
        }
      }
    }
  }
}
//输入捕获中断
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    pulse_count++; 
  }
}
// volatile uint8_t adc_ready_flag = 0;  // ADC就绪标志

// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
// 	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
//           adc_ready_flag = 1; // 设置 ADC 就绪标志
//          pulse_count++; 
//     }
// }