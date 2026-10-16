/**
  ******************************************************************************
  * @file    Level_Detect.c
  * @brief   液位开关检测
  *          引脚    : PA7 (已在 gpio.c 配置为 GPIO_MODE_INPUT)
  *          开关类型 : NPN 型开漏输出
  *                    有液位 -> NPN 导通 -> PA7 低电平
  *                    无液位 -> NPN 截止 -> PA7 高电平(靠外部上拉电阻)
  ******************************************************************************
  */
#include "Level_Detect.h"

/* 防抖参数 */
#define LEVEL_SAMPLE_MS      10   // 采样间隔(ms)。本函数每 1ms 调用一次，故每 10 次调用采样一次
#define LEVEL_CONFIRM_CNT    100    // 连续一致确认次数，100 * 10ms = 1000ms 防抖

/**
  * @brief  获取当前液位状态（含软件防抖，内部连续确认）
  *         说明：需周期性调用本函数，建议放在 1ms 定时中断里每 1ms 调用一次，
  *               内部会自动按 LEVEL_SAMPLE_MS 间隔采样，连续 LEVEL_CONFIRM_CNT
  *               次一致后才更新输出状态，防止液面波动/抖动误判。
  * @retval 1 = 有液位, 0 = 无液位
  */
uint8_t Level_GetState(void)
{
    static uint8_t sample_cnt  = 0;  // 采样间隔计数
    static uint8_t last_raw    = 0;  // 上一次原始电平(归一化后: 1=有液位, 0=无液位)
    static uint8_t confirm_cnt = 0;  // 连续一致计数
    static uint8_t level_state = 0;  // 当前确认的液位状态(返回值)

    /* 按采样间隔分频，每 LEVEL_SAMPLE_MS 才真正采样一次 */
    if(++sample_cnt < LEVEL_SAMPLE_MS)
    {
        return level_state;
    }
    sample_cnt = 0;

    /* NPN 型: 有液位 = NPN 导通 = PA7 低电平 -> 归一化为 1 */
    uint8_t raw = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) ? 1 : 0;

    /* 连续一致确认: 相同则累加，不同则清零重新计数 */
    if(raw == last_raw)
    {
        confirm_cnt++;
    }
    else
    {
        last_raw    = raw;
        confirm_cnt = 0;
    }

    /* 连续确认达到阈值才更新状态 */
    if(confirm_cnt >= LEVEL_CONFIRM_CNT)
    {
        level_state = last_raw;
    }

    return level_state;
}
