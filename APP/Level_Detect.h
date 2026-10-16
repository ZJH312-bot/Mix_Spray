/**
  ******************************************************************************
  * @file    Level_Detect.h
  * @brief   液位开关检测模块头文件
  ******************************************************************************
  */
#ifndef __LEVEL_DETECT_H__
#define __LEVEL_DETECT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
  * @brief  获取当前液位状态（需周期性调用，建议每 1ms 调用一次）
  * @retval 1 = 有液位, 0 = 无液位
  */
uint8_t Level_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __LEVEL_DETECT_H__ */
