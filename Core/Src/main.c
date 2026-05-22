/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor_Control.h"
#include "usb.h"
#include "GND_Scan.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  USB_SoftReset();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)&mix_pwm_duty, 1);	// 启动搅拌电机
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)&spray_pwm_duty, 1);	// 启动喷涂电机
  HAL_TIM_Base_Start_IT(&htim14);  // 启动1秒更新中断
  HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);  // 开启捕获中断
	
	// USB_SoftReset(); // 强制重新枚举,放在MX_USB_DEVICE_Init()前面，确保每次上电都能正确枚举

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(GND_State==0)// 已接地，正常工作
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // LED2接地指示灯亮
      
    }else if(GND_State==2) // 接地不良，进入警告模式
    {

      if(LED_flag==1)HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2); // LED2接地指示灯闪烁
    }
    else      // 未接地，进入安全模式
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // LED2接地指示灯灭
    }
		
		if(GET_BIT(system_state_data.Motor_Control, SPRAY_STATE))// 喷涂电机打开
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); 
			if(system_state_data.spray_motor_speed<10)system_state_data.spray_motor_speed=10;
      Motor_SPRAY_SPEED_Set(system_state_data.spray_motor_speed);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); 
      Motor_SPRAY_SPEED_Set(0);
    }
    if(GET_BIT(system_state_data.Motor_Control, Auto_MIX_STATE))// 搅拌电机自动模式
    {
      if(system_state_data.Auto_State == 0) // 当前在搅拌时间计数
      {
        if(system_state_data.mix_motor_speed<10)system_state_data.mix_motor_speed=10;
        Motor_MIX_SPEED_Set(system_state_data.mix_motor_speed);
      }
      else // 当前在间隔时间计数
      {
        Motor_MIX_SPEED_Set(0);
      }
     }
     else
     {
        system_state_data.Current_Auto_continuous_time_Cnt = 0; // 退出自动模式，重置搅拌时间计数
        system_state_data.Current_Auto_interval_time_Cnt = 0;   // 退出自动模式，重置间隔时间计数
        system_state_data.Auto_State = 0; // 退出自动模式，重置自动模式状态
         if(GET_BIT(system_state_data.Motor_Control, MIX_STATE))// 搅拌电机手动打开
        {
          if(system_state_data.mix_motor_speed<10)system_state_data.mix_motor_speed=10;
          Motor_MIX_SPEED_Set(system_state_data.mix_motor_speed);
        }
        else
        {
          Motor_MIX_SPEED_Set(0);
        }
     }
    if(TX_flag==1&&USB_Receive_flag==0)
		{
			USB_Tx_SendFrame(0x00,system_state_data.Motor_Control,system_state_data.spray_motor_speed,
                                  system_state_data.mix_motor_speed,system_state_data.Auto_continuous_time,
                                  system_state_data.Auto_interval_time,GND_State);//上位机数据上报
			TX_flag=0;
		}
		if(USB_Receive_flag)//USB数据处理
    {
      USB_Rx_Parse(usb_Receive_buf, &usb_Receive_Len);
      USB_Receive_flag = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // LED1指示灯亮，表示上下位机交互正常
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
