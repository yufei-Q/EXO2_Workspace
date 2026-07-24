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
#include "can.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dm_d4340p.h"
#include "usb_motor_comm.h"
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
USB_MotorCommand_t motor_command;
uint8_t motor_enabled[DM_D4340P_MOTOR_COUNT];
uint8_t usb_command_received;
uint32_t last_usb_command_tick;
//float torque=0;
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
  CAN_FilterTypeDef can_filter;
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
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* D4340P motor IDs on CAN1. */
  d4340p_motors[0].motor_id = 1U;
  d4340p_motors[1].motor_id = 2U;
  d4340p_motors[2].motor_id = 3U;
  d4340p_motors[3].motor_id = 4U;

  /* Receive all standard CAN frames in FIFO0. */
  can_filter.FilterBank = 0U;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0U;
  can_filter.FilterIdLow = 0U;
  can_filter.FilterMaskIdHigh = 0U;
  can_filter.FilterMaskIdLow = 0U;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(
        &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  if (DM_D4340P_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
//	DM_D4340P_Enable(&d4340p_motors[0]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		DM_D4340P_MitControl(&d4340p_motors[0],0,0,0,0,torque);
//		HAL_Delay(1);
    uint8_t index;

    if (USB_MotorComm_GetCommand(&motor_command) != 0U)
    {
      usb_command_received = 1U;
      last_usb_command_tick = HAL_GetTick();

      for (index = 0U; index < DM_D4340P_MOTOR_COUNT; ++index)
      {
        if ((motor_command.motor[index].flags &
             USB_MOTOR_FLAG_CLEAR_ERROR) != 0U)
        {
          (void)DM_D4340P_ClearError(&d4340p_motors[index]);
          HAL_Delay(1U);
        }

        if ((motor_command.motor[index].flags &
             USB_MOTOR_FLAG_SET_ZERO) != 0U)
        {
          (void)DM_D4340P_SetZero(&d4340p_motors[index]);
          HAL_Delay(1U);
        }

        if (((motor_command.motor[index].flags &
              USB_MOTOR_FLAG_ENABLE) != 0U) &&
            (motor_enabled[index] == 0U))
        {
          if (DM_D4340P_Enable(&d4340p_motors[index]) == HAL_OK)
          {
            motor_enabled[index] = 1U;
          }
          HAL_Delay(1U);
        }
        else if (((motor_command.motor[index].flags &
                   USB_MOTOR_FLAG_ENABLE) == 0U) &&
                 (motor_enabled[index] != 0U))
        {
          if (DM_D4340P_Disable(&d4340p_motors[index]) == HAL_OK)
          {
            motor_enabled[index] = 0U;
          }
          HAL_Delay(1U);
        }
      }
    }

    if ((usb_command_received != 0U) &&
        ((HAL_GetTick() - last_usb_command_tick) > 100U))
    {
      for (index = 0U; index < DM_D4340P_MOTOR_COUNT; index++)
      {
        if (motor_enabled[index] != 0U)
        {
          (void)DM_D4340P_Disable(&d4340p_motors[index]);
          motor_enabled[index] = 0U;
          HAL_Delay(1U);
        }
      }
      usb_command_received = 0U;
    }

    for (index = 0U; index < DM_D4340P_MOTOR_COUNT; ++index)
    {
      if (motor_enabled[index] != 0U)
      {
        (void)DM_D4340P_MitControl(
          &d4340p_motors[index],
          motor_command.motor[index].position,
          motor_command.motor[index].velocity,
          motor_command.motor[index].kp,
          motor_command.motor[index].kd,
          motor_command.motor[index].torque);
      }
    }
		HAL_Delay(3U);
    (void)USB_MotorComm_SendFeedback();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
