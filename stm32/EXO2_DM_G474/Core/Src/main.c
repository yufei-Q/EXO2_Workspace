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
#include "fdcan.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <dm_motor.h>
#include <usb_motor_comm.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_ACTION_CLEAR_ERROR  0x01U
#define MOTOR_ACTION_SET_ZERO     0x02U
#define MOTOR_ACTION_ENABLE       0x04U
#define MOTOR_ACTION_DISABLE      0x08U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static USB_MotorCommand_t motor_command;
static volatile uint8_t motor_enabled[DM_MOTOR_COUNT];
static uint8_t motor_action_pending[DM_MOTOR_COUNT];
static uint8_t motor_action_index;
static volatile uint8_t motor_control_index;
static volatile uint8_t feedback_pending;
static volatile uint8_t usb_command_received;
static uint32_t last_usb_command_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_ProcessPendingAction(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_ProcessPendingAction(void)
{
  HAL_StatusTypeDef status;
  uint32_t interrupt_state;
  uint8_t action;
  uint8_t index;
  uint8_t offset;

  for (offset = 0U; offset < DM_MOTOR_COUNT; ++offset)
  {
    index = (uint8_t)((motor_action_index + offset) % DM_MOTOR_COUNT);
    action = motor_action_pending[index];
    if (action == 0U)
    {
      continue;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    if ((action & MOTOR_ACTION_CLEAR_ERROR) != 0U)
    {
      status = DM_Motor_ClearError(&dm_motors[index]);
      action = MOTOR_ACTION_CLEAR_ERROR;
    }
    else if ((action & MOTOR_ACTION_SET_ZERO) != 0U)
    {
      status = DM_Motor_SetZero(&dm_motors[index]);
      action = MOTOR_ACTION_SET_ZERO;
    }
    else if ((action & MOTOR_ACTION_DISABLE) != 0U)
    {
      status = DM_Motor_Disable(&dm_motors[index]);
      action = MOTOR_ACTION_DISABLE;
    }
    else
    {
      status = DM_Motor_Enable(&dm_motors[index]);
      action = MOTOR_ACTION_ENABLE;
    }

    if (interrupt_state == 0U)
    {
      __enable_irq();
    }

    if (status == HAL_OK)
    {
      motor_action_pending[index] &= (uint8_t)~action;
      if (action == MOTOR_ACTION_ENABLE)
      {
        motor_enabled[index] = 1U;
      }
    }

    motor_action_index = index + 1U;
    if (motor_action_index >= DM_MOTOR_COUNT)
    {
      motor_action_index = 0U;
    }
    return;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  FDCAN_FilterTypeDef fdcan_filter;

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
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  fdcan_filter.IdType = FDCAN_STANDARD_ID;
  fdcan_filter.FilterIndex = 0U;
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  fdcan_filter.FilterID1 = 0U;
  fdcan_filter.FilterID2 = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 32U, 0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(
        &hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (DM_Motor_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t index;

    if (USB_MotorComm_GetCommand(&motor_command) != 0U)
    {
      usb_command_received = 1U;
      last_usb_command_tick = HAL_GetTick();

      for (index = 0U; index < DM_MOTOR_COUNT; ++index)
      {
        if ((motor_command.motor[index].flags &
             USB_MOTOR_FLAG_CLEAR_ERROR) != 0U)
        {
          motor_action_pending[index] |= MOTOR_ACTION_CLEAR_ERROR;
        }

        if ((motor_command.motor[index].flags &
             USB_MOTOR_FLAG_SET_ZERO) != 0U)
        {
          motor_action_pending[index] |= MOTOR_ACTION_SET_ZERO;
        }

        if ((motor_command.motor[index].flags &
             USB_MOTOR_FLAG_ENABLE) != 0U)
        {
          if (motor_enabled[index] == 0U)
          {
            motor_action_pending[index] &=
              (uint8_t)~MOTOR_ACTION_DISABLE;
            motor_action_pending[index] |= MOTOR_ACTION_ENABLE;
          }
        }
        else if ((motor_enabled[index] != 0U) ||
                 ((motor_action_pending[index] &
                   MOTOR_ACTION_ENABLE) != 0U))
        {
          motor_enabled[index] = 0U;
          motor_action_pending[index] &= (uint8_t)~MOTOR_ACTION_ENABLE;
          motor_action_pending[index] |= MOTOR_ACTION_DISABLE;
        }
      }
    }

    if ((usb_command_received != 0U) &&
        ((HAL_GetTick() - last_usb_command_tick) > 100U))
    {
      usb_command_received = 0U;
      for (index = 0U; index < DM_MOTOR_COUNT; ++index)
      {
        if ((motor_enabled[index] != 0U) ||
            ((motor_action_pending[index] & MOTOR_ACTION_ENABLE) != 0U))
        {
          motor_enabled[index] = 0U;
          motor_action_pending[index] &= (uint8_t)~MOTOR_ACTION_ENABLE;
          motor_action_pending[index] |= MOTOR_ACTION_DISABLE;
        }
      }
    }

    Motor_ProcessPendingAction();

    if (feedback_pending != 0U)
    {
      feedback_pending = 0U;
      if (USB_MotorComm_SendFeedback() != 0U)
      {
        feedback_pending = 1U;
      }
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t index;

  if (htim->Instance != TIM6)
  {
    return;
  }

  index = motor_control_index;
  if ((usb_command_received != 0U) && (motor_enabled[index] != 0U))
  {
    (void)DM_Motor_MitControl(
      &dm_motors[index],
      motor_command.motor[index].position,
      motor_command.motor[index].velocity,
      motor_command.motor[index].kp,
      motor_command.motor[index].kd,
      motor_command.motor[index].torque);
  }

  ++index;
  if (index >= DM_MOTOR_COUNT)
  {
    index = 0U;
    feedback_pending = 1U;
  }
  motor_control_index = index;
}

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
