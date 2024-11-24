/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "param.h"
#include "sbus.h"
#include "can_bsp.h"
#include "motor.h"
#include "force_position.h"

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
double         pos_des[MOTOR_NUM_PER_LEG*2] = {-0.33522, 1.34904, -2.00320,
                                                      -0.33822, -1.29971, 1.98062};  // [rad]
const static uint8_t* motor_id_ptr;
static int32_t*        current_ptr;
static uint8_t*       uart5_rx_buf_ptr;
static double*        motor_angle_increment_ptr;
static uint8_t*       selected_motor_id_ptr;
const static double*   motor_logical_initial_pos_ptr;
static uint8_t        flag = 1; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// get static variable pointers
static void init_motors()
{
  motor_init();
  motor_id_ptr = motor_get_id_ptr();
  motor_logical_initial_pos_ptr = motor_get_logical_initial_pos();
  can_bsp_init();
}

static void init_rc()
{
  sbus_init();
  uart5_rx_buf_ptr = sbus_get_uart5_rx_buf();
  motor_angle_increment_ptr = sbus_get_motor_angle_increment();
  selected_motor_id_ptr = sbus_get_selected_motor_id();
}

static void init_force_position_control()
{
  fp_init();
  current_ptr = fp_get_current_ptr();
}

static void init_system()
{
  init_rc();
  init_motors();
}

static void start_sbus_reception()
{
//  HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf_ptr, UART5_RX_BUFFER_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_buf_ptr, UART5_RX_BUFFER_SIZE*2); // use the size of 2 times, because DMA will trigger interrupt when RxHalfCplt
}

static void start_pd_control()
{
  HAL_TIM_Base_Start_IT(&htim6);
}
static void start_send_motor_cmd()
{
  HAL_TIM_Base_Start_IT(&htim7);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_FDCAN2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  init_system();
  start_sbus_reception();
  HAL_Delay(999);
	motor_enable();
  HAL_Delay(999);
  // start_pd_control();
	start_send_motor_cmd();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(9); // 10ms delay for pos_des update
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    pos_des[*selected_motor_id_ptr-1] = motor_logical_initial_pos_ptr[*selected_motor_id_ptr-1] + motor_angle_increment_ptr[0];
//    pos_des[*selected_motor_id_ptr-1+MOTOR_NUM_PER_LEG] = motor_logical_initial_pos_ptr[*selected_motor_id_ptr-1+MOTOR_NUM_PER_LEG] + motor_angle_increment_ptr[1];
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief   send command to motor, callback for TIM6
 * @details the frequency of this function is 300us, which means each motor will receive its cmd every 900us.
 * @return  none
 */
static void send_cmd()
{
  // fp_calc_output(flag);
  // convert mA to A
  // motor_control_current(motor_id_ptr[flag-1], (int32_t)(current_ptr[flag-1] / TORQUE_CONSTANT * 1000.0), (int32_t)(current_ptr[flag-1+MOTOR_NUM_PER_LEG] / TORQUE_CONSTANT *1000.0));
 motor_control_current(motor_id_ptr[flag-1], 0, 0);
	switch (flag) // change id of motor to be sent cmd
  {
    case 1:
      flag = 2;
      break;
    case 2:
      flag = 3;
      break;
    case 3:
      flag = 1;
      break;
    default:
      flag = 1;
      break;
  }
}
/**
 * @brief   500Hz TIM6 callback function.
 * @details update motor pd parameters
 * 
 */
static void update_motor_pd()
{
  for(int i = 0; i < MOTOR_NUM_PER_LEG; i++)
  {
    fp_set_target(pos_des);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 300us
{
  if(htim->Instance == TIM7) // 300us period, nearly 1ms for each motor
  {
    send_cmd();
  }
  else if(htim->Instance == TIM6) // 500Hz period, update motor pd parameters
  {
    update_motor_pd();
  }
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

#ifdef  USE_FULL_ASSERT
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
