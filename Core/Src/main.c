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
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include "pd.h"
#include "stdio.h"

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
// float pos_des = 0.0;
// static int sign = 1;
// static int spd_des = 0;
static int            pos_des = 1;  // [rad]
static MotorTxCmd*    motor_cmd_ptr;
static int            flag = 0;
static uint8_t*       motor_id_ptr;
static MotorRxMsg*    motor_msg_ptr;
static MotorFeedback* motor_fb_ptr;
static PDStruct*      motor_fp_pd_ptr; // motor force-position PD Pointer
static uint16_t       can_reid = 0;
static MotorRxMsg     buf;
static uint32_t       rx_cnt[3] = {0, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  can_bsp_init();
  motor_id_ptr = motor_get_id_ptr();
  motor_msg_ptr = motor_msgs_init();
  motor_cmd_ptr = motor_cmds_init();
  motor_fb_ptr = motor_feedback_init();
  motor_fp_pd_ptr = pds_init();
  HAL_Delay(1999);
  motor_enable();
  HAL_Delay(999);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(999);
    if(pos_des == 1)
    {
      pos_des = -1;
    }
    else if(pos_des == -1)
    {
      pos_des = 1;
    }
    // printf("feedback angle: %.2f \r\n", motor_fb_ptr[2].angle);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  if(flag == 0)
  {
      // double current0 = pd_calc_output(&motor_fp_pd_ptr[0]);
      motor_control_position(&motor_cmd_ptr[0], motor_id_ptr[0], (int32_t)(pos_des*180/M_PI*100*MOTOR_REDUCTION_RATIO), 1.0, 0, 0.2, 333, 15000, 0);
      // motor_control_current(&motor_cmd_ptr[0], motor_id_ptr[0], current0, 20000);
      flag = 1;
  }
  else if(flag == 1)
  {
      // double current1 = pd_calc_output(&motor_fp_pd_ptr[1]);
      motor_control_position(&motor_cmd_ptr[1], motor_id_ptr[1], (int32_t)(pos_des*180/M_PI*100*MOTOR_REDUCTION_RATIO), 1.0, 0, 0.2, 333, 15000, 0);
      // motor_control_current(&motor_cmd_ptr[1], motor_id_ptr[1], current1, 20000);
      flag = 2;
  }
  else if(flag == 2)
  {
      // double current2 = pd_calc_output(&motor_fp_pd_ptr[2]);
      motor_control_position(&motor_cmd_ptr[2], motor_id_ptr[2], (int32_t)(pos_des*180/M_PI*100*MOTOR_REDUCTION_RATIO), 1.0, 0, 0.2, 333, 15000, 0);
      // motor_control_current(&motor_cmd_ptr[2], motor_id_ptr[2], current2, 20000);
      flag = 0;
  }
}
/**
 * @brief   500Hz TIM7 callback function.
 * @details update motor pd parameters
 * 
 */
static void update_motor_pd_params()
{
  for(int i = 0; i < MOTOR_NUM; i++)
  {
    motor_fp_pd_ptr[i].kp = FP_KP;
    motor_fp_pd_ptr[i].kd = FP_KD;
    motor_fp_pd_ptr[i].pos_target = pos_des;
  }
  printf("rx_cnt: %d,%d,%d\r\n", rx_cnt[0], rx_cnt[1], rx_cnt[2]);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 300us
{
  if(htim->Instance == TIM6)
  {
    send_cmd();
  }
  else if(htim->Instance == TIM7)
  {
    update_motor_pd_params();
  }
}
/**
 * @brief   callback function for FDCAN1 RX FIFO0
 * @details receive motor feedback, update pd error
 * 
 */
void fdcan1_rx_callback(void)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &pRxHeader, (uint8_t*)&buf)==HAL_OK)
	{
	}
	if(pRxHeader.Identifier == 0x789) // ID of response message is 0x789
	{
		can_reid = buf.label[1]; // get the motor id
    rx_cnt[can_reid-1]++; // update the receive count of the motor
		// memcpy(&motor_msg_ptr[can_reid-1], &buf, sizeof(MotorRxMsg)); // copy the received message to the corresponding motor message pointer
    // motor_update_feedback(motor_msg_ptr, motor_fb_ptr);
    // motor_pd_update_real_value(motor_fp_pd_ptr, motor_fb_ptr);
    
	}
}
/**
 * @brief FDCAN1 RX FIFO0 Callback function.
 * 
 * @param hfdcan 
 * @param RxFifo0ITs 
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1)
    {
		  fdcan1_rx_callback();
    }
}
// retarget printf to USB
int _write(int fd, char *ptr, int len)
{
    CDC_Transmit_HS((uint8_t*)ptr, len);
    return len;
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
