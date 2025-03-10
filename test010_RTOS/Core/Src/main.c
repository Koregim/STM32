/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId myTask01Handle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osSemaphoreId myBinarySem01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void myStartTask01(void const * argument);
void myStartTask02(void const * argument);
void myStartTask03(void const * argument);
void myStartTask04(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int mode = 0;
int step = 0;
int d;
int spinning = 0;
void microDelay(int us) {
	htim2.Instance->CNT = 0;
	while((htim2.Instance->CNT < us));
}
void Trigger()
{
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
	microDelay(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 1);
	microDelay(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
}
void LD2Test()
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
//void HAL_GPIO_EXTI_Callback(uint16_t btn) //External Interrupt ISR
//{
////	switch(btn)
////	{
////	case B1_Pin :
////		d += 45;
////		mode = mode ? 0 : 1;
////		break;
////	}
//	osSemaphoreRelease(myBinarySem01Handle);
//}
double dist;
int t1 = 0, t2 = 0;
int angle = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	if(GPIO_PIN == Echo_Pin)
	{
		if(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1)
		{
			//htim2.Instance->CNT = 0;
			t1 = htim2.Instance->CNT;
		}
		else
		{
			t2 = htim2.Instance->CNT;
			dist = (t2 - t1) * 0.17;
		}
	}
//	if(dist < 100)
//	{
//		angle = 90 * 2048 / 360;
//		spinning = 1;
//	}
//	else if(dist >= 100 && dist < 300)
//	{
//		angle = 45 * 2048 / 360;
//		spinning = 1;
//	}

}
void step_Wave(int step)
{
	switch(step)
	{
	case 0:
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
		break;
	case 1:
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
		break;
	case 2:
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
		break;
	case 3:
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
		break;

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of myTask01 */
  osThreadDef(myTask01, myStartTask01, osPriorityAboveNormal, 0, 512);
  myTask01Handle = osThreadCreate(osThread(myTask01), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, myStartTask02, osPriorityNormal, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, myStartTask03, osPriorityLow, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, myStartTask04, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  ProgramStart("RTOS Test"); // before the osKernelStart
  HAL_TIM_Base_Start(&htim2);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  LD2Test();
//	  HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4_Pin|IN1_Pin|IN3_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trig_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN1_Pin IN3_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN1_Pin|IN3_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int n = 0;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_myStartTask01 */
/**
  * @brief  Function implementing the myTask01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_myStartTask01 */
void myStartTask01(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
	  {
//		  printf("Distance1 : %.2f\r\n", dist);
//		  printf("Angle0 : %d\r\n", angle);
		  Trigger();
		  printf("Spinning : %d\r\n", spinning);
		  printf("Distance2 : %6.2f\r\n", dist);
		  printf("Angle1 : %d\r\n", angle);
		  osSemaphoreRelease(myBinarySem01Handle);
		  printf("2222222222222222222\r\n");

		  osDelay(60);
		  printf("123123123123\r\n");
	  }
	  else if(spinning) {
		  printf("3333333333333\r\n");
		  osDelay(1);
//		  osSemaphoreRelease(myBinarySem01Handle);
	  }
	  osSemaphoreRelease(myBinarySem01Handle);
	  osDelay(1);
//	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
//	  {
//		  printf("Task 01 (Normal) --------\r\n");
//		  osSemaphoreRelease(myBinarySem01Handle);
//		    osDelay(10);
//	  }

    //LD2Test();
    //Cursor(0,0);
    //printf("LD2 flashed %d times (Task 1)\r\n", n++);

		//HAL_Delay(500); // hardware delay

//    if(n == 1000)
//    {
//    	n = 0;
//    	osDelay(300);
//    }
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_myStartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
int n2 = 0;
int cnt = 0;
/* USER CODE END Header_myStartTask02 */
void myStartTask02(void const * argument)
{
  /* USER CODE BEGIN myStartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
	  {
		printf("Angle2 : %d \r\n", angle);
		printf("Count : %d \r\n", cnt);
		step_Wave(cnt % 4);
		if(cnt < angle)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
			angle = 0;
			osSemaphoreRelease(myBinarySem01Handle);
		}
		  osDelay(1);
	  }
	  printf("spinning2 : %d\r\n", spinning);
	//Cursor(0,10);
//	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
//	  {
//		  printf("Task 02 (Normal) -------\r\n");
//		  osSemaphoreRelease(myBinarySem01Handle);
//		    osDelay(10);
//	  }

//	if(~mode)
//		printf("button is pressed");
//	else
//		printf("                 ");
//	printf("Input Degree : ");
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, mode);
//	//scanf("%d", &d);
//	step = d * 4096 / 360; // d / 360 * 2048 ==> (integer d) d / 360 ==> d<360, (d/360) == 0
//	printf("Wave(Full) : %d steps, Half : %d steps", step / 2, step);
//
//	step_Wave(cnt % 4);
//	if(cnt < (step / 2))
//	{
//
//		cnt++;
//	}
//	else
//	{
//		cnt = 0;
//		d = 0;
//	}

    osDelay(1);
  }
  /* USER CODE END myStartTask02 */
}

/* USER CODE BEGIN Header_myStartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_myStartTask03 */
void myStartTask03(void const * argument)
{
  /* USER CODE BEGIN myStartTask03 */
  /* Infinite loop */
  for(;;)
  {
//	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
//	  {
//		  printf("Task 03 (Idle) ------\r\n");
//		  osSemaphoreRelease(myBinarySem01Handle);
//		    osDelay(10);
//	  }
//	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
//	  {
//		  LD2Test();
////		  osSemaphoreRelease(myBinarySem01Handle);
//		  osDelay(10);
//	  }
	  printf("tttttest \r\n");
	  LD2Test();
	  osDelay(500);
  }
  /* USER CODE END myStartTask03 */
}

/* USER CODE BEGIN Header_myStartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_myStartTask04 */
void myStartTask04(void const * argument)
{
  /* USER CODE BEGIN myStartTask04 */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(myBinarySem01Handle, 0) == osOK)
	  {
		  printf("Motor Complete\r\n");
		  osDelay(10);
		  osSemaphoreRelease(myBinarySem01Handle);
		  //osDelay(60);
	  }
//	  printf("Distance final : %.2f\r\n", dist);
//	  osSemaphoreRelease(myBinarySem01Handle);
	  osDelay(1);
  }
  /* USER CODE END myStartTask04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
