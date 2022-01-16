/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "pid.h"

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
ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static SemaphoreHandle_t mutex_dbg;
static SemaphoreHandle_t mutex_in1;
static SemaphoreHandle_t mutex_in2;

// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata1;
struct pid_controller ctrldata2;
pid_t pid_controller1;
pid_t pid_controller2;

// Control loop input,output and setpoint variables
float input1 = 0, output1 = 0;
float setpoint1 = 15;
// Control loop gains
float kp1 = 2.5, ki1 = 1.0, kd1 = 1.0;

// Control loop input,output and setpoint variables
float input2 = 0, output2 = 0;
float setpoint2 = 15;
// Control loop gains
float kp2 = 2.5, ki2 = 1.0, kd2 = 1.0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pidTask( void *pvParameters )  ////////////////////////////////////////////////////////// https://github.com/geekfactory/PID
/////////////////////////////////////// https://deepbluembedded.com/stm32-pwm-example-timer-pwm-mode-tutorial/
{
  pid_t pid_controller = (pid_t) pvParameters;
  char dbg_msg[50];

  // Set controler output limits from 0 to 200
	pid_limits(pid_controller, 0, 200);
	// Allow PID to compute and change output
	pid_auto(pid_controller);

  sprintf(dbg_msg, "Running PID Controller with ID");
  xSemaphoreTake(mutex_dbg, portMAX_DELAY);
  HAL_UART_Transmit(&huart1, dbg_msg, sizeof(char)*strlen(dbg_msg), HAL_MAX_DELAY);
  xSemaphoreGive(mutex_dbg);

  while(1)
  {
    // Read process feedback
    // input = process_input();
    // Compute new PID output value
    pid_compute(pid_controller);
    //Change actuator value
    // process_output(output);
  }
}


void vTask1( void *pvParameters )
{
  const char *pcTaskName = "Task 1";
  char buf[50];
  volatile uint32_t ul=0; /* volatile to ensure ul is not optimized away. */
  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    sprintf(buf,"%s: %d \r\n",pcTaskName,ul);
    // Take the mutex
    xSemaphoreTake(mutex_dbg, portMAX_DELAY);
    /* Print out the name of this task. */
    HAL_UART_Transmit(&huart1, buf, sizeof(char)*strlen(buf), HAL_MAX_DELAY);
    // HAL_UART_Transmit_DMA(&huart1, pcTaskName, sizeof(pcTaskName)); //Non-blocking mode with DMA
    // Release the mutex
    xSemaphoreGive(mutex_dbg);
    vTaskDelay(1000);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    ul++;
    /* Delay for a period. */
    // for( ul = 0; ul < 1000; ul++ )
    // {
    //   /* This loop is just a very crude delay implementation. There is
    //   nothing to do in here. Later examples will replace this crude
    //   loop with a proper delay/sleep function. */
    // }
  }
}

void vTask2( void *pvParameters )
{
  const char *pcTaskName = "Task 2";
  char buf[50];
  volatile uint32_t ul=0; /* volatile to ensure ul is not optimized away. */
  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    sprintf(buf,"%s: %d \r\n",pcTaskName,ul);
    // Take the mutex
    xSemaphoreTake(mutex_dbg, portMAX_DELAY);
    /* Print out the name of this task. */
    HAL_UART_Transmit(&huart1, buf, sizeof(char)*strlen(buf), HAL_MAX_DELAY);
    // HAL_UART_Transmit_DMA(&huart1, pcTaskName, sizeof(pcTaskName)); //Non-blocking mode with DMA
    // Release the mutex
    xSemaphoreGive(mutex_dbg);
    vTaskDelay(2000);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    ul++;
    /* Delay for a period. */
    // for( ul = 0; ul < 500; ul++ )
    // {
    //   /* This loop is just a very crude delay implementation. There is
    //   nothing to do in here. Later examples will replace this crude
    //   loop with a proper delay/sleep function. */
    // }
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
  MX_ETH_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  /* TODO: fix Ethernet initialization */

  // Create mutex before starting tasks
  mutex_dbg = xSemaphoreCreateMutex();
  mutex_in1 = xSemaphoreCreateMutex();
  mutex_in2 = xSemaphoreCreateMutex();
  // Give the mutex
  xSemaphoreGive(mutex_dbg);
  xSemaphoreGive(mutex_in1);
  xSemaphoreGive(mutex_in2);

  char data[50] = {"Creating tasks...\r\n"};
  // uint8_t datalen = strlen(data)+48; //suma "0" en ASCII para que se imprima correctamente
  HAL_UART_Transmit(&huart1, &data, sizeof(char)*strlen(data), HAL_MAX_DELAY);

  // Create both controllers, one for each task and wheel
  pid_controller1 = pid_create(&ctrldata1, mutex_in1, &input1, &output1, &setpoint1, kp1, ki1, kd1);
  pid_controller2 = pid_create(&ctrldata2, mutex_in2, &input2, &output2, &setpoint2, kp2, ki2, kd2);

  /* Create one of the two tasks. Note that a real application should check
  the return value of the xTaskCreate() call to ensure the task was created
  successfully. */
  xTaskCreate( vTask1, /* Pointer to the function that implements the task. */
              "Task 1",/* Text name for the task. This is to facilitate
              debugging only. */
              1000, /* Stack depth - small microcontrollers will use much
              less stack than this. */
              NULL, /* This example does not use the task parameter. */
              1, /* This task will run at priority 1. */
              NULL ); /* This example does not use the task handle. */
  /* Create the other task in exactly the same way and at the same priority. */
  xTaskCreate( vTask2, "Task 2", 1000, NULL, 1, NULL );

  strcpy(data, "Starting Scheduler... \r\n");
  HAL_UART_Transmit(&huart1, &data, sizeof(char)*strlen(data), HAL_MAX_DELAY);
  /* Start the scheduler so the tasks start executing. */
  vTaskStartScheduler();

  /* If all is well then main() will never reach here as the scheduler will
  now be running the tasks. If main() does reach here then it is likely that
  there was insufficient heap memory available for the idle task to be created.
  Chapter 2 provides more information on heap memory management. */
  for( ;; );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
