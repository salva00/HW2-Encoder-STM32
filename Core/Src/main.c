/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define mainENCODER_PRIORITY     ( tskIDLE_PRIORITY + 2 )
#define mainTASK1_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainTASK2_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainSCOPE_PRIORITY		 ( tskIDLE_PRIORITY + 1 )
#define mainDIAG_PRIORITY        ( tskIDLE_PRIORITY + 1 )



#define BASE_PERIOD_MS 5
#define mainENCODER_TICK_FREQUENCY			pdMS_TO_TICKS( BASE_PERIOD_MS )
#define mainTASK1_TICK_FREQUENCY			pdMS_TO_TICKS( BASE_PERIOD_MS/2 )
#define mainTASK2_TICK_FREQUENCY			pdMS_TO_TICKS( BASE_PERIOD_MS/2 )
#define mainSCOPE_TICK_FREQUENCY			pdMS_TO_TICKS( BASE_PERIOD_MS*2 )
#define mainDIAG_TICK_FREQUENCY             pdMS_TO_TICKS( BASE_PERIOD_MS*2 )


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart2;

/* Definitions for encoder */
osThreadId_t encoderHandle;
osStaticThreadDef_t encoderControlBlock;
const osThreadAttr_t encoder_attributes = {
  .name = "encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) (osPriorityIdle + 2)
};
/* Definitions for rt1 */
osThreadId_t rt1Handle;
osStaticThreadDef_t rt1ControlBlock;
const osThreadAttr_t rt1_attributes = {
  .name = "rt1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) (osPriorityIdle + 3)

};
/* Definitions for rt2 */
osThreadId_t rt2Handle;
osStaticThreadDef_t rt2ControlBlock;
const osThreadAttr_t rt2_attributes = {
  .name = "rt2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) (osPriorityIdle + 3)

};
/* Definitions for scope */
osThreadId_t scopeHandle;
osStaticThreadDef_t scopeControlBlock;
const osThreadAttr_t scope_attributes = {
  .name = "scope",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) (osPriorityIdle + 1)

};
/* Definitions for diag */
osThreadId_t diagHandle;
osStaticThreadDef_t diagControlBlock;
const osThreadAttr_t diag_attributes = {
  .name = "diag",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) (osPriorityIdle + 1)

};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void startEncoder(void *argument);
void StartRt1(void *argument);
void StartRt2(void *argument);
void StartScope(void *argument);
void StartDiag(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct enc_str
{
	unsigned int slit;		//valori oscillanti tra 0 e 1
	unsigned int home_slit;	//1 se in home, 0 altrimenti
	SemaphoreHandle_t lock;
};
static struct enc_str enc_data;

struct _rising_edge{
	unsigned int count;
	SemaphoreHandle_t lock;
};
static struct _rising_edge rising_edge;

struct _round_time{
	unsigned long int time_diff;
	SemaphoreHandle_t lock;
};
static struct _round_time round_time;

struct _slack_rt{
	unsigned long int slack_time;
	SemaphoreHandle_t lock;
};

static struct _slack_rt slack_rt1;
static struct _slack_rt slack_rt2;

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  enc_data.lock = xSemaphoreCreateMutex();
  rising_edge.lock = xSemaphoreCreateMutex();
  round_time.lock = xSemaphoreCreateMutex();

  /*Creazione dei mutex per il calcolo dello slack time*/
  slack_rt1.lock = xSemaphoreCreateMutex();
  slack_rt2.lock = xSemaphoreCreateMutex();

  /* USER CODE END RTOS_MUTEX */

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

  if (enc_data.lock != NULL &&  rising_edge.lock != NULL && round_time.lock != NULL)
  {
	  /* creation of encoder */
	  encoderHandle = osThreadNew(startEncoder, NULL, &encoder_attributes);

	  /* creation of rt1 */
	  rt1Handle = osThreadNew(StartRt1, NULL, &rt1_attributes);

	  /* creation of rt2 */
	  rt2Handle = osThreadNew(StartRt2, NULL, &rt2_attributes);

	  /* creation of scope */
	  scopeHandle = osThreadNew(StartScope, NULL, &scope_attributes);

	  /* creation of diag */
	  diagHandle = osThreadNew(StartDiag, NULL, &diag_attributes);
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startEncoder */
/**
  * @brief  Function implementing the encoder thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startEncoder */
void startEncoder(void *argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = mainENCODER_TICK_FREQUENCY;
    xNextWakeTime = xTaskGetTickCount();

    ( void ) argument;

    xSemaphoreTake(enc_data.lock, portMAX_DELAY);
    enc_data.slit = 0;
    enc_data.home_slit = 0;
    xSemaphoreGive(enc_data.lock);

    unsigned int count = 0;
    unsigned int slit_count = 0;
    unsigned int prev_slit = 0;

    /* Randomized period (75-750 RPM) */
	srand(time(NULL));

	unsigned int semi_per = (rand() % 10) + 1;

  /* Infinite loop */
  for(;;)
  {
      vTaskDelayUntil( &xNextWakeTime, xBlockTime );

      xSemaphoreTake(enc_data.lock, portMAX_DELAY);
      prev_slit = enc_data.slit;
		if (count%semi_per == 0) {
			enc_data.slit++;
			enc_data.slit%=2;
		}

		if (prev_slit==0&&enc_data.slit==1) 					//fronte di salita
			slit_count=(++slit_count)%8;

		if (slit_count==0) enc_data.home_slit=enc_data.slit;
		else enc_data.home_slit=0;

		//sprintf(MSG,"%d:\t\t %d %d\n",count,enc_data.slit,enc_data.home_slit);	//DEBUG encoder
		//HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		count++;
      xSemaphoreGive(enc_data.lock);

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRt1 */
/**
* @brief Function implementing the rt1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRt1 */
void StartRt1(void *argument)
{
  /* USER CODE BEGIN StartRt1 */
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = mainTASK1_TICK_FREQUENCY;

	    /* Prevent the compiler warning about the unused parameter. */
	xNextWakeTime = xTaskGetTickCount();
	( void ) argument;
	    /*per lo slack time*/
	TickType_t xFinishTime;

	xSemaphoreTake(rising_edge.lock, portMAX_DELAY);
	rising_edge.count = 0;
	xSemaphoreGive(rising_edge.lock);

	int last_value = 0;
	uint8_t MSG[100] = {'\0'};


	unsigned long int slack;

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil( &xNextWakeTime, xBlockTime );

      xSemaphoreTake(enc_data.lock, portMAX_DELAY);
      if( last_value == 0 && enc_data.slit == 1){
    	  last_value = 1;

    	  xSemaphoreTake(rising_edge.lock, portMAX_DELAY);
    	  rising_edge.count++;
    	  xSemaphoreGive(rising_edge.lock);

      }
      else if(last_value == 1 && enc_data.slit == 0){
    	  last_value = 0;
      }

      xSemaphoreGive(enc_data.lock);


      xFinishTime = xTaskGetTickCount();
      slack = (xBlockTime + xNextWakeTime - xFinishTime) * portTICK_PERIOD_MS  ;
      if(slack>=0){
          xSemaphoreTake(slack_rt1.lock, portMAX_DELAY);
          slack_rt1.slack_time = slack;
          xSemaphoreGive(slack_rt1.lock);
      }
      else{
          sprintf(MSG,"Task rt1: deadline miss. deadline: %lu, finish time: %lu\n",xNextWakeTime + xBlockTime ,xFinishTime);
          HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
      }

    osDelay(1);
  }
  /* USER CODE END StartRt1 */
}

/* USER CODE BEGIN Header_StartRt2 */
/**
* @brief Function implementing the rt2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRt2 */
void StartRt2(void *argument)
{
  /* USER CODE BEGIN StartRt2 */
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = mainTASK2_TICK_FREQUENCY;
	xNextWakeTime = xTaskGetTickCount();
	( void ) argument;

	TickType_t time_home;
	TickType_t last_time_home;

	int first_measure = 1;
	int last_home_slit = 0;

	TickType_t xFinishTime;
	unsigned long int slack;
	uint8_t MSG[100] = {'\0'};


  /* Infinite loop */
	for(;;)
	{
		 vTaskDelayUntil( &xNextWakeTime, xBlockTime );
		 xSemaphoreTake(enc_data.lock, portMAX_DELAY);
		 if(enc_data.home_slit == 1 && last_home_slit == 0){
			 last_home_slit = 1;
			 if(first_measure){
				 last_time_home = xTaskGetTickCount();
				 first_measure = 0;
			 }
			 else{
				 time_home = xTaskGetTickCount();
				 xSemaphoreTake(round_time.lock, portMAX_DELAY);
				 round_time.time_diff = portTICK_PERIOD_MS*(time_home - last_time_home);
				 xSemaphoreGive(round_time.lock);
				 last_time_home = time_home;
			 }
		 }
		 else if(enc_data.home_slit == 0){
			 last_home_slit = 0;
		 }
		 xSemaphoreGive(enc_data.lock);

		 xFinishTime = xTaskGetTickCount();
		 slack = (xBlockTime + xNextWakeTime - xFinishTime)*portTICK_PERIOD_MS;
		 if(slack >= 0){
			 xSemaphoreTake(slack_rt2.lock, portMAX_DELAY);
			 slack_rt2.slack_time = slack;
			 xSemaphoreGive(slack_rt2.lock);
		 }
		 else{
			printf("Task rt2: deadline: %lu, finish time: %lu\n",xNextWakeTime + xBlockTime ,xFinishTime);
			sprintf(MSG,"Task rt2: deadline: %lu, finish time: %lu\n",xNextWakeTime + xBlockTime ,xFinishTime);
			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	     }

    osDelay(1);
  }
  /* USER CODE END StartRt2 */
}

/* USER CODE BEGIN Header_StartScope */
/**
* @brief Function implementing the scope thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartScope */
void StartScope(void *argument)
{
  /* USER CODE BEGIN StartScope */
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = mainTASK2_TICK_FREQUENCY;
    /* Prevent the compiler warning about the unused parameter. */
	xNextWakeTime = xTaskGetTickCount();
	( void ) argument;

	unsigned int count=0;
	float diff_us = 0;
	unsigned int rpm = 0;
	uint8_t MSG[100] = {'\0'};

  /* Infinite loop */
	for(;;)
	{
		vTaskDelayUntil( &xNextWakeTime, xBlockTime );

		xSemaphoreTake(rising_edge.lock, portMAX_DELAY);
		count = rising_edge.count;
		xSemaphoreGive(rising_edge.lock);

		printf("Rising Edge Counter : %d\t",count);

		xSemaphoreTake(round_time.lock, portMAX_DELAY);
		diff_us = round_time.time_diff * 1000;        //difference in microseconds
		xSemaphoreGive(round_time.lock);

		rpm = (unsigned int)((float)60*1000000/diff_us);

		//printf("diff : %f\t",diff_us);				//DEBUG

		printf( "RPM : %u\n",rpm);
		sprintf(MSG, "RPM : %u\n",rpm);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		osDelay(1);
	}
  /* USER CODE END StartScope */
}

/* USER CODE BEGIN Header_StartDiag */
/**
* @brief Function implementing the diag thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDiag */
void StartDiag(void *argument)
{
  /* USER CODE BEGIN StartDiag */
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = mainDIAG_TICK_FREQUENCY;

	/* Prevent the compiler warning about the unused parameter. */
	( void ) argument;
	xNextWakeTime = xTaskGetTickCount();

	unsigned long int avg_slack=0;
	int i = 0;
	int rounds = 100;
	uint8_t MSG[100] = {'\0'};
	/* Infinite loop */
	for(;;)
	{
		vTaskDelayUntil( &xNextWakeTime, xBlockTime );

    	xSemaphoreTake(slack_rt1.lock, portMAX_DELAY);
    	xSemaphoreTake(slack_rt2.lock, portMAX_DELAY);

    	avg_slack += (slack_rt1.slack_time + slack_rt2.slack_time)/2 * 1000; 	//average in microseconds

    	xSemaphoreGive(slack_rt1.lock);
    	xSemaphoreGive(slack_rt2.lock);

    	i++;
    	if(i == rounds){
    		avg_slack = avg_slack/rounds;
    		sprintf(MSG, "**********SLACK TIME: %ld us**********\n",avg_slack);
    		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
    		i = 0;
    	}
    	osDelay(1);
	}
  /* USER CODE END StartDiag */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
