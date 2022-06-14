/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
/* USER CODE BEGIN Variables */

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


/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



static void encoderTask( void * pvParameters );
static void rt1Task( void * pvParameters );
static void rt2Task( void * pvParameters );
static void scopeTask( void * pvParameters );
static void diagnosticTask( void* pvParameters );


/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void encoderTask(void * pvParameters){
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = mainENCODER_TICK_FREQUENCY;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    xNextWakeTime = xTaskGetTickCount();
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

    for (;;)
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

		//printf("%d:\t\t %d %d\n",count,enc_data.slit,enc_data.home_slit);	//DEBUG encoder
		count++;
        xSemaphoreGive(enc_data.lock);

    }
}

/*-----------------------------------------------------------*/

static void rt1Task(void * pvParameters){
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = mainTASK1_TICK_FREQUENCY;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    xNextWakeTime = xTaskGetTickCount();

    /*per lo slack time*/
    TickType_t xFinishTime;

    xSemaphoreTake(rising_edge.lock, portMAX_DELAY);
    rising_edge.count = 0;
    xSemaphoreGive(rising_edge.lock);

    int last_value = 0;

    unsigned long int slack;

    for (;;)
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
            printf("Task rt1: deadline miss. deadline: %lu, finish time: %lu\n",xNextWakeTime + xBlockTime ,xFinishTime);
        }

    }
}
/*-----------------------------------------------------------*/

static void rt2Task(void * pvParameters){
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = mainTASK2_TICK_FREQUENCY;
    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    xNextWakeTime = xTaskGetTickCount();


    TickType_t time_home;
	TickType_t last_time_home;

	int first_measure = 1;
	int last_home_slit = 0;

    TickType_t xFinishTime;
    unsigned long int slack;

    for (;;)
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
        }

    }
}
/*-----------------------------------------------------------*/

static void scopeTask(void * pvParameters){
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = mainTASK2_TICK_FREQUENCY;
    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    xNextWakeTime = xTaskGetTickCount();
    unsigned int count=0;
	float diff_us = 0;
	unsigned int rpm = 0;

    for (;;)
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
        printf("RPM : %u\n",rpm);

    }

}

/*-----------------------------------------------------------*/
static void diagnosticTask( void* pvParameters ){

    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = mainDIAG_TICK_FREQUENCY;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    xNextWakeTime = xTaskGetTickCount();

    unsigned long int avg_slack=0;
	int i = 0;
	int rounds = 100;

    for( ; ;){

        vTaskDelayUntil( &xNextWakeTime, xBlockTime );

        xSemaphoreTake(slack_rt1.lock, portMAX_DELAY);
        xSemaphoreTake(slack_rt2.lock, portMAX_DELAY);

        avg_slack += (slack_rt1.slack_time + slack_rt2.slack_time)/2 * 1000; 	//average in microseconds

        xSemaphoreGive(slack_rt1.lock);
        xSemaphoreGive(slack_rt2.lock);

        i++;
		if(i == rounds){
			avg_slack = avg_slack/rounds;
			printf("**********SLACK TIME: %ld us**********\n",avg_slack);
			i = 0;
		}
    }

}



/* USER CODE END Application */

