/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "usart.h"
#include <string.h>
#include "adc.h"
#include "pid.h"
#include "bldc.h"
#include "modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
extern float g_adc_u_value[ADC_CH_NUM]; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommutationTask */
osThreadId_t CommutationTaskHandle;
uint32_t CommutationTaskBuffer[ 128 ];
osStaticThreadDef_t CommutationTaskControlBlock;
const osThreadAttr_t CommutationTask_attributes = {
  .name = "CommutationTask",
  .cb_mem = &CommutationTaskControlBlock,
  .cb_size = sizeof(CommutationTaskControlBlock),
  .stack_mem = &CommutationTaskBuffer[0],
  .stack_size = sizeof(CommutationTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for BEMF_Task */
osThreadId_t BEMF_TaskHandle;
const osThreadAttr_t BEMF_Task_attributes = {
  .name = "BEMF_Task",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for SpeedTask */
osThreadId_t SpeedTaskHandle;
const osThreadAttr_t SpeedTask_attributes = {
  .name = "SpeedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StartupTask */
osThreadId_t StartupTaskHandle;
const osThreadAttr_t StartupTask_attributes = {
  .name = "StartupTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CommutationTaskFun(void *argument);
void BEMF_Task_Fun(void *argument);
void SpeedTaskFun(void *argument);
void StartupTaskFun(void *argument);
void MonitorTaskFun(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CommutationTask */
  CommutationTaskHandle = osThreadNew(CommutationTaskFun, NULL, &CommutationTask_attributes);

  /* creation of BEMF_Task */
  BEMF_TaskHandle = osThreadNew(BEMF_Task_Fun, NULL, &BEMF_Task_attributes);

  /* creation of SpeedTask */
  SpeedTaskHandle = osThreadNew(SpeedTaskFun, NULL, &SpeedTask_attributes);

  /* creation of StartupTask */
  StartupTaskHandle = osThreadNew(StartupTaskFun, NULL, &StartupTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(MonitorTaskFun, NULL, &MonitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CommutationTaskFun */
/**
* @brief Function implementing the CommutationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommutationTaskFun */
void CommutationTaskFun(void *argument)
{
  /* USER CODE BEGIN CommutationTaskFun */
  /* Infinite loop */
  for(;;)
  {
		//uart_print("CommutationTask\r\n");
    osDelay(1);
  }
  /* USER CODE END CommutationTaskFun */
}

/* USER CODE BEGIN Header_BEMF_Task_Fun */
/**
* @brief Function implementing the BEMF_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BEMF_Task_Fun */
void BEMF_Task_Fun(void *argument)
{
  /* USER CODE BEGIN BEMF_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
		//uart_print("BEMFTask\r\n");
    osDelay(1);
  }
  /* USER CODE END BEMF_Task_Fun */
}

/* USER CODE BEGIN Header_SpeedTaskFun */
/**
* @brief Function implementing the SpeedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SpeedTaskFun */
void SpeedTaskFun(void *argument)
{
  /* USER CODE BEGIN SpeedTaskFun */
	
  /* Infinite loop */
  for(;;)
  {
		//uart_print("SpeedTask\r\n");
		
    osDelay(1);
  }
  /* USER CODE END SpeedTaskFun */
}

/* USER CODE BEGIN Header_StartupTaskFun */
/**
* @brief Function implementing the StartupTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartupTaskFun */
void StartupTaskFun(void *argument)
{
  /* USER CODE BEGIN StartupTaskFun */
  /* Infinite loop */
  for(;;)
  {
		//uart_print("StartupTask\r\n");
    osDelay(500);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		osDelay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END StartupTaskFun */
}

/* USER CODE BEGIN Header_MonitorTaskFun */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MonitorTaskFun */
void MonitorTaskFun(void *argument)
{
  /* USER CODE BEGIN MonitorTaskFun */
	extern GetValue_t sensor_data;//adc data
	extern UART_Frame_t UART_Rx_Data;//uart data
	extern UART_Frame_t UART_Tx_Data;//uart data
	TickType_t lastTick = 0;
	g_speed_pid.SetPoint = -3000;
  /* Infinite loop */
  for(;;)
  {
//get data test code,test result is oK.
//				if(UART_Rx_Data.status)
//				{
//						UART_Rx_Data.status = 0;
//						HAL_UART_Transmit(
//								&huart1,
//								UART_Rx_Data.buf,
//								UART_Rx_Data.len,
//								100);
//					Uart_SendDMA(&huart1,
//                       UART_Rx_Data.buf,
//                       UART_Rx_Data.len);
//				}
				
				if(UART_Rx_Data.status)
				{
						UART_Rx_Data.status = 0;

						Modbus_Parse(
								UART_Rx_Data.buf,
								UART_Rx_Data.len);
				}
				
        if ((xTaskGetTickCount() - lastTick) > 1000)
        {
            lastTick = xTaskGetTickCount();
        }
				
    
    osDelay(1);
		
  }
  /* USER CODE END MonitorTaskFun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

