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
  .priority = (osPriority_t) osPriorityRealtime7,
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
    osDelay(1000);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		osDelay(1000);
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
  //"Monitor":��ء�����ϵͳ״̬����Դʹ�����
  uint8_t ch;
  static uint8_t uartBuf[UART_BUF_LEN];  
  static uint8_t uartIndex = 0;          
	static uint8_t lastCh = 0;        
  extern QueueHandle_t uartRxQueue;
	extern QueueHandle_t uartTxQueue;
	extern GetValue_t sensor_data;
	static uint8_t txBuf[UART_TX_BUF_LEN];
	TickType_t lastTick = 0;
	
  /* Infinite loop */
  for(;;)
  {
			
       // ÿ�� 1 ���ӡһ��ջʣ��ռ�
        if ((xTaskGetTickCount() - lastTick) > 1000)
        {
            lastTick = xTaskGetTickCount();

            // ��ӡÿ������ʣ��ջ�ռ�
            UartPrintf("MonitorTask free stack: %u\r\n",
                       uxTaskGetStackHighWaterMark(MonitorTaskHandle));
//					  UartPrintf("CommutationTaskHandle free stack: %u\r\n",
//                       uxTaskGetStackHighWaterMark(CommutationTaskHandle));
					  //adc_temp = (g_adc_u_value[8]-0.76f)/0.0025f+25;  // �ȸ�ֵ���ֲ�����
					  UartPrintf("ntc  :  %.2f\t", sensor_data.ntc);
            UartPrintf("temperature : %.2f\r\n", sensor_data.temperature);
					
					  UartPrintf("emf_m : %.2f\t", sensor_data.emf_m);
					  UartPrintf("emf_u : %.2f\t", sensor_data.emf_u);
					  UartPrintf("emf_v : %.2f\t", sensor_data.emf_v);
					  UartPrintf("emf_w : %.2f\r\n", sensor_data.emf_w);
					
					  UartPrintf("i_u : %.2f\t", sensor_data.i_u);
					  UartPrintf("i_v : %.2f\t", sensor_data.i_v);
					  UartPrintf("i_w : %.2f\r\n", sensor_data.i_w);
					
					  UartPrintf("v12 : %.2f\t", sensor_data.v12);
					  UartPrintf("v5  :  %.2f\t", sensor_data.v5);
					  UartPrintf("Vbus_v : %.2f\r\n", sensor_data.Vbus_v);		
        }
				
        //��ѯ���ڽ��ն���
        if (xQueueReceive(uartRxQueue, &ch, 0) == pdPASS)
        {
            if (uartIndex < UART_BUF_LEN - 1) 
            {
                uartBuf[uartIndex++] = ch;
            }
						else
            {
                uartIndex = 0;
            }
            if (lastCh == '\r' && ch == '\n') 
            {
                uartBuf[uartIndex] = 0; 
                uartIndex = 0;           
							  UartPrintf("%s", uartBuf);//����ʵ������printf����
							  //�ӽ�������
							
            }
						lastCh = ch; 
        }
				
				//��ѯ���ڷ��Ͷ���
				if (xQueueReceive(uartTxQueue, txBuf, 0) == pdPASS)
        {
            uint16_t txLen = 0U;
            while ((txLen < UART_TX_BUF_LEN) && (txBuf[txLen] != '\0'))
            {
              txLen++;
            }
            if (txLen > 0U)
            {
              HAL_UART_Transmit(&huart1, txBuf, txLen, 100);
            }
        }
    osDelay(1);
		//uart_print("MonitorTask\r\n");
  }
  /* USER CODE END MonitorTaskFun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

