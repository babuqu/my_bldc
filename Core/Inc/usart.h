/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
typedef uint8_t (*uart_rx_callback_t)(UART_HandleTypeDef *huart, uint8_t byte, void *user_ctx);
typedef struct
{
    uint8_t buf[256];
    uint16_t len;
	  uint8_t status;
}UART_Frame_t;// data store for uart
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define UART_TX_BUF_LEN 128     
#define UART_BUF_LEN 64
#define UART1_RX_DMA_BUF_LEN 64
HAL_StatusTypeDef Uart_SendDMA(UART_HandleTypeDef *huart,
                               uint8_t *buf,
                               uint16_t len);
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void Uart_RegisterRxCallback(UART_HandleTypeDef *huart, uart_rx_callback_t callback, void *user_ctx);
HAL_StatusTypeDef Uart_StartRxToIdleDMA(UART_HandleTypeDef *huart);
void uart_print(const char *str);
void UartPrintf(const char *fmt, ...);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

