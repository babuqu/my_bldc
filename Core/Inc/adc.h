/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
//ADC use
#define ADC_CH_NUM  12     
#define ADC_COLL    36    
#define ADC_SUM     (ADC_CH_NUM * ADC_COLL) 

#define ADC2CURT    (float)(3.3f / 4096)
#define CURTREF     (float)(1.25f)
#define CURTGAIN    (float)(6.0f)
	

#define ADC2VBUS    (float)(3.3f * 25 / 4096)
#define ADC2EMF     (float)(3.3f * 37.0f/ 4096)
#define ADC212V     (float)(3.3f * 12.0f/ 4096)
#define ADC25V      (float)(3.3f * 12.0f/ 4096)	

// �¶ȴ�����У׼ֵ���洢��ϵͳ�洢����
#define V25        (float)0.76f      // 25��Cʱ�ĵ�ѹ (V)
#define AVG_SLOPE  (float)0.0025f    // ƽ��б�� 2.5mV/��C
#define VREF       (float)3.3f       // ADC�ο���ѹ (V)
#define ADC_RES    (float)4095.0f     // 12λADC�ֱ���

// 假设：Vref = 3.3V, ADC为12位 (分辨率4095)，R_ref = 10.0kΩ
#define R_REF 4700.0f
#define R0 10000.0f    // 25°C时的电阻
#define T0 298.15f     // 25°C对应的开尔文温度
#define B 3950.0f      // B常数
typedef struct {
  float i_w ;
	float i_u ;
	float i_v ;
	float emf_w ;
	float emf_u ;
	float emf_v ;
	float emf_m ;
	float v12 ;
	float v5 ;
	float Vbus_v ;
	float temperature ;
	float ntc;
}GetValue_t;
	
extern uint16_t g_adc_value[ADC_SUM];
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
void calc_adc_val(uint16_t *adc_buf);
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

