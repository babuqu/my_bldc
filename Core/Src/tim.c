/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "adc.h"
#include "bldc.h"
#include "pid.h"
#include "zero_ctr.h"
extern _bldc_obj g_bldc_motor1;
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8400-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	  HAL_TIM_Base_Start_IT(&htim1);  // 启用更新中断
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 200-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}
/* TIM6 init function */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8400-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PB12     ------> TIM1_BKIN
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    PA2     ------> TIM5_CH3
    */
    GPIO_InitStruct.Pin = PM1_ZEROU_Pin|PM1_ZEROV_Pin|PM1_ZEROW_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = PM1_PWM_UH_Pin|PM1_PWM_VH_Pin|PM1_PWM_WH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PB12     ------> TIM1_BKIN
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOA, PM1_PWM_UH_Pin|PM1_PWM_VH_Pin|PM1_PWM_WH_Pin);

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    PA2     ------> TIM5_CH3
    */
    HAL_GPIO_DeInit(GPIOA, PM1_ZEROU_Pin|PM1_ZEROV_Pin|PM1_ZEROW_Pin);

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**********************************************************************************************/
int32_t temp_pwm1 = 0.0;                                /* 存放速度环PID结果 */
//int32_t motor_pwm_s = 0;                                /*  一阶滤波结果*/
float motor_pwm_s = 0; 
//float motor_pwm_sL = 0;
/*停机状态下电流采集使用  */
#define ADC_AMP_OFFSET_TIMES 50
uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1];
uint8_t adc_amp_offset_p = 0;
int16_t adc_amp[3];
int16_t adc_amp_un[3];
float  adc_amp_bus=0.0f;
uint8_t clc = 0;                                        /* 等待进入闭环控制的时间 */
float debug_data_temp = 0.0;

volatile uint16_t adc_val_m1[ADC_CH_NUM];
/**
 * @brief       
 * @param       
 * @retval      
 */
 

 float MY_FirstOrderRC_LPF(float Yn_1,float Xn,float a){
     
    	 Yn_1 = (1-a)*Yn_1 + a*(float)Xn;
	    if(Yn_1 <0&&Yn_1 >= -600) {
			  return  -600.0;
			}else if(Yn_1 >0&&Yn_1 <= 600) 
			{
			  return	600.0;
			}else
			{
			  return	Yn_1;
			}
 }
 
 
void HAL_TIM_PeriodElapsedCallback_MY(TIM_HandleTypeDef *htim)
{
    uint8_t i;
    static uint8_t times_count = 0;                     /* 定时器时间记录*/
    if(htim->Instance == TIM1)                 /* 55us */
    {
			  g_bldc_motor1.run_flag = RUN;//调试使用
        if(g_bldc_motor1.run_flag == RUN)
        {
            /*****************************  ****************************/
            for(i=0; i<3; i++)
            {
                //adc_val_m1[i] = g_adc_value[i+2];
							  if(i==0) adc_val_m1[i] = g_adc_value[0];
							  if(i==1) adc_val_m1[i] = g_adc_value[3];
							  if(i==2) adc_val_m1[i] = g_adc_value[5];
							
                adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];
                if(adc_amp[i] < 0)  
									adc_amp_un[i] = 0;                  /* 反电动势电压为悬空时，绕组清零 */
                else if(adc_amp[i] >= 0)                /* 去除反电动势引起的负电流数据 */
                    adc_amp_un[i] = adc_amp[i];
            }
            /*计算母线电流之和*/
            adc_amp_bus = (adc_amp_un[0] + adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;
        }
//#ifdef H_PWM_L_ON
        /* 过零点检测*/
        zero_ctr_loop();
         //g_zero_ctr_status=3;
        /***************************** 速度环PID控制 *****************************/
        if(g_bldc_motor1.run_flag == RUN && g_zero_ctr_status == 3) /* ¾ßÓÐÒ»¶¨ËÙ¶È£¨ÓÐËÙ¶È²âÁ¿ËµÃ÷ÒÑ¾­½øÈë¹ýÁã±Õ»·×´Ì¬£©ºó²ÅÄÜ½øÈëPID±Õ»·¿ØÖÆ */
        {
                temp_pwm1 = increment_pid_ctrl(&g_speed_pid,g_bldc_motor1.speed);

                //FirstOrderRC_LPF(motor_pwm_s,(float)temp_pwm1,0.085);
					       motor_pwm_s = MY_FirstOrderRC_LPF(motor_pwm_s,(float)temp_pwm1,0.085);
                /* 最低速度限制 */
                if(motor_pwm_s < 0)
                {
                    /* 加速启动速度 */
//									  if(motor_pwm_s >= -600) {
//										    motor_pwm_s = -600;
//                    }
                    g_bldc_motor1.pwm_duty = -(int32_t)motor_pwm_s;          /* 取绝对值*/
                }
                else
                {
									    /* 加速启动速度 */
//									  if(motor_pwm_s <= 600) {
//										    motor_pwm_s = 600;
//                    }
                    g_bldc_motor1.pwm_duty = (int32_t)motor_pwm_s;
                }
        }

//#endif
    }
    if(htim->Instance == TIM6)
    {
        /*****************************基准电压测量 *****************************/
        times_count++;
        if(g_bldc_motor1.run_flag == STOP)
        {
            uint8_t i;
            uint32_t avg[3] = {0,0,0};
            adc_amp_offset[0][adc_amp_offset_p] = g_adc_value[1];         /* 得到还未开始运动时的三相基准电压 */
            adc_amp_offset[1][adc_amp_offset_p] = g_adc_value[2];         
            adc_amp_offset[2][adc_amp_offset_p] = g_adc_value[4];         
            adc_amp_offset_p ++;
            NUM_CLEAR(adc_amp_offset_p,ADC_AMP_OFFSET_TIMES);           /* 最大采集ADC_AMP_OFFSET_TIMES 5次 */
            for(i=0; i<ADC_AMP_OFFSET_TIMES; i++)                       /* 将采集的每个通道值累加 */
            {
                avg[0] += adc_amp_offset[0][i];
                avg[1] += adc_amp_offset[1][i];
                avg[2] += adc_amp_offset[2][i];
            }
            for(i=0; i<3; i++)
            {
                avg[i] /= ADC_AMP_OFFSET_TIMES;                         /* 取平均值 */
                adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];       /* 得到还未运动时的基准电压 */
            }
        }
        if(times_count == SMAPLSE_PID_SPEED)
        {
#if (LOCK_TAC == 2)
            /***************************** 堵塞处理 *****************************/
            if(g_bldc_motor1.locked_rotor==1)// 1      /* 堵塞 */
            {
                clc++;
                if(clc > 50)                        /* 延迟1s后重新启动*/
                {
                    clc = 0;
                    pid_init();
                    stop_motor1();
                    g_speed_pid.SetPoint = 400.0; /* 400PRM */
                    g_bldc_motor1.dir = CW;       /* 初始方向正转 */
                    g_bldc_motor1.pwm_duty = 600; /* 加速启动速度 */
									  g_bldc_motor1.run_flag = RUN; /* 开机运行 */
									  start_motor1();               /* 运行电机*/
                    g_bldc_motor1.locked_rotor = 0;
                    g_zero_ctr_status = 0;        /* 重新定位初始位置 */
                }
            }
#endif
            times_count = 0;
        }
    }
}
/* USER CODE END 1 */
