/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "PWM-Basic.h"
#include "PWM-Regulator.h"
#include "PWM-Tri.h"
//#include "hrtim.h"
#include "tim.h"
#include "adc.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId T01Handle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId GUIHandle;
uint32_t GUIBuffer[ 800 ];
osStaticThreadDef_t GUIControlBlock;

/* USER CODE BEGIN Variables */
PWM_Basic_t PWM_BASIC[3];
PWM_Tri_t   PWM_TRI;
uint16_t pu16CurrentADC[6]; // ADC2使用DMA方式采集电流反馈
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartGUI(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void PWM_DriverInit(void);
/* USER CODE END FunctionPrototypes */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */ 
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

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

  /* Create the thread(s) */
  /* definition and creation of T01 */
  osThreadStaticDef(T01, StartDefaultTask, osPriorityNormal, 0, 64, defaultTaskBuffer, &defaultTaskControlBlock);
  T01Handle = osThreadCreate(osThread(T01), NULL);

  /* definition and creation of GUI */
  osThreadStaticDef(GUI, StartGUI, osPriorityIdle, 0, 512, GUIBuffer, &GUIControlBlock);
  GUIHandle = osThreadCreate(osThread(GUI), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
__weak void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  PWM_DriverInit();

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartGUI function */
__weak void StartGUI(void const * argument)
{
  /* USER CODE BEGIN StartGUI */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGUI */
}

/* USER CODE BEGIN Application */
void PWM_DriverInit(void)
{
  for (int i = 0; i < 3; i++)
  {
    PWM_BASIC[i].Frequence = 40e3;
    PWM_BASIC[i].ReloadValue = 3600;
    PWM_BASIC[i].PWM_ID = i + 1;
    PWM_BASIC[i].CC_Max = 3600;
    PWM_BASIC[i].CC_Min = 0;
    PWM_BASIC[i].DutyCycle = 0.0f;
    PWM_BASIC[i].CC = 0;
  }

  PWM_TRI.pPWM_A = PWM_BASIC;
  PWM_TRI.pPWM_B = PWM_BASIC + 1;
  PWM_TRI.pPWM_C = PWM_BASIC + 2;
  PWM_TRI.PID.Kp = 0.5;
  PWM_TRI.PID.Ki = 0.03;
  PWM_TRI.PID.Kd = 0;
  PWM_TRI.fTargetVrms = 5;
  PWM_TRI.fInputVoltage = 50;
  PWM_TRI.fPhaseOffsetB = 0;
  PWM_TRI.fPhaseOffsetC = 0;
  PWM_TRI.fTargetFreq = 50.0f;
  PWM_TRI.uSampleFreq = 5e3;
  PWM_TRI_Init(&PWM_TRI);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(10);
  
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)pu16CurrentADC, 6);

  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start_IT(&htim1);
}
void PWM_CALLBACK(void)
{
      PWM_TRI_STEP(&PWM_TRI);
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
