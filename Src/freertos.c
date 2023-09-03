/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
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
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "usart.h"

/* USER CODE END Includes */

#define PCKT_SIZE 16

/* Variables -----------------------------------------------------------------*/

/* USER CODE BEGIN Variables */
TaskHandle_t spiTaskHandle;
TaskHandle_t usartTaskHandle;

QueueHandle_t usartTxQueue;
QueueHandle_t spiTxQueue;

static uint8_t spiRxItem[PCKT_SIZE];
static uint8_t usartRxItem[PCKT_SIZE];
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void MX_FREERTOS_Init(void); 

/* USER CODE BEGIN FunctionPrototypes */
void LEDTask(void *);
void USARTTask(void *);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  SemaphoreHandle_t xSemaphore = NULL;
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(USARTTask, "USARTTask", 128, (void *)1, tskIDLE_PRIORITY+1, &usartTaskHandle);
  xTaskCreate(USARTTask, "SPITask", 128, (void *)1, tskIDLE_PRIORITY+2, &spiTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Application */
void USARTTask(void *pvParameters) {
	uint8_t usartTxItem[PCKT_SIZE];
	uint16_t idx = 0;

	if ( (usartTxQueue = xQueueCreate(4, PCKT_SIZE)) == NULL)
		return;

  HAL_UART_Receive_IT(&huart1, &byte, 1);

	while(1) {
    
		if (xQueueReceive(usartTxQueue, usartTxItem, 10) != pdPASS)
			continue;

    HAL_UART_Transmit(&huart1, &usartTxItem, PCKT_SIZE, 100);
	  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == USART2)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      HAL_UART_Receive_IT(&huart1, &byte, 1);
      usartRxItem[idx++] = byte & 0xff;
      if(idx >= PCKT_SIZE) {
          status = xQueueSendToFrontFromISR( spiTxQueue, &usartRxItem, &xHigherPriorityTaskWoken );
          idx = 0;
      }
  }
}

/* USER CODE BEGIN Application */
void SPITTask(void *pvParameters) {
	uint8_t spiTxItem[PCKT_SIZE];
  
	if ( (spiTxQueue = xQueueCreate(4, PCKT_SIZE)) == NULL)
		return;
  xSemaphore = xSemaphoreCreateMutex();

	while(1) {
    // UART Rx -> SPI Tx
		if (xQueueReceive(spiTxQueue, spiTxItem, 10) == pdPASS) 
    {
      if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
      {
        response = HAL_SPI_Transmit_DMA(&hspi1, spiTxItem, PCKT_SIZE);
      }
    }
    // SPI Rx -> UART Tx Queue
    if (ulTaskNotifyTake(pdTRUE, 0xFFFFFFFF) != 0 ) {
      if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
      {
        response = HAL_SPI_TransmitReceive_DMA(&hspi1, sprTxItem, spiRxItem, 2);
      }
    }
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    xSemaphoreGive( SemaphoreHandle_t xSemaphore);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToFrontFromISR( spiTxQueue, &spiRxItem, &xHigherPriorityTaskWoken );
    xSemaphoreGive( SemaphoreHandle_t xSemaphore);
}

/*Your general GPIO_EXTI Callbacks here*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE */  
   BaseType_t xHigherPriorityTaskWoken;
   xHigherPriorityTaskWoken = pdFALSE;
   
   /* your user button callback here*/
   if(GPIO_Pin == SPI_DATA_READY_PIN)
   {
      /* Send the notification directly to the task handler*/
      vTaskNotifyGiveFromISR( xTaskHandler, &xHigherPriorityTaskWoken );
      /*If xHigherPriorityTaskWoken was set to pdTRUE inside vTaskNotifyGiveFromISR()
        then calling portYIELD_FROM_ISR() will request a context switch. */      
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
   }
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
