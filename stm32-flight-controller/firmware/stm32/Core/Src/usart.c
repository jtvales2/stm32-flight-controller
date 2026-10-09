/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "sbus.h"

#define SBUS_UART6_DMA_RX_SIZE  256u

static uint8_t  s_sbus_dma_rx[SBUS_UART6_DMA_RX_SIZE];
static volatile uint16_t s_sbus_dma_pos = 0u;
static volatile uint8_t  s_sbus_dma_busy = 0u;
static volatile uint8_t  s_sbus_dma_restart_pending = 0u;

static volatile uint32_t s_sbus_dma_rx_bytes = 0u;
static volatile uint32_t s_sbus_dma_rx_events = 0u;
static volatile uint32_t s_sbus_dma_idle_events = 0u;
static volatile uint32_t s_sbus_dma_poll_events = 0u;
static volatile uint32_t s_sbus_dma_errors = 0u;
static volatile uint32_t s_sbus_dma_overruns = 0u;
static volatile uint32_t s_sbus_dma_restarts = 0u;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

static uint32_t sbus_irq_save(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void sbus_irq_restore(uint32_t primask)
{
  if (primask == 0u) {
    __enable_irq();
  }
}

static uint8_t sbus_dma_try_lock(void)
{
  uint8_t ok = 0u;
  uint32_t primask = sbus_irq_save();
  if (!s_sbus_dma_busy) {
    s_sbus_dma_busy = 1u;
    ok = 1u;
  }
  sbus_irq_restore(primask);
  return ok;
}

static void sbus_dma_unlock(void)
{
  uint32_t primask = sbus_irq_save();
  s_sbus_dma_busy = 0u;
  sbus_irq_restore(primask);
}

static uint16_t sbus_uart6_dma_current_pos(void)
{
  uint16_t ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
  uint16_t pos = (uint16_t)(SBUS_UART6_DMA_RX_SIZE - ndtr);
  if (pos >= SBUS_UART6_DMA_RX_SIZE) {
    pos = 0u;
  }
  return pos;
}

static uint16_t sbus_uart6_dma_service_pos(uint16_t pos)
{
  uint16_t old;
  uint16_t count = 0u;

  if (pos >= SBUS_UART6_DMA_RX_SIZE) {
    pos = 0u;
  }

  if (!sbus_dma_try_lock()) {
    return 0u;
  }

  old = s_sbus_dma_pos;
  if (pos == old) {
    sbus_dma_unlock();
    return 0u;
  }

  if (pos > old) {
    count = (uint16_t)(pos - old);
    sbus_feed_bytes(&s_sbus_dma_rx[old], count);
  } else {
    count = (uint16_t)(SBUS_UART6_DMA_RX_SIZE - old);
    if (count > 0u) {
      sbus_feed_bytes(&s_sbus_dma_rx[old], count);
    }
    if (pos > 0u) {
      sbus_feed_bytes(&s_sbus_dma_rx[0], pos);
      count = (uint16_t)(count + pos);
    }
  }

  s_sbus_dma_pos = pos;
  s_sbus_dma_rx_bytes += count;
  sbus_dma_unlock();
  return count;
}

static void sbus_uart6_dma_start(void)
{
  uint32_t primask;

  HAL_UART_DMAStop(&huart6);
  __HAL_UART_CLEAR_IDLEFLAG(&huart6);
  __HAL_UART_CLEAR_OREFLAG(&huart6);

  primask = sbus_irq_save();
  s_sbus_dma_pos = 0u;
  s_sbus_dma_busy = 0u;
  sbus_irq_restore(primask);

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                   s_sbus_dma_rx,
                                   SBUS_UART6_DMA_RX_SIZE) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    s_sbus_dma_restarts++;
  } else {
    s_sbus_dma_restart_pending = 1u;
  }
}

void sbus_uart6_dma_poll(void)
{
  uint16_t pos;
  uint16_t count;

  if (s_sbus_dma_restart_pending) {
    uint32_t primask = sbus_irq_save();
    s_sbus_dma_restart_pending = 0u;
    sbus_irq_restore(primask);
    sbus_uart6_dma_start();
  }

  pos = sbus_uart6_dma_current_pos();
  count = sbus_uart6_dma_service_pos(pos);
  if (count > 0u) {
    s_sbus_dma_poll_events++;
  }
}

void sbus_uart6_dma_get_stats(sbus_uart6_dma_stats_t *out)
{
  uint32_t primask;
  if (out == 0) {
    return;
  }

  primask = sbus_irq_save();
  out->rx_bytes    = s_sbus_dma_rx_bytes;
  out->rx_events   = s_sbus_dma_rx_events;
  out->idle_events = s_sbus_dma_idle_events;
  out->poll_events = s_sbus_dma_poll_events;
  out->errors      = s_sbus_dma_errors;
  out->overruns    = s_sbus_dma_overruns;
  out->restarts    = s_sbus_dma_restarts;
  out->last_pos    = s_sbus_dma_pos;
  sbus_irq_restore(primask);
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 100000;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_2;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
	sbus_uart6_dma_start();  // 如果 start 是 static，就只保留包里的调用方式
  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG9     ------> USART6_RX
    PG14     ------> USART6_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG9     ------> USART6_RX
    PG14     ------> USART6_TX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9|GPIO_PIN_14);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART6) {
    s_sbus_dma_rx_events++;
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) {
      s_sbus_dma_idle_events++;
    }
    (void)sbus_uart6_dma_service_pos(Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) {
    uint32_t err = huart->ErrorCode;
    s_sbus_dma_errors++;
    if ((err & HAL_UART_ERROR_ORE) != 0u) {
      s_sbus_dma_overruns++;
    }
    s_sbus_dma_restart_pending = 1u;
  }
}

/* USER CODE END 1 */
