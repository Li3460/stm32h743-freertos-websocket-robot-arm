#include "usart.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

void MX_USART2_UART_Init(void)
{
  uint32_t peripheral_clock;

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200U;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  /* Fixed 115200-8-N-1 setup. The handle state matches HAL_UART_Init so the
     HAL ReceiveToIdle and DMA APIs remain fully usable. */
  huart2.Lock = HAL_UNLOCKED;
  huart2.gState = HAL_UART_STATE_BUSY;
  huart2.RxState = HAL_UART_STATE_BUSY_RX;
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
  huart2.ReceptionType = HAL_UART_RECEPTION_STANDARD;
  huart2.RxEventType = HAL_UART_RXEVENT_TC;
  HAL_UART_MspInit(&huart2);
  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  WRITE_REG(USART2->CR1, USART_CR1_TE | USART_CR1_RE);
  WRITE_REG(USART2->CR2, 0U);
  WRITE_REG(USART2->CR3, 0U);
  WRITE_REG(USART2->PRESC, UART_PRESCALER_DIV1);
  peripheral_clock = HAL_RCC_GetPCLK1Freq();
  WRITE_REG(USART2->BRR,
            (peripheral_clock + (huart2.Init.BaudRate / 2U)) /
            huart2.Init.BaudRate);
  SET_BIT(USART2->CR1, USART_CR1_UE);
  huart2.gState = HAL_UART_STATE_READY;
  huart2.RxState = HAL_UART_STATE_READY;
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart_handle)
{
  GPIO_InitTypeDef gpio = {0};

  if (uart_handle->Instance != USART2) { return; }
  __HAL_RCC_USART234578_CONFIG(RCC_USART234578CLKSOURCE_D2PCLK1);
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &gpio);

  hdma_usart2_rx.Instance = DMA1_Stream0;
  hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
  hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_rx.Init.Mode = DMA_NORMAL;
  hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) { Error_Handler(); }
  __HAL_LINKDMA(uart_handle, hdmarx, hdma_usart2_rx);

  hdma_usart2_tx.Instance = DMA1_Stream1;
  hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
  hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) { Error_Handler(); }
  __HAL_LINKDMA(uart_handle, hdmatx, hdma_usart2_tx);

  HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uart_handle)
{
  if (uart_handle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    HAL_DMA_DeInit(uart_handle->hdmarx);
    HAL_DMA_DeInit(uart_handle->hdmatx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}
