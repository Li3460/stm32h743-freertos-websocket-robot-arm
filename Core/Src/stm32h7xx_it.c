#include "main.h"
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

void NMI_Handler(void) { while (1) { } }
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void) { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void DebugMon_Handler(void) { }

void SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
#else
  xPortSysTickHandler();
#endif
}

void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void USART2_IRQHandler(void)
{
  uint32_t isr = READ_REG(huart2.Instance->ISR);
  uint32_t cr1 = READ_REG(huart2.Instance->CR1);
  uint32_t error_flags = isr & (USART_ISR_PE | USART_ISR_FE |
                                USART_ISR_NE | USART_ISR_ORE);

  if (error_flags != 0U)
  {
    if ((error_flags & USART_ISR_PE) != 0U) { huart2.ErrorCode |= HAL_UART_ERROR_PE; }
    if ((error_flags & USART_ISR_NE) != 0U) { huart2.ErrorCode |= HAL_UART_ERROR_NE; }
    if ((error_flags & USART_ISR_FE) != 0U) { huart2.ErrorCode |= HAL_UART_ERROR_FE; }
    if ((error_flags & USART_ISR_ORE) != 0U) { huart2.ErrorCode |= HAL_UART_ERROR_ORE; }
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_NEF | UART_CLEAR_OREF);
    CLEAR_BIT(huart2.Instance->CR1, USART_CR1_PEIE | USART_CR1_IDLEIE);
    CLEAR_BIT(huart2.Instance->CR3, USART_CR3_EIE | USART_CR3_DMAR);
    if (huart2.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE(huart2.hdmarx);
      huart2.hdmarx->State = HAL_DMA_STATE_READY;
      huart2.hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
    }
    huart2.RxState = HAL_UART_STATE_READY;
    huart2.ReceptionType = HAL_UART_RECEPTION_STANDARD;
    HAL_UART_ErrorCallback(&huart2);
    huart2.ErrorCode = HAL_UART_ERROR_NONE;
    return;
  }

  if (((isr & USART_ISR_IDLE) != 0U) && ((cr1 & USART_CR1_IDLEIE) != 0U))
  {
    uint16_t remaining;
    uint16_t received;

    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_IDLEF);
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if ((remaining > 0U) && (remaining < huart2.RxXferSize))
    {
      received = (uint16_t)(huart2.RxXferSize - remaining);
      huart2.RxXferCount = remaining;
      CLEAR_BIT(huart2.Instance->CR1, USART_CR1_PEIE | USART_CR1_IDLEIE);
      CLEAR_BIT(huart2.Instance->CR3, USART_CR3_EIE | USART_CR3_DMAR);
      __HAL_DMA_DISABLE(huart2.hdmarx);
      huart2.hdmarx->State = HAL_DMA_STATE_READY;
      huart2.hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
      huart2.RxState = HAL_UART_STATE_READY;
      huart2.ReceptionType = HAL_UART_RECEPTION_STANDARD;
      huart2.RxEventType = HAL_UART_RXEVENT_IDLE;
      HAL_UARTEx_RxEventCallback(&huart2, received);
    }
    return;
  }

  if (((isr & USART_ISR_TC) != 0U) && ((cr1 & USART_CR1_TCIE) != 0U))
  {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TCF);
    CLEAR_BIT(huart2.Instance->CR1, USART_CR1_TCIE);
    huart2.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&huart2);
  }
}

void TIM6_DAC_IRQHandler(void)
{
  if (((TIM6->SR & TIM_SR_UIF) != 0U) &&
      ((TIM6->DIER & TIM_DIER_UIE) != 0U))
  {
    CLEAR_BIT(TIM6->SR, TIM_SR_UIF);
    HAL_IncTick();
  }
}
