#include "stm32h7xx_hal.h"

HAL_StatusTypeDef HAL_InitTick(uint32_t tick_priority)
{
  RCC_ClkInitTypeDef clock_config;
  uint32_t flash_latency;
  uint32_t timer_clock;
  uint32_t prescaler;

  __HAL_RCC_TIM6_CLK_ENABLE();
  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  timer_clock = HAL_RCC_GetPCLK1Freq();
  if (clock_config.APB1CLKDivider != RCC_APB1_DIV1)
  {
    timer_clock *= 2U;
  }
  prescaler = (timer_clock / 1000000U) - 1U;
  CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
  WRITE_REG(TIM6->PSC, prescaler);
  WRITE_REG(TIM6->ARR, 1000U - 1U);
  WRITE_REG(TIM6->EGR, TIM_EGR_UG);
  WRITE_REG(TIM6->SR, 0U);
  SET_BIT(TIM6->DIER, TIM_DIER_UIE);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, tick_priority, 0U);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  uwTickPrio = tick_priority;
  SET_BIT(TIM6->CR1, TIM_CR1_CEN);
  return HAL_OK;
}

void HAL_SuspendTick(void)
{
  CLEAR_BIT(TIM6->DIER, TIM_DIER_UIE);
}

void HAL_ResumeTick(void)
{
  SET_BIT(TIM6->DIER, TIM_DIER_UIE);
}
