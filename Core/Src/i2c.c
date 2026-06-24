#include "i2c.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

static void I2C_CommonInit(I2C_HandleTypeDef *handle)
{
  handle->Init.Timing = 0x307075B1U;
  handle->Init.OwnAddress1 = 0U;
  handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  handle->Init.OwnAddress2 = 0U;
  handle->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if ((HAL_I2C_Init(handle) != HAL_OK) ||
      (HAL_I2CEx_ConfigAnalogFilter(handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK) ||
      (HAL_I2CEx_ConfigDigitalFilter(handle, 0U) != HAL_OK))
  {
    Error_Handler();
  }
}

void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  I2C_CommonInit(&hi2c1);
}

void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  I2C_CommonInit(&hi2c2);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c_handle)
{
  GPIO_InitTypeDef gpio = {0};

  if ((i2c_handle->Instance != I2C1) && (i2c_handle->Instance != I2C2))
  {
    return;
  }
  __HAL_RCC_I2C123_CONFIG(RCC_I2C123CLKSOURCE_D2PCLK1);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  if (i2c_handle->Instance == I2C1)
  {
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
  else
  {
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_I2C2_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2c_handle)
{
  if (i2c_handle->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
  }
  else if (i2c_handle->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
  }
}
