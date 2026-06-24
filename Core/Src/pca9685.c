#include "pca9685.h"

#define PCA9685_MODE1          0x00U
#define PCA9685_MODE2          0x01U
#define PCA9685_LED0_ON_L      0x06U
#define PCA9685_PRESCALE       0xFEU
#define PCA9685_MODE1_RESTART  0x80U
#define PCA9685_MODE1_AI       0x20U
#define PCA9685_MODE1_SLEEP    0x10U
#define PCA9685_MODE2_OUTDRV   0x04U
#define PCA9685_TIMEOUT_MS     100U

static I2C_HandleTypeDef *s_i2c;

static HAL_StatusTypeDef PCA9685_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};

  if (s_i2c == NULL)
  {
    return HAL_ERROR;
  }
  return HAL_I2C_Master_Transmit(s_i2c, PCA9685_I2C_ADDR, data,
                                 sizeof(data), PCA9685_TIMEOUT_MS);
}

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *i2c)
{
  uint32_t denominator;
  uint32_t prescale_value;
  uint8_t prescale;
  HAL_StatusTypeDef status;

  if (i2c == NULL)
  {
    return HAL_ERROR;
  }
  s_i2c = i2c;
  if (HAL_I2C_IsDeviceReady(s_i2c, PCA9685_I2C_ADDR, 2U,
                            PCA9685_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  status = PCA9685_WriteRegister(PCA9685_MODE1,
                                 PCA9685_MODE1_AI | PCA9685_MODE1_SLEEP);
  if (status != HAL_OK) { return status; }
  status = PCA9685_WriteRegister(PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
  if (status != HAL_OK) { return status; }

  denominator = 4096U * PCA9685_SERVO_FREQ_HZ;
  prescale_value = (25000000U + (denominator / 2U)) / denominator;
  if (prescale_value > 0U) { prescale_value--; }
  if (prescale_value > 255U) { prescale_value = 255U; }
  prescale = (uint8_t)prescale_value;
  status = PCA9685_WriteRegister(PCA9685_PRESCALE, prescale);
  if (status != HAL_OK) { return status; }
  status = PCA9685_WriteRegister(PCA9685_MODE1, PCA9685_MODE1_AI);
  if (status != HAL_OK) { return status; }
  HAL_Delay(5U);
  return PCA9685_WriteRegister(PCA9685_MODE1,
                               PCA9685_MODE1_RESTART | PCA9685_MODE1_AI);
}

HAL_StatusTypeDef PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
  uint8_t data[5];

  if ((s_i2c == NULL) || (channel > 15U))
  {
    return HAL_ERROR;
  }
  if (on > 4095U) { on = 4095U; }
  if (off > 4095U) { off = 4095U; }
  data[0] = (uint8_t)(PCA9685_LED0_ON_L + (4U * channel));
  data[1] = (uint8_t)(on & 0xFFU);
  data[2] = (uint8_t)((on >> 8U) & 0x0FU);
  data[3] = (uint8_t)(off & 0xFFU);
  data[4] = (uint8_t)((off >> 8U) & 0x0FU);
  return HAL_I2C_Master_Transmit(s_i2c, PCA9685_I2C_ADDR, data,
                                 sizeof(data), PCA9685_TIMEOUT_MS);
}
