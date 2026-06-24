#ifndef PCA9685_H
#define PCA9685_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define PCA9685_I2C_ADDR       (0x40U << 1U)
#define PCA9685_SERVO_FREQ_HZ  50U

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

#endif /* PCA9685_H */
