#ifndef SSD1306_H
#define SSD1306_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define SSD1306_WIDTH   128U
#define SSD1306_HEIGHT  64U

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c);
void SSD1306_ClearBuffer(void);
void SSD1306_WriteLine(uint8_t line, const char *text);
HAL_StatusTypeDef SSD1306_Update(void);
uint16_t SSD1306_GetAddress(void);

#endif /* SSD1306_H */
