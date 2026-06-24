#ifndef OLED_STATUS_H
#define OLED_STATUS_H

#include "system_status.h"
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef OLEDStatus_Init(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef OLEDStatus_ShowStartup(void);
HAL_StatusTypeDef OLEDStatus_Render(const SystemStatus_t *status,
                                    uint32_t page_counter);

#endif /* OLED_STATUS_H */
