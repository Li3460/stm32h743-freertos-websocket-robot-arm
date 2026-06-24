#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define SERVO_COUNT              6U
#define SERVO_MOVE_HOME_ON_BOOT  0
#define SERVO_DEFAULT_SPEED      30U

typedef enum
{
  SERVO_CMD_SINGLE = 0,
  SERVO_CMD_ALL,
  SERVO_CMD_HOME,
  SERVO_CMD_STOP
} ServoCommandType_t;

typedef struct
{
  ServoCommandType_t type;
  uint32_t seq;
  uint8_t servo_id;
  int16_t angles[SERVO_COUNT];
  uint16_t speed_deg_s;
} ServoCommand_t;

typedef struct
{
  float current_angle;
  float target_angle;
  float home_angle;
  float min_angle;
  float max_angle;
  float max_speed_deg_s;
  uint16_t min_pulse_us;
  uint16_t max_pulse_us;
  int8_t direction;
  uint8_t channel;
} ServoConfig_t;

void ServoManager_Init(void);
HAL_StatusTypeDef ServoManager_InitHardware(I2C_HandleTypeDef *i2c);
bool ServoManager_ValidateCommand(const ServoCommand_t *command,
                                  const char **error_code);
void ServoManager_ApplyCommand(const ServoCommand_t *command);
HAL_StatusTypeDef ServoManager_Update20ms(void);
uint16_t ServoManager_GetPulseUs(uint8_t servo_id, float angle);
const ServoConfig_t *ServoManager_GetConfig(uint8_t servo_id);

#endif /* SERVO_MANAGER_H */
