#include "servo_manager.h"

#include "pca9685.h"
#include "system_status.h"

/* Temporary bench values. Recalibrate limits, pulse widths and direction
   against the real mechanical arm before applying servo power. */
static ServoConfig_t s_servos[SERVO_COUNT] =
{
  {90.0f, 90.0f, 90.0f, 10.0f, 170.0f, 100.0f, 500U, 2500U, 1, 0U},
  {90.0f, 90.0f, 90.0f, 20.0f, 160.0f, 100.0f, 500U, 2500U, 1, 1U},
  {90.0f, 90.0f, 90.0f, 20.0f, 160.0f, 100.0f, 500U, 2500U, 1, 2U},
  {90.0f, 90.0f, 90.0f, 10.0f, 170.0f, 100.0f, 500U, 2500U, 1, 3U},
  {90.0f, 90.0f, 90.0f, 10.0f, 170.0f, 100.0f, 500U, 2500U, 1, 4U},
  {30.0f, 30.0f, 30.0f, 10.0f, 100.0f, 100.0f, 500U, 2500U, 1, 5U}
};

static uint16_t s_speeds[SERVO_COUNT];
static uint16_t s_pulses[SERVO_COUNT];
static bool s_output_active[SERVO_COUNT];
static bool s_needs_output[SERVO_COUNT];
static bool s_hardware_ready;
static uint8_t s_last_servo_id = 1U;

static int16_t ServoManager_Round(float value)
{
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void ServoManager_PublishStatus(void)
{
  int16_t angles[SERVO_COUNT];
  int16_t targets[SERVO_COUNT];
  bool moving = false;
  uint8_t index;
  float delta;

  for (index = 0U; index < SERVO_COUNT; index++)
  {
    angles[index] = ServoManager_Round(s_servos[index].current_angle);
    targets[index] = ServoManager_Round(s_servos[index].target_angle);
    delta = s_servos[index].target_angle - s_servos[index].current_angle;
    if ((delta > 0.01f) || (delta < -0.01f))
    {
      moving = true;
    }
  }
  SystemStatus_SetServoSnapshot(angles, targets, s_pulses, moving,
                                s_last_servo_id);
}

void ServoManager_Init(void)
{
  uint8_t index;

  for (index = 0U; index < SERVO_COUNT; index++)
  {
    s_servos[index].current_angle = s_servos[index].home_angle;
    s_servos[index].target_angle = s_servos[index].home_angle;
    s_speeds[index] = SERVO_DEFAULT_SPEED;
    s_pulses[index] = 0U;
#if SERVO_MOVE_HOME_ON_BOOT
    s_output_active[index] = true;
    s_needs_output[index] = true;
#else
    s_output_active[index] = false;
    s_needs_output[index] = false;
#endif
  }
  s_hardware_ready = false;
  s_last_servo_id = 1U;
  ServoManager_PublishStatus();
}

HAL_StatusTypeDef ServoManager_InitHardware(I2C_HandleTypeDef *i2c)
{
  HAL_StatusTypeDef status = PCA9685_Init(i2c);

  s_hardware_ready = (status == HAL_OK);
  if (s_hardware_ready)
  {
    uint8_t index;
    for (index = 0U; index < SERVO_COUNT; index++)
    {
      if (s_output_active[index]) { s_needs_output[index] = true; }
    }
  }
  SystemStatus_SetPca9685Ready(s_hardware_ready);
  return status;
}

const ServoConfig_t *ServoManager_GetConfig(uint8_t servo_id)
{
  if ((servo_id == 0U) || (servo_id > SERVO_COUNT))
  {
    return NULL;
  }
  return &s_servos[servo_id - 1U];
}

bool ServoManager_ValidateCommand(const ServoCommand_t *command,
                                  const char **error_code)
{
  uint8_t first;
  uint8_t last;
  uint8_t index;

  if (error_code != NULL) { *error_code = "FORMAT_ERROR"; }
  if (command == NULL) { return false; }
  if (command->type == SERVO_CMD_STOP) { return true; }
  if ((command->speed_deg_s < 1U) || (command->speed_deg_s > 100U))
  {
    if (error_code != NULL) { *error_code = "SPEED_OUT_OF_RANGE"; }
    return false;
  }
  if (command->type == SERVO_CMD_HOME) { return true; }
  if (command->type == SERVO_CMD_SINGLE)
  {
    if ((command->servo_id == 0U) || (command->servo_id > SERVO_COUNT))
    {
      if (error_code != NULL) { *error_code = "INVALID_ID"; }
      return false;
    }
    first = (uint8_t)(command->servo_id - 1U);
    last = first;
  }
  else if (command->type == SERVO_CMD_ALL)
  {
    first = 0U;
    last = SERVO_COUNT - 1U;
  }
  else
  {
    return false;
  }
  for (index = first; index <= last; index++)
  {
    if (((float)command->angles[index] < s_servos[index].min_angle) ||
        ((float)command->angles[index] > s_servos[index].max_angle))
    {
      if (error_code != NULL) { *error_code = "ANGLE_OUT_OF_RANGE"; }
      return false;
    }
  }
  return true;
}

void ServoManager_ApplyCommand(const ServoCommand_t *command)
{
  uint8_t first = 0U;
  uint8_t last = SERVO_COUNT - 1U;
  uint8_t index;

  if (command == NULL) { return; }
  if (command->type == SERVO_CMD_STOP)
  {
    for (index = 0U; index < SERVO_COUNT; index++)
    {
      s_servos[index].target_angle = s_servos[index].current_angle;
    }
    ServoManager_PublishStatus();
    return;
  }
  if (command->type == SERVO_CMD_SINGLE)
  {
    first = (uint8_t)(command->servo_id - 1U);
    last = first;
  }
  for (index = first; index <= last; index++)
  {
    if (command->type == SERVO_CMD_HOME)
    {
      s_servos[index].target_angle = s_servos[index].home_angle;
    }
    else
    {
      s_servos[index].target_angle = (float)command->angles[index];
    }
    s_speeds[index] = command->speed_deg_s;
    s_output_active[index] = true;
    s_needs_output[index] = true;
  }
  s_last_servo_id = (command->type == SERVO_CMD_SINGLE) ?
                    command->servo_id : 1U;
  ServoManager_PublishStatus();
}

uint16_t ServoManager_GetPulseUs(uint8_t servo_id, float angle)
{
  const ServoConfig_t *servo = ServoManager_GetConfig(servo_id);
  float effective_angle;
  float ratio;
  float pulse;

  if (servo == NULL) { return 0U; }
  if (angle < servo->min_angle) { angle = servo->min_angle; }
  if (angle > servo->max_angle) { angle = servo->max_angle; }
  effective_angle = (servo->direction >= 0) ? angle :
                    (servo->min_angle + servo->max_angle - angle);
  ratio = (effective_angle - servo->min_angle) /
          (servo->max_angle - servo->min_angle);
  pulse = (float)servo->min_pulse_us +
          ratio * (float)(servo->max_pulse_us - servo->min_pulse_us);
  return (uint16_t)(pulse + 0.5f);
}

HAL_StatusTypeDef ServoManager_Update20ms(void)
{
  uint8_t index;
  float delta;
  float speed;
  float step;
  uint16_t off_count;
  HAL_StatusTypeDef overall = HAL_OK;

  if (!s_hardware_ready) { return HAL_ERROR; }
  for (index = 0U; index < SERVO_COUNT; index++)
  {
    delta = s_servos[index].target_angle - s_servos[index].current_angle;
    speed = (float)s_speeds[index];
    if (speed > s_servos[index].max_speed_deg_s)
    {
      speed = s_servos[index].max_speed_deg_s;
    }
    step = speed * 0.02f;
    if (delta > step) { s_servos[index].current_angle += step; }
    else if (delta < -step) { s_servos[index].current_angle -= step; }
    else if ((delta > 0.01f) || (delta < -0.01f))
    {
      s_servos[index].current_angle = s_servos[index].target_angle;
    }

    if (s_output_active[index] &&
        (s_needs_output[index] || (delta > 0.01f) || (delta < -0.01f)))
    {
      s_pulses[index] = ServoManager_GetPulseUs((uint8_t)(index + 1U),
                                                 s_servos[index].current_angle);
      off_count = (uint16_t)((((uint32_t)s_pulses[index] * 4096U) +
                              10000U) / 20000U);
      if (off_count > 4095U) { off_count = 4095U; }
      if (PCA9685_SetPWM(s_servos[index].channel, 0U, off_count) != HAL_OK)
      {
        overall = HAL_ERROR;
      }
      s_needs_output[index] = false;
    }
  }
  ServoManager_PublishStatus();
  return overall;
}
