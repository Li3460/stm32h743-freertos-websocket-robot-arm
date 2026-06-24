#include "system_status.h"

#include "task.h"
#include <string.h>

static SystemStatus_t s_status;
static SemaphoreHandle_t s_status_mutex;

static void Status_Lock(void)
{
  if ((s_status_mutex != NULL) &&
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
  {
    (void)xSemaphoreTake(s_status_mutex, portMAX_DELAY);
  }
}

static void Status_Unlock(void)
{
  if ((s_status_mutex != NULL) &&
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
  {
    (void)xSemaphoreGive(s_status_mutex);
  }
}

static void Status_CopyText(char *destination, size_t capacity, const char *source)
{
  size_t length = 0U;

  if ((destination == NULL) || (capacity == 0U))
  {
    return;
  }
  if (source != NULL)
  {
    while ((length + 1U < capacity) && (source[length] != '\0'))
    {
      length++;
    }
    (void)memcpy(destination, source, length);
  }
  destination[length] = '\0';
}

void SystemStatus_Reset(uint32_t now_tick)
{
  uint8_t index;

  (void)memset(&s_status, 0, sizeof(s_status));
  s_status.network_state = NET_BOOT;
  s_status.last_heartbeat_tick = now_tick;
  s_status.last_servo_id = 1U;
  for (index = 0U; index < SYSTEM_STATUS_SERVO_COUNT; index++)
  {
    s_status.servo_angles[index] = (index == 5U) ? 30 : 90;
    s_status.servo_targets[index] = s_status.servo_angles[index];
  }
  s_status.last_target_angle = 90;
  s_status.last_current_angle = 90;
  Status_CopyText(s_status.last_error, sizeof(s_status.last_error), "NONE");
}

void SystemStatus_AttachMutex(SemaphoreHandle_t mutex)
{
  s_status_mutex = mutex;
}

void SystemStatus_GetSnapshot(SystemStatus_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }
  Status_Lock();
  *snapshot = s_status;
  Status_Unlock();
}

void SystemStatus_SetNetwork(NetworkState_t state, const char *ip_address)
{
  Status_Lock();
  s_status.network_state = state;
  if (ip_address != NULL)
  {
    Status_CopyText(s_status.ip_address, sizeof(s_status.ip_address), ip_address);
  }
  if ((state == NET_FAILED) || (state == NET_LOST) || (state == NET_BOOT))
  {
    s_status.http_ready = false;
    s_status.websocket_ready = false;
    s_status.websocket_clients = 0U;
  }
  Status_Unlock();
}

void SystemStatus_SetHttpReady(bool ready)
{
  Status_Lock();
  s_status.http_ready = ready;
  Status_Unlock();
}

void SystemStatus_SetWebSocketReady(bool ready)
{
  Status_Lock();
  s_status.websocket_ready = ready;
  Status_Unlock();
}

void SystemStatus_SetWebSocketClients(uint8_t clients)
{
  Status_Lock();
  s_status.websocket_clients = clients;
  Status_Unlock();
}

void SystemStatus_Heartbeat(uint32_t now_tick)
{
  Status_Lock();
  s_status.last_heartbeat_tick = now_tick;
  s_status.link_alive = true;
  Status_Unlock();
}

void SystemStatus_SetLinkAlive(bool alive)
{
  Status_Lock();
  s_status.link_alive = alive;
  Status_Unlock();
}

void SystemStatus_SetOledReady(bool ready)
{
  Status_Lock();
  s_status.oled_ready = ready;
  Status_Unlock();
}

void SystemStatus_SetPca9685Ready(bool ready)
{
  Status_Lock();
  s_status.pca9685_ready = ready;
  Status_Unlock();
}

void SystemStatus_RequestStop(uint32_t seq)
{
  Status_Lock();
  s_status.stop_requested = true;
  s_status.stop_seq = seq;
  Status_Unlock();
}

bool SystemStatus_ConsumeStopRequest(uint32_t *seq)
{
  bool requested;

  Status_Lock();
  requested = s_status.stop_requested;
  if (requested && (seq != NULL))
  {
    *seq = s_status.stop_seq;
  }
  s_status.stop_requested = false;
  Status_Unlock();
  return requested;
}

void SystemStatus_SetSafetyStop(bool stopped, const char *reason)
{
  Status_Lock();
  s_status.safety_stop = stopped;
  if (stopped && (reason != NULL))
  {
    Status_CopyText(s_status.last_error, sizeof(s_status.last_error), reason);
  }
  Status_Unlock();
}

void SystemStatus_SetServoSnapshot(const int16_t angles[SYSTEM_STATUS_SERVO_COUNT],
                                   const int16_t targets[SYSTEM_STATUS_SERVO_COUNT],
                                   const uint16_t pulses[SYSTEM_STATUS_SERVO_COUNT],
                                   bool moving,
                                   uint8_t last_servo_id)
{
  uint8_t index;

  if ((angles == NULL) || (targets == NULL) || (pulses == NULL))
  {
    return;
  }
  Status_Lock();
  for (index = 0U; index < SYSTEM_STATUS_SERVO_COUNT; index++)
  {
    s_status.servo_angles[index] = angles[index];
    s_status.servo_targets[index] = targets[index];
    s_status.servo_pulses[index] = pulses[index];
  }
  s_status.servo_moving = moving;
  if ((last_servo_id >= 1U) && (last_servo_id <= SYSTEM_STATUS_SERVO_COUNT))
  {
    s_status.last_servo_id = last_servo_id;
  }
  index = (uint8_t)(s_status.last_servo_id - 1U);
  s_status.last_current_angle = s_status.servo_angles[index];
  s_status.last_target_angle = s_status.servo_targets[index];
  s_status.last_pulse_us = s_status.servo_pulses[index];
  Status_Unlock();
}

void SystemStatus_Increment(SystemStatusCounter_t counter, uint32_t amount)
{
  Status_Lock();
  switch (counter)
  {
    case STATUS_COUNTER_CRC:           s_status.crc_error_count += amount; break;
    case STATUS_COUNTER_UART_OVERFLOW: s_status.uart_overflow_count += amount; break;
    case STATUS_COUNTER_QUEUE_FULL:    s_status.queue_full_count += amount; break;
    case STATUS_COUNTER_I2C:           s_status.i2c_error_count += amount; break;
    case STATUS_COUNTER_UART:          s_status.uart_error_count += amount; break;
    case STATUS_COUNTER_PCA9685:       s_status.pca9685_error_count += amount; break;
    case STATUS_COUNTER_OLED:          s_status.oled_error_count += amount; break;
    case STATUS_COUNTER_LOW_STACK:     s_status.low_stack_count += amount; break;
    default: break;
  }
  Status_Unlock();
}

void SystemStatus_SetError(const char *error_text)
{
  Status_Lock();
  Status_CopyText(s_status.last_error, sizeof(s_status.last_error), error_text);
  Status_Unlock();
}
