#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>
#include <stdint.h>

#define SYSTEM_STATUS_SERVO_COUNT 6U

typedef enum
{
  NET_BOOT = 0,
  NET_CONNECTING,
  NET_CONNECTED,
  NET_FAILED,
  NET_LOST
} NetworkState_t;

typedef enum
{
  STATUS_COUNTER_CRC = 0,
  STATUS_COUNTER_UART_OVERFLOW,
  STATUS_COUNTER_QUEUE_FULL,
  STATUS_COUNTER_I2C,
  STATUS_COUNTER_UART,
  STATUS_COUNTER_PCA9685,
  STATUS_COUNTER_OLED,
  STATUS_COUNTER_LOW_STACK
} SystemStatusCounter_t;

typedef struct
{
  NetworkState_t network_state;
  bool http_ready;
  bool websocket_ready;
  uint8_t websocket_clients;
  bool link_alive;
  bool servo_moving;
  bool oled_ready;
  bool pca9685_ready;
  char ip_address[16];
  uint32_t last_heartbeat_tick;
  uint32_t crc_error_count;
  uint32_t uart_overflow_count;
  uint32_t queue_full_count;
  uint32_t i2c_error_count;

  uint32_t uart_error_count;
  uint32_t pca9685_error_count;
  uint32_t oled_error_count;
  uint32_t low_stack_count;
  bool safety_stop;
  bool stop_requested;
  uint32_t stop_seq;
  uint8_t last_servo_id;
  int16_t last_target_angle;
  int16_t last_current_angle;
  uint16_t last_pulse_us;
  int16_t servo_angles[SYSTEM_STATUS_SERVO_COUNT];
  int16_t servo_targets[SYSTEM_STATUS_SERVO_COUNT];
  uint16_t servo_pulses[SYSTEM_STATUS_SERVO_COUNT];
  char last_error[24];
} SystemStatus_t;

void SystemStatus_Reset(uint32_t now_tick);
void SystemStatus_AttachMutex(SemaphoreHandle_t mutex);
void SystemStatus_GetSnapshot(SystemStatus_t *snapshot);
void SystemStatus_SetNetwork(NetworkState_t state, const char *ip_address);
void SystemStatus_SetHttpReady(bool ready);
void SystemStatus_SetWebSocketReady(bool ready);
void SystemStatus_SetWebSocketClients(uint8_t clients);
void SystemStatus_Heartbeat(uint32_t now_tick);
void SystemStatus_SetLinkAlive(bool alive);
void SystemStatus_SetOledReady(bool ready);
void SystemStatus_SetPca9685Ready(bool ready);
void SystemStatus_RequestStop(uint32_t seq);
bool SystemStatus_ConsumeStopRequest(uint32_t *seq);
void SystemStatus_SetSafetyStop(bool stopped, const char *reason);
void SystemStatus_SetServoSnapshot(const int16_t angles[SYSTEM_STATUS_SERVO_COUNT],
                                   const int16_t targets[SYSTEM_STATUS_SERVO_COUNT],
                                   const uint16_t pulses[SYSTEM_STATUS_SERVO_COUNT],
                                   bool moving,
                                   uint8_t last_servo_id);
void SystemStatus_Increment(SystemStatusCounter_t counter, uint32_t amount);
void SystemStatus_SetError(const char *error_text);

#endif /* SYSTEM_STATUS_H */
