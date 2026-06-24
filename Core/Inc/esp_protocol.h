#ifndef ESP_PROTOCOL_H
#define ESP_PROTOCOL_H

#include "servo_manager.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESP_PROTOCOL_MAX_FRAME    160U
#define ESP_PROTOCOL_MAX_PAYLOAD  144U

typedef enum
{
  ESP_ERROR_NONE = 0,
  ESP_ERROR_CRC,
  ESP_ERROR_FORMAT,
  ESP_ERROR_UNKNOWN_COMMAND,
  ESP_ERROR_INVALID_ID,
  ESP_ERROR_ANGLE_RANGE,
  ESP_ERROR_SPEED_RANGE,
  ESP_ERROR_BUFFER_OVERFLOW,
  ESP_ERROR_QUEUE_FULL,
  ESP_ERROR_I2C,
  ESP_ERROR_PCA9685
} EspErrorCode_t;

typedef enum
{
  ESP_RX_NONE = 0,
  ESP_RX_SERVO_COMMAND,
  ESP_RX_HEARTBEAT,
  ESP_RX_QUERY,
  ESP_RX_NET_BOOT,
  ESP_RX_NET_CONNECTING,
  ESP_RX_NET_CONNECTED,
  ESP_RX_NET_FAILED,
  ESP_RX_NET_LOST,
  ESP_RX_HTTP_READY,
  ESP_RX_WS_READY,
  ESP_RX_WS_CLIENTS
} EspRxType_t;

typedef struct
{
  EspRxType_t type;
  ServoCommand_t servo;
  uint32_t seq;
  uint8_t clients;
  uint16_t port;
  char ip_address[16];
} EspRxPacket_t;

typedef enum
{
  ESP_TX_READY = 0,
  ESP_TX_ACK_OK,
  ESP_TX_ACK_ERROR,
  ESP_TX_STATE,
  ESP_TX_STACK_WARNING
} EspTxMessageType_t;

typedef struct
{
  EspTxMessageType_t type;
  uint32_t seq;
  uint8_t servo_id;
  int16_t angle;
  uint16_t pulse_us;
  EspErrorCode_t error;
  int16_t angles[SERVO_COUNT];
  bool moving;
  char task_name[16];
  uint16_t stack_words;
} EspTxMessage_t;

typedef struct
{
  char data[ESP_PROTOCOL_MAX_FRAME + 1U];
  uint16_t length;
  bool receiving;
  bool overflow;
} EspFrameCollector_t;

typedef enum
{
  ESP_COLLECT_NONE = 0,
  ESP_COLLECT_FRAME,
  ESP_COLLECT_OVERFLOW
} EspCollectResult_t;

void ESPProtocol_CollectorInit(EspFrameCollector_t *collector);
EspCollectResult_t ESPProtocol_CollectByte(EspFrameCollector_t *collector,
                                          uint8_t byte);
EspErrorCode_t ESPProtocol_ParseFrame(const char *frame, size_t frame_length,
                                      EspRxPacket_t *packet);
const char *ESPProtocol_ErrorText(EspErrorCode_t error);
size_t ESPProtocol_FormatTxFrame(const EspTxMessage_t *message,
                                 char *output, size_t output_size);

#endif /* ESP_PROTOCOL_H */
