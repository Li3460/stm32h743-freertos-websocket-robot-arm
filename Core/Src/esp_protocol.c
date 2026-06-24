#include "esp_protocol.h"

#include <string.h>

#define ESP_MAX_FIELDS 12U

static bool Protocol_ParseLong(const char *text, long minimum, long maximum,
                               long *value)
{
  uint32_t magnitude = 0U;
  uint32_t digit;
  uint32_t limit;
  bool negative = false;

  if ((text == NULL) || (*text == '\0') || (value == NULL)) { return false; }
  if (*text == '-')
  {
    negative = true;
    text++;
    if (*text == '\0') { return false; }
  }
  limit = negative ? (uint32_t)(-(minimum + 1L)) + 1U : (uint32_t)maximum;
  while (*text != '\0')
  {
    if ((*text < '0') || (*text > '9')) { return false; }
    digit = (uint32_t)(*text - '0');
    if ((digit > limit) || (magnitude > ((limit - digit) / 10U)))
    {
      return false;
    }
    magnitude = magnitude * 10U + digit;
    text++;
  }
  *value = negative ? -(long)magnitude : (long)magnitude;
  return (*value >= minimum) && (*value <= maximum);
}

static bool Protocol_ParseU32(const char *text, uint32_t *value)
{
  uint32_t parsed = 0U;
  uint32_t digit;

  if ((text == NULL) || (*text == '\0') || (value == NULL)) { return false; }
  while (*text != '\0')
  {
    if ((*text < '0') || (*text > '9')) { return false; }
    digit = (uint32_t)(*text - '0');
    if (parsed > ((UINT32_MAX - digit) / 10U)) { return false; }
    parsed = parsed * 10U + digit;
    text++;
  }
  *value = parsed;
  return true;
}

static bool Protocol_ValidIp(const char *text)
{
  uint8_t octets = 0U;
  uint16_t value = 0U;
  uint8_t digits = 0U;

  if ((text == NULL) || (*text == '\0') || (strlen(text) > 15U))
  {
    return false;
  }
  while (true)
  {
    if ((*text >= '0') && (*text <= '9'))
    {
      value = (uint16_t)(value * 10U + (uint16_t)(*text - '0'));
      digits++;
      if ((digits > 3U) || (value > 255U)) { return false; }
    }
    else if ((*text == '.') || (*text == '\0'))
    {
      if (digits == 0U) { return false; }
      octets++;
      if (*text == '\0') { return octets == 4U; }
      value = 0U;
      digits = 0U;
    }
    else
    {
      return false;
    }
    text++;
  }
}

static uint8_t Protocol_HexNibble(char character)
{
  if ((character >= '0') && (character <= '9')) return (uint8_t)(character - '0');
  if ((character >= 'A') && (character <= 'F')) return (uint8_t)(character - 'A' + 10);
  if ((character >= 'a') && (character <= 'f')) return (uint8_t)(character - 'a' + 10);
  return 0xFFU;
}

static size_t Protocol_SplitFields(char *payload, char *fields[ESP_MAX_FIELDS])
{
  size_t count = 0U;
  char *cursor = payload;

  if ((payload == NULL) || (*payload == '\0')) { return 0U; }
  fields[count++] = cursor;
  while (*cursor != '\0')
  {
    if (*cursor == ',')
    {
      if (count >= ESP_MAX_FIELDS) { return 0U; }
      *cursor = '\0';
      fields[count++] = cursor + 1;
    }
    cursor++;
  }
  return count;
}

void ESPProtocol_CollectorInit(EspFrameCollector_t *collector)
{
  if (collector != NULL) { (void)memset(collector, 0, sizeof(*collector)); }
}

EspCollectResult_t ESPProtocol_CollectByte(EspFrameCollector_t *collector,
                                          uint8_t byte)
{
  if (collector == NULL) { return ESP_COLLECT_NONE; }
  if (byte == (uint8_t)'$')
  {
    collector->length = 0U;
    collector->overflow = false;
    collector->receiving = true;
    collector->data[collector->length++] = '$';
    return ESP_COLLECT_NONE;
  }
  if (!collector->receiving) { return ESP_COLLECT_NONE; }
  if (collector->length >= ESP_PROTOCOL_MAX_FRAME) { collector->overflow = true; }
  else { collector->data[collector->length++] = (char)byte; }
  if (byte == (uint8_t)'\n')
  {
    collector->receiving = false;
    collector->data[collector->length] = '\0';
    if (collector->overflow)
    {
      collector->length = 0U;
      collector->overflow = false;
      return ESP_COLLECT_OVERFLOW;
    }
    return ESP_COLLECT_FRAME;
  }
  return ESP_COLLECT_NONE;
}

EspErrorCode_t ESPProtocol_ParseFrame(const char *frame, size_t frame_length,
                                      EspRxPacket_t *packet)
{
  char payload[ESP_PROTOCOL_MAX_FRAME + 1U];
  char *fields[ESP_MAX_FIELDS];
  size_t star;
  size_t payload_length;
  size_t field_count;
  size_t index;
  uint8_t crc = 0U;
  uint8_t high;
  uint8_t low;
  long value;

  if ((frame == NULL) || (packet == NULL) || (frame_length < 7U) ||
      (frame_length > ESP_PROTOCOL_MAX_FRAME) || (frame[0] != '$'))
  {
    return ESP_ERROR_FORMAT;
  }
  for (star = 1U; (star < frame_length) && (frame[star] != '*'); star++) { }
  if ((star >= frame_length) || ((star + 3U) >= frame_length))
  {
    return ESP_ERROR_FORMAT;
  }
  high = Protocol_HexNibble(frame[star + 1U]);
  low = Protocol_HexNibble(frame[star + 2U]);
  if ((high > 0x0FU) || (low > 0x0FU)) { return ESP_ERROR_FORMAT; }
  if (frame[star + 3U] == '\r')
  {
    if (((star + 5U) != frame_length) || (frame[star + 4U] != '\n'))
    {
      return ESP_ERROR_FORMAT;
    }
  }
  else if ((frame[star + 3U] != '\n') || ((star + 4U) != frame_length))
  {
    return ESP_ERROR_FORMAT;
  }
  for (index = 1U; index < star; index++) { crc ^= (uint8_t)frame[index]; }
  if (crc != (uint8_t)((high << 4U) | low)) { return ESP_ERROR_CRC; }

  payload_length = star - 1U;
  (void)memcpy(payload, &frame[1], payload_length);
  payload[payload_length] = '\0';
  (void)memset(packet, 0, sizeof(*packet));
  field_count = Protocol_SplitFields(payload, fields);
  if (field_count == 0U) { return ESP_ERROR_FORMAT; }

  if (strcmp(fields[0], "SV") == 0)
  {
    if (field_count != 5U) { return ESP_ERROR_FORMAT; }
    packet->type = ESP_RX_SERVO_COMMAND;
    packet->servo.type = SERVO_CMD_SINGLE;
    if (!Protocol_ParseLong(fields[1], 1L, SERVO_COUNT, &value)) return ESP_ERROR_INVALID_ID;
    packet->servo.servo_id = (uint8_t)value;
    if (!Protocol_ParseLong(fields[2], -32768L, 32767L, &value)) return ESP_ERROR_FORMAT;
    packet->servo.angles[packet->servo.servo_id - 1U] = (int16_t)value;
    if (!Protocol_ParseLong(fields[3], 0L, 65535L, &value)) return ESP_ERROR_FORMAT;
    packet->servo.speed_deg_s = (uint16_t)value;
    if (!Protocol_ParseU32(fields[4], &packet->servo.seq)) return ESP_ERROR_FORMAT;
  }
  else if (strcmp(fields[0], "ALL") == 0)
  {
    if (field_count != 9U) { return ESP_ERROR_FORMAT; }
    packet->type = ESP_RX_SERVO_COMMAND;
    packet->servo.type = SERVO_CMD_ALL;
    for (index = 0U; index < SERVO_COUNT; index++)
    {
      if (!Protocol_ParseLong(fields[index + 1U], -32768L, 32767L, &value)) return ESP_ERROR_FORMAT;
      packet->servo.angles[index] = (int16_t)value;
    }
    if (!Protocol_ParseLong(fields[7], 0L, 65535L, &value)) return ESP_ERROR_FORMAT;
    packet->servo.speed_deg_s = (uint16_t)value;
    if (!Protocol_ParseU32(fields[8], &packet->servo.seq)) return ESP_ERROR_FORMAT;
  }
  else if ((strcmp(fields[0], "HOME") == 0) ||
           (strcmp(fields[0], "STOP") == 0))
  {
    if (field_count != 2U) { return ESP_ERROR_FORMAT; }
    packet->type = ESP_RX_SERVO_COMMAND;
    packet->servo.type = (fields[0][0] == 'H') ? SERVO_CMD_HOME : SERVO_CMD_STOP;
    packet->servo.speed_deg_s = SERVO_DEFAULT_SPEED;
    if (!Protocol_ParseU32(fields[1], &packet->servo.seq)) return ESP_ERROR_FORMAT;
  }
  else if ((strcmp(fields[0], "HB") == 0) ||
           (strcmp(fields[0], "QUERY") == 0))
  {
    if (field_count != 2U) { return ESP_ERROR_FORMAT; }
    packet->type = (fields[0][0] == 'H') ? ESP_RX_HEARTBEAT : ESP_RX_QUERY;
    if (!Protocol_ParseU32(fields[1], &packet->seq)) return ESP_ERROR_FORMAT;
  }
  else if (strcmp(fields[0], "NET") == 0)
  {
    if (field_count < 2U) { return ESP_ERROR_FORMAT; }
    if ((strcmp(fields[1], "BOOT") == 0) && (field_count == 2U)) packet->type = ESP_RX_NET_BOOT;
    else if ((strcmp(fields[1], "WIFI_CONNECTING") == 0) && (field_count == 2U)) packet->type = ESP_RX_NET_CONNECTING;
    else if ((strcmp(fields[1], "WIFI_CONNECTED") == 0) && (field_count == 3U))
    {
      if (!Protocol_ValidIp(fields[2])) { return ESP_ERROR_FORMAT; }
      packet->type = ESP_RX_NET_CONNECTED;
      (void)memcpy(packet->ip_address, fields[2], strlen(fields[2]) + 1U);
    }
    else if ((strcmp(fields[1], "WIFI_FAILED") == 0) && (field_count == 2U)) packet->type = ESP_RX_NET_FAILED;
    else if ((strcmp(fields[1], "WIFI_LOST") == 0) && (field_count == 2U)) packet->type = ESP_RX_NET_LOST;
    else if (((strcmp(fields[1], "HTTP_READY") == 0) ||
              (strcmp(fields[1], "WS_READY") == 0)) && (field_count == 3U))
    {
      if (!Protocol_ParseLong(fields[2], 1L, 65535L, &value)) return ESP_ERROR_FORMAT;
      packet->port = (uint16_t)value;
      packet->type = (fields[1][0] == 'H') ? ESP_RX_HTTP_READY : ESP_RX_WS_READY;
    }
    else if ((strcmp(fields[1], "WS_CLIENTS") == 0) && (field_count == 3U))
    {
      if (!Protocol_ParseLong(fields[2], 0L, 255L, &value)) return ESP_ERROR_FORMAT;
      packet->clients = (uint8_t)value;
      packet->type = ESP_RX_WS_CLIENTS;
    }
    else { return ESP_ERROR_UNKNOWN_COMMAND; }
  }
  else { return ESP_ERROR_UNKNOWN_COMMAND; }
  return ESP_ERROR_NONE;
}

const char *ESPProtocol_ErrorText(EspErrorCode_t error)
{
  static const char *const texts[] =
  {
    "NONE", "CRC_ERROR", "FORMAT_ERROR", "UNKNOWN_COMMAND", "INVALID_ID",
    "ANGLE_OUT_OF_RANGE", "SPEED_OUT_OF_RANGE", "BUFFER_OVERFLOW",
    "QUEUE_FULL", "I2C_ERROR", "PCA9685_ERROR"
  };
  if ((unsigned int)error >= (sizeof(texts) / sizeof(texts[0])))
  {
    return "UNKNOWN_COMMAND";
  }
  return texts[error];
}

typedef struct
{
  char *buffer;
  size_t capacity;
  size_t length;
  bool valid;
} ProtocolBuilder_t;

static void Protocol_AppendChar(ProtocolBuilder_t *builder, char value)
{
  if ((!builder->valid) || ((builder->length + 1U) >= builder->capacity))
  {
    builder->valid = false;
    return;
  }
  builder->buffer[builder->length++] = value;
  builder->buffer[builder->length] = '\0';
}

static void Protocol_AppendText(ProtocolBuilder_t *builder, const char *text)
{
  if (text == NULL) { builder->valid = false; return; }
  while (*text != '\0') { Protocol_AppendChar(builder, *text++); }
}

static void Protocol_AppendU32(ProtocolBuilder_t *builder, uint32_t value)
{
  char digits[10];
  uint8_t count = 0U;

  if (value == 0U) { Protocol_AppendChar(builder, '0'); return; }
  while ((value > 0U) && (count < sizeof(digits)))
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while (count > 0U) { Protocol_AppendChar(builder, digits[--count]); }
}

static void Protocol_AppendI32(ProtocolBuilder_t *builder, int32_t value)
{
  uint32_t magnitude;

  if (value < 0)
  {
    Protocol_AppendChar(builder, '-');
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else { magnitude = (uint32_t)value; }
  Protocol_AppendU32(builder, magnitude);
}

static void Protocol_AppendHex(ProtocolBuilder_t *builder, uint8_t value)
{
  static const char hex[] = "0123456789ABCDEF";
  Protocol_AppendChar(builder, hex[(value >> 4U) & 0x0FU]);
  Protocol_AppendChar(builder, hex[value & 0x0FU]);
}

size_t ESPProtocol_FormatTxFrame(const EspTxMessage_t *message,
                                 char *output, size_t output_size)
{
  char payload[ESP_PROTOCOL_MAX_PAYLOAD + 1U];
  ProtocolBuilder_t builder;
  size_t index;
  uint8_t crc = 0U;

  if ((message == NULL) || (output == NULL) || (output_size < 8U)) return 0U;
  builder.buffer = payload;
  builder.capacity = sizeof(payload);
  builder.length = 0U;
  builder.valid = true;
  payload[0] = '\0';
  switch (message->type)
  {
    case ESP_TX_READY:
      Protocol_AppendText(&builder, "STM32,READY");
      break;
    case ESP_TX_ACK_OK:
      Protocol_AppendText(&builder, "ACK,");
      Protocol_AppendU32(&builder, message->seq);
      Protocol_AppendChar(&builder, ',');
      Protocol_AppendU32(&builder, message->servo_id);
      Protocol_AppendChar(&builder, ',');
      Protocol_AppendI32(&builder, message->angle);
      Protocol_AppendChar(&builder, ',');
      Protocol_AppendU32(&builder, message->pulse_us);
      Protocol_AppendText(&builder, ",OK");
      break;
    case ESP_TX_ACK_ERROR:
      Protocol_AppendText(&builder, "ACK,");
      Protocol_AppendU32(&builder, message->seq);
      Protocol_AppendText(&builder, ",ERROR,");
      Protocol_AppendText(&builder, ESPProtocol_ErrorText(message->error));
      break;
    case ESP_TX_STATE:
      Protocol_AppendText(&builder, "STATE");
      for (index = 0U; index < SERVO_COUNT; index++)
      {
        Protocol_AppendChar(&builder, ',');
        Protocol_AppendI32(&builder, message->angles[index]);
      }
      Protocol_AppendChar(&builder, ',');
      Protocol_AppendU32(&builder, message->moving ? 1U : 0U);
      break;
    case ESP_TX_STACK_WARNING:
      Protocol_AppendText(&builder, "WARN,STACK,");
      Protocol_AppendText(&builder, message->task_name);
      Protocol_AppendChar(&builder, ',');
      Protocol_AppendU32(&builder, message->stack_words);
      break;
    default:
      return 0U;
  }
  if (!builder.valid) { return 0U; }
  for (index = 0U; index < builder.length; index++) { crc ^= (uint8_t)payload[index]; }

  builder.buffer = output;
  builder.capacity = output_size;
  builder.length = 0U;
  builder.valid = true;
  output[0] = '\0';
  Protocol_AppendChar(&builder, '$');
  Protocol_AppendText(&builder, payload);
  Protocol_AppendChar(&builder, '*');
  Protocol_AppendHex(&builder, crc);
  Protocol_AppendText(&builder, "\r\n");
  return builder.valid ? builder.length : 0U;
}
