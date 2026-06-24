#include "oled_status.h"

#include "ssd1306.h"

static uint8_t OLEDStatus_Length(const char *text, uint8_t maximum)
{
  uint8_t length = 0U;
  while ((length < maximum) && (text[length] != '\0')) { length++; }
  return length;
}

static void OLEDStatus_Copy(char *destination, uint8_t capacity,
                            const char *source)
{
  uint8_t used = 0U;

  if ((destination == NULL) || (capacity == 0U)) { return; }
  while ((source != NULL) && (*source != '\0') &&
         (used < (uint8_t)(capacity - 1U)))
  {
    destination[used++] = *source++;
  }
  destination[used] = '\0';
}

static void OLEDStatus_AppendU32(char *destination, uint8_t capacity,
                                 uint32_t value)
{
  char digits[10];
  uint8_t count = 0U;
  uint8_t used = OLEDStatus_Length(destination, capacity);

  if (value == 0U) { digits[count++] = '0'; }
  while ((value > 0U) && (count < sizeof(digits)))
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while ((count > 0U) && (used < (uint8_t)(capacity - 1U)))
  {
    destination[used++] = digits[--count];
  }
  destination[used] = '\0';
}

static void OLEDStatus_AppendI32(char *destination, uint8_t capacity,
                                 int32_t value)
{
  uint8_t used;
  uint32_t magnitude;

  if (value < 0)
  {
    used = OLEDStatus_Length(destination, capacity);
    if (used < (uint8_t)(capacity - 1U))
    {
      destination[used++] = '-';
      destination[used] = '\0';
    }
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else { magnitude = (uint32_t)value; }
  OLEDStatus_AppendU32(destination, capacity, magnitude);
}

HAL_StatusTypeDef OLEDStatus_Init(I2C_HandleTypeDef *i2c)
{
  return SSD1306_Init(i2c);
}

HAL_StatusTypeDef OLEDStatus_ShowStartup(void)
{
  SSD1306_ClearBuffer();
  SSD1306_WriteLine(0U, "ARM SERVO DEBUG");
  SSD1306_WriteLine(2U, "STM32 STARTING");
  return SSD1306_Update();
}

HAL_StatusTypeDef OLEDStatus_Render(const SystemStatus_t *status,
                                    uint32_t page_counter)
{
  char line[18];

  if (status == NULL) { return HAL_ERROR; }
  SSD1306_ClearBuffer();
  if (!status->pca9685_ready)
  {
    SSD1306_WriteLine(0U, "PCA ERROR");
    SSD1306_WriteLine(2U, "SERVO DISABLED");
  }
  else if (status->safety_stop)
  {
    SSD1306_WriteLine(0U, status->last_error);
    SSD1306_WriteLine(2U, "TARGETS HELD");
  }
  else if ((status->crc_error_count > 0U) && ((page_counter % 10U) == 3U))
  {
    SSD1306_WriteLine(0U, "CRC ERROR");
    OLEDStatus_Copy(line, sizeof(line), "COUNT: ");
    OLEDStatus_AppendU32(line, sizeof(line), status->crc_error_count);
    SSD1306_WriteLine(2U, line);
  }
  else if ((page_counter % 5U) == 4U)
  {
    OLEDStatus_Copy(line, sizeof(line), "SERVO: ");
    OLEDStatus_AppendU32(line, sizeof(line), status->last_servo_id);
    SSD1306_WriteLine(0U, line);
    OLEDStatus_Copy(line, sizeof(line), "TARGET: ");
    OLEDStatus_AppendI32(line, sizeof(line), status->last_target_angle);
    SSD1306_WriteLine(1U, line);
    OLEDStatus_Copy(line, sizeof(line), "CURRENT: ");
    OLEDStatus_AppendI32(line, sizeof(line), status->last_current_angle);
    SSD1306_WriteLine(2U, line);
    OLEDStatus_Copy(line, sizeof(line), "PWM: ");
    OLEDStatus_AppendU32(line, sizeof(line), status->last_pulse_us);
    SSD1306_WriteLine(3U, line);
  }
  else if (status->network_state == NET_BOOT)
  {
    SSD1306_WriteLine(0U, "ESP8266 BOOT");
    SSD1306_WriteLine(2U, "UART READY");
  }
  else if (status->network_state == NET_CONNECTING)
  {
    SSD1306_WriteLine(0U, "WIFI CONNECTING");
    SSD1306_WriteLine(2U, "SSID: iPhone14");
  }
  else if (status->network_state == NET_FAILED)
  {
    SSD1306_WriteLine(0U, "WIFI FAILED");
    SSD1306_WriteLine(2U, status->last_error);
  }
  else if (status->network_state == NET_LOST)
  {
    SSD1306_WriteLine(0U, "LINK LOST");
    SSD1306_WriteLine(2U, "WIFI LOST");
  }
  else if (!status->websocket_ready)
  {
    SSD1306_WriteLine(0U, "WIFI CONNECTED");
    SSD1306_WriteLine(1U, "IP:");
    SSD1306_WriteLine(2U, status->ip_address);
    if (status->http_ready) { SSD1306_WriteLine(3U, "HTTP READY: 80"); }
  }
  else
  {
    SSD1306_WriteLine(0U, status->websocket_clients > 0U ?
                      "WS ONLINE" : "WS WAITING");
    OLEDStatus_Copy(line, sizeof(line), "CLIENTS: ");
    OLEDStatus_AppendU32(line, sizeof(line), status->websocket_clients);
    SSD1306_WriteLine(1U, line);
    SSD1306_WriteLine(2U, "PORT: 81");
    SSD1306_WriteLine(3U, status->ip_address);
  }
  return SSD1306_Update();
}
