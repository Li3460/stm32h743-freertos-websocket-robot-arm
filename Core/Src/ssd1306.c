#include "ssd1306.h"

#include "fonts.h"
#include <string.h>

#define SSD1306_ADDRESS_PRIMARY   (0x3CU << 1U)
#define SSD1306_ADDRESS_FALLBACK  (0x3DU << 1U)
#define SSD1306_TIMEOUT_MS        50U

static I2C_HandleTypeDef *s_i2c;
static uint16_t s_address;
static uint8_t s_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8U];

static HAL_StatusTypeDef SSD1306_Command(uint8_t command)
{
  uint8_t data[2] = {0x00U, command};

  return HAL_I2C_Master_Transmit(s_i2c, s_address, data, sizeof(data),
                                 SSD1306_TIMEOUT_MS);
}

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c)
{
  static const uint8_t commands[] =
  {
    0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
    0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
    0x40, 0x8D, 0x14, 0xAF
  };
  uint8_t index;

  if (i2c == NULL) { return HAL_ERROR; }
  s_i2c = i2c;
  s_address = SSD1306_ADDRESS_PRIMARY;
  if (HAL_I2C_IsDeviceReady(s_i2c, s_address, 2U,
                            SSD1306_TIMEOUT_MS) != HAL_OK)
  {
    s_address = SSD1306_ADDRESS_FALLBACK;
    if (HAL_I2C_IsDeviceReady(s_i2c, s_address, 2U,
                              SSD1306_TIMEOUT_MS) != HAL_OK)
    {
      s_address = 0U;
      return HAL_ERROR;
    }
  }
  for (index = 0U; index < sizeof(commands); index++)
  {
    if (SSD1306_Command(commands[index]) != HAL_OK) { return HAL_ERROR; }
  }
  SSD1306_ClearBuffer();
  return SSD1306_Update();
}

void SSD1306_ClearBuffer(void)
{
  (void)memset(s_buffer, 0, sizeof(s_buffer));
}

void SSD1306_WriteLine(uint8_t line, const char *text)
{
  uint8_t glyph[5];
  uint8_t column;
  uint8_t x = 0U;
  uint16_t page_offset;

  if ((text == NULL) || (line >= 4U)) { return; }
  page_offset = (uint16_t)line * 2U * SSD1306_WIDTH;
  while ((*text != '\0') && ((uint16_t)x + 5U < SSD1306_WIDTH))
  {
    Fonts_Get5x7(*text++, glyph);
    for (column = 0U; column < 5U; column++)
    {
      s_buffer[page_offset + x + column] = glyph[column];
    }
    s_buffer[page_offset + x + 5U] = 0U;
    x = (uint8_t)(x + 6U);
  }
}

HAL_StatusTypeDef SSD1306_Update(void)
{
  uint8_t page;

  if ((s_i2c == NULL) || (s_address == 0U)) { return HAL_ERROR; }
  for (page = 0U; page < 8U; page++)
  {
    if ((SSD1306_Command((uint8_t)(0xB0U + page)) != HAL_OK) ||
        (SSD1306_Command(0x00U) != HAL_OK) ||
        (SSD1306_Command(0x10U) != HAL_OK))
    {
      return HAL_ERROR;
    }
    if (HAL_I2C_Mem_Write(s_i2c, s_address, 0x40U,
                          I2C_MEMADD_SIZE_8BIT,
                          &s_buffer[(uint16_t)page * SSD1306_WIDTH],
                          SSD1306_WIDTH, SSD1306_TIMEOUT_MS) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

uint16_t SSD1306_GetAddress(void)
{
  return s_address;
}
