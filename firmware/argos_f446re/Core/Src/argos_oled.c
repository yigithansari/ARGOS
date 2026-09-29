#include "argos_oled.h"

#include <stdio.h>
#include <string.h>

#define OLED_SCL_PIN GPIO_PIN_6
#define OLED_SDA_PIN GPIO_PIN_7
#define OLED_PORT GPIOA
#define OLED_ADDRESS 0x3CU
#define OLED_REFRESH_MS 500U

static uint8_t oled_ready;
static uint8_t network_connected;
static char network_mode[4] = "AP";
static char network_address[16] = "192.168.4.1";
static uint8_t imu_ok;
static uint8_t tof_left_ok;
static uint8_t tof_right_ok;
static uint16_t tof_left_mm;
static uint16_t tof_right_mm;
static int16_t motor_left;
static int16_t motor_right;
static char safety_text[13] = "STOP";
static uint32_t last_refresh_ms;
static uint8_t dirty = 1U;

static void Oled_Delay(void)
{
  for (volatile uint32_t i = 0U; i < 22U; ++i)
  {
    __NOP();
  }
}

static void Oled_Sda(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OLED_PORT, OLED_SDA_PIN, state);
}

static void Oled_Scl(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OLED_PORT, OLED_SCL_PIN, state);
}

static void Oled_Start(void)
{
  Oled_Sda(GPIO_PIN_SET);
  Oled_Scl(GPIO_PIN_SET);
  Oled_Delay();
  Oled_Sda(GPIO_PIN_RESET);
  Oled_Delay();
  Oled_Scl(GPIO_PIN_RESET);
}

static void Oled_Stop(void)
{
  Oled_Sda(GPIO_PIN_RESET);
  Oled_Scl(GPIO_PIN_SET);
  Oled_Delay();
  Oled_Sda(GPIO_PIN_SET);
  Oled_Delay();
}

static void Oled_WriteByte(uint8_t value)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    Oled_Sda((value & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Oled_Scl(GPIO_PIN_SET);
    Oled_Delay();
    Oled_Scl(GPIO_PIN_RESET);
  }

  Oled_Sda(GPIO_PIN_SET);
  Oled_Scl(GPIO_PIN_SET);
  Oled_Delay();
  Oled_Scl(GPIO_PIN_RESET);
}

static void Oled_Command(uint8_t command)
{
  Oled_Start();
  Oled_WriteByte((uint8_t)(OLED_ADDRESS << 1U));
  Oled_WriteByte(0x00U);
  Oled_WriteByte(command);
  Oled_Stop();
}

static void Oled_Glyph(char character, uint8_t glyph[5])
{
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static const uint8_t letters[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
  };
  static const uint8_t digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
    {0x06,0x49,0x49,0x29,0x1E}
  };

  memset(glyph, 0, 5U);
  const char *letter = strchr(alphabet, character);
  if (letter != NULL)
  {
    memcpy(glyph, letters[letter - alphabet], 5U);
  }
  else if ((character >= '0') && (character <= '9'))
  {
    memcpy(glyph, digits[character - '0'], 5U);
  }
  else if (character == '-') { glyph[0]=0x08; glyph[1]=0x08; glyph[2]=0x08; glyph[3]=0x08; glyph[4]=0x08; }
  else if (character == ':') { glyph[1]=0x36; glyph[2]=0x36; }
  else if (character == '.') { glyph[2]=0x60; glyph[3]=0x60; }
  else if (character == '+') { glyph[1]=0x08; glyph[2]=0x1C; glyph[3]=0x08; }
  else if (character == '/') { glyph[0]=0x20; glyph[1]=0x10; glyph[2]=0x08; glyph[3]=0x04; glyph[4]=0x02; }
  else if (character == '%') { glyph[0]=0x63; glyph[1]=0x13; glyph[2]=0x08; glyph[3]=0x64; glyph[4]=0x63; }
}

static void Oled_WriteRow(uint8_t page, const char *text)
{
  uint8_t glyph[5];
  uint8_t columns = 0U;

  Oled_Command((uint8_t)(0xB0U + page));
  Oled_Command(0x00U);
  Oled_Command(0x10U);
  Oled_Start();
  Oled_WriteByte((uint8_t)(OLED_ADDRESS << 1U));
  Oled_WriteByte(0x40U);

  while ((*text != '\0') && (columns <= 120U))
  {
    char character = *text++;
    if ((character >= 'a') && (character <= 'z'))
    {
      character = (char)(character - ('a' - 'A'));
    }
    Oled_Glyph(character, glyph);
    for (uint8_t i = 0U; i < 5U; ++i)
    {
      Oled_WriteByte(glyph[i]);
    }
    Oled_WriteByte(0x00U);
    columns = (uint8_t)(columns + 6U);
  }
  while (columns < 128U)
  {
    Oled_WriteByte(0x00U);
    columns++;
  }
  Oled_Stop();
}

static void Oled_Render(void)
{
  char line[22];
  Oled_WriteRow(0U, "ARGOS");
  Oled_WriteRow(1U, network_connected ? "WIFI CLIENT ONLINE" : "WIFI WAITING");
  snprintf(line, sizeof(line), "%s %s", network_mode, network_address);
  Oled_WriteRow(2U, line);
  snprintf(line, sizeof(line), "SAFE %s", safety_text);
  Oled_WriteRow(3U, line);
  snprintf(line, sizeof(line), "MOTOR L%+d R%+d", motor_left, motor_right);
  Oled_WriteRow(4U, line);
  snprintf(line, sizeof(line), "TOF L%s%u", tof_left_ok ? "" : "-", tof_left_ok ? tof_left_mm : 0U);
  Oled_WriteRow(5U, line);
  snprintf(line, sizeof(line), "TOF R%s%u", tof_right_ok ? "" : "-", tof_right_ok ? tof_right_mm : 0U);
  Oled_WriteRow(6U, line);
  Oled_WriteRow(7U, imu_ok ? "IMU OK  UART 115200" : "IMU WAIT UART 115200");
}

void Argos_Oled_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_PORT, &gpio);
  Oled_Scl(GPIO_PIN_SET);
  Oled_Sda(GPIO_PIN_SET);
  HAL_Delay(50U);

  const uint8_t init[] = {0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x02,
                          0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF};
  for (uint32_t i = 0U; i < sizeof(init); ++i)
  {
    Oled_Command(init[i]);
  }
  oled_ready = 1U;
  Oled_Render();
}

void Argos_Oled_Task(void)
{
  if ((oled_ready != 0U) && ((dirty != 0U) || ((HAL_GetTick() - last_refresh_ms) >= OLED_REFRESH_MS)))
  {
    last_refresh_ms = HAL_GetTick();
    dirty = 0U;
    Oled_Render();
  }
}

void Argos_Oled_SetNetwork(uint8_t connected) { network_connected = connected ? 1U : 0U; dirty = 1U; }

void Argos_Oled_SetNetworkInfo(const char *mode, const char *address)
{
  snprintf(network_mode, sizeof(network_mode), "%s", mode);
  snprintf(network_address, sizeof(network_address), "%s", address);
  dirty = 1U;
}

void Argos_Oled_SetImu(uint8_t ok) { imu_ok = ok ? 1U : 0U; dirty = 1U; }

void Argos_Oled_SetTof(uint16_t left_mm, uint8_t left_ok, uint16_t right_mm, uint8_t right_ok)
{
  tof_left_mm = left_mm;
  tof_right_mm = right_mm;
  tof_left_ok = left_ok ? 1U : 0U;
  tof_right_ok = right_ok ? 1U : 0U;
  dirty = 1U;
}

void Argos_Oled_SetMotors(int16_t left_percent, int16_t right_percent, const char *safety)
{
  motor_left = left_percent;
  motor_right = right_percent;
  strncpy(safety_text, safety, sizeof(safety_text) - 1U);
  safety_text[sizeof(safety_text) - 1U] = '\0';
  dirty = 1U;
}
