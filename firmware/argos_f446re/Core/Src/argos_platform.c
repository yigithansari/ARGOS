#include "argos_platform.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

static void TestPlatform_Init(void);
static void TestPlatform_Task(void);
static void TestPlatform_PrintMenu(void);
static void TestPlatform_HandleCommand(uint8_t command);
static void TestPlatform_HandleLine(char *line);
static void TestPlatform_Printf(const char *format, ...);
static void TestPlatform_PrintLine(const char *text);
static uint8_t I2C_WaitFlagSet(volatile uint32_t *reg, uint32_t flag, uint32_t timeout_ms);
static uint8_t I2C_WaitFlagClear(volatile uint32_t *reg, uint32_t flag, uint32_t timeout_ms);
static void I2C1_BusInit(void);
static uint8_t I2C1_StartAndAddress(uint8_t address, uint8_t read);
static uint8_t I2C1_Probe(uint8_t address);
static uint8_t I2C1_WriteReg(uint8_t address, uint8_t reg, uint8_t value);
static uint8_t I2C1_WriteReg16(uint8_t address, uint8_t reg, uint16_t value);
static uint8_t I2C1_ReadReg(uint8_t address, uint8_t reg, uint8_t *value);
static uint8_t I2C1_ReadReg16(uint8_t address, uint8_t reg, uint16_t *value);
static uint8_t MPU6050_ReadWord(uint8_t reg, int16_t *value);
static int32_t RawAccelToMg(int16_t raw);
static int32_t RawGyroToCentiDps(int16_t raw);
static int32_t RawTempToCentiC(int16_t raw);
static void Run_I2CScan(void);
static void Run_MPU6050Test(void);
static void Run_MPU6050Calibrate(void);
static void ToF_SetXshut(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
static void ToF_AssignAddresses(void);
static uint8_t VL53L0X_ApplyTuning(uint8_t address);
static uint8_t VL53L0X_BasicInit(uint8_t address);
static uint8_t VL53L0X_ReadRangeMm(uint8_t address, uint16_t *range_mm);
static void Run_ToFTest(void);
static void Run_ToFSingleTest(const char *name, uint8_t address, GPIO_TypeDef *int_port, uint16_t int_pin);
static void Run_ToFStreamSample(void);
static const char *PinLevel(GPIO_TypeDef *port, uint16_t pin);
static const char *PinOnOff(GPIO_TypeDef *port, uint16_t pin);
static const char *PinSetReset(GPIO_TypeDef *port, uint16_t pin);
static uint8_t Encoder_ReadState(GPIO_TypeDef *port, uint16_t pin_a, uint16_t pin_b);
static void Encoder_UpdateOne(volatile uint8_t *old_state, uint8_t new_state, volatile int32_t *count);
static void Encoder_ReconfigureInputs(uint32_t pull);
static uint16_t PercentToPwm(int16_t percent);
static int16_t ClampPercent(int16_t percent);
static void Motor_WriteLeft(int16_t percent);
static void Motor_WriteRight(int16_t percent);
static void Motor_Init(void);
static void Motor_SetSpeeds(int16_t left_percent, int16_t right_percent);
static void Motor_Stop(void);
static void Motor_Task(void);
static void Motor_ResetEncoders(void);
static void Run_MotorStatus(void);
static void Run_EncoderRawMonitor(void);
static void Run_EncoderPullDiagnostic(void);
static void Print_ToFGPIOState(void);
static void Run_GPIOTest(void);
static void Run_AllTests(void);

static uint8_t mpu_stream_enabled = 0U;
static uint8_t tof_stream_enabled = 0U;
#define MPU6050_ADDR        0x68U
#define MPU6050_WHO_AM_I    0x75U
#define MPU6050_PWR_MGMT_1  0x6BU
#define MPU6050_ACCEL_XOUT  0x3BU
#define MPU6050_GYRO_CONFIG 0x1BU
#define MPU6050_ACC_CONFIG  0x1CU

#define VL53L0X_DEFAULT_ADDR 0x29U
#define VL53L0X_LEFT_ADDR    0x30U
#define VL53L0X_RIGHT_ADDR   0x31U
#define VL53L0X_I2C_ADDR_REG 0x8AU
#define VL53L0X_MODEL_ID_REG 0xC0U
#define VL53L0X_MODULE_ID_REG 0xC1U
#define VL53L0X_REVISION_REG 0xC2U
#define VL53L0X_SYSRANGE_START 0x00U
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR 0x0BU
#define VL53L0X_RESULT_INTERRUPT_STATUS 0x13U
#define VL53L0X_RESULT_RANGE_STATUS 0x14U
#define TOF_LEFT_YAW_DEG     -15
#define TOF_RIGHT_YAW_DEG     15
#define TOF_UP_TILT_DEG        5
#define TOF_BASELINE_MM       70
#define TOF_STREAM_PERIOD_MS  500U
#define TOF_RANGE_TIMEOUT_MS  160U
#define TOF_INTER_SENSOR_MS    60U

#define I2C_TIMEOUT_MS      20U
#define MOTOR_PWM_MAX        1000U
#define MOTOR_STATUS_PERIOD_MS 250U
#define MOTOR_COMMAND_TIMEOUT_MS 1200U
#define MOTOR_DIRECTION_CHANGE_STOP_MS 60U
#define ENCODER_RAW_MONITOR_MS 8000U
#define ENCODER_COUNTS_PER_REV 1440L

static int16_t gyro_offset_x = 0;
static int16_t gyro_offset_y = 0;
static int16_t gyro_offset_z = 0;
static uint8_t imu_identity_reported = 0U;
static uint8_t tof_addresses_assigned = 0U;
static uint8_t tof_left_ready = 0U;
static uint8_t tof_right_ready = 0U;
static uint8_t tof_left_initialized = 0U;
static uint8_t tof_right_initialized = 0U;
static int16_t left_motor_percent = 0;
static int16_t right_motor_percent = 0;
static volatile int32_t left_encoder_count = 0;
static volatile int32_t right_encoder_count = 0;
static volatile uint8_t left_encoder_state = 0U;
static volatile uint8_t right_encoder_state = 0U;
static uint8_t motor_status_stream_enabled = 1U;
static uint32_t last_motor_command_ms = 0U;
static uint8_t motor_timeout_latched = 0U;
static int32_t last_left_encoder_report = 0;
static int32_t last_right_encoder_report = 0;
static uint32_t last_motor_report_ms = 0U;

static void TestPlatform_Init(void)
{
  I2C1_BusInit();
  Motor_Init();
  Motor_Stop();
  ToF_AssignAddresses();
  TestPlatform_PrintLine("\r\nARGOS test platform hazir.");
  TestPlatform_PrintLine("MPU6050: SCL PB8/CN10-3, SDA PB9/CN10-5, INT PB10/CN10-25, AD0 GND => 0x68");
  TestPlatform_PrintLine("VL53L0X Left: XSHUT PA8, INT PC7, assigned 0x30");
  TestPlatform_PrintLine("VL53L0X Right: XSHUT PA9, INT PA10, assigned 0x31");
  TestPlatform_PrintLine("Motors: LEFT PC6/PC8, RIGHT PB14/PB15, encoders L PC4/PC5 R PC2/PC3");
  TestPlatform_PrintMenu();
}

static void TestPlatform_Task(void)
{
  uint8_t command;
  static char line[40];
  static uint8_t line_len = 0U;
  static uint32_t last_stream_ms = 0U;
  static uint32_t last_tof_stream_ms = 0U;
  static uint32_t last_motor_status_ms = 0U;

  Motor_Task();

  if (HAL_UART_Receive(&huart2, &command, 1U, 0U) == HAL_OK)
  {
    if ((command == '\r') || (command == '\n'))
    {
      if (line_len > 0U)
      {
        line[line_len] = '\0';
        TestPlatform_HandleLine(line);
        line_len = 0U;
      }
    }
    else if (line_len < (sizeof(line) - 1U))
    {
      line[line_len] = (char)command;
      line_len++;
    }
  }

  if ((mpu_stream_enabled != 0U) && ((HAL_GetTick() - last_stream_ms) >= 250U))
  {
    last_stream_ms = HAL_GetTick();
    Run_MPU6050Test();
  }

  if ((tof_stream_enabled != 0U) && ((HAL_GetTick() - last_tof_stream_ms) >= TOF_STREAM_PERIOD_MS))
  {
    last_tof_stream_ms = HAL_GetTick();
    Run_ToFStreamSample();
  }

  if ((motor_status_stream_enabled != 0U) && ((HAL_GetTick() - last_motor_status_ms) >= MOTOR_STATUS_PERIOD_MS))
  {
    last_motor_status_ms = HAL_GetTick();
    Run_MotorStatus();
  }
}

static void TestPlatform_PrintMenu(void)
{
  TestPlatform_PrintLine("\r\nKomutlar:");
  TestPlatform_PrintLine("  h : bu menuyu yazdir");
  TestPlatform_PrintLine("  s : I2C adres taramasi");
  TestPlatform_PrintLine("  m : MPU6050 tek okuma");
  TestPlatform_PrintLine("  M : MPU6050 surekli akis ac/kapat");
  TestPlatform_PrintLine("  c : gyro offset kalibrasyonu (kart sabit dursun)");
  TestPlatform_PrintLine("  T : ToF XSHUT reset ve adres atama");
  TestPlatform_PrintLine("  t : iki ToF sensoru test et");
  TestPlatform_PrintLine("  D : ToF surekli mesafe akisi ac/kapat");
  TestPlatform_PrintLine("  L : sol ToF test (0x30)");
  TestPlatform_PrintLine("  R : sag ToF test (0x31)");
  TestPlatform_PrintLine("  g : GPIO/LED/buton/MPU INT testi");
  TestPlatform_PrintLine("  E : motor/encoder durumunu yazdir");
  TestPlatform_PrintLine("  Q : encoder raw A/B izle (8 sn, sonra motor akisini durdurur)");
  TestPlatform_PrintLine("  V : encoder pull-up/pull-down teshisi");
  TestPlatform_PrintLine("  Z : encoder sayaclarini sifirla");
  TestPlatform_PrintLine("  P L R : motor hizi yuzde, ornek P 50 -50");
  TestPlatform_PrintLine("  x : motor stop");
  TestPlatform_PrintLine("  f/b : iki motor ileri/geri %50");
  TestPlatform_PrintLine("  </> : sola/saga donus %50");
  TestPlatform_PrintLine("  a : hepsini sirayla test et");
  TestPlatform_PrintLine("UART: 115200 8N1");
}

static void TestPlatform_HandleLine(char *line)
{
  int left;
  int right;

  TestPlatform_Printf("\r\n> %s\r\n", line);

  if ((line[0] == 'P') || (line[0] == 'p'))
  {
    if (sscanf(line + 1, "%d %d", &left, &right) == 2)
    {
      Motor_SetSpeeds((int16_t)left, (int16_t)right);
      Run_MotorStatus();
    }
    else
    {
      TestPlatform_PrintLine("Motor komutu hatali. Ornek: P 50 -50");
    }
    return;
  }

  TestPlatform_HandleCommand((uint8_t)line[0]);
}

static void TestPlatform_HandleCommand(uint8_t command)
{
  switch (command)
  {
    case 'h':
    case '?':
      TestPlatform_PrintMenu();
      break;

    case 's':
      Run_I2CScan();
      break;

    case 'm':
      Run_MPU6050Test();
      break;

    case 'M':
      mpu_stream_enabled = (mpu_stream_enabled == 0U) ? 1U : 0U;
      TestPlatform_Printf("MPU6050 surekli akis: %s\r\n", (mpu_stream_enabled != 0U) ? "ACIK" : "KAPALI");
      break;

    case 'c':
      Run_MPU6050Calibrate();
      break;

    case 'T':
      ToF_AssignAddresses();
      break;

    case 't':
      Run_ToFTest();
      break;

    case 'D':
      tof_stream_enabled = (tof_stream_enabled == 0U) ? 1U : 0U;
      if (tof_stream_enabled != 0U)
      {
        ToF_AssignAddresses();
      }
      TestPlatform_Printf("ToF surekli akis: %s\r\n", (tof_stream_enabled != 0U) ? "ACIK" : "KAPALI");
      break;

    case 'L':
      Run_ToFSingleTest("LEFT", VL53L0X_LEFT_ADDR, TOF_LEFT_INT_Port, TOF_LEFT_INT_Pin);
      break;

    case 'R':
      Run_ToFSingleTest("RIGHT", VL53L0X_RIGHT_ADDR, TOF_RIGHT_INT_Port, TOF_RIGHT_INT_Pin);
      break;

    case 'g':
      Run_GPIOTest();
      break;

    case 'E':
      Run_MotorStatus();
      break;

    case 'Q':
    case 'q':
      Run_EncoderRawMonitor();
      break;

    case 'V':
    case 'v':
      Run_EncoderPullDiagnostic();
      break;

    case 'Z':
    case 'z':
      Motor_ResetEncoders();
      Run_MotorStatus();
      break;

    case 'x':
    case 'X':
      Motor_Stop();
      Run_MotorStatus();
      break;

    case 'f':
    case 'F':
      Motor_SetSpeeds(50, 50);
      Run_MotorStatus();
      break;

    case 'b':
    case 'B':
      Motor_SetSpeeds(-50, -50);
      Run_MotorStatus();
      break;

    case '<':
      Motor_SetSpeeds(-50, 50);
      Run_MotorStatus();
      break;

    case '>':
      Motor_SetSpeeds(50, -50);
      Run_MotorStatus();
      break;

    case 'a':
      Run_AllTests();
      break;

    default:
      TestPlatform_PrintLine("Bilinmeyen komut. 'h' ile menuyu gorebilirsin.");
      break;
  }
}

static void TestPlatform_Printf(const char *format, ...)
{
  char buffer[320];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length < 0)
  {
    return;
  }

  if ((size_t)length >= sizeof(buffer))
  {
    length = (int)sizeof(buffer) - 1;
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
}

static void TestPlatform_PrintLine(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, HAL_MAX_DELAY);
}

static uint8_t I2C_WaitFlagSet(volatile uint32_t *reg, uint32_t flag, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (((*reg) & flag) == 0U)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t I2C_WaitFlagClear(volatile uint32_t *reg, uint32_t flag, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (((*reg) & flag) != 0U)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return 0U;
    }
  }

  return 1U;
}

static void I2C1_BusInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  I2C1->CR1 = I2C_CR1_SWRST;
  I2C1->CR1 = 0U;
  I2C1->CR2 = 42U;
  I2C1->CCR = 210U;
  I2C1->TRISE = 43U;
  I2C1->CR1 = I2C_CR1_PE;
}

static uint8_t I2C1_StartAndAddress(uint8_t address, uint8_t read)
{
  uint32_t dummy;
  uint32_t start = HAL_GetTick();

  if (I2C_WaitFlagClear(&I2C1->SR2, I2C_SR2_BUSY, I2C_TIMEOUT_MS) == 0U)
  {
    return 0U;
  }

  I2C1->CR1 |= I2C_CR1_START;
  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_SB, I2C_TIMEOUT_MS) == 0U)
  {
    return 0U;
  }

  I2C1->DR = (uint8_t)((address << 1U) | (read & 0x01U));

  while ((I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0U)
  {
    if ((HAL_GetTick() - start) >= I2C_TIMEOUT_MS)
    {
      I2C1->CR1 |= I2C_CR1_STOP;
      return 0U;
    }
  }

  if ((I2C1->SR1 & I2C_SR1_AF) != 0U)
  {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  dummy = I2C1->SR1;
  dummy = I2C1->SR2;
  (void)dummy;

  return 1U;
}

static uint8_t I2C1_Probe(uint8_t address)
{
  uint8_t ok = I2C1_StartAndAddress(address, 0U);
  I2C1->CR1 |= I2C_CR1_STOP;
  return ok;
}

static uint8_t I2C1_WriteReg(uint8_t address, uint8_t reg, uint8_t value)
{
  if (I2C1_StartAndAddress(address, 0U) == 0U)
  {
    return 0U;
  }

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = reg;

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = value;

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_BTF, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->CR1 |= I2C_CR1_STOP;
  return 1U;
}

static uint8_t I2C1_WriteReg16(uint8_t address, uint8_t reg, uint16_t value)
{
  if (I2C1_StartAndAddress(address, 0U) == 0U)
  {
    return 0U;
  }

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = reg;
  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = (uint8_t)(value >> 8U);
  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = (uint8_t)(value & 0xFFU);
  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_BTF, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->CR1 |= I2C_CR1_STOP;
  return 1U;
}

static uint8_t I2C1_ReadReg(uint8_t address, uint8_t reg, uint8_t *value)
{
  if (I2C1_StartAndAddress(address, 0U) == 0U)
  {
    return 0U;
  }

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = reg;

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_TXE, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->CR1 |= I2C_CR1_START;
  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_SB, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->DR = (uint8_t)((address << 1U) | 0x01U);

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_ADDR, I2C_TIMEOUT_MS) == 0U)
  {
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
  }

  I2C1->CR1 &= ~I2C_CR1_ACK;
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  I2C1->CR1 |= I2C_CR1_STOP;

  if (I2C_WaitFlagSet(&I2C1->SR1, I2C_SR1_RXNE, I2C_TIMEOUT_MS) == 0U)
  {
    return 0U;
  }

  *value = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_ACK;
  return 1U;
}

static uint8_t I2C1_ReadReg16(uint8_t address, uint8_t reg, uint16_t *value)
{
  uint8_t msb;
  uint8_t lsb;

  if ((I2C1_ReadReg(address, reg, &msb) == 0U) ||
      (I2C1_ReadReg(address, (uint8_t)(reg + 1U), &lsb) == 0U))
  {
    return 0U;
  }

  *value = ((uint16_t)msb << 8U) | lsb;
  return 1U;
}

static uint8_t MPU6050_ReadWord(uint8_t reg, int16_t *value)
{
  uint8_t msb;
  uint8_t lsb;

  if ((I2C1_ReadReg(MPU6050_ADDR, reg, &msb) == 0U) ||
      (I2C1_ReadReg(MPU6050_ADDR, (uint8_t)(reg + 1U), &lsb) == 0U))
  {
    return 0U;
  }

  *value = (int16_t)((uint16_t)msb << 8U | lsb);
  return 1U;
}

static uint8_t MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                               int16_t *temp_raw, int16_t *gx, int16_t *gy, int16_t *gz)
{
  return ((MPU6050_ReadWord(MPU6050_ACCEL_XOUT, ax) != 0U) &&
          (MPU6050_ReadWord((uint8_t)(MPU6050_ACCEL_XOUT + 2U), ay) != 0U) &&
          (MPU6050_ReadWord((uint8_t)(MPU6050_ACCEL_XOUT + 4U), az) != 0U) &&
          (MPU6050_ReadWord((uint8_t)(MPU6050_ACCEL_XOUT + 6U), temp_raw) != 0U) &&
          (MPU6050_ReadWord((uint8_t)(MPU6050_ACCEL_XOUT + 8U), gx) != 0U) &&
          (MPU6050_ReadWord((uint8_t)(MPU6050_ACCEL_XOUT + 10U), gy) != 0U) &&
          (MPU6050_ReadWord((uint8_t)(MPU6050_ACCEL_XOUT + 12U), gz) != 0U)) ? 1U : 0U;
}

static int32_t RawAccelToMg(int16_t raw)
{
  return ((int32_t)raw * 1000L) / 16384L;
}

static int32_t RawGyroToCentiDps(int16_t raw)
{
  return ((int32_t)raw * 100L) / 131L;
}

static int32_t RawTempToCentiC(int16_t raw)
{
  return (((int32_t)raw * 100L) / 340L) + 3653L;
}

static void Run_I2CScan(void)
{
  uint8_t found = 0U;

  TestPlatform_PrintLine("I2C taraniyor...");
  for (uint8_t address = 0x08U; address <= 0x77U; address++)
  {
    if (I2C1_Probe(address) != 0U)
    {
      TestPlatform_Printf("  bulundu: 0x%02X\r\n", address);
      found++;
    }
  }

  if (found == 0U)
  {
    TestPlatform_PrintLine("  cihaz yok. 3V3/GND, PB8/PB9 ve pull-up baglantilarini kontrol et.");
  }
  else
  {
    TestPlatform_Printf("  toplam %u cihaz bulundu\r\n", found);
  }
}

static void Run_MPU6050Test(void)
{
  uint8_t who_am_i;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp_raw;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t gx_cal;
  int16_t gy_cal;
  int16_t gz_cal;

  if (I2C1_ReadReg(MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i) == 0U)
  {
    TestPlatform_PrintLine("MPU6050 yok/cevap yok. Beklenen adres 0x68.");
    return;
  }

  if ((who_am_i != 0x68U) &&
      (who_am_i != 0x70U) &&
      (who_am_i != 0x71U) &&
      (who_am_i != 0x72U) &&
      (who_am_i != 0x73U))
  {
    TestPlatform_Printf("IMU WHO_AM_I taninmiyor: 0x%02X\r\n", who_am_i);
    return;
  }

  if ((who_am_i != 0x68U) && (imu_identity_reported == 0U))
  {
    TestPlatform_Printf("IMU WHO_AM_I: 0x%02X, MPU6050 uyumlu okuma deneniyor.\r\n", who_am_i);
    imu_identity_reported = 1U;
  }

  (void)I2C1_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00U);
  (void)I2C1_WriteReg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00U);
  (void)I2C1_WriteReg(MPU6050_ADDR, MPU6050_ACC_CONFIG, 0x00U);
  HAL_Delay(10U);

  if (MPU6050_ReadAll(&ax, &ay, &az, &temp_raw, &gx, &gy, &gz) == 0U)
  {
    TestPlatform_PrintLine("MPU6050 veri okuma hatasi.");
    return;
  }

  gx_cal = (int16_t)(gx - gyro_offset_x);
  gy_cal = (int16_t)(gy - gyro_offset_y);
  gz_cal = (int16_t)(gz - gyro_offset_z);

  TestPlatform_Printf("MPU OK | WHO:0x%02X | ACC raw X:%6d Y:%6d Z:%6d | ACC mg X:%5ld Y:%5ld Z:%5ld | GYR raw X:%6d Y:%6d Z:%6d | GYR cdps X:%6ld Y:%6ld Z:%6ld | TEMP cC:%5ld | INT:%u\r\n",
                      who_am_i, ax, ay, az,
                      RawAccelToMg(ax), RawAccelToMg(ay), RawAccelToMg(az),
                      gx, gy, gz,
                      RawGyroToCentiDps(gx_cal), RawGyroToCentiDps(gy_cal), RawGyroToCentiDps(gz_cal),
                      RawTempToCentiC(temp_raw),
                      (HAL_GPIO_ReadPin(MPU_INT_GPIO_Port, MPU_INT_Pin) == GPIO_PIN_SET) ? 1U : 0U);
}

static void Run_MPU6050Calibrate(void)
{
  int32_t sum_x = 0L;
  int32_t sum_y = 0L;
  int32_t sum_z = 0L;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp_raw;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  const uint16_t samples = 120U;

  TestPlatform_PrintLine("Gyro kalibrasyon basladi. Kart 3 saniye sabit dursun...");
  (void)I2C1_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00U);
  (void)I2C1_WriteReg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00U);
  (void)I2C1_WriteReg(MPU6050_ADDR, MPU6050_ACC_CONFIG, 0x00U);
  HAL_Delay(50U);

  for (uint16_t i = 0U; i < samples; i++)
  {
    if (MPU6050_ReadAll(&ax, &ay, &az, &temp_raw, &gx, &gy, &gz) == 0U)
    {
      TestPlatform_PrintLine("Gyro kalibrasyon okuma hatasi.");
      return;
    }

    sum_x += gx;
    sum_y += gy;
    sum_z += gz;
    HAL_Delay(25U);
  }

  gyro_offset_x = (int16_t)(sum_x / samples);
  gyro_offset_y = (int16_t)(sum_y / samples);
  gyro_offset_z = (int16_t)(sum_z / samples);

  TestPlatform_Printf("Gyro offset OK | raw X:%d Y:%d Z:%d | cdps X:%ld Y:%ld Z:%ld\r\n",
                      gyro_offset_x, gyro_offset_y, gyro_offset_z,
                      RawGyroToCentiDps(gyro_offset_x),
                      RawGyroToCentiDps(gyro_offset_y),
                      RawGyroToCentiDps(gyro_offset_z));
}

static void ToF_SetXshut(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  HAL_GPIO_WritePin(port, pin, state);
}

static void ToF_AssignAddresses(void)
{
  uint8_t left_ok;
  uint8_t right_ok;

  TestPlatform_PrintLine("ToF adres atama basladi...");

  ToF_SetXshut(TOF_LEFT_XSHUT_Port, TOF_LEFT_XSHUT_Pin, GPIO_PIN_RESET);
  ToF_SetXshut(TOF_RIGHT_XSHUT_Port, TOF_RIGHT_XSHUT_Pin, GPIO_PIN_RESET);
  tof_left_ready = 0U;
  tof_right_ready = 0U;
  tof_left_initialized = 0U;
  tof_right_initialized = 0U;
  HAL_Delay(20U);

  ToF_SetXshut(TOF_LEFT_XSHUT_Port, TOF_LEFT_XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(80U);
  left_ok = I2C1_Probe(VL53L0X_DEFAULT_ADDR);
  if (left_ok != 0U)
  {
    (void)I2C1_WriteReg(VL53L0X_DEFAULT_ADDR, VL53L0X_I2C_ADDR_REG, VL53L0X_LEFT_ADDR);
    HAL_Delay(10U);
    left_ok = I2C1_Probe(VL53L0X_LEFT_ADDR);
  }

  ToF_SetXshut(TOF_RIGHT_XSHUT_Port, TOF_RIGHT_XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(80U);
  right_ok = I2C1_Probe(VL53L0X_DEFAULT_ADDR);
  if (right_ok != 0U)
  {
    (void)I2C1_WriteReg(VL53L0X_DEFAULT_ADDR, VL53L0X_I2C_ADDR_REG, VL53L0X_RIGHT_ADDR);
    HAL_Delay(10U);
    right_ok = I2C1_Probe(VL53L0X_RIGHT_ADDR);
  }

  tof_left_ready = (left_ok != 0U) ? 1U : 0U;
  tof_right_ready = (right_ok != 0U) ? 1U : 0U;
  tof_addresses_assigned = ((left_ok != 0U) || (right_ok != 0U)) ? 1U : 0U;
  TestPlatform_Printf("ToF assign | LEFT 0x30:%s | RIGHT 0x31:%s\r\n",
                      (left_ok != 0U) ? "OK" : "YOK",
                      (right_ok != 0U) ? "OK" : "YOK");
}

static uint8_t VL53L0X_ApplyTuning(uint8_t address)
{
  static const uint8_t tuning[][2] = {
    {0xFFU, 0x01U}, {0x00U, 0x00U}, {0xFFU, 0x00U}, {0x09U, 0x00U},
    {0x10U, 0x00U}, {0x11U, 0x00U}, {0x24U, 0x01U}, {0x25U, 0xFFU},
    {0x75U, 0x00U}, {0xFFU, 0x01U}, {0x4EU, 0x2CU}, {0x48U, 0x00U},
    {0x30U, 0x20U}, {0xFFU, 0x00U}, {0x30U, 0x09U}, {0x54U, 0x00U},
    {0x31U, 0x04U}, {0x32U, 0x03U}, {0x40U, 0x83U}, {0x46U, 0x25U},
    {0x60U, 0x00U}, {0x27U, 0x00U}, {0x50U, 0x06U}, {0x51U, 0x00U},
    {0x52U, 0x96U}, {0x56U, 0x08U}, {0x57U, 0x30U}, {0x61U, 0x00U},
    {0x62U, 0x00U}, {0x64U, 0x00U}, {0x65U, 0x00U}, {0x66U, 0xA0U},
    {0xFFU, 0x01U}, {0x22U, 0x32U}, {0x47U, 0x14U}, {0x49U, 0xFFU},
    {0x4AU, 0x00U}, {0xFFU, 0x00U}, {0x7AU, 0x0AU}, {0x7BU, 0x00U},
    {0x78U, 0x21U}, {0xFFU, 0x01U}, {0x23U, 0x34U}, {0x42U, 0x00U},
    {0x44U, 0xFFU}, {0x45U, 0x26U}, {0x46U, 0x05U}, {0x40U, 0x40U},
    {0x0EU, 0x06U}, {0x20U, 0x1AU}, {0x43U, 0x40U}, {0xFFU, 0x00U},
    {0x34U, 0x03U}, {0x35U, 0x44U}, {0xFFU, 0x01U}, {0x31U, 0x04U},
    {0x4BU, 0x09U}, {0x4CU, 0x05U}, {0x4DU, 0x04U}, {0xFFU, 0x00U},
    {0x44U, 0x00U}, {0x45U, 0x20U}, {0x47U, 0x08U}, {0x48U, 0x28U},
    {0x67U, 0x00U}, {0x70U, 0x04U}, {0x71U, 0x01U}, {0x72U, 0xFEU},
    {0x76U, 0x00U}, {0x77U, 0x00U}, {0xFFU, 0x01U}, {0x0DU, 0x01U},
    {0xFFU, 0x00U}, {0x80U, 0x01U}, {0x01U, 0xF8U}, {0xFFU, 0x01U},
    {0x8EU, 0x01U}, {0x00U, 0x01U}, {0xFFU, 0x00U}, {0x80U, 0x00U}
  };

  for (uint32_t i = 0U; i < (sizeof(tuning) / sizeof(tuning[0])); i++)
  {
    if (I2C1_WriteReg(address, tuning[i][0], tuning[i][1]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t VL53L0X_BasicInit(uint8_t address)
{
  if (I2C1_Probe(address) == 0U)
  {
    return 0U;
  }

  (void)I2C1_WriteReg(address, 0x88U, 0x00U);
  (void)I2C1_WriteReg(address, 0x80U, 0x01U);
  (void)I2C1_WriteReg(address, 0xFFU, 0x01U);
  (void)I2C1_WriteReg(address, 0x00U, 0x00U);
  (void)I2C1_WriteReg(address, 0x00U, 0x01U);
  (void)I2C1_WriteReg(address, 0xFFU, 0x00U);
  (void)I2C1_WriteReg(address, 0x80U, 0x00U);

  if (VL53L0X_ApplyTuning(address) == 0U)
  {
    return 0U;
  }

  (void)I2C1_WriteReg(address, 0x01U, 0xFFU);
  (void)I2C1_WriteReg(address, 0x0AU, 0x04U);
  (void)I2C1_WriteReg(address, 0x0BU, 0x01U);
  (void)I2C1_WriteReg16(address, 0x44U, 0x0020U);
  HAL_Delay(10U);

  return 1U;
}

static uint8_t VL53L0X_ReadRangeMm(uint8_t address, uint16_t *range_mm)
{
  uint8_t status;
  uint32_t start;

  if (I2C1_Probe(address) == 0U)
  {
    return 0U;
  }

  (void)I2C1_WriteReg(address, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01U);
  if (I2C1_WriteReg(address, VL53L0X_SYSRANGE_START, 0x01U) == 0U)
  {
    return 0U;
  }

  start = HAL_GetTick();
  do
  {
    if (I2C1_ReadReg(address, VL53L0X_RESULT_INTERRUPT_STATUS, &status) == 0U)
    {
      return 0U;
    }

    if ((status & 0x07U) != 0U)
    {
      break;
    }
  } while ((HAL_GetTick() - start) < TOF_RANGE_TIMEOUT_MS);

  if ((status & 0x07U) == 0U)
  {
    return 0U;
  }

  if (I2C1_ReadReg16(address, (uint8_t)(VL53L0X_RESULT_RANGE_STATUS + 10U), range_mm) == 0U)
  {
    return 0U;
  }

  (void)I2C1_WriteReg(address, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01U);

  if ((*range_mm == 0U) || (*range_mm > 4000U))
  {
    return 0U;
  }

  return 1U;
}

static void Run_ToFTest(void)
{
  if (tof_addresses_assigned == 0U)
  {
    ToF_AssignAddresses();
  }

  TestPlatform_PrintLine("ToF testi:");
  Run_ToFSingleTest("LEFT", VL53L0X_LEFT_ADDR, TOF_LEFT_INT_Port, TOF_LEFT_INT_Pin);
  Run_ToFSingleTest("RIGHT", VL53L0X_RIGHT_ADDR, TOF_RIGHT_INT_Port, TOF_RIGHT_INT_Pin);
}

static void Run_ToFStreamSample(void)
{
  uint16_t left_mm = 0U;
  uint16_t right_mm = 0U;
  uint8_t left_ok;
  uint8_t right_ok;

  if (tof_addresses_assigned == 0U)
  {
    ToF_AssignAddresses();
  }

  if ((tof_left_ready != 0U) && (tof_left_initialized == 0U))
  {
    tof_left_initialized = VL53L0X_BasicInit(VL53L0X_LEFT_ADDR);
  }

  if ((tof_right_ready != 0U) && (tof_right_initialized == 0U))
  {
    tof_right_initialized = VL53L0X_BasicInit(VL53L0X_RIGHT_ADDR);
  }

  left_ok = (tof_left_initialized != 0U) ? VL53L0X_ReadRangeMm(VL53L0X_LEFT_ADDR, &left_mm) : 0U;
  HAL_Delay(TOF_INTER_SENSOR_MS);
  right_ok = (tof_right_initialized != 0U) ? VL53L0X_ReadRangeMm(VL53L0X_RIGHT_ADDR, &right_mm) : 0U;

  TestPlatform_Printf("TOF RANGE | LEFT:%s %u mm | RIGHT:%s %u mm | YAW L:%d R:%d | UP:%d | BASE:%d\r\n",
                      (left_ok != 0U) ? "OK" : "YOK",
                      (left_ok != 0U) ? left_mm : 0U,
                      (right_ok != 0U) ? "OK" : "YOK",
                      (right_ok != 0U) ? right_mm : 0U,
                      TOF_LEFT_YAW_DEG,
                      TOF_RIGHT_YAW_DEG,
                      TOF_UP_TILT_DEG,
                      TOF_BASELINE_MM);
}

static void Run_ToFSingleTest(const char *name, uint8_t address, GPIO_TypeDef *int_port, uint16_t int_pin)
{
  uint8_t model = 0U;
  uint8_t module = 0U;
  uint8_t revision = 0U;
  uint8_t int_state = (HAL_GPIO_ReadPin(int_port, int_pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint16_t range_mm = 0U;
  uint8_t range_ok = 0U;

  if (I2C1_Probe(address) == 0U)
  {
    TestPlatform_Printf("TOF %s | ADDR 0x%02X | STATUS:YOK | INT:%u\r\n", name, address, int_state);
    return;
  }

  if (address == VL53L0X_LEFT_ADDR)
  {
    if (tof_left_initialized == 0U)
    {
      tof_left_initialized = VL53L0X_BasicInit(address);
    }
    range_ok = (tof_left_initialized != 0U) ? VL53L0X_ReadRangeMm(address, &range_mm) : 0U;
  }
  else if (address == VL53L0X_RIGHT_ADDR)
  {
    if (tof_right_initialized == 0U)
    {
      tof_right_initialized = VL53L0X_BasicInit(address);
    }
    range_ok = (tof_right_initialized != 0U) ? VL53L0X_ReadRangeMm(address, &range_mm) : 0U;
  }

  (void)I2C1_ReadReg(address, VL53L0X_MODEL_ID_REG, &model);
  (void)I2C1_ReadReg(address, VL53L0X_MODULE_ID_REG, &module);
  (void)I2C1_ReadReg(address, VL53L0X_REVISION_REG, &revision);

  TestPlatform_Printf("TOF %s | ADDR 0x%02X | STATUS:OK | MODEL:0x%02X MODULE:0x%02X REV:0x%02X | RANGE:%s %u mm | INT:%u\r\n",
                      name, address, model, module, revision,
                      (range_ok != 0U) ? "OK" : "YOK",
                      (range_ok != 0U) ? range_mm : 0U,
                      int_state);
}

static const char *PinLevel(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "HIGH" : "LOW";
}

static const char *PinOnOff(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "ON" : "OFF";
}

static const char *PinSetReset(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "SET" : "RESET";
}

static uint8_t Encoder_ReadState(GPIO_TypeDef *port, uint16_t pin_a, uint16_t pin_b)
{
  uint8_t a = (HAL_GPIO_ReadPin(port, pin_a) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(port, pin_b) == GPIO_PIN_SET) ? 1U : 0U;
  return (uint8_t)((a << 1U) | b);
}

static void Encoder_UpdateOne(volatile uint8_t *old_state, uint8_t new_state, volatile int32_t *count)
{
  static const int8_t delta_table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  uint8_t transition = (uint8_t)((*old_state << 2U) | new_state);

  *count += delta_table[transition & 0x0FU];
  *old_state = new_state;
}

static void Encoder_ReconfigureInputs(uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LEFT_ENC_A_Pin | LEFT_ENC_B_Pin | RIGHT_ENC_A_Pin | RIGHT_ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = pull;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  left_encoder_state = Encoder_ReadState(LEFT_ENC_Port, LEFT_ENC_A_Pin, LEFT_ENC_B_Pin);
  right_encoder_state = Encoder_ReadState(RIGHT_ENC_Port, RIGHT_ENC_A_Pin, RIGHT_ENC_B_Pin);
}

static uint16_t PercentToPwm(int16_t percent)
{
  int16_t magnitude = percent;

  if (magnitude < 0)
  {
    magnitude = (int16_t)-magnitude;
  }

  if (magnitude > 100)
  {
    magnitude = 100;
  }

  return (uint16_t)(((uint32_t)magnitude * MOTOR_PWM_MAX) / 100U);
}

static int16_t ClampPercent(int16_t percent)
{
  if (percent > 100)
  {
    return 100;
  }

  if (percent < -100)
  {
    return -100;
  }

  return percent;
}

static void Motor_WriteLeft(int16_t percent)
{
  uint16_t pwm = PercentToPwm(percent);

  if (percent > 0)
  {
    TIM3->CCR1 = pwm;
    TIM3->CCR3 = 0U;
  }
  else if (percent < 0)
  {
    TIM3->CCR1 = 0U;
    TIM3->CCR3 = pwm;
  }
  else
  {
    TIM3->CCR1 = 0U;
    TIM3->CCR3 = 0U;
  }
}

static void Motor_WriteRight(int16_t percent)
{
  uint16_t pwm = PercentToPwm(percent);

  if (percent > 0)
  {
    TIM12->CCR1 = pwm;
    TIM12->CCR2 = 0U;
  }
  else if (percent < 0)
  {
    TIM12->CCR1 = 0U;
    TIM12->CCR2 = pwm;
  }
  else
  {
    TIM12->CCR1 = 0U;
    TIM12->CCR2 = 0U;
  }
}

static void Motor_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM12_CLK_ENABLE();

  TIM3->PSC = 0U;
  TIM3->ARR = MOTOR_PWM_MAX;
  TIM3->CCR1 = 0U;
  TIM3->CCR3 = 0U;
  TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
  TIM3->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM3->CCMR2 &= ~TIM_CCMR2_OC3M;
  TIM3->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;
  TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

  TIM12->PSC = 0U;
  TIM12->ARR = MOTOR_PWM_MAX;
  TIM12->CCR1 = 0U;
  TIM12->CCR2 = 0U;
  TIM12->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM12->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM12->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM12->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

  left_encoder_state = Encoder_ReadState(LEFT_ENC_Port, LEFT_ENC_A_Pin, LEFT_ENC_B_Pin);
  right_encoder_state = Encoder_ReadState(RIGHT_ENC_Port, RIGHT_ENC_A_Pin, RIGHT_ENC_B_Pin);
}

static void Motor_SetSpeeds(int16_t left_percent, int16_t right_percent)
{
  int16_t new_left = ClampPercent(left_percent);
  int16_t new_right = ClampPercent(right_percent);
  uint8_t left_reversing = ((left_motor_percent > 0) && (new_left < 0)) ||
                           ((left_motor_percent < 0) && (new_left > 0));
  uint8_t right_reversing = ((right_motor_percent > 0) && (new_right < 0)) ||
                            ((right_motor_percent < 0) && (new_right > 0));

  if ((left_reversing != 0U) || (right_reversing != 0U))
  {
    Motor_WriteLeft(0);
    Motor_WriteRight(0);
    HAL_Delay(MOTOR_DIRECTION_CHANGE_STOP_MS);
  }

  left_motor_percent = new_left;
  right_motor_percent = new_right;
  last_motor_command_ms = HAL_GetTick();
  motor_timeout_latched = 0U;
  Motor_WriteLeft(left_motor_percent);
  Motor_WriteRight(right_motor_percent);
}

static void Motor_Stop(void)
{
  Motor_SetSpeeds(0, 0);
}

static void Motor_Task(void)
{
  if (((left_motor_percent != 0) || (right_motor_percent != 0)) &&
      ((HAL_GetTick() - last_motor_command_ms) >= MOTOR_COMMAND_TIMEOUT_MS))
  {
    left_motor_percent = 0;
    right_motor_percent = 0;
    Motor_WriteLeft(0);
    Motor_WriteRight(0);
    motor_timeout_latched = 1U;
  }
}

static void Motor_ResetEncoders(void)
{
  __disable_irq();
  left_encoder_count = 0;
  right_encoder_count = 0;
  left_encoder_state = Encoder_ReadState(LEFT_ENC_Port, LEFT_ENC_A_Pin, LEFT_ENC_B_Pin);
  right_encoder_state = Encoder_ReadState(RIGHT_ENC_Port, RIGHT_ENC_A_Pin, RIGHT_ENC_B_Pin);
  __enable_irq();

  last_left_encoder_report = 0;
  last_right_encoder_report = 0;
  last_motor_report_ms = HAL_GetTick();
  TestPlatform_PrintLine("Encoder sayaclari sifirlandi.");
}

static void Run_MotorStatus(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed_ms = now - last_motor_report_ms;
  int32_t left_count;
  int32_t right_count;
  uint8_t left_state;
  uint8_t right_state;
  int32_t left_delta;
  int32_t right_delta;
  int32_t left_rpm_x100 = 0;
  int32_t right_rpm_x100 = 0;

  if (elapsed_ms == 0U)
  {
    elapsed_ms = 1U;
  }

  left_count = left_encoder_count;
  right_count = right_encoder_count;
  left_state = left_encoder_state;
  right_state = right_encoder_state;
  left_delta = left_count - last_left_encoder_report;
  right_delta = right_count - last_right_encoder_report;

  if (ENCODER_COUNTS_PER_REV > 0L)
  {
    left_rpm_x100 = (int32_t)(((int64_t)left_delta * 6000000LL) /
                              ((int64_t)ENCODER_COUNTS_PER_REV * (int64_t)elapsed_ms));
    right_rpm_x100 = (int32_t)(((int64_t)right_delta * 6000000LL) /
                               ((int64_t)ENCODER_COUNTS_PER_REV * (int64_t)elapsed_ms));
  }

  last_left_encoder_report = left_count;
  last_right_encoder_report = right_count;
  last_motor_report_ms = now;

  TestPlatform_Printf("MOTOR | LEFT:%d RPWM:%lu LPWM:%lu ENC:%ld DENC:%ld RPMx100:%ld AB:%u | RIGHT:%d RPWM:%lu LPWM:%lu ENC:%ld DENC:%ld RPMx100:%ld AB:%u | SAFE:%s\r\n",
                      left_motor_percent,
                      TIM3->CCR1,
                      TIM3->CCR3,
                      left_count,
                      left_delta,
                      left_rpm_x100,
                      left_state,
                      right_motor_percent,
                      TIM12->CCR1,
                      TIM12->CCR2,
                      right_count,
                      right_delta,
                      right_rpm_x100,
                      right_state,
                      (motor_timeout_latched != 0U) ? "TIMEOUT_STOP" : (((left_motor_percent == 0) && (right_motor_percent == 0)) ? "STOP" : "RUN"));
}

static void Run_EncoderRawMonitor(void)
{
  uint32_t start_ms = HAL_GetTick();
  uint32_t last_ms = start_ms;
  uint8_t last_left_a = (HAL_GPIO_ReadPin(LEFT_ENC_Port, LEFT_ENC_A_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t last_left_b = (HAL_GPIO_ReadPin(LEFT_ENC_Port, LEFT_ENC_B_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t last_right_a = (HAL_GPIO_ReadPin(RIGHT_ENC_Port, RIGHT_ENC_A_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t last_right_b = (HAL_GPIO_ReadPin(RIGHT_ENC_Port, RIGHT_ENC_B_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint32_t left_a_edges = 0U;
  uint32_t left_b_edges = 0U;
  uint32_t right_a_edges = 0U;
  uint32_t right_b_edges = 0U;
  int32_t left_b_only_count = 0;
  int32_t right_b_only_count = 0;

  TestPlatform_PrintLine("ENC RAW basladi: tekeri 8 saniye boyunca elle cevir.");

  while ((HAL_GetTick() - start_ms) < ENCODER_RAW_MONITOR_MS)
  {
    uint8_t left_state = Encoder_ReadState(LEFT_ENC_Port, LEFT_ENC_A_Pin, LEFT_ENC_B_Pin);
    uint8_t right_state = Encoder_ReadState(RIGHT_ENC_Port, RIGHT_ENC_A_Pin, RIGHT_ENC_B_Pin);
    uint8_t left_a = (uint8_t)(left_state >> 1U);
    uint8_t left_b = (uint8_t)(left_state & 1U);
    uint8_t right_a = (uint8_t)(right_state >> 1U);
    uint8_t right_b = (uint8_t)(right_state & 1U);

    if (left_a != last_left_a)
    {
      left_a_edges++;
      last_left_a = left_a;
    }

    if (left_b != last_left_b)
    {
      left_b_edges++;
      left_b_only_count += (left_motor_percent < 0) ? -1 : 1;
      last_left_b = left_b;
    }

    if (right_a != last_right_a)
    {
      right_a_edges++;
      last_right_a = right_a;
    }

    if (right_b != last_right_b)
    {
      right_b_edges++;
      right_b_only_count += (right_motor_percent < 0) ? -1 : 1;
      last_right_b = right_b;
    }

    if ((HAL_GetTick() - last_ms) >= 200U)
    {
      last_ms = HAL_GetTick();
      TestPlatform_Printf("ENC RAW | L_AB:%u LA:%u LB:%u L_CNT:%ld LAE:%lu LBE:%lu L_BCNT:%ld | R_AB:%u RA:%u RB:%u R_CNT:%ld RAE:%lu RBE:%lu R_BCNT:%ld\r\n",
                          left_state,
                          left_a,
                          left_b,
                          left_encoder_count,
                          left_a_edges,
                          left_b_edges,
                          left_b_only_count,
                          right_state,
                          right_a,
                          right_b,
                          right_encoder_count,
                          right_a_edges,
                          right_b_edges,
                          right_b_only_count);
    }
  }

  motor_status_stream_enabled = 0U;
  TestPlatform_PrintLine("ENC RAW bitti. Motor durum akisi durduruldu; ekran sabit kalacak.");
}

static void Run_EncoderPullDiagnostic(void)
{
  static const struct
  {
    const char *name;
    uint32_t pull;
  } configs[] = {
    {"NOPULL", GPIO_NOPULL},
    {"PULLDOWN", GPIO_PULLDOWN},
    {"PULLUP", GPIO_PULLUP}
  };

  TestPlatform_PrintLine("ENC PULL teshisi: motorlari durdur, tekerleri cevirmeden oku.");

  for (uint8_t i = 0U; i < (uint8_t)(sizeof(configs) / sizeof(configs[0])); i++)
  {
    Encoder_ReconfigureInputs(configs[i].pull);
    HAL_Delay(80U);

    TestPlatform_Printf("ENC PULL | %s | LA:%u LB:%u | RA:%u RB:%u\r\n",
                        configs[i].name,
                        (HAL_GPIO_ReadPin(LEFT_ENC_Port, LEFT_ENC_A_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                        (HAL_GPIO_ReadPin(LEFT_ENC_Port, LEFT_ENC_B_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                        (HAL_GPIO_ReadPin(RIGHT_ENC_Port, RIGHT_ENC_A_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                        (HAL_GPIO_ReadPin(RIGHT_ENC_Port, RIGHT_ENC_B_Pin) == GPIO_PIN_SET) ? 1U : 0U);
  }

  Encoder_ReconfigureInputs(GPIO_PULLUP);
  TestPlatform_PrintLine("ENC PULL bitti. Encoder girisleri PULLUP moda geri alindi.");
}

static void Print_ToFGPIOState(void)
{
  TestPlatform_Printf("TOF GPIO | LEFT_XSHUT PA8:%s LEFT_INT PC7:%s | RIGHT_XSHUT PA9:%s RIGHT_INT PA10:%s\r\n",
                      PinLevel(TOF_LEFT_XSHUT_Port, TOF_LEFT_XSHUT_Pin),
                      PinLevel(TOF_LEFT_INT_Port, TOF_LEFT_INT_Pin),
                      PinLevel(TOF_RIGHT_XSHUT_Port, TOF_RIGHT_XSHUT_Pin),
                      PinLevel(TOF_RIGHT_INT_Port, TOF_RIGHT_INT_Pin));
}

static void Run_GPIOTest(void)
{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  TestPlatform_Printf("GPIO | B1:%s | LD2:%s | MPU_INT PB10:%s\r\n",
                      PinSetReset(B1_GPIO_Port, B1_Pin),
                      PinOnOff(LD2_GPIO_Port, LD2_Pin),
                      PinLevel(MPU_INT_GPIO_Port, MPU_INT_Pin));
  Print_ToFGPIOState();
}

static void Run_AllTests(void)
{
  Run_GPIOTest();
  Run_I2CScan();
  Run_MPU6050Test();
  Run_ToFTest();
}

void Argos_Platform_Init(void)
{
  TestPlatform_Init();
}

void Argos_Platform_Task(void)
{
  TestPlatform_Task();
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == LEFT_ENC_A_Pin) || (GPIO_Pin == LEFT_ENC_B_Pin))
  {
    uint8_t state = Encoder_ReadState(LEFT_ENC_Port, LEFT_ENC_A_Pin, LEFT_ENC_B_Pin);
    Encoder_UpdateOne(&left_encoder_state, state, &left_encoder_count);
  }

  if ((GPIO_Pin == RIGHT_ENC_A_Pin) || (GPIO_Pin == RIGHT_ENC_B_Pin))
  {
    uint8_t state = Encoder_ReadState(RIGHT_ENC_Port, RIGHT_ENC_A_Pin, RIGHT_ENC_B_Pin);
    Encoder_UpdateOne(&right_encoder_state, state, &right_encoder_count);
  }
}
