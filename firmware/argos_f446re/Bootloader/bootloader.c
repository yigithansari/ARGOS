#include <stdint.h>
#include "stm32f446xx.h"

#define APP_ADDRESS       0x08008000UL
#define APP_LIMIT         0x08080000UL
#define APP_MAX_SIZE      (APP_LIMIT - APP_ADDRESS)
#define META_ADDRESS      0x08004000UL
#define META_MAGIC        0x4152474FUL
#define META_VALID        0x56414C49UL
#define BOOT_WAIT_MS      1800UL
#define UPDATE_TIMEOUT_MS 15000UL
#define CHUNK_SIZE        256UL

typedef struct
{
  uint32_t magic;
  uint32_t state;
  uint32_t size;
  uint32_t crc32;
  uint32_t build_id;
  uint32_t record_crc32;
} ImageMetadata;

static volatile uint32_t ticks_ms;
static uint8_t chunk_buffer[CHUNK_SIZE];

void SystemInit(void) {}

void SysTick_Handler(void)
{
  ticks_ms++;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
  while (length-- != 0UL)
  {
    crc ^= *data++;
    for (uint32_t bit = 0UL; bit < 8UL; bit++)
    {
      crc = (crc >> 1U) ^ ((crc & 1UL) ? 0xEDB88320UL : 0UL);
    }
  }
  return crc;
}

static uint32_t crc32_bytes(const uint8_t *data, uint32_t length)
{
  return ~crc32_update(0xFFFFFFFFUL, data, length);
}

static void gpio_safe_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
  (void)RCC->AHB1ENR;

  GPIOA->BSRR = ((1UL << 0U) | (1UL << 1U)) << 16U;
  GPIOC->BSRR = ((1UL << 6U) | (1UL << 8U)) << 16U;
  GPIOA->MODER = (GPIOA->MODER & ~((3UL << 0U) | (3UL << 2U))) |
                 (1UL << 0U) | (1UL << 2U);
  GPIOC->MODER = (GPIOC->MODER & ~((3UL << 12U) | (3UL << 16U))) |
                 (1UL << 12U) | (1UL << 16U);
}

static void uart_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  (void)RCC->APB1ENR;

  GPIOA->MODER = (GPIOA->MODER & ~((3UL << 4U) | (3UL << 6U))) |
                 (2UL << 4U) | (2UL << 6U);
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFUL << 8U) | (0xFUL << 12U))) |
                  (7UL << 8U) | (7UL << 12U);
  GPIOA->OSPEEDR |= (3UL << 4U) | (3UL << 6U);
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << 6U)) | (1UL << 6U);

  USART2->BRR = 139UL;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_putc(char value)
{
  while ((USART2->SR & USART_SR_TXE) == 0UL) {}
  USART2->DR = (uint8_t)value;
}

static void uart_puts(const char *text)
{
  while (*text != '\0')
  {
    uart_putc(*text++);
  }
}

static void uart_put_u32(uint32_t value)
{
  char digits[10];
  uint32_t count = 0UL;
  do
  {
    digits[count++] = (char)('0' + (value % 10UL));
    value /= 10UL;
  } while ((value != 0UL) && (count < sizeof(digits)));
  while (count != 0UL)
  {
    uart_putc(digits[--count]);
  }
}

static int uart_getc_timeout(uint8_t *value, uint32_t timeout_ms)
{
  uint32_t started = ticks_ms;
  while ((ticks_ms - started) < timeout_ms)
  {
    if ((USART2->SR & USART_SR_RXNE) != 0UL)
    {
      *value = (uint8_t)USART2->DR;
      return 1;
    }
  }
  return 0;
}

static int uart_read_line(char *line, uint32_t capacity, uint32_t timeout_ms)
{
  uint32_t started = ticks_ms;
  uint32_t length = 0UL;
  while ((ticks_ms - started) < timeout_ms)
  {
    uint8_t value;
    uint32_t remaining = timeout_ms - (ticks_ms - started);
    if (!uart_getc_timeout(&value, remaining))
    {
      break;
    }
    if (value == '\n')
    {
      line[length] = '\0';
      return (length != 0UL);
    }
    if ((value != '\r') && (length + 1UL < capacity))
    {
      line[length++] = (char)value;
    }
  }
  line[length] = '\0';
  return 0;
}

static int text_equal(const char *left, const char *right)
{
  while ((*left != '\0') && (*left == *right))
  {
    left++;
    right++;
  }
  return (*left == *right);
}

static int text_prefix(const char *text, const char *prefix)
{
  while (*prefix != '\0')
  {
    if (*text++ != *prefix++)
    {
      return 0;
    }
  }
  return 1;
}

static int parse_u32(const char **cursor, uint32_t *value)
{
  uint32_t result = 0UL;
  int found = 0;
  while (**cursor == ' ') { (*cursor)++; }
  while ((**cursor >= '0') && (**cursor <= '9'))
  {
    found = 1;
    result = (result * 10UL) + (uint32_t)(**cursor - '0');
    (*cursor)++;
  }
  *value = result;
  return found;
}

static void flash_wait(void)
{
  while ((FLASH->SR & FLASH_SR_BSY) != 0UL) {}
}

static void flash_clear_status(void)
{
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR |
              FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
}

static void flash_unlock(void)
{
  if ((FLASH->CR & FLASH_CR_LOCK) != 0UL)
  {
    FLASH->KEYR = 0x45670123UL;
    FLASH->KEYR = 0xCDEF89ABUL;
  }
}

static int flash_erase_sector(uint32_t sector)
{
  flash_wait();
  flash_clear_status();
  FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB | FLASH_CR_PSIZE)) |
              FLASH_CR_SER | FLASH_CR_PSIZE_1 |
              (sector << FLASH_CR_SNB_Pos);
  FLASH->CR |= FLASH_CR_STRT;
  flash_wait();
  FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  return ((FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                        FLASH_SR_PGPERR | FLASH_SR_PGSERR)) == 0UL);
}

static int flash_program_word(uint32_t address, uint32_t value)
{
  flash_wait();
  flash_clear_status();
  FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE) | FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  *(volatile uint32_t *)address = value;
  flash_wait();
  FLASH->CR &= ~FLASH_CR_PG;
  return ((FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                        FLASH_SR_PGPERR | FLASH_SR_PGSERR)) == 0UL);
}

static int app_vectors_valid(void)
{
  uint32_t stack = *(const uint32_t *)APP_ADDRESS;
  uint32_t reset = *(const uint32_t *)(APP_ADDRESS + 4UL);
  return ((stack >= 0x20000000UL) && (stack <= 0x20020000UL) &&
          (reset >= APP_ADDRESS) && (reset < APP_LIMIT) && ((reset & 1UL) != 0UL));
}

static int metadata_valid(void)
{
  const ImageMetadata *meta = (const ImageMetadata *)META_ADDRESS;
  uint32_t record_crc;
  if ((meta->magic != META_MAGIC) || (meta->state != META_VALID) ||
      (meta->size < 8UL) || (meta->size > APP_MAX_SIZE) || !app_vectors_valid())
  {
    return 0;
  }
  record_crc = crc32_bytes((const uint8_t *)meta, 5UL * sizeof(uint32_t));
  if (record_crc != meta->record_crc32)
  {
    return 0;
  }
  return (crc32_bytes((const uint8_t *)APP_ADDRESS, meta->size) == meta->crc32);
}

static int write_metadata(uint32_t size, uint32_t crc, uint32_t build_id)
{
  ImageMetadata meta;
  const uint32_t *words = (const uint32_t *)&meta;
  meta.magic = META_MAGIC;
  meta.state = META_VALID;
  meta.size = size;
  meta.crc32 = crc;
  meta.build_id = build_id;
  meta.record_crc32 = crc32_bytes((const uint8_t *)&meta, 5UL * sizeof(uint32_t));
  if (!flash_erase_sector(1UL))
  {
    return 0;
  }
  for (uint32_t index = 0UL; index < sizeof(meta) / sizeof(uint32_t); index++)
  {
    if (!flash_program_word(META_ADDRESS + (index * 4UL), words[index]))
    {
      return 0;
    }
  }
  return 1;
}

static void jump_to_application(void)
{
  uint32_t stack = *(const uint32_t *)APP_ADDRESS;
  uint32_t reset = *(const uint32_t *)(APP_ADDRESS + 4UL);
  void (*entry)(void) = (void (*)(void))reset;

  __disable_irq();
  SysTick->CTRL = 0UL;
  SysTick->LOAD = 0UL;
  SysTick->VAL = 0UL;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
  for (uint32_t index = 0UL; index < 8UL; index++)
  {
    NVIC->ICER[index] = 0xFFFFFFFFUL;
    NVIC->ICPR[index] = 0xFFFFFFFFUL;
  }
  USART2->CR1 = 0UL;
  SCB->VTOR = APP_ADDRESS;
  __DSB();
  __ISB();
  __set_MSP(stack);
  __enable_irq();
  entry();
  while (1) {}
}

static int receive_update(void)
{
  char line[96];
  uint32_t image_size;
  uint32_t image_crc;
  uint32_t build_id;
  uint32_t offset = 0UL;

  uart_puts("BL READY 1\r\n");
  if (!uart_read_line(line, sizeof(line), UPDATE_TIMEOUT_MS) ||
      !text_prefix(line, "BL BEGIN "))
  {
    uart_puts("BL ERROR BEGIN_TIMEOUT\r\n");
    return 0;
  }

  const char *cursor = line + 9;
  if (!parse_u32(&cursor, &image_size) || !parse_u32(&cursor, &image_crc) ||
      !parse_u32(&cursor, &build_id) || (image_size < 8UL) ||
      (image_size > APP_MAX_SIZE))
  {
    uart_puts("BL ERROR BEGIN_VALUES\r\n");
    return 0;
  }

  flash_unlock();
  if (!flash_erase_sector(1UL))
  {
    uart_puts("BL ERROR META_ERASE\r\n");
    return 0;
  }
  for (uint32_t sector = 2UL; sector <= 7UL; sector++)
  {
    if (!flash_erase_sector(sector))
    {
      uart_puts("BL ERROR APP_ERASE\r\n");
      return 0;
    }
  }
  uart_puts("BL ERASED\r\n");

  while (offset < image_size)
  {
    uint32_t frame_offset;
    uint32_t frame_length;
    uint32_t frame_crc;
    if (!uart_read_line(line, sizeof(line), UPDATE_TIMEOUT_MS) ||
        !text_prefix(line, "BL DATA "))
    {
      uart_puts("BL ERROR DATA_HEADER\r\n");
      return 0;
    }
    cursor = line + 8;
    if (!parse_u32(&cursor, &frame_offset) || !parse_u32(&cursor, &frame_length) ||
        !parse_u32(&cursor, &frame_crc) || (frame_offset != offset) ||
        (frame_length == 0UL) || (frame_length > CHUNK_SIZE) ||
        (frame_length > (image_size - offset)))
    {
      uart_puts("BL ERROR DATA_VALUES\r\n");
      return 0;
    }
    for (uint32_t index = 0UL; index < frame_length; index++)
    {
      if (!uart_getc_timeout(&chunk_buffer[index], 2000UL))
      {
        uart_puts("BL ERROR DATA_TIMEOUT\r\n");
        return 0;
      }
    }
    if (crc32_bytes(chunk_buffer, frame_length) != frame_crc)
    {
      uart_puts("BL ERROR DATA_CRC\r\n");
      return 0;
    }
    for (uint32_t index = 0UL; index < frame_length; index += 4UL)
    {
      uint32_t word = 0xFFFFFFFFUL;
      uint32_t remaining = frame_length - index;
      uint8_t *word_bytes = (uint8_t *)&word;
      uint32_t copy = (remaining >= 4UL) ? 4UL : remaining;
      for (uint32_t byte = 0UL; byte < copy; byte++)
      {
        word_bytes[byte] = chunk_buffer[index + byte];
      }
      if (!flash_program_word(APP_ADDRESS + offset + index, word))
      {
        uart_puts("BL ERROR FLASH_WRITE\r\n");
        return 0;
      }
    }
    offset += frame_length;
    uart_puts("BL ACK ");
    uart_put_u32(offset);
    uart_puts("\r\n");
  }

  if (!uart_read_line(line, sizeof(line), UPDATE_TIMEOUT_MS) ||
      !text_equal(line, "BL END"))
  {
    uart_puts("BL ERROR END_TIMEOUT\r\n");
    return 0;
  }
  if (crc32_bytes((const uint8_t *)APP_ADDRESS, image_size) != image_crc)
  {
    uart_puts("BL ERROR IMAGE_CRC\r\n");
    return 0;
  }
  if (!app_vectors_valid() || !write_metadata(image_size, image_crc, build_id))
  {
    uart_puts("BL ERROR FINALIZE\r\n");
    return 0;
  }
  uart_puts("BL OK ");
  uart_put_u32(image_crc);
  uart_puts("\r\n");
  return 1;
}

int main(void)
{
  char line[48];
  gpio_safe_init();
  uart_init();
  SysTick_Config(16000000UL / 1000UL);
  __enable_irq();

  uart_puts("ARGOS BL | WAIT | V1\r\n");
  uint32_t wait_started = ticks_ms;
  while ((ticks_ms - wait_started) < BOOT_WAIT_MS)
  {
    uint32_t remaining = BOOT_WAIT_MS - (ticks_ms - wait_started);
    if (uart_read_line(line, sizeof(line), remaining) && text_equal(line, "BL SYNC"))
    {
      if (receive_update())
      {
        for (volatile uint32_t wait = 0UL; wait < 300000UL; wait++) {}
        jump_to_application();
      }
      break;
    }
  }

  if (metadata_valid())
  {
    jump_to_application();
  }

  uart_puts("ARGOS BL | RECOVERY | NO_VALID_APP\r\n");
  while (1)
  {
    if (uart_read_line(line, sizeof(line), 2000UL) && text_equal(line, "BL SYNC"))
    {
      if (receive_update())
      {
        jump_to_application();
      }
      uart_puts("ARGOS BL | RECOVERY | RETRY\r\n");
    }
  }
}
