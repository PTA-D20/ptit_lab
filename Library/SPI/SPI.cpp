
#include <stdio.h>

#include "Core.h"
#include "SPI.h"


inline void RCC_enable_peripheral_clock(void *periph) {
       if (periph == SPI1)   BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_SPI1);
  else if (periph == SPI2)   BITMASK_SET(RCC->APB1ENR, RCC_APB1Periph_SPI2);
  else if (periph == USART1) BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_USART1);
  else if (periph == USART2) BITMASK_SET(RCC->APB1ENR, RCC_APB1Periph_USART2);
  else if (periph == USART3) BITMASK_SET(RCC->APB1ENR, RCC_APB1Periph_USART3);
  else if (periph == GPIOA)  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_GPIOA);
  else if (periph == GPIOB)  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_GPIOB);
  else if (periph == GPIOC)  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_GPIOC);
}


SPIClass::SPIClass(SPI_TypeDef *_dev, void(*_beginCb)()) :
  dev(_dev),
  onBeginCallback(_beginCb) {

  // enable SPI clock
  RCC_enable_peripheral_clock(dev);
}


SPISettings::SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
  if (__builtin_constant_p(clock)) {
    init_AlwaysInline(clock, bitOrder, dataMode);
  } else {
    init_MightInline(clock, bitOrder, dataMode);
  }
}


SPISettings::SPISettings() {
//init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
}


void SPISettings::init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
  init_AlwaysInline(clock, bitOrder, dataMode);
}


void SPISettings::init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
//    // Clock settings are defined as follows. Note that this shows SPI2X
//    // inverted, so the bits form increasing numbers. Also note that
//    // fosc/64 appears twice
//    // SPR1 SPR0 ~SPI2X Freq
//    //   0    0     0   fosc/2
//    //   0    0     1   fosc/4
//    //   0    1     0   fosc/8
//    //   0    1     1   fosc/16
//    //   1    0     0   fosc/32
//    //   1    0     1   fosc/64
//    //   1    1     0   fosc/64
//    //   1    1     1   fosc/128
//
//    // We find the fastest clock that is less than or equal to the
//    // given clock rate. The clock divider that results in clock_setting
//    // is 2 ^^ (clock_div + 1). If nothing is slow enough, we'll use the
//    // slowest (128 == 2 ^^ 7, so clock_div = 6).
//    uint8_t clockDiv;
//
//    // When the clock is known at compiletime, use this if-then-else
//    // cascade, which the compiler knows how to completely optimize
//    // away. When clock is not known, use a loop instead, which generates
//    // shorter code.
//    if (__builtin_constant_p(clock)) {
//      if (clock >= F_CPU / 2) {
//        clockDiv = 0;
//      } else if (clock >= F_CPU / 4) {
//        clockDiv = 1;
//      } else if (clock >= F_CPU / 8) {
//        clockDiv = 2;
//      } else if (clock >= F_CPU / 16) {
//        clockDiv = 3;
//      } else if (clock >= F_CPU / 32) {
//        clockDiv = 4;
//      } else if (clock >= F_CPU / 64) {
//        clockDiv = 5;
//      } else {
//        clockDiv = 6;
//      }
//    } else {
//      uint32_t clockSetting = F_CPU / 2;
//      clockDiv = 0;
//      while (clockDiv < 6 && clock < clockSetting) {
//        clockSetting /= 2;
//        clockDiv++;
//      }
//    }
//
//    // Compensate for the duplicate fosc/64
//    if (clockDiv == 6)
//    clockDiv = 7;
//
//    // Invert the SPI2X bit
//    clockDiv ^= 0x1;
//
//    // Pack into the SPISettings class
////    spcr = _BV(SPE) | _BV(MSTR) | ((bitOrder == LSBFIRST) ? _BV(DORD) : 0) |
////      (dataMode & SPI_MODE_MASK) | ((clockDiv >> 1) & SPI_CLOCK_MASK);
//    spsr = clockDiv & SPI_2XCLOCK_MASK;
}


uint8_t SPIClass::initialized   = 0;
uint8_t SPIClass::interruptMode = 0;
uint8_t SPIClass::interruptMask = 0;
uint8_t SPIClass::interruptSave = 0;

#ifdef SPI_TRANSACTION_MISMATCH_LED
  uint8_t SPIClass::inTransactionFlag = 0;
#endif


void SPIClass::begin() {
  onBeginCallback();
}

void SPIClass::end() {
  SPI_I2S_DeInit(dev);

//  #ifdef RF24_USART
//    USART_DeInit(RF24_USART);
//  #endif
}

uint8_t SPIClass::transfer(uint8_t data) {
  while ((dev->SR & SPI_I2S_FLAG_TXE) == 0);
  dev->DR = data;
  while ((dev->SR & SPI_I2S_FLAG_RXNE) == 0);
  return dev->DR;
}

void SPIClass::transfer(void *buf, int len) {
    if (len == 0)
      return;

    uint8_t *p = (uint8_t *)buf;
    while (len-- > 0)
      transfer(*p++);
}


//uint16_t SPIClass::transfer16(uint16_t data) {
//  return 0;
//}


void SPIClass::setBitOrder(uint8_t bitOrder) {
  BITPOS_SET_X1(dev->CR1, 7, bitOrder);  // bit 7: SPI_CR1_LSBFIRST
}


void SPIClass::setDataMode(uint8_t dataMode) {
  BITPOS_SET_X2(dev->CR1, 0, dataMode);
}


void SPIClass::setClockDivider(uint8_t clockDiv) {
  if (IS_SPI_BAUDRATE_PRESCALER(clockDiv))
    dev->CR1 = (dev->CR1 & ~(SPI_CR1_BR)) | clockDiv;
}


void SPIClass::attachInterrupt() {
//  SPCR |= _BV(SPIE);
}

void SPIClass::detachInterrupt() {
//  SPCR &= ~_BV(SPIE);
}


//// mapping of interrupt numbers to bits within SPI_AVR_EIMSK
//#if defined(__AVR_ATmega32U4__)
//  #define SPI_INT0_MASK  (1<<INT0)
//  #define SPI_INT1_MASK  (1<<INT1)
//  #define SPI_INT2_MASK  (1<<INT2)
//  #define SPI_INT3_MASK  (1<<INT3)
//  #define SPI_INT4_MASK  (1<<INT6)
//#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
//  #define SPI_INT0_MASK  (1<<INT0)
//  #define SPI_INT1_MASK  (1<<INT1)
//  #define SPI_INT2_MASK  (1<<INT2)
//  #define SPI_INT3_MASK  (1<<INT3)
//  #define SPI_INT4_MASK  (1<<INT4)
//  #define SPI_INT5_MASK  (1<<INT5)
//  #define SPI_INT6_MASK  (1<<INT6)
//  #define SPI_INT7_MASK  (1<<INT7)
//#elif defined(EICRA) && defined(EICRB) && defined(EIMSK)
//  #define SPI_INT0_MASK  (1<<INT4)
//  #define SPI_INT1_MASK  (1<<INT5)
//  #define SPI_INT2_MASK  (1<<INT0)
//  #define SPI_INT3_MASK  (1<<INT1)
//  #define SPI_INT4_MASK  (1<<INT2)
//  #define SPI_INT5_MASK  (1<<INT3)
//  #define SPI_INT6_MASK  (1<<INT6)
//  #define SPI_INT7_MASK  (1<<INT7)
//#else
//  #ifdef INT0
//  #define SPI_INT0_MASK  (1<<INT0)
//  #endif
//  #ifdef INT1
//  #define SPI_INT1_MASK  (1<<INT1)
//  #endif
//  #ifdef INT2
//  #define SPI_INT2_MASK  (1<<INT2)
//  #endif
//#endif


void SPIClass::usingInterrupt(uint8_t interruptNumber) {
//  uint8_t mask = 0;
//  uint8_t sreg = SREG;
//  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
//  switch (interruptNumber) {
//  #ifdef SPI_INT0_MASK
//  case 0: mask = SPI_INT0_MASK; break;
//  #endif
//  #ifdef SPI_INT1_MASK
//  case 1: mask = SPI_INT1_MASK; break;
//  #endif
//  #ifdef SPI_INT2_MASK
//  case 2: mask = SPI_INT2_MASK; break;
//  #endif
//  #ifdef SPI_INT3_MASK
//  case 3: mask = SPI_INT3_MASK; break;
//  #endif
//  #ifdef SPI_INT4_MASK
//  case 4: mask = SPI_INT4_MASK; break;
//  #endif
//  #ifdef SPI_INT5_MASK
//  case 5: mask = SPI_INT5_MASK; break;
//  #endif
//  #ifdef SPI_INT6_MASK
//  case 6: mask = SPI_INT6_MASK; break;
//  #endif
//  #ifdef SPI_INT7_MASK
//  case 7: mask = SPI_INT7_MASK; break;
//  #endif
//  default:
//    interruptMode = 2;
//    break;
//  }
//  interruptMask |= mask;
//  if (!interruptMode)
//    interruptMode = 1;
//  SREG = sreg;
}

void SPIClass::notUsingInterrupt(uint8_t interruptNumber) {
//  // Once in mode 2 we can't go back to 0 without a proper reference count
//  if (interruptMode == 2)
//    return;
//  uint8_t mask = 0;
//  uint8_t sreg = SREG;
//  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
//  switch (interruptNumber) {
//  #ifdef SPI_INT0_MASK
//  case 0: mask = SPI_INT0_MASK; break;
//  #endif
//  #ifdef SPI_INT1_MASK
//  case 1: mask = SPI_INT1_MASK; break;
//  #endif
//  #ifdef SPI_INT2_MASK
//  case 2: mask = SPI_INT2_MASK; break;
//  #endif
//  #ifdef SPI_INT3_MASK
//  case 3: mask = SPI_INT3_MASK; break;
//  #endif
//  #ifdef SPI_INT4_MASK
//  case 4: mask = SPI_INT4_MASK; break;
//  #endif
//  #ifdef SPI_INT5_MASK
//  case 5: mask = SPI_INT5_MASK; break;
//  #endif
//  #ifdef SPI_INT6_MASK
//  case 6: mask = SPI_INT6_MASK; break;
//  #endif
//  #ifdef SPI_INT7_MASK
//  case 7: mask = SPI_INT7_MASK; break;
//  #endif
//  default:
//    break;
//    // this case can't be reached
//  }
//  interruptMask &= ~mask;
//  if (!interruptMask)
//    interruptMode = 0;
//  SREG = sreg;
}


/************************************************************************/
static void SPI_F103_Init_SPI(SPI_TypeDef *spi) {
  spi->CR2 = 0;
  spi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI  // NSS Software mode
           | SPI_FirstBit_MSB
           | SPI_DataSize_8b
           | SPI_CPOL_Low
           | SPI_CPHA_1Edge

           | SPI_CLOCK_DIV128    // default clock (minimum)
//           | SPI_CLOCK_DIV8

           | SPI_CR1_MSTR        // master mode
           | SPI_CR1_SPE;        // SPI enable
}

#define SPI_GPIO_Speed   GPIO_Speed_10MHz

static void SPI_F103_Init_GPIO(GPIO_TypeDef *port, int sck, int miso, int mosi) {
  RCC_enable_peripheral_clock(port);
  F103_GPIO_pinMode_output(port, sck,  GPIO_Mode_AF_PP | SPI_GPIO_Speed);
  F103_GPIO_pinMode_input( port, miso, GPIO_Mode_IPU);
  F103_GPIO_pinMode_output(port, mosi, GPIO_Mode_AF_PP | SPI_GPIO_Speed);
}


/************************************************************************/
static void SPI_F103_Init_SPI1Default() {
  // Step 1: Initialize SPI
  SPI_F103_Init_SPI(SPI1);

  // Step 2: Initialize GPIO: SCK_A5, MISO_A6, MOSI_A7
  SPI_F103_Init_GPIO(GPIOA, 5, 6, 7);

  printf("SPI done\n");
}


static void SPI_F103_Init_SPI1Remap() {
  // Step 1: Initialize SPI
  SPI_F103_Init_SPI(SPI1);

  // Step 2: Initialize GPIO_Remap: SCK_B3, MISO_B4, MOSI_B5
  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_AFIO);  // enable AFIO clock
  BITMASK_SET(AFIO->MAPR, GPIO_Remap_SPI1);
  SPI_F103_Init_GPIO(GPIOB, 3, 4, 5);
}


static void SPI_F103_Init_SPI2Default() {
  // Step 1: Initialize SPI
  SPI_F103_Init_SPI(SPI2);

  // Step 2: Initialize GPIO: SCK_B13, MISO_B14, MOSI_B15
  SPI_F103_Init_GPIO(GPIOB, 13, 14, 15);
}

/************************************************************************/
/// Select the communication: can be one of the following options
#if defined (STM32F10X_MD)
                                                   //  SCK    |  MISO   |  MOSI   | Clock_source (MHz) | Max baudrate (MHz)
  SPIClass SPI(SPI1, SPI_F103_Init_SPI1Default);   //  PA.5   |  PA.6   |  PA.7   |    72 (RCC_APB2)   |    36
//SPIClass SPI(SPI1, SPI_F103_Init_SPI1Remap);     //  PB.3   |  PB.4   |  PB.5   |    72 (RCC_APB2)   |    36
//SPIClass SPI(SPI2, SPI_F103_Init_SPI2Default);   //  PB.13  |  PB.14  |  PB.15  |    36 (RCC_APB1)   |    18

//#define UART1_SYNC_DEFAULT  //  PA.8   |  PA.10  |  PA.9   |    72 (RCC_APB2)   |    4.5
//#define UART1_SYNC_REMAP    //  PA.8   |  PB.7   |  PB.6   |    72 (RCC_APB2)   |    4.5
//#define UART2_SYNC_DEFAULT  //  PA.4   |  PA.3   |  PA.2   |    36 (RCC_APB1)   |    2.25
//#define UART3_SYNC_DEFAULT  //  PB.12  |  PB.11  |  PB.10  |    36 (RCC_APB1)   |    2.25
#endif


/************************************************************************/
