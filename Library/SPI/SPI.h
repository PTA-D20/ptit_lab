
#ifndef SPI_H
#define SPI_H




#include "basic.h"
//#include <Arduino.h>

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

// SPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that SPI_HAS_TRANSACTION as documented above is
// available too.
#define SPI_ATOMIC_VERSION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use SPI.endTransaction() for
// each SPI.beginTransaction().  Connect an LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define SPI_TRANSACTION_MISMATCH_LED  5

//#ifndef LSBFIRST
//  #define LSBFIRST 0
//#endif
//
//#ifndef MSBFIRST
//  #define MSBFIRST 1
//#endif

#define SPI_MSBFIRST  0
#define SPI_LSBFIRST  1


#define SPI_CLOCK_DIV2    SPI_BaudRatePrescaler_2
#define SPI_CLOCK_DIV4    SPI_BaudRatePrescaler_4
#define SPI_CLOCK_DIV8    SPI_BaudRatePrescaler_8
#define SPI_CLOCK_DIV16   SPI_BaudRatePrescaler_16
#define SPI_CLOCK_DIV32   SPI_BaudRatePrescaler_32
#define SPI_CLOCK_DIV64   SPI_BaudRatePrescaler_64
#define SPI_CLOCK_DIV128  SPI_BaudRatePrescaler_128
#define SPI_CLOCK_DIV256  SPI_BaudRatePrescaler_256

#define SPI_MODE0         0x00
#define SPI_MODE1         0x01
#define SPI_MODE2         0x02
#define SPI_MODE3         0x03

//#define SPI_MODE_MASK     0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
//#define SPI_CLOCK_MASK    0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
//#define SPI_2XCLOCK_MASK  0x01  // SPI2X = bit 0 on SPSR

//#define F_CPU   (72000000UL)

class SPISettings {
  public:
    SPISettings();
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);

  private:
    void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
    void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
  //  uint8_t spcr;
  //  uint8_t spsr;
    friend class SPIClass;
};


class SPIClass {
public:
  SPIClass(SPI_TypeDef *_dev, void(*begin_cb)());

  // Initialize the SPI library
  void begin();

  // If SPI is used from within an interrupt, this function registers
  // that interrupt with the SPI library, so beginTransaction() can
  // prevent conflicts.  The input interruptNumber is the number used
  // with attachInterrupt.  If SPI is used from a different interrupt
  // (eg, a timer), interruptNumber should be 255.
  static void usingInterrupt(uint8_t interruptNumber);
  // And this does the opposite.
  static void notUsingInterrupt(uint8_t interruptNumber);
  // Note: the usingInterrupt and notUsingInterrupt functions should
  // not to be called from ISR context or inside a transaction.
  // For details see:
  // https://github.com/arduino/Arduino/pull/2381
  // https://github.com/arduino/Arduino/pull/2449

  // Before using SPI.transfer() or asserting chip select pins,
  // this function is used to gain exclusive access to the SPI bus
  // and configure the correct settings.
  void beginTransaction(SPISettings settings) {
//    if (interruptMode > 0) {
//      uint8_t sreg = SREG;
//      noInterrupts();
//
//      #ifdef SPI_AVR_EIMSK
//      if (interruptMode == 1) {
//        interruptSave = SPI_AVR_EIMSK;
//        SPI_AVR_EIMSK &= ~interruptMask;
//        SREG = sreg;
//      } else
//      #endif
//      {
//        interruptSave = sreg;
//      }
//    }
//
//    #ifdef SPI_TRANSACTION_MISMATCH_LED
//    if (inTransactionFlag) {
//      pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
//      digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
//    }
//    inTransactionFlag = 1;
//    #endif
//
//    SPCR = settings.spcr;
//    SPSR = settings.spsr;
  }


  uint8_t transfer(uint8_t data);
  void transfer(void *buf, int len);
//uint16_t transfer16(uint16_t data);

  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the SPI bus
  void endTransaction(void) {
//    #ifdef SPI_TRANSACTION_MISMATCH_LED
//    if (!inTransactionFlag) {
//      pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
//      digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
//    }
//    inTransactionFlag = 0;
//    #endif
//
//    if (interruptMode > 0) {
//      #ifdef SPI_AVR_EIMSK
//      uint8_t sreg = SREG;
//      #endif
//      noInterrupts();
//      #ifdef SPI_AVR_EIMSK
//      if (interruptMode == 1) {
//        SPI_AVR_EIMSK = interruptSave;
//        SREG = sreg;
//      } else
//      #endif
//      {
//        SREG = interruptSave;
//      }
//    }
  }

  // Disable the SPI bus
  void end();

  void setBitOrder(uint8_t bitOrder);
  void setDataMode(uint8_t dataMode);
  void setClockDivider(uint8_t clockDiv);
  void attachInterrupt();
  void detachInterrupt();


private:
  SPI_TypeDef *dev;

  // Called before initialization
  void (*onBeginCallback)();

  static uint8_t initialized;
  static uint8_t interruptMode; // 0=none, 1=mask, 2=global
  static uint8_t interruptMask; // which interrupts to mask
  static uint8_t interruptSave; // temp storage, to restore state

  #ifdef SPI_TRANSACTION_MISMATCH_LED
    static uint8_t inTransactionFlag;
  #endif
};

extern SPIClass SPI;

#endif // SPI_H
