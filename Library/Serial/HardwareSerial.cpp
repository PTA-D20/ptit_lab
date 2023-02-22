
//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <inttypes.h>

//#include <util/atomic.h>
//#include "Arduino.h"

#include "Core.h"
#include "HardwareSerial.h"
//#include "HardwareSerial_private.h"

// Public Methods //////////////////////////////////////////////////////////////
void HardwareSerial::begin(unsigned long baud) {
//_written = false;
  USART1_begin_A9A10(baud);
}

void HardwareSerial::end() {
  flush();       // wait for transmission of outgoing data
  USART1_end();  // disable Hardware USART1
}

int HardwareSerial::available(void) {
  return USART1_available();
}

int HardwareSerial::peek(void) {
  return USART1_peek();
}

int HardwareSerial::read(void) {
  return USART1_read();
}

int HardwareSerial::availableForWrite(void) {
  return 1;
}

void HardwareSerial::flush() {
  USART1_flushTx();
//USART1_flushRx();
}

size_t HardwareSerial::write(uint8_t c) {
//_written = true;

  if (c != '\n')
    printf("%c", c);
  else
    printf("\n");

  return 1;
}

/// definition
HardwareSerial Serial;
