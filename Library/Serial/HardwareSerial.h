#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Print.h"

class HardwareSerial : public Print {
  protected:
    // Has any byte been written to the UART since begin()
  //bool _written;

  public:
//     HardwareSerial() {}
//    ~HardwareSerial() {}

    void begin(unsigned long baud);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual int availableForWrite(void);
    virtual void flush(void);

    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n)          { return write((uint8_t)n); }
    inline size_t write(unsigned int n)  { return write((uint8_t)n); }
    inline size_t write(int n)           { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }

//    // Interrupt handlers - Not intended to be called externally
//    inline void _rx_complete_irq(void);
//    void _tx_udr_empty_irq(void);
};

/// declaration
extern HardwareSerial Serial;  // definition in HardwareSerial.cpp

#endif
