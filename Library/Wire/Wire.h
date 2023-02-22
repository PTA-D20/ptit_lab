/*
 * TwoWire.h - TWI/I2C library for MakerLabBoard(Arduino compatible)
 * Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com)
 * ported to stm32
 * Copyright (c) 2011 Cristian Maglie <c.maglie@bug.st>.
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TwoWire_h
#define TwoWire_h

#include "basic.h"

#define  TWI_CLOCK      400000

#define  BUFFER_LENGTH  32

class  TwoWire;
extern TwoWire Wire;

class TwoWire {
  public:
    TwoWire(I2C_TypeDef *_dev, IRQn_Type _irq, void(*begin_cb)(uint8_t addr));
    void begin(uint8_t addr = 0x00);
    void beginTransmission(uint8_t addr);
    int endTransmission(bool sendStop = true);
    int requestFrom(uint8_t addr, int quantity, bool sendStop = true);

    int WriteN(uint8_t addr, const uint8_t *buffer, int len, bool sendStop = true);
    int Write1(uint8_t addr, uint8_t wdata, bool sendStop = true);
    int Write1N(uint8_t addr, uint8_t wdata, const uint8_t *buffer, int len);
    int Write1N_16HL(uint8_t addr, uint8_t wdata, const uint16_t *buffer, int len);
		
    int ReadN(uint8_t addr, uint8_t *buffer, int len, bool sendStop = true);
    int Read1N(uint8_t addr, uint8_t wdata, uint8_t *buffer, int len);

    int write(uint8_t data);
    int write(const uint8_t *data, int quantity);
    int available(void);
    int read(void);
    int peek(void);
    void flush(void);

    void onReceive(void(*function)(int));
    void onRequest(void(*function)(void));
    void onService(void);

  private:

    bool beginTransfer(uint8_t addr);
    void checkStopTx(bool sendStop);
    void checkStopRx(bool sendStop);

    uint8_t txAddress;

    typedef struct {
      int     index;
      int     length;
      uint8_t data[BUFFER_LENGTH];
    } TWI_BUFFER;

    TWI_BUFFER txBuffer;
    TWI_BUFFER rxBuffer;
    TWI_BUFFER srvBuffer;

    bool waitEvent(uint32_t event, int32_t timeout);
    bool waitStop(int32_t timeout);

    static bool writeBuffer(TWI_BUFFER &buffer, uint8_t data);
    static int  writeBuffer(TWI_BUFFER &buffer, const uint8_t *data, int quantity);

    // Callback user functions
    void (*onRequestCallback)(void);
    void (*onReceiveCallback)(int);

    // Called before initialization
    void (*onBeginCallback)(uint8_t address);

    I2C_TypeDef *dev;
    IRQn_Type    devIRQ;

    // TWI state
    enum TwoWireStatus {
      UNINITIALIZED,
      MASTER_IDLE,
      MASTER_SEND,
      MASTER_RECV,
      SLAVE_IDLE,
      SLAVE_RECV,
      SLAVE_SEND
    };
    TwoWireStatus status;

    // Timeout: (us)
    static const uint32_t EVENT_TIMEOUT = 10000;
    static const uint32_t RECV_TIMEOUT  = 10000;
    static const uint32_t XMIT_TIMEOUT  = 10000;
};


#endif

