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

#include "Wire.h"
#include "Core.h"


#define I2C_ADDR_TRANSMITTER(addr)     (((addr) << 1) | 0)
#define I2C_ADDR_RECEIVER(addr)        (((addr) << 1) | 1)


TwoWire::TwoWire(I2C_TypeDef *_dev, IRQn_Type _irq, void(*_beginCb)(uint8_t addr)) :
	dev(_dev),
	devIRQ(_irq),
  status(UNINITIALIZED),
  onBeginCallback(_beginCb) {
  // empty
}


void TwoWire::begin(uint8_t addr) {
	onBeginCallback(addr);

  BITMASK_SET(SCB->SCR, SCB_SCR_SEVONPEND_Msk);  // enable system events
  BITMASK_SET(dev->CR2, I2C_IT_EVT);             // enable event interrupt

	status = MASTER_IDLE;
}


bool TwoWire::beginTransfer(uint8_t addr) {
  /// Step 0: Wait until STOP bit is cleared by hardware
  if (!waitStop(EVENT_TIMEOUT))
    return false;

  /// Step 1: Start
  BITMASK_SET(dev->CR1, I2C_CR1_START);
	if (!waitEvent(I2C_EVENT_MASTER_MODE_SELECT, EVENT_TIMEOUT))
    return false;

  /// Step 2: Send Slave address:
  dev->DR = addr;

  uint32_t event   = (addr & 1) ? I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED : I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
  uint32_t timeout = (addr & 1) ? RECV_TIMEOUT : XMIT_TIMEOUT;
  if (!waitEvent(event, timeout))
    return false;

  return true;
}


void TwoWire::checkStopTx(bool sendStop) {
	if (sendStop == true)
		BITMASK_SET(dev->CR1, I2C_CR1_STOP);  // send STOP condition
	else
    dev->DR;  // clear BTF event flag
}


void TwoWire::checkStopRx(bool sendStop) {
  if (sendStop == true)
    dev->CR1 = (dev->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;  // disable ACK & send STOP
}


bool TwoWire::waitStop(int32_t timeout) {
	while ((dev->CR1 & I2C_CR1_STOP) != 0) {
    delay_us(1);
    if ((--timeout) <= 0)
      return false;
  }

  return true;
}


bool TwoWire::waitEvent(uint32_t event, int32_t timeout) {
  NVIC_ClearPendingIRQ(devIRQ);  // clear prev pending
  timeout_start_us(timeout);
  while ((LH32(dev->SR1, dev->SR2) & event) != event) {
    if (!timeout_is_busy()) {    // sleep, wait new pending
      timeout_end();
      return false;
    }
  }
  timeout_end();
  return true;
}


int TwoWire::requestFrom(uint8_t addr, int quantity, bool sendStop) {
  if (quantity <= 0)
    return 0;

  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_RECEIVER(addr)) )
    return 0;

	/// Step 3: read into buffer
  BITMASK_SET(dev->CR2, I2C_IT_BUF);  // enable interrupt: I2C_EVENT_MASTER_BYTE_RECEIVED

	if (quantity > BUFFER_LENGTH)
		quantity = BUFFER_LENGTH;

  for (int i = 0 ; i < (quantity - 1); i++) {
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, RECV_TIMEOUT)) {
      BITMASK_CLEAR(dev->CR2, I2C_IT_BUF);  // disable interrupt
      return 0;
    }
    rxBuffer.data[i] = (uint8_t)dev->DR;  // read a byte from the Slave
  }

  checkStopRx(sendStop);

  if (!waitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, RECV_TIMEOUT)) {
    BITMASK_CLEAR(dev->CR2, I2C_IT_BUF);  // disable interrupt
    return 0;
  }
  // Read a byte from the Slave: I2C_ReceiveData(twi);
  rxBuffer.data[quantity - 1] = (uint8_t)dev->DR;
  BITMASK_CLEAR(dev->CR2, I2C_IT_BUF);  // disable interrupt

	// Enable Acknowledgement to be ready for another reception
	BITMASK_SET(dev->CR1, I2C_CR1_ACK);

	// set rx buffer iterator vars
  rxBuffer.index = 0;
  rxBuffer.length = quantity;
	return quantity;
}


void TwoWire::beginTransmission(uint8_t addr) {
	status = MASTER_SEND;
	// save address of target and empty buffer
	txAddress = addr;
  txBuffer.length = 0;
}


//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
int TwoWire::endTransmission(bool sendStop) {
  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_TRANSMITTER(txAddress)) )
    return -1;

  /// Step 3: write buffer
	for (int i = 0; i < txBuffer.length; i++) {
		dev->DR = txBuffer.data[i];  // send the current byte to slave
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
      return -1;
	}

  checkStopTx(sendStop);

	// reset tx buffer iteratorvars
  txBuffer.length = 0;
	status = MASTER_IDLE;
	return 0;
}


int TwoWire::WriteN(uint8_t addr, const uint8_t *buffer, int len, bool sendStop) {
  if (len <= 0)
    return 0;

  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_TRANSMITTER(addr)) )
    return -1;

  /// Step 3: write buffer
	for (int i = 0; i < len; i++) {
		// Send the current byte to slave
		dev->DR = buffer[i];
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
      return -1;
	}

	checkStopTx(sendStop);

	return 0;
}


int TwoWire::Write1(uint8_t addr, uint8_t wdata, bool sendStop) {
  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_TRANSMITTER(addr)) )
    return -1;

  /// Step 3: Write data
  dev->DR = wdata;
  if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
    return -1;

	checkStopTx(sendStop);

	return 0;
}


int TwoWire::Write1N(uint8_t addr, uint8_t wdata, const uint8_t *buffer, int len) {
  if (len <= 0)
    return -1;

  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_TRANSMITTER(addr)) )
    return -1;

  /// Step 3: Write wdata
  dev->DR = wdata;
  if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
    return -1;

  /// Step 4: Write buffer
	for (int i = 0; i < len; i++) {
		dev->DR = buffer[i];
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
      return -1;
	}

  delay_us(1);
  BITMASK_SET(dev->CR1, I2C_CR1_STOP);  // send STOP condition
	return 0;
}


int TwoWire::Write1N_16HL(uint8_t addr, uint8_t wdata, const uint16_t *buffer, int len) {
  if (len <= 0)
    return -1;

  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_TRANSMITTER(addr)) )
    return -1;

  /// Step 3: Write wdata
  dev->DR = wdata;
  if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
    return -1;

  /// Step 4: Write buffer
	for (int i = 0; i < len; i++) {
    dev->DR = (uint8_t)(buffer[i] >> 8);
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
      return -1;

    dev->DR = (uint8_t)buffer[i];
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, XMIT_TIMEOUT))
      return -1;
	}
	delay_us(1);
  BITMASK_SET(dev->CR1, I2C_CR1_STOP);  // send STOP condition
  return 0;
}


int TwoWire::ReadN(uint8_t addr, uint8_t *buffer, int len, bool sendStop) {
  if (len <= 0)
    return 0;

  /// Step 0, 1, 2:
  if ( !beginTransfer(I2C_ADDR_RECEIVER(addr)) )
    return 0;

	/// Step 3: Read into buffer
  BITMASK_SET(dev->CR2, I2C_IT_BUF);  // enable interrupt: I2C_EVENT_MASTER_BYTE_RECEIVED

  for (int i = 0 ; i < (len - 1); i++) {
    if (!waitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, RECV_TIMEOUT)) {
      BITMASK_CLEAR(dev->CR2, I2C_IT_BUF);  // disable interrupt
      return 0;
    }
    // Read a byte from the Slave:
    buffer[i] = (uint8_t)dev->DR;
  }

  checkStopRx(sendStop);

  if (!waitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, RECV_TIMEOUT)) {
    BITMASK_CLEAR(dev->CR2, I2C_IT_BUF);  // disable interrupt
    return 0;
  }

  // Read a byte from the Slave:
  buffer[len - 1] = (uint8_t)dev->DR;
  BITMASK_CLEAR(dev->CR2, I2C_IT_BUF);  // disable interrupt

	// Enable Acknowledgement to be ready for another reception
	BITMASK_SET(dev->CR1, I2C_CR1_ACK);

	return len;
}


int TwoWire::Read1N(uint8_t addr, uint8_t wdata, uint8_t *buffer, int len) {
  Write1(addr, wdata, false);
  return ReadN(addr, buffer, len);
}


bool TwoWire::writeBuffer(TWI_BUFFER &buffer, uint8_t data) {
  if (buffer.length >= BUFFER_LENGTH)
    return false;
  buffer.data[buffer.length++] = data;
  return true;
}

int TwoWire::writeBuffer(TWI_BUFFER &buffer, const uint8_t *data, int len) {
  for (int i = 0; i < len; i++) {
    if (!writeBuffer(buffer, data[i]))
      return i;
  }
  return len;
}

int TwoWire::write(uint8_t data) {
  switch (status) {
    case MASTER_SEND: return writeBuffer(txBuffer, data);
    default :         return writeBuffer(srvBuffer, data);
  }
}

int TwoWire::write(const uint8_t *data, int quantity) {
  switch (status) {
    case MASTER_SEND: return writeBuffer(txBuffer, data, quantity);
    default :         return writeBuffer(srvBuffer, data, quantity);
  }
}


int TwoWire::available(void) {
  return rxBuffer.length - rxBuffer.index;
}

int TwoWire::read(void) {
	if (rxBuffer.index >= rxBuffer.length)
    return -1;
	return rxBuffer.data[rxBuffer.index++];
}

int TwoWire::peek(void) {
	if (rxBuffer.index >= rxBuffer.length)
    return -1;
  return rxBuffer.data[rxBuffer.index];
}

void TwoWire::flush(void) {
	// Do nothing, use endTransmission(..) to force data transfer
}

void TwoWire::onReceive(void(*function)(int)) {
	onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void)) {
	onRequestCallback = function;
}


/************************************************************************/
void Wire_Init_F103_I2C(I2C_TypeDef *I2Cx, uint8_t addr) {
  I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.I2C_ClockSpeed = TWI_CLOCK;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = addr;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;  // phải enable để sẵn sàng nhận dữ liệu
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2Cx, &I2C_InitStruct);
  I2C_Cmd(I2Cx, ENABLE);
}

void Wire_Init_F103_GPIOB(uint32_t GPIO_SCL_SDA) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_SCL_SDA;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}


static void Wire_Init_F103_I2C1_B6B7(uint8_t address) {
  /// Step 1: Initialize I2C
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  Wire_Init_F103_I2C(I2C1, address);

  /// Step 2: Initialize GPIO as open drain alternate function
  Wire_Init_F103_GPIOB(GPIO_Pin_6 | GPIO_Pin_7);
}


/************************************************************************/
static void Wire_Init_F103_I2C1_B8B9(uint8_t address) {
  /// Step 1: Initialize I2C
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  Wire_Init_F103_I2C(I2C1, address);

  /// Step 2: Initialize GPIO as open drain alternate function
  // I2C1_GPIOB remapping
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  BITMASK_SET(AFIO->MAPR, AFIO_MAPR_I2C1_REMAP);

  Wire_Init_F103_GPIOB(GPIO_Pin_8 | GPIO_Pin_9);
}


/************************************************************************/
static void Wire_Init_F103_I2C2_B10B11(uint8_t address) {
  /// Step 1: Initialize I2C
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  Wire_Init_F103_I2C(I2C2, address);

  /// Step 2: Initialize GPIO as open drain alternate function
  Wire_Init_F103_GPIOB(GPIO_Pin_10 | GPIO_Pin_11);
}


/************************************************************************/
//TwoWire Wire = TwoWire(I2C1, I2C1_EV_IRQn, Wire_Init_F103_I2C1_B6B7);
//TwoWire Wire = TwoWire(I2C1, I2C1_EV_IRQn, Wire_Init_F103_I2C1_B8B9);
  TwoWire Wire = TwoWire(I2C2, I2C2_EV_IRQn, Wire_Init_F103_I2C2_B10B11);


/************************************************************************/
void TwoWire::onService(void) {
//#if 0
//	// Retrieve interrupt status
//	uint32_t sr = TWI_GetStatus(twi);
//
//	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
//		TWI_DisableIt(twi, TWI_IDR_SVACC);
//		TWI_EnableIt(twi, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
//				| TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);
//
//		srvBufferLength = 0;
//		srvBufferIndex = 0;
//
//		// Detect if we should go into RECV or SEND status
//		// SVREAD==1 means *master* reading -> SLAVE_SEND
//		if (!TWI_STATUS_SVREAD(sr)) {
//			status = SLAVE_RECV;
//		} else {
//			status = SLAVE_SEND;
//
//			// Alert calling program to generate a response ASAP
//			if (onRequestCallback)
//				onRequestCallback();
//			else
//				// create a default 1-byte response
//				write((uint8_t) 0);
//		}
//	}
//
//	if (status != SLAVE_IDLE) {
//		if (TWI_STATUS_TXCOMP(sr) && TWI_STATUS_EOSACC(sr)) {
//			if (status == SLAVE_RECV && onReceiveCallback) {
//				// Copy data into rxBuffer
//				// (allows to receive another packet while the
//				// user program reads actual data)
//				for (uint8_t i = 0; i < srvBufferLength; ++i)
//					rxBuffer[i] = srvBuffer[i];
//				rxBufferIndex = 0;
//				rxBufferLength = srvBufferLength;
//
//				// Alert calling program
//				onReceiveCallback( rxBufferLength);
//			}
//
//			// Transfer completed
//			TWI_EnableIt(twi, TWI_SR_SVACC);
//			TWI_DisableIt(twi, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
//					| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
//			status = SLAVE_IDLE;
//		}
//	}
//
//	if (status == SLAVE_RECV) {
//		if (TWI_STATUS_RXRDY(sr)) {
//			if (srvBufferLength < BUFFER_LENGTH)
//				srvBuffer[srvBufferLength++] = TWI_ReadByte(twi);
//		}
//	}
//
//	if (status == SLAVE_SEND) {
//		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
//			uint8_t c = 'x';
//			if (srvBufferIndex < srvBufferLength)
//				c = srvBuffer[srvBufferIndex++];
//			TWI_WriteByte(twi, c);
//		}
//	}
//#endif
}


/*==============================================================================

Change log

180830 - TranDucNam
  Bug: timeout khi gọi:
    Wire.endTransmission(false);
    Wire.requestFrom(...);

  Cause:
    Trong endTransmission(): sau khi truyền xong dữ liệu, BTF được bật
    BTF được xóa khi có STOP hoặc đọc DR
      RM0008 - STM32F10xxx Reference manual
        26.6.6 - I2C status register I2C_SR1
          Bit 2 - BTF: Byte transfer finished (page 781)

    Nếu sendStop = false, sẽ không có STOP, nên BTF không được xóa
    Dẫn tới timeout khi waitEvent() trong requestFrom()

  Fix: xóa BTF khi sendStop = false
    if (sendStop == true)
      BITMASK_SET(twi->CR1, I2C_CR1_STOP);
    else
      twi->DR;  // clear BTF event flag

==============================================================================*/
