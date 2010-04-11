/*
 AsyncTwi.cpp - Asyncronous TWI/I2C library for Wiring & Arduino
 Copyright (c) 2010 Garth Minette.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


// #include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <string.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "AsyncTwi.h"


static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static uint8_t twi_txBufferLength;
static uint8_t twi_txBufferIndex;

static volatile uint8_t twi_masterBuffer[128];
static volatile uint8_t twi_masterHeadIdx;
static volatile uint8_t twi_masterTailIdx;
static volatile uint8_t twi_masterFrameIdx;
static volatile uint8_t twi_masterPayloadEndIdx;
static volatile uint8_t twi_masterPayloadSent;
static volatile uint8_t twi_framesPending;



/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void)
{
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__)
  // activate internal pull-ups for twi
  // as per note from atmega8 manual pg167
  sbi(PORTC, 4);
  sbi(PORTC, 5);
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
  // activate internal pull-ups for twi
  sbi(PORTC, 0);
  sbi(PORTC, 1);
#else
  // activate internal pull-ups for twi
  // as per note from atmega128 manual pg204
  sbi(PORTD, 0);
  sbi(PORTD, 1);
#endif

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
   SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
   note: TWBR should be 10 or higher for master mode
   It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);

  twi_rxBufferIndex = 0;
  twi_txBufferLength = 0;
  twi_txBufferIndex = 0;
  twi_masterHeadIdx = 0;
  twi_masterTailIdx = 0;
  twi_masterFrameIdx = 0;
  twi_masterPayloadEndIdx = 0;
  twi_masterPayloadSent = 0;
  twi_framesPending = 0;
}



static void twi_rollback() {
    uint8_t tmpsreg = SREG;
    cli();
    twi_masterHeadIdx = twi_masterFrameIdx;
    SREG = tmpsreg;
}


static void twi_commit() {
    uint8_t tmpsreg = SREG;
    cli();
    twi_masterFrameIdx = twi_masterHeadIdx;
    twi_framesPending--;
    SREG = tmpsreg;
}


uint8_t twi_available_tx_bytes() {
    uint8_t tmpsreg = SREG;
    cli();
    int8_t len = twi_masterTailIdx - twi_masterFrameIdx;
    if (len < 0) {
    	len += sizeof(twi_masterBuffer);
    }
    SREG = tmpsreg;
    return sizeof(twi_masterBuffer) - len;
}


uint8_t twi_pending() {
    return twi_framesPending;
}


static uint8_t twi_getByte() {
    uint8_t tmpsreg = SREG;
    cli();
    if (twi_masterHeadIdx == twi_masterTailIdx) {
        return 0;
    }
    uint8_t val = twi_masterBuffer[twi_masterHeadIdx++];
    if (twi_masterHeadIdx >= sizeof(twi_masterBuffer)) {
	twi_masterHeadIdx -= sizeof(twi_masterBuffer);
    }
    SREG = tmpsreg;
    return val;
}


static uint8_t twi_appendByte(uint8_t val) {
    uint8_t tmpsreg = SREG;
    cli();
    if (twi_available_tx_bytes() < 1) {
        return -1;
    }
    twi_masterBuffer[twi_masterTailIdx++] = val;
    if (twi_masterTailIdx >= sizeof(twi_masterBuffer)) {
	twi_masterTailIdx -= sizeof(twi_masterBuffer);
    }
    uint8_t avail = twi_available_tx_bytes();
    SREG = tmpsreg;
    return avail;
}


static uint8_t twi_currentFrameAddress() {
    uint8_t tmpsreg = SREG;
    cli();
    if (twi_masterHeadIdx == twi_masterTailIdx) {
    	return 0;
    }
    uint8_t lenidx = twi_masterFrameIdx + 1;
    if (lenidx >= sizeof(twi_masterBuffer)) {
	lenidx -= sizeof(twi_masterBuffer);
    }
    uint8_t val = twi_masterBuffer[twi_masterFrameIdx];
    twi_masterPayloadEndIdx = twi_masterFrameIdx + 2 + twi_masterBuffer[lenidx];
    if (twi_masterPayloadEndIdx >= sizeof(twi_masterBuffer)) {
        twi_masterPayloadEndIdx -= sizeof(twi_masterBuffer);
    }
    SREG = tmpsreg;
    return val;
}


static uint8_t twi_currentPayloadSize() {
    uint8_t tmpsreg = SREG;
    cli();
    if (twi_masterHeadIdx == twi_masterTailIdx) {
    	return 0;
    }
    uint8_t idx = twi_masterFrameIdx + 1;
    if (idx >= sizeof(twi_masterBuffer)) {
	idx -= sizeof(twi_masterBuffer);
    }
    uint8_t val = twi_masterBuffer[idx];
    SREG = tmpsreg;
    return val;
}



/* 
 * Function twi_stop
 * Desc     grabs bus master status
 * Input    none
 * Output   none
 */
static void twi_start()
{
    // send start condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
}



/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
static void twi_stop(void)
{
    // send stop condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

    // wait for stop condition to be exectued on bus
    // TWINT is not set after a stop condition!
    while(TWCR & _BV(TWSTO)){
	continue;
    }
}



/* 
 * Function twi_stop_or_restart
 * Desc     if no more frames are pending, sends STOP condition, otherwise sends RESTART condition.
 * Input    none
 * Output   none
 */
static void twi_stop_or_restart()
{
    if (twi_framesPending > 0) {
	twi_start();
    } else {
	twi_stop();
    }
}



/* 
 * Function twi_releaseBus
 * Desc     releases bus control without Stop signal.
 * Input    none
 * Output   none
 */
static void twi_releaseBus(void)
{
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
}



/* 
 * Function twi_ack
 * Desc     sends ACK/NAK or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
static void twi_ack(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }
  else{
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}



/* 
 * Function twi_write
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          len: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          cb: callback routine to call when write is complete.
 * Output   number of payload data bytes sent, or 0 if call is used asyncronously.
 *          -1 if not enough buffer space was free.
 */
uint8_t twi_write(uint8_t address, uint8_t* data, uint8_t len, uint8_t wait, twi_callback_t cb)
{
    address &= 0xfe;
    address |= TW_WRITE;

    uint8_t tmpsreg = SREG;
    cli();

    // ensure frame will fit into buffer
    if(twi_available_tx_bytes() < len+4){
	SREG = tmpsreg;
	return -1;
    }

    twi_appendByte(address);
    twi_appendByte(len);
    for (uint8_t i = 0; i < len; i++) {
	twi_appendByte(data[i]);
    }
    twi_appendByte(((uint16_t)cb) & 0xff);
    twi_appendByte((((uint16_t)cb)>>8) & 0xff);
    if (twi_framesPending++ == 0) {
        twi_start();
    }
    SREG = tmpsreg;
    if (wait) {
	while (twi_framesPending > 0) {
	    continue;
	}
	return twi_masterPayloadSent;
    }
    return 0;
}


/* 
 * Function twi_read_async
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          maxlen: number of bytes to read into array
 *          cb: callback routine to call for asyncronous reads.
 * Output   number of bytes read, or 0 if call is used asyncronously
 *          or -1 if not enough buffer space was free.
 */
uint8_t twi_read_async(uint8_t address, uint8_t maxlen, twi_callback_t cb)
{
    uint8_t tmpsreg = SREG;
    cli();

    // ensure frame will fit into buffer
    if(twi_available_tx_bytes() < 4){
	SREG = tmpsreg;
	return -1;
    }

    if(TWI_BUFFER_LENGTH < maxlen){
	maxlen = TWI_BUFFER_LENGTH;
    }

    address &= 0xfe;
    address |= TW_READ;

    twi_appendByte(address);
    twi_appendByte(maxlen);
    twi_appendByte(((uint16_t)cb) & 0xff);
    twi_appendByte((((uint16_t)cb)>>8) & 0xff);

    if (twi_framesPending == 0) {
        twi_start();
    }
    twi_framesPending++;
    SREG = tmpsreg;
    if (!cb) {
	while (twi_framesPending > 0) {
	    continue;
	}
	return twi_rxBufferIndex;
    }
    return 0;
}



/* 
 * Function twi_read
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          buf: pointer to byte array to put read data into
 *          maxlen: max number of bytes to read into array
 * Output   number of bytes read, or 0 if call is used asyncronously
 *          or -1 if not enough buffer space was free.
 */
uint8_t twi_read(uint8_t address, uint8_t* buf, uint8_t maxlen)
{
    uint8_t tmpsreg = SREG;
    cli();

    // ensure frame will fit into buffer
    if(twi_available_tx_bytes() < 4){
	SREG = tmpsreg;
	return -1;
    }

    if(TWI_BUFFER_LENGTH < maxlen){
	maxlen = TWI_BUFFER_LENGTH;
    }

    address &= 0xfe;
    address |= TW_READ;

    twi_appendByte(address);
    twi_appendByte(maxlen);
    twi_appendByte(0x00);
    twi_appendByte(0x00);

    if (twi_framesPending == 0) {
        twi_start();
    }
    twi_framesPending++;
    SREG = tmpsreg;

    uint8_t numbytes = 0;
    while (1) {
	uint8_t tmpsreg = SREG;
	cli();
	if (twi_framesPending == 0) {
	    numbytes = twi_rxBufferIndex;
	    memcpy(buf, twi_rxBuffer, numbytes);
	    SREG = tmpsreg;
	    break;
	}
	SREG = tmpsreg;
	__asm__("nop\n\t");
	__asm__("nop\n\t");
	__asm__("nop\n\t");
	__asm__("nop\n\t");
    }
    return numbytes;
}



/* 
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   -1 length too long for buffer
 *          0 ok
 */
uint8_t twi_transmit(uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return -1;
  }

  // set length and copy data into tx buffer
  twi_txBufferLength = length;
  for(i = 0; i < length; ++i){
    twi_txBuffer[i] = data[i];
  }

  return 0;
}



/* 
 * Function twi_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void twi_setAddress(uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}



/* 
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) )
{
  twi_onSlaveReceive = function;
}



/* 
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveTxEvent( void (*function)(void) )
{
  twi_onSlaveTransmit = function;
}



/* Interrupt handler and TWI state engine. */
SIGNAL(SIG_2WIRE_SERIAL)
{
  if (TWCR & _BV(TWINT) == 0) {
      return;
  }
  twi_callback_t cb;
  uint16_t cba;

  switch(TW_STATUS){

  /////////////////////////////////////
  // ALL MASTER MODES
  /////////////////////////////////////
  case TW_START:     // sent start condition
  case TW_REP_START: // sent repeated start condition
    // copy device address and r/w bit to output register and ack
    TWDR = twi_currentFrameAddress();
    twi_ack(1);
    twi_getByte();
    twi_getByte();
    twi_rxBufferIndex = 0;
    break;

  /////////////////////////////////////
  // MASTER TRANSMITTER
  /////////////////////////////////////
  case TW_MT_SLA_ACK:  // slave receiver acked address
    twi_masterPayloadSent = 0;
  case TW_MT_DATA_ACK: // slave receiver acked data
    // if there is data to send, send it, otherwise stop 
    if (twi_masterHeadIdx != twi_masterPayloadEndIdx) {
      TWDR = twi_getByte();
      twi_ack(1);
      twi_masterPayloadSent++;
    }
    else{
      cba = twi_getByte();
      cba |= twi_getByte() << 8;
      if (cba) {
	cb = (twi_callback_t)cba;
	cb(twi_currentFrameAddress(), NULL, twi_masterPayloadSent);
      }
      twi_commit();
      twi_stop_or_restart();
    }
    break;

  case TW_MT_SLA_NACK:  // address sent, nack received
    // Discard rest of data in frame.
    if (twi_masterHeadIdx != twi_masterPayloadEndIdx) {
      twi_getByte();
    }
    twi_commit();   // On to the next packet.
    twi_stop_or_restart();
    // Tell callback we sent no payload data.
    twi_masterPayloadSent = 0;

    cba = twi_getByte();
    cba |= twi_getByte() << 8;
    if (cba) {
      cb = (twi_callback_t)cba;
      cb(twi_currentFrameAddress(), NULL, twi_masterPayloadSent);
    }
    break;

  case TW_MT_DATA_NACK: // data sent, nack received
    // Discard rest of data in frame.
    if (twi_masterHeadIdx != twi_masterPayloadEndIdx) {
      twi_getByte();
    }
    twi_commit();   // On to the next packet.
    twi_stop_or_restart();
    // Tell callback how many payload bytes we sent.
    cba = twi_getByte();
    cba |= twi_getByte() << 8;
    if (cba) {
      cb = (twi_callback_t)cba;
      cb(twi_currentFrameAddress(), NULL, twi_masterPayloadSent);
    }
    break;

  case TW_MT_ARB_LOST: // We lost bus arbitration
    twi_rollback();      // We'll resend this packet later.
    twi_releaseBus();    // Let the other master have bus control.
    twi_start();         // Try to re-gain bus after other master finishes.
    break;

  /////////////////////////////////////
  // MASTER RECEIVER
  /////////////////////////////////////
  case TW_MR_DATA_ACK: // data received, ack sent
    // put byte into buffer
    twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
    // ack if more bytes are expected, otherwise nack
    if(twi_rxBufferIndex < twi_currentPayloadSize()-1) {
      twi_ack(1);
    }
    else{
      twi_ack(0);
    }
    break;

  case TW_MR_SLA_ACK:  // address sent, ack received
    // ack if more bytes are expected, otherwise nack
    if(twi_rxBufferIndex < twi_currentPayloadSize()-1) {
      twi_ack(1);
    }
    else{
      twi_ack(0);
    }
    break;

  case TW_MR_DATA_NACK: // data received, nack sent
    // put final byte into buffer
    twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
  case TW_MR_SLA_NACK: // address sent, nack received
    cba = twi_getByte();
    cba |= twi_getByte() << 8;
    if (cba) {
      cb = (twi_callback_t)cba;
      cb(twi_currentFrameAddress(), twi_rxBuffer, twi_rxBufferIndex);
    }
    twi_commit();
    twi_stop_or_restart();
    break;

    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

  /////////////////////////////////////
  // SLAVE RECEIVER
  /////////////////////////////////////
  case TW_SR_SLA_ACK:   // addressed, returned ack
  case TW_SR_GCALL_ACK: // addressed generally, returned ack
  case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
  case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
    // indicate that rx buffer can be overwritten and ack
    twi_rxBufferIndex = 0;
    twi_ack(1);
    break;

  case TW_SR_DATA_ACK:       // data received, returned ack
  case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
    // if there is still room in the rx buffer
    if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
      // put byte in buffer and ack
      twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
      twi_ack(1);
    }
    else{
      // otherwise nack
      twi_ack(0);
    }
    break;

  case TW_SR_STOP: // stop or repeated start condition received
    // put a null char after data if there's room
    if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
      twi_rxBuffer[twi_rxBufferIndex] = '\0';
    }
    // callback to user defined callback
    // TODO: Fix this
    twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
    // ack future responses
    twi_ack(1);
    break;

  case TW_SR_DATA_NACK:       // data received, returned nack
  case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
    // nack back at master
    twi_ack(0);
    break;

  /////////////////////////////////////
  // SLAVE TRANSMITTER
  /////////////////////////////////////
  case TW_ST_SLA_ACK:          // addressed, returned ack
  case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
    // ready the tx buffer index for iteration
    twi_txBufferIndex = 0;
    // set tx buffer length to be zero, to verify if user changes it
    twi_txBufferLength = 0;
    // request for txBuffer to be filled and length to be set
    // note: user must call twi_transmit(bytes, length) to do this
    twi_onSlaveTransmit();
    // if they didn't change buffer & length, initialize it
    if(0 == twi_txBufferLength){
      twi_txBufferLength = 1;
      twi_txBuffer[0] = 0x00;
    }
    // transmit first byte from buffer, fall
  case TW_ST_DATA_ACK: // byte sent, ack returned
    // copy data to output register
    TWDR = twi_txBuffer[twi_txBufferIndex++];
    // if there is more to send, ack, otherwise nack
    if(twi_txBufferIndex < twi_txBufferLength){
      twi_ack(1);
    }
    else{
      twi_ack(0);
    }
    break;
  case TW_ST_DATA_NACK: // received nack, we are done 
  case TW_ST_LAST_DATA: // received ack, but we are done already!
    // ack future responses
    twi_ack(1);
    break;

  /////////////////////////////////////
  // WIERD STATES
  /////////////////////////////////////
  case TW_NO_INFO:   // no state information
    break;
  case TW_BUS_ERROR: // bus error, illegal stop/start
    twi_stop();
    if (twi_framesPending > 0) {
      twi_start();  // Attempt to restart.
    }
    break;
  }
}


