/*
 AsyncTwi.h - Asyncronous TWI/I2C library for Wiring & Arduino
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

#ifndef _ASYNC_TWI_H
#define _ASYNC_TWI_H

#include <stdint.h> 

#define TWI_BUFFER_LENGTH 32
#define TWI_FREQ 100000

// On write completion callback, data is null and len is how many bytes
//  were written before the client NACKed or we finished.
// On read completion callback, data points to recv buffer, and len is how
//  many bytes were read.
typedef void (*twi_callback_t)(uint8_t address, uint8_t* data, uint8_t len);

/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void);


/* 
 * Function twi_available_tx_bytes
 * Desc     returns how many bytes are free in twi send buffer.
 *          a twi_read or twi_write call will need a number of bytes free
 *          that is >= the number of payload data bytes + 4 bytes.
 * Input    none
 * Output   number of bytes free in twi send buffer.
 */
uint8_t twi_available_tx_bytes();


/* 
 * Function twi_available_tx_bytes
 * Desc     returns how many twi_read() and twi_write() calls are still
 *          pending completion.
 * Input    none
 * Output   number of pending read and write calls.
 */
uint8_t twi_pending();


/* 
 * Function twi_write
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          len: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          cb: callback routine to call when write is complete.
 * Output   number of payload data bytes sent, or 0 if call is used async.
 *          -1 if not enough buffer space was free.
 */
uint8_t twi_write(uint8_t address, uint8_t* data, uint8_t len, uint8_t wait, twi_callback_t cb);


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
uint8_t twi_read_async(uint8_t address, uint8_t maxlen, twi_callback_t cb);


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
uint8_t twi_read(uint8_t address, uint8_t *buf, uint8_t maxlen);



/* 
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   -1 length too long for buffer
 *          -2 not slave transmitter
 *          0 ok
 */
uint8_t twi_transmit(uint8_t* data, uint8_t length);


/* 
 * Function twi_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void twi_setAddress(uint8_t address);


/* 
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) );


/* 
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveTxEvent( void (*function)(void) );


#endif // _ASYNC_TWI_H

