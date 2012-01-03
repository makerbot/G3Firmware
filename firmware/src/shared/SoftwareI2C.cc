/* 
 * Software I2C Library To Run On Analog Pins
 * This implements a master and is designed to work with the BlinkM.
 * May or may not work with other I2C devices
 *
 * Reference:
 *	http://www.robot-electronics.co.uk/acatalog/I2C_Tutorial.html
 *	http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html
 *
 *
 * Copyright Dec, 2011 by Jetty840
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */


#include <util/delay.h>
#include "SoftwareI2C.hh"


//Pin high and low definitions
#define SCL_LOW  {sclPin.setValue(false);	sclPin.setDirection(true);}
#define SCL_HIGH {sclPin.setDirection(false);	sclPin.setValue(true);}
#define SDA_LOW  {sdaPin.setValue(false);	sdaPin.setDirection(true);}
#define SDA_HIGH {sdaPin.setDirection(false);	sdaPin.setValue(true);}

#define kI2CBitTimeuS	50			//50 microseconds
#define I2C_DLY		_delay_us(kI2CBitTimeuS)


//Transmit 1 bit

void SoftwareI2C::txBit(uint8_t b) {
	//Set up the data bit
	if ( b )	SDA_HIGH	//; in macro
	else		SDA_LOW		//; in macro

	//Clock it to the slave
	SCL_HIGH;
	I2C_DLY;

	SCL_LOW;
	I2C_DLY;

	SDA_LOW;
	I2C_DLY;
}


//Receive 1 bit

uint8_t SoftwareI2C::rxBit(void) {
	SDA_HIGH;
	SCL_HIGH;
	I2C_DLY;

	uint8_t b = sdaPin.getValue();

	SCL_LOW;
	I2C_DLY;
	
	return b;
}


SoftwareI2C::SoftwareI2C() {
}


SoftwareI2C::SoftwareI2C(Pin sda, Pin sdl) {
	init(sda, sdl);
}


void SoftwareI2C::init(Pin sda, Pin scl) {
	//Store pins for use later
	sclPin = scl;
	sdaPin = sda;

	//These should really be set high at exactly the same time 
	//Set the bus to inactive
	SDA_HIGH;
	SCL_HIGH;

	I2C_DLY;
}



//Start a transmission to an address.  readReq specifies wether it's
//a read or write transmission

uint8_t SoftwareI2C::start(uint8_t address, bool readReq) {
	//These should really be set high at exactly the same time 
	SDA_HIGH;
	SCL_HIGH;
	I2C_DLY;

	//Signal the start of a transmission

	//Set data low
	SDA_LOW;
	I2C_DLY;

	//Set clock low
	SCL_LOW;
	I2C_DLY;

	//Shift the address by 1 bit, and add on the read/write bit
  	uint8_t addr = address << 1;
  	if ( readReq )	addr |= 0x01;

	//Transmit the address byte
	return txByte(addr);
}


//Finish the transmission

uint8_t SoftwareI2C::stop(void) {
	//Signal the end of a transmission
	SCL_HIGH;
	I2C_DLY;

	SDA_HIGH;
	I2C_DLY;
}


//Transmit one byte

uint8_t SoftwareI2C::txByte(uint8_t c) {
	for (int i = 0; i < 8; i ++, c = c << 1)
		txBit(c & 0x80);

	return rxBit();
}


//Receive one byte

//When reading a number of bytes in succession, ack is set to 1
//for every byte except the last byte

uint8_t SoftwareI2C::rxByte(uint8_t ack) {
	uint8_t ret = 0;
	
	for (int i = 0; i < 8; i ++ ) {
		ret = ret << 1;
		ret |= rxBit();
	}

	txBit( ! ack );
	I2C_DLY;
	
	return ret;
}
