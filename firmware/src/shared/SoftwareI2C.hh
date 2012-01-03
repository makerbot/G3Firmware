/* 
 * Software I2C Library To Run On Analog Pins
 * This implements a master and is designed to work with the BlinkM.
 * May or may not work with other I2C devices
 *
 * Reference:
 *      http://www.robot-electronics.co.uk/acatalog/I2C_Tutorial.html
 *      http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html
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


#ifndef SOFTWAREI2C_HH_
#define SOFTWAREI2C_HH_

#include "Pin.hh"

class SoftwareI2C {
public:
	SoftwareI2C();
	SoftwareI2C(Pin sda, Pin sdl);

	void init(Pin sda, Pin scl);

	uint8_t start(uint8_t address, bool readReq);
	uint8_t stop(void);
	uint8_t txByte(uint8_t c);
	uint8_t rxByte(uint8_t ack);

private:
	Pin sdaPin, sclPin;

	void    txBit(uint8_t b);
	uint8_t rxBit(void);
};

#endif // SOFTWAREI2C_HH_
