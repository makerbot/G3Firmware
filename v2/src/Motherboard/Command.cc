/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
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

#include "Command.hh"
#include "Steppers.hh"
#include "Commands.hh"
#include "Tool.hh"
#include "Configuration.hh"
#include "Timeout.hh"
#include "CircularBuffer.hh"
#include <util/atomic.h>
#include "SDCard.hh"
#include <avr/eeprom.h>

namespace command {

#define COMMAND_BUFFER_SIZE 256
uint8_t buffer_data[COMMAND_BUFFER_SIZE];
CircularBuffer command_buffer(COMMAND_BUFFER_SIZE, buffer_data);

bool outstanding_tool_command = false;

bool paused = false;

uint16_t getRemainingCapacity() {
	uint16_t sz;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		sz = command_buffer.getRemainingCapacity();
	}
	return sz;
}

void pause(bool pause) {
	paused = pause;
}

bool isPaused() {
	return paused;
}

bool isEmpty() {
	return command_buffer.isEmpty();
}

void push(uint8_t byte) {
	command_buffer.push(byte);
}

uint8_t pop8() {
	return command_buffer.pop();
}

int16_t pop16() {
	union {
		// AVR is little-endian
		int16_t a;
		struct {
			uint8_t data[2];
		} b;
	} shared;
	shared.b.data[0] = command_buffer.pop();
	shared.b.data[1] = command_buffer.pop();
	return shared.a;
}

int32_t pop32() {
	union {
		// AVR is little-endian
		int32_t a;
		struct {
			uint8_t data[4];
		} b;
	} shared;
	shared.b.data[0] = command_buffer.pop();
	shared.b.data[1] = command_buffer.pop();
	shared.b.data[2] = command_buffer.pop();
	shared.b.data[3] = command_buffer.pop();
	return shared.a;
}

enum {
	READY,
	MOVING,
	DELAY,
	HOMING,
	WAIT_ON_TOOL
} mode = READY;

Timeout delay_timeout;
Timeout homing_timeout;

void reset() {
	command_buffer.reset();
	mode = READY;
}

// A fast slice for processing commands and refilling the stepper queue, etc.
void runCommandSlice() {
	if (sdcard::isPlaying()) {
		while (command_buffer.getRemainingCapacity() > 0 && sdcard::playbackHasNext()) {
			command_buffer.push(sdcard::playbackNext());
		}
	}
	if (paused) { return; }
	if (mode == HOMING) {
		if (!steppers::isRunning()) {
			mode = READY;
		} else if (homing_timeout.hasElapsed()) {
			steppers::abort();
			mode = READY;
		}
	}
	if (mode == MOVING) {
		if (!steppers::isRunning()) { mode = READY; }
	}
	if (mode == DELAY) {
		// check timers
		if (delay_timeout.hasElapsed()) {
			mode = READY;
		}
	}
	if (mode == WAIT_ON_TOOL) {
		if (tool::getLock()) {
			OutPacket& out = tool::getOutPacket();
			InPacket& in = tool::getInPacket();
			out.reset();
			out.append8(0); // TODO: TOOL INDEX
			out.append8(SLAVE_CMD_IS_TOOL_READY);
			tool::startTransaction();
			// WHILE: bounded by timeout in runToolSlice
			while (!tool::isTransactionDone()) {
				tool::runToolSlice();
			}
			if (!in.hasError()) {
				if (in.read8(1) != 0) {
					mode = READY;
				}
			}
			tool::releaseLock();
		}
	}
	if (mode == READY) {
		// process next command on the queue.
		if (command_buffer.getLength() > 0) {
			uint8_t command = command_buffer[0];
			if (command == HOST_CMD_QUEUE_POINT_ABS) {
				// check for completion
				if (command_buffer.getLength() >= 17) {
					command_buffer.pop(); // remove the command code
					mode = MOVING;
					int32_t x = pop32();
					int32_t y = pop32();
					int32_t z = pop32();
					int32_t dda = pop32();
					steppers::setTarget(Point(x,y,z),dda);
				}
			} else if (command == HOST_CMD_CHANGE_TOOL) {
				if (command_buffer.getLength() >= 2) {
					command_buffer.pop(); // remove the command code
					uint8_t tool_index = command_buffer.pop();
				}
			} else if (command == HOST_CMD_ENABLE_AXES) {
				if (command_buffer.getLength() >= 2) {
					command_buffer.pop(); // remove the command code
					uint8_t axes = command_buffer.pop();
					bool enable = (axes & 0x80) != 0;
					for (int i = 0; i < 3; i++) {
						if ((axes & _BV(i)) != 0) {
							steppers::enableAxis(i, enable);
						}
					}
				}
			} else if (command == HOST_CMD_SET_POSITION) {
				// check for completion
				if (command_buffer.getLength() >= 13) {
					command_buffer.pop(); // remove the command code
					int32_t x = pop32();
					int32_t y = pop32();
					int32_t z = pop32();
					steppers::definePosition(Point(x,y,z));
				}
			} else if (command == HOST_CMD_DELAY) {
				if (command_buffer.getLength() >= 5) {
					mode = DELAY;
					command_buffer.pop(); // remove the command code
					// parameter is in milliseconds; timeouts need microseconds
					uint32_t microseconds = pop32() * 1000;
					delay_timeout.start(microseconds);
				}
			} else if (command == HOST_CMD_FIND_AXES_MINIMUM ||
					command == HOST_CMD_FIND_AXES_MAXIMUM) {
				if (command_buffer.getLength() >= 8) {
					command_buffer.pop(); // remove the command
					uint8_t flags = pop8();
					uint32_t feedrate = pop32(); // feedrate in us per step
					uint16_t timeout_s = pop16();
					bool direction = command == HOST_CMD_FIND_AXES_MAXIMUM;
					mode = HOMING;
					homing_timeout.start(timeout_s * 1000L * 1000L);
					steppers::startHoming(command==HOST_CMD_FIND_AXES_MAXIMUM,
							flags,
							feedrate);
				}
			} else if (command == HOST_CMD_FIRST_AUTO_RAFT) { //Super beta testing phase! Please pardon our dust!
					//Command pop only removes the first in the queue. pop multiple times to erase something big. pop also returns the value of the thing. Use command_buffer[something] if you want to read without poping. pop afterward please! Other code lives here too!
				if (command_buffer.getLength() >= 8) {
					//first we need to zero our position (We are at 0,0,0. AKA the center of the build platform and at the right hight.)
					int32_t x = 0; //set x
					int32_t y = 0; //set y
					int32_t z = 0; //set z
					steppers::definePosition(Point(x,y,z)); //set the position in steps
					command_buffer.pop(); // remove the command
					uint8_t flags = pop8(); //get the axis
					uint32_t feedrate = pop32(); // feedrate in us per step
					uint16_t timeout_s = pop16(); //The time to home for before giving up.
					bool direction = false;
					mode = HOMING;
					homing_timeout.start(timeout_s * 1000L * 1000L); 
					steppers::startHoming(direction,
							flags,
							feedrate);
					//now need to wait 'till done homing and save the distance traveled.
					while (mode == HOMING) {
						if (!steppers::isRunning()) { //wait 'till done homing
						mode = READY; //ok!
						const Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
						/*int32_t currentX = currentPosition[0]; //set x to current pos
						int32_t currentY = currentPosition[1]; //set y to current pos
						int32_t currentZ = currentPosition[2]; //set z to current pos (I think) */

						//save it in EEPROM!
						
						
						uint8_t data[3]; //data to write to eeprom
						data[0] = currentPosition[0]; //x
						data[1] = currentPosition[1]; //y
						data[2] = currentPosition[2]; //z
						uint16_t offset = 0x100;
						uint8_t length = 0x4; //32 is four bytes
						//eeprom_read_block(data, (const void*) offset, length);
						//for (int i = 0; i < length; i++) {
							//data[i] = from_host.read8(i + 4);
						//}
						uint32_t dataa = currentPosition[2]; //need to write this to eeprom, but it doesn't let me!
						eeprom_write_block(data, (void*) offset, length); //save it in slot 0x100,101 and 102!
						//next move back up the same amount (aka build platform height)
						mode = MOVING;
						int32_t x = 0;
						int32_t y = 0;
						int32_t z = 0;
						int32_t dda = 1250; // max feedrate for Z stage
						steppers::setTarget(Point(x,y,z),dda);
						
								}// end of stepper is running if
								} //end of homing while
								}//end of command buffer if 
				
				


			} else if (command == HOST_CMD_AUTO_RAFT) { //Super beta testing phase! Please pardon our dust!
					//Command pop only removes the first in the queue. pop multiple times to erase something big. pop also returns the value of the thing. Use command_buffer[something] if you want to read without poping. pop afterward please! Other code lives here too!
				if (command_buffer.getLength() >= 8) {
					//first we need to zero our position to get rid of any crazy numbers.
					int32_t x = 0; //set x
					int32_t y = 0; //set y
					int32_t z = 0; //set z
					steppers::definePosition(Point(x,y,z)); //set the position in steps
					command_buffer.pop(); // remove the command
					uint8_t flags = pop8(); //get the axis
					uint32_t feedrate = pop32(); // feedrate in us per step
					uint16_t timeout_s = pop16(); //The time to home for before giving up.
					bool direction = false;
					mode = HOMING;
					homing_timeout.start(timeout_s * 1000L * 1000L); 
					steppers::startHoming(direction,
							flags,
							feedrate);
					//now need to wait 'till done homing and save the distance traveled.
					while (mode == HOMING) {
						if (!steppers::isRunning()) { //wait 'till done homing
						mode = READY; //ok!
						//const Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
						/*int32_t currentX = currentPosition[0]; //set x to current pos
						int32_t currentY = currentPosition[1]; //set y to current pos
						int32_t currentZ = currentPosition[2]; //set z to current pos (I think) */

						//move back up the amount saved in EEPROM.
						
						
						uint8_t data[3]; //data to write to eeprom
						//data[0] = currentPosition[0]; //x
						//data[1] = currentPosition[1]; //y
						//data[2] = currentPosition[2]; //z
						uint16_t offset = 0x100 ;
						uint8_t length = 0x4 ;
						//eeprom_read_block(data, (const void*) offset, length);
						//for (int i = 0; i < length; i++) {
							//data[i] = from_host.read8(i + 4);
						//}
						eeprom_write_block(data, (void*) offset, length); //save it in slot 0x100,101 and 102!
						//next move back up the same amount (aka build platform height)
						mode = MOVING;
						int32_t x = 0;
						int32_t y = 0;
						int32_t z = 0;
						int32_t dda = 1250; // max feedrate for Z stage
						steppers::setTarget(Point(x,y,z),dda);
						
								}// end of stepper is running if
								} //end of homing while
								}//end of command buffer if 
				
				


			} else if (command == HOST_CMD_WAIT_FOR_TOOL) {
				if (command_buffer.getLength() >= 6) {
					mode = WAIT_ON_TOOL;
					command_buffer.pop();
					uint8_t currentToolIndex = command_buffer.pop();
					uint16_t toolPingDelay = (uint16_t)pop16();
					uint16_t toolTimeout = (uint16_t)pop16();
				}
			} else if (command == HOST_CMD_TOOL_COMMAND) {
				if (command_buffer.getLength() >= 4) { // needs a payload
					uint8_t payload_length = command_buffer[3];
					if (command_buffer.getLength() >= 4+payload_length) {
						// command is ready
						if (tool::getLock()) {
							OutPacket& out = tool::getOutPacket();
							out.reset();
							command_buffer.pop(); // remove the command code
							out.append8(command_buffer.pop()); // copy tool index
							out.append8(command_buffer.pop()); // copy command code
							int len = pop8(); // get payload length
							for (int i = 0; i < len; i++) {
								out.append8(command_buffer.pop());
							}
							// we don't care about the response, so we can release
							// the lock after we initiate the transfer
							tool::startTransaction();
							tool::releaseLock();
						}
					}
				}
			} else {
			}
		}
	}
}

}
