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
#include <avr/eeprom.h>
#include "EepromMap.hh"
#include "SDCard.hh"

namespace command {

#define COMMAND_BUFFER_SIZE 512
uint8_t buffer_data[COMMAND_BUFFER_SIZE];
CircularBuffer command_buffer(COMMAND_BUFFER_SIZE, buffer_data);

bool outstanding_tool_command = false;

bool paused = false;

uint16_t getRemainingCapacity() {
	//ATOMIC_BLOCK(ATOMIC_FORCEON) 
    {
		return command_buffer.getRemainingCapacity();
	}
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

inline uint8_t pop8() {
	return command_buffer.pop();
}

inline int16_t pop16() {
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

inline int32_t pop32() {
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
	WAIT_ON_TOOL,
	WAIT_ON_PLATFORM
} mode = READY;

Timeout delay_timeout;
Timeout homing_timeout;
Timeout tool_wait_timeout;

void reset() {
    DEBUG_CMD_SLICE_PIN::setDirection(true);
    DEBUG_CMD_SLICE_PIN::setValue(false);
	command_buffer.reset();
	mode = READY;
}

// A fast slice for processing commands and refilling the stepper queue, etc.
void runCommandSlice() {
    DEBUG_CMD_SLICE_PIN::setValue(true);
	if (sdcard::isPlaying()) {
		while (command_buffer.getRemainingCapacity() > 0 && sdcard::playbackHasNext()) {
			command_buffer.push(sdcard::playbackNext());
		}
	}
	if (!paused)
    {
        switch(mode)
        {
            case HOMING: {
		        if (!steppers::isRunning()) {
			        mode = READY;
		        } else if (homing_timeout.hasElapsed()) {
			        steppers::abort();
			        mode = READY;
		        }
                break;
	        }
            case MOVING: {
		        if (!steppers::isRunning()) { mode = READY; }
                break;
	        }
            case DELAY: {
		        // check timers
		        if (delay_timeout.hasElapsed()) {
			        mode = READY;
		        }
                break;
	        }
            case WAIT_ON_TOOL: {
		        if (tool_wait_timeout.hasElapsed()) {
			        mode = READY;
		        } else if (tool::getLock()) {
			        OutPacket& out = tool::getOutPacket();
			        InPacket& in = tool::getInPacket();
			        out.reset();
			        out.append8(tool::getCurrentToolheadIndex());
			        out.append8(SLAVE_CMD_GET_TOOL_STATUS);
			        tool::startTransaction();
                    tool::waitForTransaction();
			        if (!in.hasError()) {
				        if (in.read8(1) & 0x01) {
					        mode = READY;
				        }
			        }
			        tool::releaseLock();
		        }
                break;
	        }
            case WAIT_ON_PLATFORM: {
		        // FIXME: Duplicates most code from WAIT_ON_TOOL
		        if (tool_wait_timeout.hasElapsed()) {
			        mode = READY;
		        } else if (tool::getLock()) {
			        OutPacket& out = tool::getOutPacket();
			        InPacket& in = tool::getInPacket();
			        out.reset();
			        out.append8(tool::getCurrentToolheadIndex());
			        out.append8(SLAVE_CMD_IS_PLATFORM_READY);
			        tool::startTransaction();
                    tool::waitForTransaction();
			        if (!in.hasError()) {
				        if (in.read8(1) != 0) {
					        mode = READY;
				        }
			        }
			        tool::releaseLock();
		        }
                break;
	        }
        }

	    if (mode == READY) {
		    // process next command on the queue.
		    if (command_buffer.getLength() > 0) {
			    uint8_t command = command_buffer[0];
                switch(command)
                {
                    case HOST_CMD_QUEUE_POINT_ABS:{
				        // check for completion
				        if (command_buffer.getLength() >= 17) {
					        command_buffer.pop(); // remove the command code
					        mode = MOVING;
					        Point pt(pop32(), pop32(), pop32() );
					        int32_t dda = pop32();
					        steppers::setTarget(pt,dda);
				        }
                        break;
			        } 
                    case HOST_CMD_QUEUE_POINT_ABS_16: {
				        // check for completion
				        if (command_buffer.getLength() >= 15) {
					        command_buffer.pop(); // remove the command code
					        mode = MOVING;
					        Point pt(pop16(), pop16(), pop16(), pop16(), pop16() );
					        int32_t dda = pop32();
					        steppers::setTarget(pt,dda);
				        }
                        break;
			        } 
                    case HOST_CMD_QUEUE_POINT_NEW_16: {
				        // check for completion
				        if (command_buffer.getLength() >= 16) {
					        command_buffer.pop(); // remove the command code
					        mode = MOVING;
					        Point pt(pop16(), pop16(), pop16(), pop16(), pop16() );
					        int32_t us = pop32();
					        uint8_t relative = pop8();
					        steppers::setTargetNew(pt,us,relative);
				        }
                        break;
			        } 
                    case HOST_CMD_QUEUE_POINT_EXT: {
				        // check for completion
				        if (command_buffer.getLength() >= 25) {
					        command_buffer.pop(); // remove the command code
					        mode = MOVING;
					        Point pt(pop32(), pop32(), pop32(), pop32(), pop32() );
					        int32_t dda = pop32();
					        steppers::setTarget(pt,dda);
				        }
                        break;
			        } 
                    case HOST_CMD_QUEUE_POINT_NEW: {
				        // check for completion
				        if (command_buffer.getLength() >= 26) {
					        command_buffer.pop(); // remove the command code
					        mode = MOVING;
					        Point pt(pop32(), pop32(), pop32(), pop32(), pop32() );
					        int32_t us = pop32();
					        uint8_t relative = pop8();
					        steppers::setTargetNew(pt,us,relative);
				        }
                        break;
			        } 
                    case HOST_CMD_CHANGE_TOOL: {
				        if (command_buffer.getLength() >= 2) {
					        command_buffer.pop(); // remove the command code
                            tool::setCurrentToolheadIndex(command_buffer.pop());
				        }
                        break;
			        } 
                    case HOST_CMD_ENABLE_AXES: {
				        if (command_buffer.getLength() >= 2) {
					        command_buffer.pop(); // remove the command code
					        uint8_t axes = command_buffer.pop();
					        bool enable = (axes & 0x80) != 0;
					        for (int i = 0; i < STEPPER_COUNT; i++) {
						        if ((axes & _BV(i)) != 0) {
							        steppers::enableAxis(i, enable);
						        }
					        }
				        }
                        break;
			        } 
                    case HOST_CMD_SET_POSITION: {
				        // check for completion
				        if (command_buffer.getLength() >= 13) {
					        command_buffer.pop(); // remove the command code
					        Point pt(pop32(), pop32(), pop32());
					        steppers::definePosition(pt);
				        }
                        break;
			        } 
                    case HOST_CMD_SET_POSITION_EXT: {
				        // check for completion
				        if (command_buffer.getLength() >= 21) {
					        command_buffer.pop(); // remove the command code
					        Point pt(pop32(), pop32(), pop32(), pop32(), pop32() );
					        steppers::definePosition(pt);
				        }
                        break;
			        } 
                    case HOST_CMD_SET_POSITION_16: {
				        // check for completion
				        if (command_buffer.getLength() >= 11) {
					        command_buffer.pop(); // remove the command code
					        Point pt(pop16(), pop16(), pop16(), pop16(), pop16() );
					        steppers::definePosition(pt);
				        }
                        break;
			        } 
                    case HOST_CMD_DELAY: {
				        if (command_buffer.getLength() >= 5) {
					        mode = DELAY;
					        command_buffer.pop(); // remove the command code
					        // parameter is in milliseconds; timeouts need microseconds
					        uint32_t microseconds = pop32() * 1000;
					        delay_timeout.start(microseconds);
				        }
                        break;
			        } 
                    case HOST_CMD_FIND_AXES_MINIMUM:
                    case HOST_CMD_FIND_AXES_MAXIMUM: {
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
                        break;
			        } 
                    case HOST_CMD_WAIT_FOR_TOOL: {
				        if (command_buffer.getLength() >= 6) {
					        mode = WAIT_ON_TOOL;
					        command_buffer.pop();
					        uint8_t currentToolIndex = command_buffer.pop();
					        uint16_t toolPingDelay = (uint16_t)pop16();
					        uint16_t toolTimeout = (uint16_t)pop16();
					        tool_wait_timeout.start(toolTimeout*1000000L);
				        }
                        break;
			        } 
                    case HOST_CMD_WAIT_FOR_PLATFORM: {
                // FIXME: Almost equivalent to WAIT_FOR_TOOL
				        if (command_buffer.getLength() >= 6) {
					        mode = WAIT_ON_PLATFORM;
					        command_buffer.pop();
					        uint8_t currentToolIndex = command_buffer.pop();
					        uint16_t toolPingDelay = (uint16_t)pop16();
					        uint16_t toolTimeout = (uint16_t)pop16();
					        tool_wait_timeout.start(toolTimeout*1000000L);
				        }
                        break;
			        } 
                    case HOST_CMD_STORE_HOME_POSITION: {

				        // check for completion
				        if (command_buffer.getLength() >= 2) {
					        command_buffer.pop();
					        uint8_t axes = pop8();

					        // Go through each axis, and if that axis is specified, read it's value,
					        // then record it to the eeprom.
                            Point point;
                            steppers::getPosition(&point);
                            uint16_t offset = eeprom::AXIS_HOME_POSITIONS;
					        for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
						        if ( axes & (1 << i) ) {
							        uint32_t position = point[i];
							        eeprom_write_block(&position, (void*) offset, 4);
						        }
						        offset += 4;
					        }
				        }
                        break;
			        } 
                    case HOST_CMD_RECALL_HOME_POSITION: {
				        // check for completion
				        if (command_buffer.getLength() >= 2) {
					        command_buffer.pop();
					        uint8_t axes = pop8();

					        Point newPoint;
                            steppers::getPosition(&newPoint); // load existing position first

                            uint16_t offset = eeprom::AXIS_HOME_POSITIONS;
					        for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
						        if ( axes & (1 << i) ) {
							        eeprom_read_block(&(newPoint[i]), (void*) offset, 4);
						        }
						        offset += 4;
					        }

					        steppers::definePosition(newPoint);
				        }

                        break;
			        } 
                    case HOST_CMD_TOOL_COMMAND: {
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
                        break;
			        }
                }
            }
		}
	}
    DEBUG_CMD_SLICE_PIN::setValue(false);
}
}
