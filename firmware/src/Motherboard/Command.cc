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
#include "ExtruderControl.hh"

namespace command {

#define COMMAND_BUFFER_SIZE 512
uint8_t buffer_data[COMMAND_BUFFER_SIZE];
CircularBuffer command_buffer(COMMAND_BUFFER_SIZE, buffer_data);

bool outstanding_tool_command = false;

bool paused = false;

uint16_t statusDivisor = 0;
volatile uint32_t recentCommandClock = 0;
volatile uint32_t recentCommandTime = 0;

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
	WAIT_ON_TOOL,
	WAIT_ON_PLATFORM
} mode = READY;

Timeout delay_timeout;
Timeout homing_timeout;
Timeout tool_wait_timeout;

void reset() {
	command_buffer.reset();
	mode = READY;
}


//Executes a slave command
//Returns true if everything is okay, otherwise false in case of error
//The result from the command is stored in result

bool querySlaveCmd(uint8_t cmd, uint8_t *result) {
	bool ret = false;

	if (tool::getLock()) {
		OutPacket& out = tool::getOutPacket();
		InPacket& in = tool::getInPacket();
		out.reset();
		out.append8(tool::getCurrentToolheadIndex());
		out.append8(SLAVE_CMD_GET_TOOL_STATUS);
		tool::startTransaction();

		// WHILE: bounded by timeout in runToolSlice
		while (!tool::isTransactionDone()) {
			tool::runToolSlice();
		}
		if (!in.hasError()) {
			*result = in.read8(1);
			ret = true;
		}
		else ret = false;
		tool::releaseLock();
	}

	return ret;
}


bool isToolReady() {
	uint8_t result;

	if (querySlaveCmd(SLAVE_CMD_GET_TOOL_STATUS, &result)) {
		if (result & 0x01) return true;
	}

	return false;
}


bool isPlatformReady() {
	uint8_t result;

	if (querySlaveCmd(SLAVE_CMD_IS_PLATFORM_READY, &result)) {
		if (result != 0)  return true;
	}
	return false;
}


// A fast slice for processing commands and refilling the stepper queue, etc.
void runCommandSlice() {
	recentCommandClock ++;

#ifdef HAS_MOOD_LIGHT
	updateMoodStatus();
#endif

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
		if (tool_wait_timeout.hasElapsed()) {
			mode = READY;
		} else if (isToolReady()) {
			mode = READY;
		}
	}
	if (mode == WAIT_ON_PLATFORM) {
		if (tool_wait_timeout.hasElapsed()) {
			mode = READY;
		} else if (isPlatformReady()) {
			mode = READY;
		}
	}
	if (mode == READY) {
		// process next command on the queue.
		if (command_buffer.getLength() > 0) {
			uint8_t command = command_buffer[0];
			if (command == HOST_CMD_QUEUE_POINT_ABS) {
				recentCommandTime = recentCommandClock;
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
			} else if (command == HOST_CMD_QUEUE_POINT_EXT) {
				recentCommandTime = recentCommandClock;
				// check for completion
				if (command_buffer.getLength() >= 25) {
					command_buffer.pop(); // remove the command code
					mode = MOVING;
					int32_t x = pop32();
					int32_t y = pop32();
					int32_t z = pop32();
					int32_t a = pop32();
					int32_t b = pop32();
					int32_t dda = pop32();
					steppers::setTarget(Point(x,y,z,a,b),dda);
				}
			} else if (command == HOST_CMD_QUEUE_POINT_NEW) {
				recentCommandTime = recentCommandClock;
				// check for completion
				if (command_buffer.getLength() >= 26) {
					command_buffer.pop(); // remove the command code
					mode = MOVING;
					int32_t x = pop32();
					int32_t y = pop32();
					int32_t z = pop32();
					int32_t a = pop32();
					int32_t b = pop32();
					int32_t us = pop32();
					uint8_t relative = pop8();
					steppers::setTargetNew(Point(x,y,z,a,b),us,relative);
				}
			} else if (command == HOST_CMD_CHANGE_TOOL) {
				if (command_buffer.getLength() >= 2) {
					command_buffer.pop(); // remove the command code
                                        tool::setCurrentToolheadIndex(command_buffer.pop());
				}
			} else if (command == HOST_CMD_ENABLE_AXES) {
				recentCommandTime = recentCommandClock;
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
			} else if (command == HOST_CMD_SET_POSITION) {
				// check for completion
				if (command_buffer.getLength() >= 13) {
					command_buffer.pop(); // remove the command code
					int32_t x = pop32();
					int32_t y = pop32();
					int32_t z = pop32();
					steppers::definePosition(Point(x,y,z));
				}
			} else if (command == HOST_CMD_SET_POSITION_EXT) {
				// check for completion
				if (command_buffer.getLength() >= 21) {
					command_buffer.pop(); // remove the command code
					int32_t x = pop32();
					int32_t y = pop32();
					int32_t z = pop32();
					int32_t a = pop32();
					int32_t b = pop32();
					steppers::definePosition(Point(x,y,z,a,b));
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
			} else if (command == HOST_CMD_WAIT_FOR_TOOL) {
				if (command_buffer.getLength() >= 6) {
					mode = WAIT_ON_TOOL;
					command_buffer.pop();
					uint8_t currentToolIndex = command_buffer.pop();
					uint16_t toolPingDelay = (uint16_t)pop16();
					uint16_t toolTimeout = (uint16_t)pop16();
					tool_wait_timeout.start(toolTimeout*1000000L);
				}
			} else if (command == HOST_CMD_WAIT_FOR_PLATFORM) {
        // FIXME: Almost equivalent to WAIT_FOR_TOOL
				if (command_buffer.getLength() >= 6) {
					mode = WAIT_ON_PLATFORM;
					command_buffer.pop();
					uint8_t currentToolIndex = command_buffer.pop();
					uint16_t toolPingDelay = (uint16_t)pop16();
					uint16_t toolTimeout = (uint16_t)pop16();
					tool_wait_timeout.start(toolTimeout*1000000L);
				}
			} else if (command == HOST_CMD_STORE_HOME_POSITION) {

				// check for completion
				if (command_buffer.getLength() >= 2) {
					command_buffer.pop();
					uint8_t axes = pop8();

					// Go through each axis, and if that axis is specified, read it's value,
					// then record it to the eeprom.
					for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
						if ( axes & (1 << i) ) {
							uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*i;
							uint32_t position = steppers::getPosition()[i];
							cli();
							eeprom_write_block(&position, (void*) offset, 4);
							sei();
						}
					}
				}
			} else if (command == HOST_CMD_RECALL_HOME_POSITION) {
				// check for completion
				if (command_buffer.getLength() >= 2) {
					command_buffer.pop();
					uint8_t axes = pop8();

					Point newPoint = steppers::getPosition();

					for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
						if ( axes & (1 << i) ) {
							uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*i;
							cli();
							eeprom_read_block(&(newPoint[i]), (void*) offset, 4);
							sei();
						}
					}

					steppers::definePosition(newPoint);
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
			} else if (command == HOST_CMD_MOOD_LIGHT_SET_RGB ) {
				// check for completion
				if (command_buffer.getLength() >= 21) {
					command_buffer.pop(); // remove the command code
					int32_t r = pop32();
					int32_t g = pop32();
					int32_t b = pop32();
					int32_t fadeSpeed = pop32();
					int32_t writeToEeprom = pop32();
#ifdef HAS_MOOD_LIGHT
					Motherboard::getBoard().MoodLightSetRGBColor((uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)fadeSpeed, (uint8_t)writeToEeprom);
#endif
				}
			} else if (command == HOST_CMD_MOOD_LIGHT_SET_HSB ) {
				// check for completion
				if (command_buffer.getLength() >= 17) {
					command_buffer.pop(); // remove the command code
					int32_t h = pop32();
					int32_t s = pop32();
					int32_t b = pop32();
					int32_t fadeSpeed = pop32();
#ifdef HAS_MOOD_LIGHT
					Motherboard::getBoard().MoodLightSetHSBColor((uint8_t)h, (uint8_t)s, (uint8_t)b, (uint8_t)fadeSpeed);
#endif
				}
			} else if (command == HOST_CMD_MOOD_LIGHT_PLAY_SCRIPT ) {
				// check for completion
				if (command_buffer.getLength() >= 9) {
					command_buffer.pop(); // remove the command code
					int32_t scriptId = pop32();
					int32_t writeToEeprom = pop32();
#ifdef HAS_MOOD_LIGHT
					Motherboard::getBoard().MoodLightPlayScript((uint8_t)scriptId, (uint8_t)writeToEeprom);
#endif
				}
			} else {
			}
		}
	}
}


#ifdef HAS_MOOD_LIGHT

#define STOCHASTIC_PERCENT(v, a, b)		(((v - a) / (b - a)) * 100.0)
#define MAX2(a,b)				((a >= b)?a:b)
#define STATUS_DIVISOR_TIME_PER_STATUS_CHANGE	4000
#define RECENT_COMMAND_TIMEOUT			4000 * 200

void updateMoodStatus() {
	//Implement a divisor so we don't get called on every turn, we don't
	//want to overload the interrupt loop when we don't need frequent changes	
	statusDivisor ++;

	if ( statusDivisor < STATUS_DIVISOR_TIME_PER_STATUS_CHANGE )	return;
	statusDivisor = 0;

	MoodLightController moodLight = Motherboard::getBoard().getMoodLightController();

	//If we're not set to the Bot Status Script, then there's no need to check anything
	if ( moodLight.getLastScriptPlayed() != 0 ) return;


	//Certain states don't require us to check as often,
	//save some CPU cycles

	enum moodLightStatus lastMlStatus = moodLight.getLastStatus();

	//If printing and recent commands, do nothing
	if ((lastMlStatus == MOOD_LIGHT_STATUS_PRINTING) &&
	    ( recentCommandTime >= (recentCommandClock - RECENT_COMMAND_TIMEOUT )))	return;
	
 	enum moodLightStatus mlStatus = MOOD_LIGHT_STATUS_IDLE;

	//Get the status of the tool head and platform

	//Figure out how hot or cold we are
	bool toolReady     = isToolReady();
	bool platformReady = isPlatformReady();

	OutPacket responsePacket;
	uint16_t toolTemp=0, toolTempSetPoint=0, platformTemp=0, platformTempSetPoint=0;

	if (extruderControl(SLAVE_CMD_GET_TEMP, EXTDR_CMD_GET, responsePacket, 0))
		toolTemp = responsePacket.read16(1);

	if (extruderControl(SLAVE_CMD_GET_SP, EXTDR_CMD_GET, responsePacket, 0))
		toolTempSetPoint = responsePacket.read16(1);

	if (extruderControl(SLAVE_CMD_GET_PLATFORM_TEMP, EXTDR_CMD_GET, responsePacket, 0))
		platformTemp = responsePacket.read16(1);

	if (extruderControl(SLAVE_CMD_GET_PLATFORM_SP, EXTDR_CMD_GET, responsePacket, 0))
		platformTempSetPoint = responsePacket.read16(1);
	

	float percentHotTool, percentHotPlatform;

	if ( toolTempSetPoint == 0 ) {
		//We're cooling.  0% = 48C  100% = 240C
		percentHotTool = STOCHASTIC_PERCENT((float)toolTemp, 48.0, 240.0);
		if ( percentHotTool > 100.0 )	percentHotTool = 100.0;
		if ( percentHotTool < 0.0 )	percentHotTool = 0.0;
	} else {
		//We're heating.  0% = 18C  100% = Set Point 
		percentHotTool = STOCHASTIC_PERCENT((float)toolTemp, 18.0, (float)toolTempSetPoint);
		if ( percentHotTool > 100.0 )	percentHotTool = 100.0;
		if ( percentHotTool < 0.0 )	percentHotTool = 0.0;

		if ( toolReady )			percentHotTool = 100.0;
		else if ( percentHotTool >= 100.0 )	percentHotTool = 99.0;
	}

	if ( platformTempSetPoint == 0 ) {
		//We're cooling.  0% = 48C  100% = 120C
		percentHotPlatform = STOCHASTIC_PERCENT((float)platformTemp, 48.0, 120.0);
		if ( percentHotPlatform > 100.0 )	percentHotPlatform = 100.0;
		if ( percentHotPlatform < 0.0 )		percentHotPlatform = 0.0;
	} else {
		//We're heating.  0% = 18C  100% = Set Point 
		percentHotPlatform = STOCHASTIC_PERCENT((float)platformTemp, 18.0, (float)platformTempSetPoint);
		if ( percentHotPlatform > 100.0 )	percentHotPlatform = 100.0;
		if ( percentHotPlatform < 0.0 )		percentHotPlatform = 0.0;

		if ( platformReady )			percentHotPlatform = 100.0;
		else if ( percentHotPlatform >= 100.0 )	percentHotPlatform = 99.0;
	}

	//Are we heating or cooling
	bool heating = false;
	if (( toolTempSetPoint != 0 ) || ( platformTempSetPoint != 0 ))	heating = true;

	if ( heating ) {
		//If we're heating and tool and platform are 100%, then we're not cooling anymore, we're printing
		if (( percentHotTool >= 100.0 ) && ( percentHotPlatform >= 100.0 ) &&
	    	    ( recentCommandTime >= (recentCommandClock - RECENT_COMMAND_TIMEOUT ))) {
			mlStatus = MOOD_LIGHT_STATUS_PRINTING;
		} else {
			//We can't go from printing back to heating
			if ( lastMlStatus != MOOD_LIGHT_STATUS_PRINTING ) mlStatus = MOOD_LIGHT_STATUS_HEATING;
			else						  mlStatus = MOOD_LIGHT_STATUS_PRINTING;
		}
	} else {
		//If we're cooling and tool and platform are 0%, then we're not cooling anymore
		if  (( percentHotTool <= 0.0 ) && ( percentHotPlatform <= 0.0 )) {
			mlStatus = MOOD_LIGHT_STATUS_IDLE;
		} else {
			//We can't go from idle back to cooling
			if ( lastMlStatus != MOOD_LIGHT_STATUS_IDLE ) mlStatus = MOOD_LIGHT_STATUS_COOLING;
			else					      mlStatus = MOOD_LIGHT_STATUS_IDLE;
		}
	}


	float percentHot = MAX2(percentHotTool, percentHotPlatform);

	Motherboard::getBoard().getMoodLightController().displayStatus(mlStatus, percentHot);

	lastMlStatus = mlStatus;
}

#endif

}
