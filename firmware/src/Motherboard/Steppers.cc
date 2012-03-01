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

#define __STDC_LIMIT_MACROS
#include "Steppers.hh"
#include "StepperAxis.hh"
#include <stdint.h>
#include <math.h>
#include <math.h>
#include <avr/eeprom.h>
#include "EepromMap.hh"
#include "Eeprom.hh"

#ifdef HAS_STEPPER_ACCELERATION
#include "StepperAccel.hh"
#endif

namespace steppers {


volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;
StepperAxis axes[STEPPER_COUNT];
volatile bool is_homing;

#ifdef HAS_STEPPER_ACCELERATION
	bool acceleration = false;
	bool planner = false;
	uint8_t plannerMaxBufferSize;
	volatile bool force_acceleration_off = false;

	Point lastTarget;
#endif

bool holdZ = false;

bool isRunning() {
	return is_running || is_homing;
}

bool isHoming() {
	return is_homing;
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
                axes[i] = StepperAxis(motherboard.getStepperInterface(i));
	}

	reset();
}

void abort() {
#ifdef HAS_STEPPER_ACCELERATION
	if ( acceleration )	quickStop();
#endif
	is_running = false;
	is_homing = false;
}

float convertAxisMMToFloat(int64_t st ) {
        float aspmf = (float)st;
        for (uint8_t i=0 ; i < STEPS_PER_MM_PRECISION; i ++ )
                aspmf /= 10.0;
	return aspmf;
}

void reset() {
#ifdef HAS_STEPPER_ACCELERATION
	//Get the acceleration settings
	uint8_t accel = eeprom::getEeprom8(eeprom::STEPPER_DRIVER, 0);
	
	acceleration = accel & 0x01;
	planner = (accel & 0x02)?true:false;

	if ( acceleration ) {
		//Good description of the settings can be found here:
		//http://wiki.ultimaker.com/Marlin_firmware_for_the_Ultimaker
		//and here:  https://github.com/ErikZalm/Marlin

		//Here's more documentation on the various settings / features
		//http://wiki.ultimaker.com/Marlin_firmware_for_the_Ultimaker
		//https://github.com/ErikZalm/Marlin/commits/Marlin_v1
		//http://forums.reprap.org/read.php?147,94689,94689
		//http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
		//http://www.brokentoaster.com/blog/?p=358

		//Same as axis steps:mm in the firmware
		//Temporarily placed here, should be read from eeprom
		axis_steps_per_unit[X_AXIS] = convertAxisMMToFloat(eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_X, STEPS_PER_MM_X_DEFAULT));
		axis_steps_per_unit[Y_AXIS] = convertAxisMMToFloat(eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_Y, STEPS_PER_MM_Y_DEFAULT));
		axis_steps_per_unit[Z_AXIS] = convertAxisMMToFloat(eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_Z, STEPS_PER_MM_Z_DEFAULT));
		axis_steps_per_unit[E_AXIS] = (float)eeprom::getEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM,44) / 10.0;
 
		//M201 - Set max acceleration in units/s^2 for print moves
		// X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
		max_acceleration_units_per_sq_second[X_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_X, 2000);
		max_acceleration_units_per_sq_second[Y_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Y, 2000);
		max_acceleration_units_per_sq_second[Z_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Z, 150);
		max_acceleration_units_per_sq_second[E_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_A, 60000);

		for (uint8_t i = 0; i < NUM_AXIS; i ++)
			axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
		
		//M203 - Set maximum feedrate that your machine can sustain in mm/sec
		max_feedrate[X_AXIS] = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_X, 160);
		max_feedrate[Y_AXIS] = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Y, 160);
		max_feedrate[Z_AXIS] = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Z, 10);
		max_feedrate[E_AXIS] = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_A, 100);

		//M204 - Set default accelerationm for "Normal Moves (acceleration)" and "filament only moves (retraction)" in mm/sec^2

		// X, Y, Z and E max acceleration in mm/s^2 for printing moves 
		p_acceleration		= (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_NORM, 5000);

		// X, Y, Z and E max acceleration in mm/s^2 for r retracts
		p_retract_acceleration	= (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_RETRACT, 3000);

		//M205 - Advanced Settings
		//minimumfeedrate   - minimum travel speed while printing
		minimumfeedrate	  = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MIN_FEED_RATE,0)	 / 10.0;

		//mintravelfeedrate - minimum travel speed while travelling
    		mintravelfeedrate = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MIN_TRAVEL_FEED_RATE,0) / 10.0;

		//max_xy_jerk     - maximum xy jerk (mm/sec)
    		max_xy_jerk	  = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_XY_JERK,2)	 / 10.0;
		max_xy_jerk_squared = max_xy_jerk * max_xy_jerk;

		//max_z_jerk        - maximum z jerk (mm/sec)
    		max_z_jerk	  = (float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_Z_JERK,100)	 / 10.0;

    		float advanceK	 	= (float)eeprom::getEepromUInt32(eeprom::ACCEL_ADVANCE_K,50)		/ 100000.0;
    		float filamentDiameter  = (float)eeprom::getEepromUInt32(eeprom::ACCEL_FILAMENT_DIAMETER,175)	/ 100.0;

		force_acceleration_off = false;

		if ( planner ) 	plannerMaxBufferSize = BLOCK_BUFFER_SIZE - 1;
		else		plannerMaxBufferSize = 1;

		plan_init(advanceK, filamentDiameter, axis_steps_per_unit[E_AXIS]);	//Initialize planner
  		st_init();								//Initialize stepper

		lastTarget = Point(st_get_position(X_AXIS), st_get_position(Y_AXIS), st_get_position(Z_AXIS), st_get_position(E_AXIS), 0);
	}
	else lastTarget = getPosition();
#endif
}

/// Define current position as given point
void definePosition(const Point& position) {
#ifdef HAS_STEPPER_ACCELERATION
	if (( acceleration ) && ( ! force_acceleration_off )) {
		plan_set_position(position[0], position[1], position[2], position[3]);
	}
	else {
#endif
		for (int i = 0; i < STEPPER_COUNT; i++) {
			axes[i].definePosition(position[i]);
		}
#ifdef HAS_STEPPER_ACCELERATION
	}
	lastTarget = position;
#endif
}

/// Get current position
const Point getPosition() {
#if STEPPER_COUNT > 3
#ifdef HAS_STEPPER_ACCELERATION
	if (( acceleration ) && ( ! force_acceleration_off ))
			    return lastTarget;
	else		    
#endif
		  	    return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,axes[4].position);
#else
#ifdef HAS_STEPPER_ACCELERATION
	if (( acceleration ) && ( ! force_acceleration_off ))
			    return Point(lastTarget[0], lastTarget[1], lastTarget[2]);
	else		    
#endif
			    return Point(axes[0].position,axes[1].position,axes[2].position);
#endif
}

void setHoldZ(bool holdZ_in) {
	holdZ = holdZ_in;
}

#ifdef HAS_STEPPER_ACCELERATION

//Calculates the feedrate based on moving from "from" to "to"

float calcFeedRate(const Point& from, const Point& to, int32_t interval ) {

	//Calculate the distance in mm's by:
	//Calculate the delta distances and convert to mm's
	//Then sqr of sum of squares of the deltas for the distance

	//We also calculate at the same time,  master_steps in steps by finding the dominant axis (for all 5)
	//You would think it would be for X/Y/Z only, but they "mistakenly" use all 5 in rep g.
	//so we do the same here for consistancy

	int32_t master_steps = 0;
	float   distance = 0.0;

	for (uint8_t i = 0; i < AXIS_COUNT; i ++ ) {
		int32_t delta = to[i] - from[i];
		if ( delta < 0 ) delta *= -1;

		if ( delta > master_steps ) master_steps = delta;

		const float delta_mm = (float)delta / axis_steps_per_unit[i];
		distance += delta_mm * delta_mm;
	}
	distance = sqrt(distance);

	return (distance * 60000000.0) / ((float)interval * (float)master_steps);
}

#endif

void setTarget(const Point& target, int32_t dda_interval) {
#ifdef HAS_STEPPER_ACCELERATION
	if (( acceleration ) && ( ! force_acceleration_off )) {
		float feedRate = calcFeedRate(lastTarget, target, dda_interval );

		//Figure out the distance between x1,y1 and x2,y2 using pythagoras
		plan_buffer_line(target[0], target[1], target[2], target[3], feedRate, 0);
	} else {
#endif
		int32_t max_delta = 0;
		for (int i = 0; i < AXIS_COUNT; i++) {
			axes[i].setTarget(target[i], false);
			const int32_t delta = axes[i].delta;
			// Only shut z axis on inactivity
			if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
			else if (delta != 0) axes[i].enableStepper(true);
			if (delta > max_delta) {
				max_delta = delta;
			}
		}
		// compute number of intervals for this move
		intervals = ((max_delta * dda_interval) / INTERVAL_IN_MICROSECONDS);
		intervals_remaining = intervals;
		const int32_t negative_half_interval = -intervals / 2;
		for (int i = 0; i < AXIS_COUNT; i++) {
			axes[i].counter = negative_half_interval;
		}
		is_running = true;

#ifdef HAS_STEPPER_ACCELERATION
	}

	lastTarget = target;
#endif
}

void setTargetNew(const Point& target, int32_t us, uint8_t relative) {
#ifdef HAS_STEPPER_ACCELERATION
	if (( acceleration ) && ( ! force_acceleration_off )) {
		Point newPosition = target;
		for (uint8_t i = 0; i < AXIS_COUNT; i ++)  {
			if ((relative & (1 << i)) != 0)
				newPosition[i] = lastTarget[i] + target[i];
		}

		int32_t max_delta = 0;
		for (int i = 0; i < AXIS_COUNT; i++) {
			int32_t delta = newPosition[i] - lastTarget[i];
			if ( delta < 0 ) delta *= -1;
			if (delta > max_delta) {
				max_delta = delta;
			}
		}
		int32_t dda_interval = us / max_delta;
		float feedRate = calcFeedRate(lastTarget, newPosition, dda_interval);

		plan_buffer_line(newPosition[0], newPosition[1], newPosition[2], newPosition[3], feedRate, 0);
		lastTarget = newPosition;
	} else {
#endif
		for (int i = 0; i < AXIS_COUNT; i++) {
			axes[i].setTarget(target[i], (relative & (1 << i)) != 0);
			// Only shut z axis on inactivity
			const int32_t delta = axes[i].delta;
			if (i == 2 && !holdZ) {
				axes[i].enableStepper(delta != 0);
			} else if (delta != 0) {
				axes[i].enableStepper(true);
			}

#ifdef HAS_STEPPER_ACCELERATION
			lastTarget[i] = axes[i].absoluteTarget;
#endif
		}
		// compute number of intervals for this move
		intervals = us / INTERVAL_IN_MICROSECONDS;
		intervals_remaining = intervals;
		const int32_t negative_half_interval = -intervals / 2;
		for (int i = 0; i < AXIS_COUNT; i++) {
			axes[i].counter = negative_half_interval;
		}
		is_running = true;

#ifdef HAS_STEPPER_ACCELERATION
	}
#endif
}

#ifdef HAS_STEPPER_ACCELERATION
void switchToRegularDriver() {
	if ( ! acceleration )	return;

	cli();

	//Get the current position of the accelerated driver and
	//store it in the regular driver
	Point currentPosition = getPosition();
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].definePosition(currentPosition[i]);
	}

	force_acceleration_off = true;

	//Change the interrupt frequency to the regular driver
	Motherboard::getBoard().setupFixedStepperTimer();

	sei();
}

void switchToAcceleratedDriver() {
	if ( ! acceleration )	return;

	//Get the current position of the regular driver and
	//store it in the accelerated driver

	cli();

	force_acceleration_off = false;

	Point currentPosition = Point(axes[0].position, axes[1].position, axes[2].position, axes[3].position, axes[4].position);
	definePosition(currentPosition);

	//Change the interrupt frequency to the accelerated driver
	Motherboard::getBoard().setupAccelStepperTimer();
	
	sei();
}
#endif

/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step) {

#ifdef HAS_STEPPER_ACCELERATION
	if ( acceleration ) switchToRegularDriver();
#endif

	intervals_remaining = INT32_MAX;
	intervals = us_per_step / INTERVAL_IN_MICROSECONDS;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		if ((axes_enabled & (1<<i)) != 0) {
			axes[i].setHoming(maximums);
		} else {
			axes[i].delta = 0;
		}
	}
	is_homing = true;
}

/// Enable/disable the given axis.
void enableAxis(uint8_t index, bool enable) {
        if (index < STEPPER_COUNT) {
                axes[index].enableStepper(enable);
	}
}

/// Report if the given axis is enabled or disabled
bool isEnabledAxis(uint8_t index) {
        if (index < STEPPER_COUNT) {
                axes[index].isEnabledStepper();
	}
}

bool isAtMaximum(uint8_t index) {
        if (index < STEPPER_COUNT) {
                return axes[index].isAtMaximum();
	}
	return false;
}


bool isAtMinimum(uint8_t index) {
        if (index < STEPPER_COUNT) {
                return axes[index].isAtMinimum();
	}
	return false;
}

void doLcd() {
//	Motherboard::getBoard().lcd.setCursor(0,3);
//	Motherboard::getBoard().lcd.writeFloat((float)movesplanned(),1);
//	Motherboard::getBoard().lcd.write(' ');
}

bool doInterrupt() {
#ifdef HAS_STEPPER_ACCELERATION
	if (( acceleration ) && ( ! force_acceleration_off )) {
		//st_interrupt runs all the time, is_running reflects if we have space in the buffer or not,
		//i.e. it enables us to add more commands if it's false

		if ( movesplanned() >=  plannerMaxBufferSize)	is_running = true;
		else						is_running = false;

		st_interrupt();
	
		return is_running;
	} else {
#endif
		if (is_running) {
			if (intervals_remaining-- == 0) {
				is_running = false;
			} else {
				for (int i = 0; i < STEPPER_COUNT; i++) {
					axes[i].doInterrupt(intervals);
				}
			}
			return is_running;
		} else if (is_homing) {
			is_homing = false;
			for (int i = 0; i < STEPPER_COUNT; i++) {
				bool still_homing = axes[i].doHoming(intervals);
				is_homing = still_homing || is_homing;
			}

#ifdef HAS_STEPPER_ACCELERATION
			//If homing has finished and we're accelerated, switch back to the accelerated drived
			if (( ! is_homing ) && ( acceleration )) switchToAcceleratedDriver();
#endif

			return is_homing;
		}
#ifdef HAS_STEPPER_ACCELERATION
	}
#endif
	return false;
}

#ifdef HAS_STEPPER_ACCELERATION

bool doAdvanceInterrupt() {
#ifdef ADVANCE
	if ( acceleration ) st_advance_interrupt();
#endif
}

#endif

}
