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
#include <stdint.h>

namespace steppers {

class Axis {
public:
	Axis() : interface(0) {}

	Axis(StepperInterface& stepper_interface) :
		interface(&stepper_interface) {
		reset();
	}

	/// Set target coordinate and compute delta
	void setTarget(const int32_t target_in) {
		target = target_in;
		delta = target - position;
		direction = true;
		if (delta != 0) {
			interface->setEnabled(true);
		}
		if (delta < 0) {
			delta = -delta;
			direction = false;
		}
	}

	/// Set homing mode
	void setHoming(const bool direction_in) {
		direction = direction_in;
		interface->setEnabled(true);
		delta = 1;
	}

	/// Define current position as the given value
	void definePosition(const int32_t position_in) {
		position = position_in;
	}

	/// Enable/disable stepper
	void enableStepper(bool enable) {
		interface->setEnabled(enable);
	}

	/// Reset to initial state
	void reset() {
		position = 0;
		minimum = 0;
		maximum = 0;
		target = 0;
		counter = 0;
		delta = 0;
	}

	void doInterrupt(const int32_t intervals) {
		counter += delta;
		if (counter >= 0) {
			interface->setDirection(direction);
			counter -= intervals;
			if (direction) {
				if (!interface->isAtMaximum()) interface->step(true);
				position++;
			} else {
				if (!interface->isAtMinimum()) interface->step(true);
				position--;
			}
			interface->step(false);
		}
	}

	// Return true if still homing; false if done.
	bool doHoming(const int32_t intervals) {
		if (delta == 0) return false;
		counter += delta;
		if (counter >= 0) {
			interface->setDirection(direction);
			counter -= intervals;
			if (direction) {
				if (!interface->isAtMaximum()) {
					interface->step(true);
				} else {
					return false;
				}
				position++;
			} else {
				if (!interface->isAtMinimum()) {
					interface->step(true);
				} else {
					return false;
				}
				position--;
			}
			interface->step(false);
		}
		return true;
	}

	StepperInterface* interface;
	/// Current position on this axis, in steps
	volatile int32_t position;
	/// Minimum position, in steps
	int32_t minimum;
	/// Maximum position, in steps
	int32_t maximum;
	/// Target position, in steps
	volatile int32_t target;
	/// Step counter; represents the proportion of a
	/// step so far passed.  When the counter hits
	/// zero, a step is taken.
	volatile int32_t counter;
	/// Amount to increment counter per tick
	volatile int32_t delta;
	/// True for positive, false for negative
	volatile bool direction;
};

volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;
Axis axes[STEPPER_COUNT];
volatile bool is_homing;
volatile bool is_running_homing_script; //new boolean object that says if the machine is running a homing script. (We have to script the homing because we home the Z stage last to prevent nozzle crashing into BP problems.)

bool isRunning() {
	return is_running || is_homing || is_running_homing_script;
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i] = Axis(motherboard.getStepperInterface(i));
	}
}

void abort() {
	is_running = false;
	is_homing = false;
	is_running_homing_script = false;
}

/// Define current position as given point
void definePosition(const Point& position) {
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].definePosition(position[i]);
	}
}

/// Get current position
const Point getPosition() {
	return Point(axes[0].position,axes[1].position,axes[2].position);
}

void setTarget(const Point& target, int32_t dda_interval) {
	int32_t max_delta = 0;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].setTarget(target[i]);
		const int32_t delta = axes[i].delta;
		// Only shut z axis on inactivity
		if (i == 2) axes[i].enableStepper(delta != 0);
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
}

/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step) {
is_running_homing_script = true;
	intervals_remaining = INT32_MAX;
	intervals = us_per_step / INTERVAL_IN_MICROSECONDS;
	const int32_t negative_half_interval = -intervals / 2;
	if (axes_enabled <= 4 || maximums == true) { //if axis enabled does not equal XYZ (aka home all axis) or if the homing direction is positive. just wanted to give a shout out here my fellow comrade intern Mike for helping me make this line sane. Always sanity check with a buddy!
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		if ((axes_enabled & (1<<i)) != 0) { //Genius! He compared the binarys of the flag to see if that axis is enabled!
			axes[i].setHoming(maximums);
		} else {
			axes[i].delta = 0;
		}
	}
	is_homing = true;
} else { //if Axis enabled equals XYZ and direction is negative, we must take extra care not to smash the Z axis nozzle into the XY axis BP. Therefore we must home the XY axis out of the way before we home the Z axis downwards.
//must change flags to 3 instead of 7. 7 is all. 3 is xy (X =1, Y =2, Z=4).
const uint8_t axes_enabled_minus_z = axes_enabled - 4; //don't home the z axis
for (int i = 0; i < AXIS_COUNT; i++) { //start homing all of the other axis
		axes[i].counter = negative_half_interval;
		if ((axes_enabled_minus_z & (1<<i)) != 0) {
			axes[i].setHoming(maximums);
		} else {
			axes[i].delta = 0;
		}
	}
is_homing = true; //home!
while (is_homing == true) { } //wait 'till done homing
axes[2].counter = negative_half_interval; //set speed?
axes[2].setHoming(maximums); //set direction and start homing the Z axis
is_homing = true; //home baby home!
}
is_running_homing_script = false;
}

/// Enable/disable the given axis.
void enableAxis(uint8_t which, bool enable) {
	axes[which].enableStepper(enable);
}

bool doInterrupt() {
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
		return is_homing;
	}
	return false;
}

}
