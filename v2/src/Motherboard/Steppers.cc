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
volatile bool script_done_homing = false;
volatile bool script_done_moving = false;

bool isRunning() {
	return is_running || is_homing;
}

bool scripts_done_homing() {
return script_done_homing;
}

bool scripts_done_moving() {
return script_done_moving;
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




void moveCarefully(const Point& target, int32_t Z_offset) {
						script_done_moving = false;
						if (Z_offset < 0) { //if the z offset is negative (this should not usually happen). assume we want to Center the XY before Z (as if we are centering the Z stage in the - direction)
						Point currentPosition = getPosition();
						bool waiting_to_move_Zstage = true; //keep track (suspect?)
						int32_t x = target[0]; //Move XY
						int32_t y = target[1];
						int32_t z = currentPosition[2]; //Leave this alone.
						
						int32_t dda = 1017; // max feedrate for XY stage
						setTarget(Point(x,y,z),dda); //move XY
						while (waiting_to_move_Zstage == true) {
						if (!isRunning()) {
						waiting_to_move_Zstage = false;
						x = target[0];
						y = target[1];
						z = target[2]; 
						int32_t dda = 1250; // max feedrate for Z stage
						setTarget(Point(x,y,z),dda); //move everything
						
						bool waiting_to_say_done = true;//probably not necessary and will be phased out
						while (waiting_to_say_done == true) {
						if (isRunning() == false) {
						waiting_to_say_done = false;
						}
						}
						}
						}
						
						} else { //we must be moving in the positive direction so move Z before XY.
						Point currentPosition = getPosition();
						bool waiting_to_move_Zstage = false; //keep track
						int32_t x = currentPosition[0]; //leave these were they are
						int32_t y = currentPosition[1];
						int32_t z = target[2] + Z_offset; //move the Z stage back up to a bit above zero to avoid the BP hitting it.
						int32_t dda = 1250; // max feedrate for Z stage
						setTarget(Point(x,y,z),dda);
						waiting_to_move_Zstage = true;
						
						while (waiting_to_move_Zstage == true) {
						if (!isRunning()) {
						waiting_to_move_Zstage = false;
						x = target[0];
						y = target[1];
						z = target[2] + Z_offset; //keep this where it was
						int32_t dda = 1017; // max feedrate for XY stage
						setTarget(Point(x,y,z),dda); //move everything back up
						}// End of if is still running
						}//End of while waiting
						
						waiting_to_move_Zstage = true;
						
						while (waiting_to_move_Zstage == true) {
						if (!isRunning()) {
						waiting_to_move_Zstage = false;
						x = target[0];
						y = target[1];
						z = target[2]; 
						int32_t dda = 1250; // max feedrate for Z stage
						setTarget(Point(x,y,z),dda); //move everything back up
						
						bool waiting_to_say_done = true; //probably not necessary and will be phased out
						while (waiting_to_say_done == true) {
						if (isRunning() == false) {
						waiting_to_say_done = false;
						}
						}
						}// End of if is still running
						}//End of while waiting
						}
						script_done_moving = true;
}

void homeCarefully(const bool direction, uint8_t flags, const uint32_t feedrate) {
//Don't drink and home. Home carefully my friends!
script_done_homing = false;
if (flags & (1<<2) != 0 && (flags - 4) != 0 && direction == false) { //if flags says home Z and something else (we don't care what). And it's also in the negative direction then home carefully.
					flags = flags - 4; //Don't home Z.
					bool waiting_to_move_Zstage = true;
					startHoming(direction,
							flags,
							feedrate); //home the others
						while (waiting_to_move_Zstage == true) {
						if (isRunning() == false) { //wait till done
						waiting_to_move_Zstage = false;
						flags = 4; //Home Z.
						startHoming(direction,
							flags,
							feedrate);
						}
						}
						} else if (flags & (1<<2) != 0 && (flags - 4) != 0 && direction == true) { 
						//home the z up before homing xy in the positive direction.
						uint8_t other_axis_flags = flags - 4; //axis besides Z to home.
						
						flags = 4; // Home Z up.
					bool waiting_to_move_XYstage = true;
					startHoming(direction,
							flags,
							feedrate); //home the others
						while (waiting_to_move_XYstage == true) {
						if (isRunning() == false) { //wait till done
						waiting_to_move_XYstage = false;
						flags = other_axis_flags; //Home the rest of the axis (besides Z).
						startHoming(direction,
							flags,
							feedrate);
						}
						}
						} else { //no special care is needed.
						startHoming(direction,
							flags,
							feedrate);
					}
					bool waiting_to_say_done = true;
					while (waiting_to_say_done == true) {
						if (isRunning() == false) {
						waiting_to_say_done = false;
						script_done_homing = true;
						}
						}
}

}
