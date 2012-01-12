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
#include "Planner.hh"
#include <math.h>
#include <stdint.h>

namespace steppers {

volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;
struct feedrate_element {
	int32_t intervals;
	int32_t target;
};
feedrate_element feedrate_elements[3];
volatile int32_t feedrate_intervals_remaining;
volatile uint8_t current_feedrate_index;

volatile int32_t timer_counter;
volatile int8_t feedrate_scale_shift;
// The ALL_AXIS_COUNT includes the virtual "speed" axis
#define ALL_AXIS_COUNT STEPPER_COUNT+1

// STEPRATE_AXIS is the number of the virtual axis that control the steprate
#define STEPRATE_AXIS STEPPER_COUNT
StepperAxis axes[ALL_AXIS_COUNT];
volatile bool is_homing;

bool holdZ = false;

planner::Block *current_block;

enum {
	ACCELERATING = 1,
	IN_PLATEAU,
	DECELERATING
} current_phase; // pretend we are just leaving a block

bool isRunning() {
	return is_running || is_homing;
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i] = StepperAxis(motherboard.getStepperInterface(i));
	}

	axes[STEPRATE_AXIS] = StepperAxis();
	timer_counter = 0;
	feedrate_scale_shift = 0;
	current_phase = DECELERATING;
	current_block = 0;
	
	for (int i = 0; i < 3; i++) {
		feedrate_elements[i] = feedrate_element();
		feedrate_elements[i].intervals = 0;
		feedrate_elements[i].target = 0;
	}
	feedrate_intervals_remaining = 0;
	
}

void abort() {
	is_running = false;
	is_homing = false;
	timer_counter = 0;
	feedrate_scale_shift = 0;
}

/// Define current position as given point
void definePosition(const Point& position) {
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].definePosition(position[i]);
	}
}

/// Get current position
const Point getPosition() {
#if STEPPER_COUNT > 3
	return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,axes[4].position);
#else
	return Point(axes[0].position,axes[1].position,axes[2].position);
#endif
}

void setHoldZ(bool holdZ_in) {
	holdZ = holdZ_in;
}


void setTarget(const Point& target, int32_t dda_interval) {
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
		// // compute number of intervals for this move
		//intervals = ((max_delta * dda_interval) / INTERVAL_IN_MICROSECONDS);

		// reset feedrate_scale_shift and feedrate position
	if (feedrate_scale_shift != 0) {
		axes[STEPRATE_AXIS].position = axes[STEPRATE_AXIS].position << feedrate_scale_shift;
		feedrate_scale_shift = 0;
	}
	if (axes[STEPRATE_AXIS].position == 0) {
		axes[STEPRATE_AXIS].definePosition(dda_interval);
	}

	axes[STEPRATE_AXIS].setTarget(dda_interval, /*relative =*/ false);

	if (max_delta == 0) {
		is_running = false;
		return;
	}

	// WARNING: Edge case where axes[STEPRATE_AXIS].delta > INT32_MAX is unhandled
	int8_t scale_shift = 0;
	while ((axes[STEPRATE_AXIS].delta >> scale_shift) > max_delta) {
		scale_shift++;
	}
	if (scale_shift > 0) {
		feedrate_scale_shift = scale_shift;
		axes[STEPRATE_AXIS].position = axes[STEPRATE_AXIS].position >> feedrate_scale_shift;
		axes[STEPRATE_AXIS].target   = axes[STEPRATE_AXIS].target   >> feedrate_scale_shift;
		axes[STEPRATE_AXIS].delta    = axes[STEPRATE_AXIS].delta    >> feedrate_scale_shift;
	}

	// We use += here so that the odd rounded-off time from the last move is still waited out
	timer_counter += axes[STEPRATE_AXIS].position << feedrate_scale_shift;

	intervals = max_delta;
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < ALL_AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
	}
	is_running = true;
}

/*
void setTargetNew(const Point& target, int32_t us, uint8_t relative) {
for (int i = 0; i < AXIS_COUNT; i++) {
axes[i].setTarget(target[i], (relative & (1 << i)) != 0);
// Only shut z axis on inactivity
const int32_t delta = axes[i].delta;
if (i == 2 && !holdZ) {
axes[i].enableStepper(delta != 0);
} else if (delta != 0) {
axes[i].enableStepper(true);
}
}
// compute number of intervals for this move
intervals = us / INTERVAL_IN_MICROSECONDS;
intervals_remaining = intervals;
const int32_t negative_half_interval = -intervals / 2;
for (int i = 0; i < AXIS_COUNT; i++) {
axes[i].counter = negative_half_interval;
}
is_running = true;
}
*/

// load up the next movment
// WARNING: called from inside the ISR, so get out fast
bool getNextMove() {	
	is_running = false;

	if (planner::isBufferEmpty())
		return false;

	current_block = planner::getNextBlock();
	// Mark block as busy (being executed by the stepper interrupt)
	current_block->busy = true;
	Point &target = current_block->target;
	int32_t dda_interval = 1000000/current_block->nominal_rate;
	
	int32_t max_delta = 0;//current_block->step_event_count;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(target[i], false);
		const int32_t delta = axes[i].delta;

		// Only shut z axis on inactivity
		if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
		else if (delta != 0) axes[i].enableStepper(true);
		
		if (delta > max_delta) {
			max_delta = delta;
		}
	}
		
#if 0
	current_feedrate_index = 0;

	// setup acceleration
	feedrate_elements[0].intervals = current_block->accelerate_until;
	feedrate_elements[0].target    = 1000000/current_block->nominal_rate;

	// setup plateau
	feedrate_elements[1].intervals = current_block->decelerate_after - current_block->accelerate_until;
	feedrate_elements[1].target    = 1000000/current_block->nominal_rate;

	// setup deceleration
	feedrate_elements[2].intervals = current_block->step_event_count - current_block->decelerate_after;
	feedrate_elements[2].target    = 1000000/current_block->final_rate;

	if (feedrate_elements[0].intervals > 0) {
		// setup the acceleration speed
		axes[STEPRATE_AXIS].definePosition(1000000/current_block->initial_rate);
		axes[STEPRATE_AXIS].setTarget(1000000/current_block->nominal_rate, /*relative =*/ false);
	} else if (feedrate_elements[1].intervals > 0) {
		// setup the acceleration speed
		axes[STEPRATE_AXIS].definePosition(1000000/current_block->nominal_rate);
		axes[STEPRATE_AXIS].setTarget(1000000/current_block->nominal_rate, /*relative =*/ false);
		current_feedrate_index = 1;
	} else if (feedrate_elements[2].intervals > 0) {
		// setup the acceleration speed
		axes[STEPRATE_AXIS].definePosition(1000000/current_block->nominal_rate);
		axes[STEPRATE_AXIS].setTarget(1000000/current_block->final_rate, /*relative =*/ false);
		current_feedrate_index = 2;
	}

	feedrate_intervals_remaining = feedrate_elements[current_feedrate_index].intervals;
	axes[STEPRATE_AXIS].counter  = -(feedrate_elements[current_feedrate_index].intervals / 2);
#else
	axes[STEPRATE_AXIS].definePosition(dda_interval);
	axes[STEPRATE_AXIS].setTarget(dda_interval, /*relative =*/ false);
#endif

	
	// WARNING: Edge case where axes[STEPRATE_AXIS].delta > INT32_MAX is unhandled
	int8_t scale_shift = 0;
	while ((axes[STEPRATE_AXIS].delta >> scale_shift) > max_delta) {
		scale_shift++;
	}
	if (scale_shift > 0) {
		feedrate_scale_shift = scale_shift;
		axes[STEPRATE_AXIS].position = axes[STEPRATE_AXIS].position >> feedrate_scale_shift;
		axes[STEPRATE_AXIS].target   = axes[STEPRATE_AXIS].target   >> feedrate_scale_shift;
		axes[STEPRATE_AXIS].delta    = axes[STEPRATE_AXIS].delta    >> feedrate_scale_shift;
	}

	// We use += here so that the odd rounded-off time from the last move is still waited out
	timer_counter += axes[STEPRATE_AXIS].position << feedrate_scale_shift;

	intervals = max_delta;
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < ALL_AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
	}
	is_running = true;
	return true;
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
void enableAxis(uint8_t index, bool enable) {
	if (index < STEPPER_COUNT) {
		axes[index].enableStepper(enable);
	}
}

void startRunning() {
	if (is_running)
		return;
	// is_running = true;
	getNextMove();
}

bool doInterrupt() {
	if (is_running) {
		timer_counter -= INTERVAL_IN_MICROSECONDS;
		if (timer_counter <= 0) {
			if (intervals_remaining-- == 0) {
				// getNextMove();
				is_running = false;
			} else {

				for (int i = 0; i < STEPPER_COUNT; i++) {
					axes[i].doInterrupt(intervals);
				}
				#if 0
				if (feedrate_intervals_remaining-- == 0) {
					axes[STEPRATE_AXIS].position = feedrate_elements[current_feedrate_index].target;
					
					current_feedrate_index++;
	
					axes[STEPRATE_AXIS].setTarget(feedrate_elements[current_feedrate_index].target, /*relative =*/ false);
	
					feedrate_intervals_remaining = feedrate_elements[current_feedrate_index].intervals;
					axes[STEPRATE_AXIS].counter  = -(feedrate_elements[current_feedrate_index].intervals / 2);
	
					// WARNING: Edge case where axes[STEPRATE_AXIS].delta > INT32_MAX is unhandled
					int8_t scale_shift = 0;
					while ((axes[STEPRATE_AXIS].delta >> scale_shift) > feedrate_intervals_remaining) {
						scale_shift++;
					}
					if (scale_shift > 0) {
						feedrate_scale_shift = scale_shift;
						axes[STEPRATE_AXIS].position = axes[STEPRATE_AXIS].position >> feedrate_scale_shift;
						axes[STEPRATE_AXIS].target   = axes[STEPRATE_AXIS].target   >> feedrate_scale_shift;
						axes[STEPRATE_AXIS].delta    = axes[STEPRATE_AXIS].delta    >> feedrate_scale_shift;
					}
				}

				axes[STEPRATE_AXIS].doInterrupt(feedrate_elements[current_feedrate_index].intervals);
				#else
				axes[STEPRATE_AXIS].doInterrupt(intervals);
				#endif
				timer_counter += axes[STEPRATE_AXIS].position << feedrate_scale_shift;
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
