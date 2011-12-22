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
#include <stdint.h>

namespace steppers {

volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;
volatile int32_t timer_counter;
// The ALL_AXIS_COUNT includes the virtual "speed" axis
#define ALL_AXIS_COUNT STEPPER_COUNT+1

// STEPRATE_AXIS is the number of the virtual axis that control the steprate
#define STEPRATE_AXIS STEPPER_COUNT
StepperAxis axes[ALL_AXIS_COUNT];
volatile bool is_homing;

bool holdZ = false;

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
}

void abort() {
	is_running = false;
	is_homing = false;
	timer_counter = 0;
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
        // compute number of intervals for this move
        //intervals = ((max_delta * dda_interval) / INTERVAL_IN_MICROSECONDS);

		axes[STEPRATE_AXIS].setScaleShift(0);
		axes[STEPRATE_AXIS].definePosition(dda_interval);//dda_interval/INTERVAL_IN_MICROSECONDS
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
		axes[STEPRATE_AXIS].setScaleShift(scale_shift);

		// if (axes[STEPRATE_AXIS].delta > max_delta) {
		// 	max_delta = axes[STEPRATE_AXIS].delta;
		// }
		
		// We use += here so that the odd rounded-off time from the last move is still waited out
		timer_counter += axes[STEPRATE_AXIS].position << axes[STEPRATE_AXIS].scale_shift;

        intervals = max_delta;
        intervals_remaining = intervals;
        const int32_t negative_half_interval = -(intervals>>1); // same as -(intervals/2), but faster (?)
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

planner::Block *current_block = 0;
enum {
	ACCELERATING = 1,
	IN_PLATEAU,
	DECELERATING
} current_phase = DECELERATING; // pretend we are just leaving a block

// load up the next movment
// WARNING: called from inside the ISR, so get out fast
bool getNextMove() {
	uint32_t max_delta = 0;
	is_running = false;

#if 0
	if (current_phase == DECELERATING) {
		current_phase == ACCELERATING;
		current_block = 0;
	}
	else if (current_phase == ACCELERATING) {
		current_phase == IN_PLATEAU;
	}
	else if (current_phase == IN_PLATEAU) {
		current_phase == DECELERATING;
	}
	
	// step_rate is steps/sec, step_timing is sec/step
	if (!current_block) {
FetchNext:
		if (planner::block_buffer.isEmpty())
			return false;

		current_block = planner::block_buffer.getTail();
		// Mark block as busy (being executed by the stepper interrupt)
		current_block->busy = true;
		
		// bump the tail forward
		planner::block_buffer--;
		
		current_phase = ACCELERATING;
		if (current_block->accelerate_until > 0) {
			// reset steprate axis scale
			axes[STEPRATE_AXIS].setScaleShift(0);
		
			// setup all real axes
			for (int i = 0; i < STEPPER_COUNT; i++) {
				// rearrange a little to avoid fractional math.
				// Should be the same as: (accelerate_until/step_event_count) * steps[i]
				axes[i].setTarget((current_block->accelerate_until * current_block->steps[i]) / current_block->step_event_count, /*relative =*/ true);
			}
			
			// setup steprate axis
			axes[STEPRATE_AXIS].definePosition(INTERVALS_PER_SECOND/current_block->initial_rate);
			axes[STEPRATE_AXIS].setTarget(INTERVALS_PER_SECOND/current_block->nominal_rate, /*relative =*/ false);
			
			max_delta = current_block->accelerate_until;
		} else {
			current_phase = IN_PLATEAU;
		}
	}
	
	if (current_phase == IN_PLATEAU && current_block->accelerate_until != current_block->decelerate_after) {
		// reset steprate axis scale
		axes[STEPRATE_AXIS].setScaleShift(0);
		
		// setup all real axes
		for (int i = 0; i < STEPPER_COUNT; i++) {
			// rearrange a little to avoid fractional math.
			// Should be the same as: (decelerate_after/step_event_count) * steps[i]
			axes[i].setTarget((current_block->decelerate_after * current_block->steps[i]) / current_block->step_event_count, /*relative =*/ true);
		}

		// setup steprate axis, using the current position properly
		axes[STEPRATE_AXIS].setTarget(INTERVALS_PER_SECOND/current_block->nominal_rate, /*relative =*/ false);
		
		max_delta = current_block->step_event_count - current_block->accelerate_until - current_block->decelerate_after;
	} else {
		current_phase = DECELERATING;
	}
	
	if (current_phase == DECELERATING && current_block->decelerate_after != current_block->step_event_count) {
		// reset steprate axis scale
		axes[STEPRATE_AXIS].setScaleShift(0);
		
		// setup all real axes
		for (int i = 0; i < STEPPER_COUNT; i++) {
			// rearrange a little to avoid fractional math.
			// Should be the same as: (decelerate_after/step_event_count) * steps[i]
			axes[i].setTarget((current_block->decelerate_after * current_block->steps[i]) / current_block->step_event_count, /*relative =*/ true);
		}

		// setup steprate axis, using the current position properly
		axes[STEPRATE_AXIS].setTarget(INTERVALS_PER_SECOND/current_block->final_rate, /*relative =*/ false);
	} else {
		current_phase = ACCELERATING;
		// My apologies for a goto, but we're in a hurry! -Rob
		goto FetchNext;
	}
	
#else

	if (!current_block) {
		if (planner::block_buffer.isEmpty())
			return false;

		current_block = planner::block_buffer.getTail();
		// Mark block as busy (being executed by the stepper interrupt)
		current_block->busy = true;

		// bump the tail forward
		planner::block_buffer--;
	}
	
	max_delta = current_block->step_event_count;
	
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(current_block->steps[i], /*relative =*/ true);
	}
	axes[STEPRATE_AXIS].definePosition(INTERVALS_PER_SECOND/current_block->nominal_rate);
	axes[STEPRATE_AXIS].setTarget(INTERVALS_PER_SECOND/current_block->nominal_rate, /*relative =*/ false);
	
	current_block = 0;
#endif
	
#if 1
	//make sure that steprate axis is not the longest axis, by scaling in halves
	int8_t scale_shift = 0;
	while ((axes[STEPRATE_AXIS].delta >> scale_shift) > max_delta) {
		scale_shift++;
	}
	axes[STEPRATE_AXIS].setScaleShift(scale_shift);

	//compute number of intervals for this move
	intervals = max_delta;
#else	
	if (axes[STEPRATE_AXIS].delta > max_delta)
		max_delta = axes[STEPRATE_AXIS].delta;
	intervals = ((max_delta * current_block->nominal_rate) / INTERVAL_IN_MICROSECONDS);
	intervals = max_delta;
#endif

	for (int i = 0; i < STEPPER_COUNT; i++) {
		const int32_t delta = axes[i].delta;
		// Only shut z axis on inactivity
		if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
		else if (delta != 0) axes[i].enableStepper(true);
	}
	
    
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -(intervals >> 1);
	for (int i = 0; i < ALL_AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
	}
	timer_counter = axes[STEPRATE_AXIS].position<<axes[STEPRATE_AXIS].scale_shift;
	
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
	is_running = true;
	//getNextMove();
}

bool doInterrupt() {
	if (is_running) {
		timer_counter -= INTERVAL_IN_MICROSECONDS;
		if (timer_counter <= 0) {
			if (intervals_remaining-- == 0) {
				//if (!getNextMove()) {
					is_running = false;
					return false;
				//}
			}
		
			for (int i = 0; i < ALL_AXIS_COUNT; i++) {
				axes[i].doInterrupt(intervals);
			}
			
			timer_counter += axes[STEPRATE_AXIS].position<<axes[STEPRATE_AXIS].scale_shift;
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
