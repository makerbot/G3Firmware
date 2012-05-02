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
#include <stdlib.h>

namespace steppers {

volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;

struct feedrate_element {
	uint32_t rate; // interval value of the feedrate axis
	uint32_t steps;     // number of steps of the master axis to change
	uint32_t target;
};
feedrate_element feedrate_elements[3];
volatile int32_t feedrate_steps_remaining;
volatile int32_t feedrate;
volatile int32_t feedrate_target; // convenient storage to save lookup time
volatile int8_t  feedrate_dirty; // indicates if the feedrate_inverted needs recalculated
volatile int32_t feedrate_inverted;
volatile int32_t feedrate_changerate;
volatile int32_t acceleration_tick_counter;
volatile int32_t feedrate_multiplier; // should always be 2^N and > 0
volatile uint8_t current_feedrate_index;

volatile int32_t timer_counter;
StepperAxis axes[STEPPER_COUNT];
volatile bool is_homing;

// Pin stepperTimingDebugPin = STEPPER_TIMER_DEBUG;

bool holdZ = false;

planner::Block *current_block;

bool isRunning() {
	return is_running || is_homing;
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	is_homing = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i] = StepperAxis(motherboard.getStepperInterface(i));
	}
	timer_counter = 0;

	current_block = NULL;
	
	for (int i = 0; i < 3; i++) {
		feedrate_elements[i] = feedrate_element();
		feedrate_elements[i].rate = 0;
		feedrate_elements[i].target = 0;
		feedrate_elements[i].steps = 0;
	}
	
	feedrate_steps_remaining = 0;
	feedrate = 0;
	feedrate_inverted = 0;
	feedrate_dirty = 1;
        feedrate_multiplier = 1;
	acceleration_tick_counter = 0;
	current_feedrate_index = 0;
	
	// stepperTimingDebugPin.setDirection(true);
	// stepperTimingDebugPin.setValue(false);
}

void abort() {
	is_running = false;
	is_homing = false;
	timer_counter = 0;
	current_block = NULL;
	feedrate_steps_remaining = 0;
	feedrate = 0;
	feedrate_inverted = 0;
	feedrate_dirty = 1;
        feedrate_multiplier = 1;
	acceleration_tick_counter = 0;
	current_feedrate_index = 0;
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
	#if STEPPER_COUNT > 4
		return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,axes[4].position);
	#else
		return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,0);
	#endif
#else
	return Point(axes[0].position,axes[1].position,axes[2].position);
#endif
}

void setHoldZ(bool holdZ_in) {
	holdZ = holdZ_in;
}

inline void prepareFeedrateIntervals() {
	if (current_feedrate_index > 2)
		return;
	feedrate_steps_remaining  = feedrate_elements[current_feedrate_index].steps;
	feedrate_changerate       = feedrate_elements[current_feedrate_index].rate;
	feedrate_target           = feedrate_elements[current_feedrate_index].target;
	// feedrate_dirty = 1;
	// acceleration_tick_counter = 0;
}

inline void recalcFeedrate() {
	if (feedrate == 0)
		return; // SHRIEK!
	feedrate_inverted = 1000000/feedrate;

	feedrate_dirty = 0;
}

uint32_t getCurrentStep() {
	return intervals - intervals_remaining;
}

// WARNING: Freezes the current feedrate!
uint32_t getCurrentFeedrate() {
	feedrate_changerate = 0;
	return feedrate;
}

// load up the next movment
// WARNING: called from inside the ISR, so get out fast
bool getNextMove() {
	// stepperTimingDebugPin.setValue(true);
	is_running = false; // this ensures that the interrupt does not .. interrupt us

	if (current_block != NULL) {
		current_block->flags &= ~planner::Block::Busy;
		planner::doneWithNextBlock();
		current_block = NULL;
	}
	
	if (planner::isBufferEmpty()) {
		// stepperTimingDebugPin.setValue(true);
		// stepperTimingDebugPin.setValue(false);
		return false;
	}
	
	current_block = planner::getNextBlock();
	// Mark block as busy (being executed by the stepper interrupt)
	current_block->flags |= planner::Block::Busy;
	
	Point &target = current_block->target;
	
	feedrate_multiplier = 1; // setTarget sets the multiplier to one
	int32_t max_delta = current_block->step_event_count;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(target[i], false);
		const int32_t delta = axes[i].delta;

		// Only shut z axis on inactivity
		if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
		else if (delta != 0) axes[i].enableStepper(true);
		
		// if (delta > max_delta) {
		// 	max_delta = delta;
		// }
	}
		
	current_feedrate_index = 0;
	int feedrate_being_setup = 0;
	// setup acceleration
	feedrate = 0;
	if (current_block->accelerate_until > 0) {
		feedrate = current_block->initial_rate;

		feedrate_elements[feedrate_being_setup].steps     = current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}

	// setup plateau
	if (current_block->decelerate_after > current_block->accelerate_until) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;
		
		feedrate_elements[feedrate_being_setup].steps     = current_block->decelerate_after - current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = 0;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}
	

	// setup deceleration
	if (current_block->decelerate_after < current_block->step_event_count) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;

		// To prevent "falling off the end" we will say we have a "bazillion" steps left...
		feedrate_elements[feedrate_being_setup].steps     = INT32_MAX; //current_block->step_event_count - current_block->decelerate_after;
		feedrate_elements[feedrate_being_setup].rate      = -current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->final_rate;
	} else {
		// and in case there wasn't a deceleration phase, we'll do the same for whichever phase was last...
		feedrate_elements[feedrate_being_setup-1].steps     = INT32_MAX;
		// We don't setup anything else because we limit to the target speed anyway.
	}
	
	if (feedrate == 0) {
		is_running = false;
		return false;
	}
	
	prepareFeedrateIntervals();
	recalcFeedrate();
	acceleration_tick_counter = TICKS_PER_ACCELERATION;
	
	timer_counter = feedrate_inverted;

	intervals = max_delta;
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -(intervals>>1);
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].counter = negative_half_interval;
	}
	is_running = true;
	
	// stepperTimingDebugPin.setValue(false);
	return true;
}

void currentBlockChanged() {
	// stepperTimingDebugPin.setValue(true);
	// If we are here, then we are moving AND the interrupts are frozen, so get out *fast*

	current_feedrate_index = 0;
	int feedrate_being_setup = 0;
	// setup acceleration
	feedrate = 0;
	if (current_block->accelerate_until > 0) {
		feedrate = current_block->initial_rate;

		feedrate_elements[feedrate_being_setup].steps     = current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}

	// setup plateau
	if (current_block->decelerate_after > current_block->accelerate_until) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;
		
		feedrate_elements[feedrate_being_setup].steps     = current_block->decelerate_after - current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = 0;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}
	

	// setup deceleration
	if (current_block->decelerate_after < current_block->step_event_count) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;

		// To prevent "falling off the end" we will say we have a "bazillion" steps left...
		feedrate_elements[feedrate_being_setup].steps     = INT32_MAX; //current_block->step_event_count - current_block->decelerate_after;
		feedrate_elements[feedrate_being_setup].rate      = -current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->final_rate;
	} else {
		// and in case there wasn't a deceleration phase, we'll do the same for whichever phase was last...
		feedrate_elements[feedrate_being_setup-1].steps     = INT32_MAX;
		// We don't setup anything else because we limit to the target speed anyway.
	}

	prepareFeedrateIntervals();
	recalcFeedrate();
	
	timer_counter = feedrate_inverted;
	
	// the steppers themselves haven't changed...
	
	// stepperTimingDebugPin.setValue(false);
}

/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step) {
	intervals_remaining = INT32_MAX;
	intervals = us_per_step / INTERVAL_IN_MICROSECONDS;
	const int32_t negative_half_interval = -(intervals>>1);
	for (int i = 0; i < STEPPER_COUNT; i++) {
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
                // stepperTimingDebugPin.setValue(true);
		timer_counter -= INTERVAL_IN_MICROSECONDS;

		if (timer_counter <= 0) {
			if ((intervals_remaining -= feedrate_multiplier) <= 0) {
				getNextMove();
                                // stepperTimingDebugPin.setValue(false);
				return is_running;
				// is_running = false;
			} else {
				// if we are supposed to step too fast, we simulate double-size microsteps
				feedrate_multiplier = 1;
				while (timer_counter <= -feedrate_inverted) {
					feedrate_multiplier++;
					timer_counter += feedrate_inverted;
				}
	
				for (int i = 0; i < STEPPER_COUNT; i++) {
					axes[i].setStepMultiplier(feedrate_multiplier);
					axes[i].doInterrupt(intervals);
				}
				
				if ((feedrate_steps_remaining-=feedrate_multiplier) <= 0) {
					current_feedrate_index++;
					// stepperTimingDebugPin.setValue(true);
					// stepperTimingDebugPin.setValue(false);
					prepareFeedrateIntervals();
				}
				
				if (feedrate_dirty) {
					recalcFeedrate();
				}
				
				timer_counter += feedrate_inverted;
			}
		}
		
		if (feedrate_changerate != 0 && acceleration_tick_counter-- <= 0) {
			acceleration_tick_counter = TICKS_PER_ACCELERATION;
			// Change our feedrate. Here it's important to note that we can over/undershoot

			feedrate += feedrate_changerate;
			feedrate_dirty = 1;
		
			if ((feedrate_changerate > 0 && feedrate > feedrate_target)
			    || (feedrate_changerate < 0 && feedrate < feedrate_target)) {
				feedrate_changerate = 0;
				feedrate = feedrate_target;
			} 

		}
		
                // stepperTimingDebugPin.setValue(false);
		return is_running;
	} else if (is_homing) {
		is_homing = false;
		for (int i = 0; i < STEPPER_COUNT; i++) {
			bool still_homing = axes[i].doHoming(intervals);
			is_homing = still_homing || is_homing;
		}
		// if we're done, force a sync with the planner
		if (!is_homing)
			planner::abort();
		return is_homing;
	}
	return false;
}

} // namespace steppers
