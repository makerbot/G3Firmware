/*
  StepperAccel.hh - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPERACCEL_HH
#define STEPPERACCEL_HH 

#include <inttypes.h>
#include "StepperAccelPlanner.hh"

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 2.0 // (mm/sec)

const int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement

#define EXTRUDERS 1

// Initialize and start the stepper motor subsystem
void st_init();

// Returns true is there are no buffered steps to be executed
bool st_empty();

// Set current position in steps
void st_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e);
void st_set_e_position(const int32_t &e);

// Get current position in steps
int32_t st_get_position(uint8_t axis);

// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();

void st_interrupt();

void st_advance_interrupt();
  
extern block_t *current_block;  // A pointer to the block currently being traced

void quickStop();

//DEBUGGING
extern float zadvance;
#endif
