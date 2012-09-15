/*
  StepperAccelPlanner.hh - buffers movement commands and manages the acceleration profile plan
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

// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef STEPPERACCELPLANNER_HH
#define STEPPERACCELPLANNER_HH

#include <stdio.h>
#include "avrfix.h"
#include "Configuration.hh"

#ifndef NOFIXED
#define FIXED
#else
#ifdef FIXED
#undef FIXED
#endif
#endif

#ifdef FIXED
	#define FPTYPE			_Accum

	#define FPTOI(x)		ktoli(x)	//FPTYPE  -> int32_t
	#define FPTOI16(x)		ktoi(x)		//FPTYPE  -> int16_t
	#define ITOFP(x)		itok(x)		//int32_t -> FPTYPE
	#define FTOFP(x)		ftok(x)		//float   -> FPTYPE
	#define FPTOF(x)		ktof(x)		//FPTYPE  -> float

	#define FPSQUARE(x)		mulk(x,x)
	#define FPMULT2(x,y)		mulk(x,y)
	#define FPMULT3(x,y,a)		mulk(mulk(x,y),a)
	#define FPMULT4(x,y,a,b)	mulk(mulk(mulk(x,y),a),b)
	#define FPDIV(x,y)		divk(x,y)
	#define FPSQRT(x)		sqrtk(x)
	#define FPABS(x)		absk(x)
	#define FPCEIL(x)		roundk(x + KCONSTANT_0_5, 3)
	#define FPFLOOR(x)		roundk(x - KCONSTANT_0_5, 3)
		
	//Various constants we need, we preconvert these to fixed point to save time later
	#define KCONSTANT_MINUS_0_95	-62259		//ftok(-0.95)   
        #define KCONSTANT_0_001         65              //ftok(0.001)
	#define KCONSTANT_0_1		6553		//ftok(0.1)
	#define KCONSTANT_0_25		16384		//ftok(0.25)
	#define KCONSTANT_0_5		32768		//ftok(0.5)
	#define KCONSTANT_0_95		62259		//ftok(0.95)
	#define KCONSTANT_1		65536		//ftok(1.0)
	#define KCONSTANT_3             196608          //ftok(3.0)
	#define KCONSTANT_8_388608	549755		//ftok(8.388608)
	#define KCONSTANT_10		655360		//ftok(10.0)
        #define KCONSTANT_30            1966080         //ftok(30.0)
	#define KCONSTANT_100          	6553600		//ftok(100.0)
	#define KCONSTANT_256		16777216	//ftok(256.0)
	#define KCONSTANT_1000		65536000	//ftok(1000.0)
        #define KCONSTANT_1000000_LSR_16 1000000        //ftok(1000000.0) >> 16

#else
	#define FPTYPE			float

	#define FPTOI(x)		(int32_t)(x)	//FPTYPE  -> int32_t
	#define FPTOI16(x)		(int16_t)(x)	//FPTYPE  -> int16_t
	#define ITOFP(x)		(float)(x)	//int32_t -> FPTYPE
	#define FTOFP(x)		(x)		//Do nothing cos we're already float
	#define FPTOF(x)		(x)		//Do nothing cos we're already float

	#define FPSQUARE(x)		((x) * (x))
	#define FPMULT2(x,y)		((x) * (y))
	#define FPMULT3(x,y,a)		((x) * (y) * (a))
	#define FPMULT4(x,y,a,b)	((x) * (y) * (a) * (b))
	#define FPDIV(x,y)		((x) / (y))
	#define FPSQRT(x)		sqrt(x)
	#define FPABS(x)		abs(x)
	#define FPCEIL(x)		ceil(x)
	#define FPFLOOR(x)		floor(x)

#endif

//If defined, support for recording the current move within the block is compiled in
//#define DEBUG_BLOCK_BY_MOVE_INDEX

//When commented out, HOST_CMD_SET_POSITION and HOST_CMD_SET_POSITION_EXT are
//handled asynchronously.
//When not commented out, when these commands are encountered, the buffer is completely drained before executing
//them.
//#define CMD_SET_POSITION_CAUSES_DRAIN

//Oversample the dda to provide less jitter.
//To switch off oversampling, comment out
//2 is the number of bits, as in a bit shift.  So << 2 = multiply by 4
//= 4 times oversampling
//Obviously because of this oversampling is always a power of 2.
//Don't make it too large, as it will kill performance and can overflow int32_t
//#define OVERSAMPLED_DDA 2

//Keep the dda "phase" between line segments
//If false, each new line segment is started as if it was a new line segment, i.e. no prior history
//If true, each new line segment takes into account the phase of the last segment
#define DDA_KEEP_PHASE	false

#define JKN_ADVANCE

//Disabled for now, probably does work but needs a non-symmetrical prime/deprime
//If defined, enables advance_lead_prime / advance_lead_deprime
//#define JKN_ADVANCE_LEAD_DE_PRIME

//Use acceleration instead of delta velocity for advance_lead_entry / advance_lead_exit
//Doesn't work with JKN_ADVANCE_LEAD_DE_PRIME at this point
//#define JKN_ADVANCE_LEAD_ACCEL

//Drop ceil/floor calculations.  Making this available as a #define so we can test timing later
#define NO_CEIL_FLOOR

// If defined then uses final_speed() and initial_speed() instead of max_allowable_speed()
#define FIXSIGN

#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ringbuffering.
#ifdef SMALL_4K_RAM
	#define BLOCK_BUFFER_SIZE 8 // maximize block buffer
#else
	#define BLOCK_BUFFER_SIZE 16 // maximize block buffer
#endif

// When SAVE_SPACE is defined, the code doesn't take some optimizations which
// which lead to additional program space usage.
//#define SAVE_SPACE

// Use yet another jerk calc
#define YET_ANOTHER_JERK

#define  FORCE_INLINE __attribute__((always_inline)) inline



// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  int32_t steps[NUM_AXIS];		// Step count along each axis
  uint32_t step_event_count;           // The number of step events required to complete this block
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
  int32_t starting_position[NUM_AXIS];
#endif
  int32_t accelerate_until;                    // The index of the step event on which to stop acceleration
  int32_t decelerate_after;                    // The index of the step event on which to start decelerating
  int32_t acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  #ifdef JKN_ADVANCE
      bool    use_advance_lead;
      int32_t advance_lead_entry;
      int32_t advance_lead_exit;
      int32_t advance_pressure_relax;	//Decel phase only
      int32_t advance_lead_prime;
      int32_t advance_lead_deprime;
  #endif

  // Fields used by the motion planner to manage acceleration
  // FPTYPE speed_x, speed_y, speed_z, speed_e;       // Nominal mm/minute for each axis
  FPTYPE nominal_speed;                               // The nominal speed for this block in mm/min  
  FPTYPE entry_speed;                                 // Entry speed at previous-current junction in mm/min
  FPTYPE max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
  FPTYPE millimeters;                                 // The total travel of this block in mm
  FPTYPE acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  uint32_t nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  uint32_t initial_rate;                        // The jerk-adjusted step rate at start of block  
  uint32_t final_rate;                          // The minimal rate at exit
  uint32_t acceleration_st;                     // acceleration steps/sec^2
  char     use_accel;                           // Use acceleration when true
  char     speed_changed;                       // Entry speed has changed
  volatile char busy;

#ifdef SIMULATOR
  FPTYPE feed_rate;     // Original feed rate before being modified for nomimal_speed
  int    planned;       // Count of the number of times the block was passed to caclulate_trapezoid_for_block()
  char   message[1024];
#endif

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
  uint32_t move_index;
#endif

	uint8_t dda_master_axis_index;
} block_t;

// Initialize the motion plan subsystem      
void plan_init(FPTYPE extruderAdvanceK, FPTYPE extruderAdvanceK2, bool zhold);

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// steps. Feed rate specifies the speed of the motion.
bool plan_buffer_line(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e, FPTYPE feed_rate, const uint8_t &extruder, bool use_accel=true);

// Set position. Used for G92 instructions.
void plan_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e);
void plan_set_e_position(const int32_t &e);

uint8_t movesplanned(); //return the nr of buffered moves

extern uint32_t minsegmenttime;
extern FPTYPE max_feedrate[NUM_AXIS]; // set the max speeds
extern uint32_t max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
extern FPTYPE minimumfeedrate;
extern FPTYPE p_acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern FPTYPE p_retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern FPTYPE max_speed_change[NUM_AXIS]; //The speed between junctions in the planner, reduces blobbing
extern FPTYPE smallest_max_speed_change;

extern FPTYPE mintravelfeedrate;
extern FPTYPE minimumSegmentTime;
extern uint32_t axis_steps_per_sqr_second[NUM_AXIS];
extern bool acceleration_zhold;
extern FPTYPE delta_mm[NUM_AXIS];
extern FPTYPE planner_distance;
extern FPTYPE minimumPlannerSpeed;
extern uint32_t planner_master_steps;
extern int32_t planner_steps[NUM_AXIS];
extern FPTYPE extruder_only_max_feedrate;
extern int slowdown_limit;

#ifndef CMD_SET_POSITION_CAUSES_DRAIN
        extern int32_t position[NUM_AXIS];
#endif

extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail; 
extern float axis_steps_per_unit[NUM_AXIS];
// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.    
FORCE_INLINE void plan_discard_current_block()  
{
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1);  
  }
}

// Gets the current block. Returns NULL if buffer empty
FORCE_INLINE block_t *plan_get_current_block() 
{
  if (block_buffer_head == block_buffer_tail) { 
    return(NULL); 
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = true;
  return(block);
}

// Gets the current block. Returns NULL if buffer empty
FORCE_INLINE bool blocks_queued() 
{
  if (block_buffer_head == block_buffer_tail) { 
    return false; 
  }
  else
    return true;
}

extern void accelStatsGet(float *minSpeed, float *avgSpeed, float *maxSpeed);

#ifndef SIMULATOR
#define SIMULATOR_RECORD(x...)
#else
#include "SimulatorRecord.hh"
#endif

#endif
