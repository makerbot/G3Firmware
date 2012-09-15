/*
  StepperAccelPlanner.cc - buffers movement commands and manages the acceleration profile plan
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

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*  
  Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
  
  s == speed, a == acceleration, t == time, d == distance

  Basic definitions:

    Speed[s_, a_, t_] := s + (a*t) 
    Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]

  Distance to reach a specific speed with a constant acceleration:

    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
      d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()

  Speed after a given distance of travel with constant acceleration:

    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
      m -> Sqrt[2 a d + s^2]    

    DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]

  When to start braking (di) to reach a specified destionation speed (s2) after accelerating
  from initial speed s1 without ever stopping at a plateau:

    Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
      di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()

    IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
*/

#ifdef SIMULATOR
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Simulator.hh"
#endif
                                                                                                      
#include "Configuration.hh"

#ifdef HAS_STEPPER_ACCELERATION


#include "StepperAccelPlanner.hh"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#ifndef SIMULATOR
#include  <avr/interrupt.h>
#endif
#include "StepperAccel.hh"
#ifndef SIMULATOR
#include "StepperInterface.hh"
#include "Motherboard.hh"
#endif

#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))

#define VEPSILON 1.0e-5

// v1 != v2
#ifdef FIXED
	#define VNEQ(v1,v2) ((v1) != (v2))
	#define VLT(v1,v2)  ((v1) < (v2))
#else
	#define VNEQ(v1,v2) (abs((v1)-(v2)) > VEPSILON)
	#define VLT(v1,v2)  (((v1) + VEPSILON) < (v2))
#endif

//===========================================================================
//=============================public variables ============================
//===========================================================================

uint32_t minsegmenttime;
FPTYPE max_feedrate[NUM_AXIS]; // set the max speeds
uint32_t max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
FPTYPE minimumfeedrate;
FPTYPE p_acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
FPTYPE p_retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
FPTYPE smallest_max_speed_change;
FPTYPE max_speed_change[NUM_AXIS]; //The speed between junctions in the planner, reduces blobbing
FPTYPE mintravelfeedrate;
FPTYPE minimumPlannerSpeed;
FPTYPE extruder_only_max_feedrate;
int slowdown_limit;
float axis_steps_per_unit[NUM_AXIS];

bool disable_slowdown = true;
uint32_t axis_steps_per_sqr_second[NUM_AXIS];
#ifdef JKN_ADVANCE
FPTYPE extruder_advance_k = 0, extruder_advance_k2 = 0;
#endif
FPTYPE delta_mm[NUM_AXIS];
FPTYPE planner_distance;
uint32_t planner_master_steps;
int32_t planner_steps[NUM_AXIS];
FPTYPE vmax_junction;
FPTYPE cosine;
#ifndef FIXED
#ifndef YET_ANOTHER_JERK
	FPTYPE compareLessThanEqualZero = 0.001;
#endif
#endif

// minimum time in seconds that a movement needs to take if the buffer is emptied.
// Increase this number if you see blobs while printing high speed & high detail.
// It will slowdown on the detailed stuff.
// Comment out to disable
FPTYPE minimumSegmentTime;

// The current position of the tool in absolute steps
int32_t position[NUM_AXIS];   //rescaled from extern when axis_steps_per_unit are changed by gcode

#ifndef YET_ANOTHER_JERK
static FPTYPE previous_delta_mm[NUM_AXIS];
#else
static FPTYPE prev_speed[NUM_AXIS];
#endif
static FPTYPE previous_inverse_millimeters;

#ifndef SIMULATOR
static StepperInterface *stepperInterface;
#else
static block_t *sblock = NULL;
static char prior_dropped[1024] = "\0";
#endif
bool acceleration_zhold = false;

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
  uint32_t current_move_index = 0;
#endif

#ifdef FIXED
#ifndef SIMULATOR

//http://members.chello.nl/j.beentjes3/Ruud/sqrt32avr.htm

static int16_t isqrt1(int16_t value)
{
    int16_t result;

    asm volatile (
";  Fast and short 16 bits AVR sqrt routine" "\n\t"
";" "\n\t"
";  R17:R16 = sqrt(R3:R2), rounded down to integer" "\n\t"
";" "\n\t"
";  Registers:" "\n\t"
";  Destroys the argument in R3:R2" "\n\t"
";" "\n\t"
";  Cycles incl call & ret = 90 - 96" "\n\t"
";" "\n\t"
";  Stack incl call = 2" "\n\t"
"Sqrt16:     ldi   %B0,0xc0          ; Rotation mask register" "\n\t"
"            ldi   %A0,0x40          ; Developing sqrt" "\n\t"
"            clc                     ; Enter loop with C=0" "\n\t"
"_sq16_1:    brcs  _sq16_2           ; C --> Bit is always 1" "\n\t"
"            cp    %B1,%A0            ; Does value fit?" "\n\t"
"            brcs  _sq16_3           ; C --> bit is 0" "\n\t"
"_sq16_2:    sub   %B1,%A0            ; Adjust argument for next bit" "\n\t"
"            or    %A0,%B0           ; Set bit to 1" "\n\t"
"_sq16_3:    lsr   %B0               ; Shift right rotation mask" "\n\t"
"            lsl   %A1" "\n\t"
"            rol   %B1                ; Shift left argument, C --> Next sub is MUST" "\n\t"
"            eor   %A0,%B0           ; Shift right test bit in developing sqrt" "\n\t"
"            andi  %B0,0xfe          ; Becomes 0 for last bit" "\n\t"
"            brne  _sq16_1           ; Develop 7 bits" "\n\t"
"            brcs  _sq16_4           ; C--> Last bit always 1" "\n\t"
"            lsl   %A1                ; Need bit 7 in C for cpc" "\n\t"
"            cpc   %A0,%B1            ; After this C is last bit" "\n\t"
"_sq16_4:    adc   %A0,%B0           ; Set last bit if C (R17=0)" "\n\t"

        : "=&r" (result)
        : "r" (value)
        );
    return result;
}

#endif
#endif

#if 0
//http://avr.15.forumer.com/a/fast-squareroot-routine-avr-asm_post264.html

static uint8_t isqrt2(uint16_t value)
{
    uint8_t result;

    asm volatile (
"; RETURN:" "\n\t"
"; return value: RetVal ( = sqrt(InputH:InputL))" "\n\t"
"; remainder:	SREG(t):InputH ( = InputH:InputL - (sqrt(InputH:InputL))^2)" "\n\t"
";               (SREG(t) - MSB (8th) of remainder)" "\n\t"

"Sqrt16:" "\n\t"
"clr	%0" "\n\t"
"clr	__tmp_reg__" "\n\t"
"ldi	r19,(1 << 7)" "\n\t"
"clt" "\n\t"

"Sqrt16Loop:" "\n\t"
"mov	r20,r19" "\n\t"
"lsr	r20" "\n\t"
"ror	__tmp_reg__" "\n\t"
"or	r20,%0" "\n\t"

"brts	Sqrt16Sub" "\n\t"

"cp	%A1,__tmp_reg__" "\n\t"
"cpc	%B1,r20" "\n\t"
"brcs	Sqrt16NoSub" "\n\t"

"Sqrt16Sub:" "\n\t"
"sub	%A1,__tmp_reg__" "\n\t"
"sbc	%B1,r20" "\n\t"
"or	%0,r19" "\n\t"

"Sqrt16NoSub:" "\n\t"
"bst	%B1,7" "\n\t"
"lsl	%A1" "\n\t"
"rol	%B1" "\n\t"

"lsr	r19" "\n\t"
"brcc	Sqrt16Loop" "\n\t"
        : "=&r" (result)
        : "r" (value)
	: "r20", "r19"
        );
    return result;
}

#endif

//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head = 0;       // Index of the next block to be pushed
volatile unsigned char block_buffer_tail = 0;       // Index of the block to process now


// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
#ifndef SAVE_SPACE
FORCE_INLINE
#endif
static uint8_t next_block_index(uint8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
#ifndef SAVE_SPACE
FORCE_INLINE
#endif
static uint8_t prev_block_index(uint8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

//===========================================================================
//=============================functions         ============================
//===========================================================================

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:

// Note the equation used below is EXACT: there's no "estimation" involved.
// As such, the name is a bit misleading.

//   t = time
//   a = acceleration (constant)
//   d(t) = distance travelled at time t
//   initial_rate = rate of travel at time t = 0
//   rate(t) = rate of travel at time t
//
// From basic kinematics, we have
//
// [1]   d(t) = d(0) + initial_rate * t + 0.5 * a * t^2,
//
// and
//
// [2]   rate(t) = initial_rate + a * t.
//
// For our purposes, d(0)
//
// [3]   d(0) = 0.
//
// Solving [2] for time t, gives us
//
// [4]   t = ( rate(t) - initial_rate ) / a.
//
// Substituting [3] and [4] into [1] produces,
//
// [5]   d(t) = initial_rate * ( rate(t) - intial_rate ) / a + ( rate(t) - initial_rate )^2 / 2a.
//
// With some algebraic simplification, we then find that d(t) is given by
//
// [6]   d(t) = ( rate(t)^2 - initial_rate^2 ) / 2a.
//
// So, if we know our desired initial rate, initial_rate, and our acceleration, the distance d
// required to reach a target rate, target_rate, is then
//
// [7]  d = ( target_rate^2 - initial_rate^2 ) / 2a.
//
// Note that if the acceleration is 0, we can never reach the target rate unless the
// initial and target rates are the same.  This stands to reason since if the acceleration
// is zero, then our speed never changes and thus no matter how far we move, we're still
// moving at our initial speed.

FORCE_INLINE int32_t estimate_acceleration_distance(int32_t initial_rate_sq, int32_t target_rate_sq, int32_t acceleration_doubled)
{
  if (acceleration_doubled!=0) {
  return((target_rate_sq-initial_rate_sq)/acceleration_doubled);
  }
  else {
    return 0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

//
//      accelerate +a         decelerate -a
//     |<---- d1 ---->|<---------- d2 ---------->|
//     |<------------- d = d1 + d2 ------------->|
//    t=0            t=t1                       t=t1+t2
//   initial_rate   peak_rate                 final_rate
//
// From basic kinematics,
//
// [1]  d1 = initial_rate t1 + 0.5 a t1^2
// [2]  d2 = final_rate t2 + 0.5 a t2^2  [think of starting at speed final_rate and accelerating by a]
// [3]  final_rate = intial_rate + a (t1 - t2)
// [4]  d2 = d - d1
//
// We wish to solve for d1 given a, d, initial_rate, and final_rate.
// By the quadratic equation,
//
// [5]  t1 = [ -initial_rate +/- sqrt( initial_rate^2 + 2 a d1 ) ] / a
// [6]  t2 = [ -final_rate +/- sqrt( final_rate^2 + 2 a d2 ) ] / a
//
// Replacing t1 and t2 in [6] then produces,
//
// [7]  final_rate = initial_rate - initial_rate +/- sqrt( initial_rate^2 + 2 a d1 ) +
//                    + final_rate -/+ sqrt( final_rate^2 + 2 a d2 )
//
// And thus,
//
// [8]  +/- sqrt( initial_rate^2 + 2 a d1 ) = +/- sqrt( final_rate^2 + 2 a d2 )
//
// Squaring both sides and substituting d2 = d - d1 [4] yields,
//
// [9]  initial_rate^2 + 2 a d1 = final_rate^2 + 2 a d - 2 a d1
//
// Solving [9] for d1 then gives our desired result,
//
// [10]  d1 =  ( final_rate^2 - initial_rate^2 + 2 a d ) / 4a

FORCE_INLINE int32_t intersection_distance(int32_t initial_rate_sq, int32_t final_rate_sq, int32_t acceleration_doubled, int32_t distance) 
{
 if (acceleration_doubled!=0) {
  return((acceleration_doubled*distance-initial_rate_sq+final_rate_sq)/(acceleration_doubled << 1) );
  }
  else {
    return 0;  // acceleration was 0, set intersection distance to 0
  }
}


#ifdef JKN_ADVANCE

//Same as final_speed, except this one works with step_rates.
//Regular final_speed will overflow if we use step_rates instead of mm/s

FORCE_INLINE FPTYPE final_speed_step_rate(uint32_t acceleration, uint32_t initial_velocity, int32_t distance) {
	uint32_t v2 = initial_velocity * initial_velocity;
#ifdef SIMULATOR
	uint64_t sum2 = (uint64_t)initial_velocity * (uint64_t)initial_velocity +
	     2 * (uint64_t)acceleration * (uint64_t)distance;
	float fres = (sum2 > 0) ? sqrt((float)sum2) : 0.0;
	FPTYPE result;
#endif
	//Although it's highly unlikely, if target_rate < initial_rate, then distance could be negative.
	if ( distance < 0 ) {
		uint32_t term2 = (acceleration * (uint32_t)abs(distance)) << 1;

		if ( term2 >= v2 )	return 0;
		v2 -= term2;
	}
	else	v2 += (acceleration * (uint32_t)distance) << 1;
	if (v2 <= 0x7fff)
#ifndef SIMULATOR
	     return ITOFP(isqrt1((uint16_t)v2));
#else
#define isqrt1(x) ((int32_t)sqrt((float)(x)))
	result = ITOFP(isqrt1((uint16_t)v2));
#endif
	else
	{
	     uint8_t n = 0;
	     while (v2 > 0x7fff)
	     {
		  v2 >>= 2;
		  n++;
	     }
#ifndef SIMULATOR
	     return ITOFP(isqrt1((int16_t)v2)) << n;
#else
	     result = ITOFP(isqrt1((int16_t)v2)) << n;
#endif
	}
#ifdef SIMULATOR
	if ((fres != 0.0) && ((fabsf(fres - FPTOF(result))/fres) > 0.01))
	{
	     char buf[1024];
	     snprintf(buf, sizeof(buf),
		      "*** final_speed_step_rate(%d, %d, %d): fixed result = %f; "
		      "float result = %f", acceleration, initial_velocity, distance,
		      FPTOF(result), fres);
	     if (sblock)
	     {
		  if (sblock->message[0] != '\0')
		       strncat(sblock->message, "\n", sizeof(sblock->message));
		  strncat(sblock->message, buf, sizeof(sblock->message));
	     }
	     else
		  printf("%s\n", buf);
	}
	return result;
#endif
}

#endif

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, FPTYPE entry_factor, FPTYPE exit_factor) {

     // If exit_factor or entry_factor are larger than unity, then we will scale
     // initial_rate or final_rate to exceed nominal_rate.  However, maximum feed rates
     // have been applied to nominal_rate and as such we should not exceed nominal_rate.

     // For example, if the next block's entry_speed exceeds this block's nominal_speed,
     // then we can be called with exit_factor = next->entry_speed / current->nominal_speed > 1.
     // Basically, that's saying that the next block's nominal speed which was subject to
     // per axis feed rates for a different combination of axes steps can override this block's
     // speed limits.  We don't want that.  For example, if this block's motion is primarily
     // Z or E axis steps and the next block's is all X axis steps, we don't want the next
     // block's X-axis limited entry_speed telling this block that it's Z or E axis limited
     // final_rate can be increased past the block's nominal_rate.

     if ( (!block->use_accel) || (entry_factor > KCONSTANT_1) ) entry_factor = KCONSTANT_1;
     if ( (!block->use_accel) || (exit_factor > KCONSTANT_1) ) exit_factor = KCONSTANT_1;

#ifdef FIXED
	#ifdef NO_CEIL_FLOOR
	  uint32_t initial_rate = (uint32_t)FPTOI(KCONSTANT_0_5 + FPMULT2(ITOFP(block->nominal_rate), entry_factor)); // (step/min)
	  uint32_t final_rate = (uint32_t)FPTOI(KCONSTANT_0_5 + FPMULT2(ITOFP(block->nominal_rate), exit_factor)); // (step/min)
	#else
	  uint32_t initial_rate = (uint32_t)FPTOI(FPCEIL(FPMULT2(ITOFP(block->nominal_rate), entry_factor))); // (step/min)
	  uint32_t final_rate = (uint32_t)FPTOI(FPCEIL(FPMULT2(ITOFP(block->nominal_rate), exit_factor))); // (step/min)
	#endif
#else
	#ifdef NO_CEIL_FLOOR
	  uint32_t initial_rate = (uint32_t)(0.5 + block->nominal_rate*entry_factor); // (step/min)
	  uint32_t final_rate = (uint32_t)(0.5 + block->nominal_rate*exit_factor); // (step/min)
	#else
	  uint32_t initial_rate = FPCEIL(block->nominal_rate*entry_factor); // (step/min)
	  uint32_t final_rate = FPCEIL(block->nominal_rate*exit_factor); // (step/min)
	#endif
#endif

  // If we really need to squeeze cyles, then we can instead do the following
  // but then we'd be testing for rates < 128
  // if (0 == (initial_rate & 0xffffff80)) initial_rate=127;  // initial_rate < 127
  // if (0 == (final_rate & 0xffffff80)) final_rate=127;    // final_rate < 127

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate <120) {initial_rate=120; }
  if(final_rate < 120) {final_rate=120;  }

  int32_t initial_rate_sq = (int32_t)(initial_rate * initial_rate);
  int32_t nominal_rate_sq = (int32_t)(block->nominal_rate * block->nominal_rate);
  int32_t final_rate_sq   = (int32_t)(final_rate   * final_rate);
  
  int32_t acceleration = block->acceleration_st;
  int32_t acceleration_doubled = acceleration << 1;
  int32_t accelerate_steps = 0;
  int32_t decelerate_steps = 0;
  if ( block->use_accel )
  {
       accelerate_steps = estimate_acceleration_distance(initial_rate_sq, nominal_rate_sq, acceleration_doubled);
       decelerate_steps = estimate_acceleration_distance(nominal_rate_sq, final_rate_sq, -acceleration_doubled);
  }

  // accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
  // accelerate_steps = min(accelerate_steps,(int32_t)block->step_event_count);

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = intersection_distance(initial_rate_sq, final_rate_sq, acceleration_doubled, (int32_t)block->step_event_count);
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min(accelerate_steps,(int32_t)block->step_event_count);
    plateau_steps = 0;
  }
  int32_t decelerate_after = accelerate_steps + plateau_steps;

  #ifdef JKN_ADVANCE
  #ifdef SIMULATOR
      sblock = block;
  #endif
      int32_t advance_lead_entry = 0, advance_lead_exit = 0, advance_lead_prime = 0, advance_lead_deprime = 0;
      int32_t advance_pressure_relax = 0;

      if ( block->use_advance_lead ) {
	uint32_t maximum_rate;

	// Note that we accelerate between step 0 & 1, between 1 & 2, ..., between
	// acclerate_steps-1 & accelerate_steps, AND accelerate_steps & accelerate_steps+1
	// So, we accelerate for accelerate_steps + 1.  This is because in st_interrupt()
	// the test is "if (step_events_completed <= accelerate_until)" which means that
	// between step accelerate_until and the next step, we're still doing acceleration.

	if ( plateau_steps == 0 ) maximum_rate = FPTOI(final_speed_step_rate(block->acceleration_st, initial_rate,
									     accelerate_steps + 1));
	else maximum_rate = block->nominal_rate;

	// Don't waste cycles computing these values if we won't use them in st_interrupt()
	if (accelerate_steps > 0) {
#ifdef JKN_ADVANCE_LEAD_ACCEL
	     advance_lead_entry = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)block->acceleration_st>>4)));
#else
	     // Acceleration dependent portion

	     // Basis of computation is as follows.  Suppose the filament velocity, Vf, should be
	     //
	     //   [1]  Vf = C1 * Vn + C2 * a
	     //
	     // where
	     //
	     //   Vn = extruded noodle velocity
	     //    a = acceleration
	     //   C1 = constant (ratio of volumes and whatnot)
	     //   C2 = another constant
	     //
	     // But we're normally just taking Vf = C1 * Vn and thus we're missing a contribution
	     // associated with acceleration (e.g., a contribution associated with energy loss to
	     // friction in the extruder nozzle).  We can then ask, well how many e-steps do we
	     // miss as a result?  Using,
	     //
	     //   [2]  distance in e-space = velocity x time
	     //
	     // we would then appear to be missing
	     //
	     //   [3]  delta-e = C2 * a * time-spent-accelerating
	     //
	     // From basic kinematics, we know the time spent accelerating under constant acceleration,
	     //
	     //   [4]  Vfinal = Vinitial + a * time-spent-accelerating
	     //
	     // and thus
	     //
	     //   [5]  time-spent-accelerating = (Vfinal - Vinitial) / a
	     //
	     // Substituting [5] into [3] yields,
	     //
	     //   [6]  delta-e = C2 * (Vfinal - Vinitial)
	     //
	     // where Vinitial and Vfinal are the initial and final speeds
	     // at the start and end of the acceleration or deceleration phase.

	     advance_lead_entry = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)(maximum_rate - initial_rate))));
#endif

#ifdef JKN_ADVANCE_LEAD_DE_PRIME
	     // Prime.  Same as advance_lead_entry, but for when we're priming from 0 to initial_rate
	     // Note that we may or may not use this value, it's only used if the previous segment was not available or had e steps of 0
	     advance_lead_prime = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)initial_rate)) - KCONSTANT_0_5);
#endif

#ifndef SIMULATOR
	     if (advance_lead_entry < 0) advance_lead_entry = 0;
	     if (advance_lead_prime < 0) advance_lead_prime = 0;
#endif
	}
	if ((decelerate_after+1) < (int32_t)block->step_event_count) {
#ifdef JKN_ADVANCE_LEAD_ACCEL
	     advance_lead_exit = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)block->acceleration_st>>4)));
#else
	     // Acceleration dependent portion
	     advance_lead_exit = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)(maximum_rate - final_rate))));
#endif

#ifdef JKN_ADVANCE_LEAD_DE_PRIME
	     // Deprime.  Same as advance_lead_exit, but for when we're depriming from final_rate to 0
	     // Note that we may or may not use this value, it's only used if the next segment is not available or has e steps of 0
	     advance_lead_deprime = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)final_rate)) - KCONSTANT_0_5);
#endif

	     //Pressure relaxation
	     if ( extruder_advance_k2 != 0 ) {
           	advance_pressure_relax = FPTOI(FPDIV(FPMULT3(extruder_advance_k2, ITOFP((int32_t)block->acceleration_st >> 4), KCONSTANT_100), ITOFP((int32_t)decelerate_steps)));
	     }
	
#ifndef SIMULATOR
	     //If we've overflowed, reset to 0
	     if ( advance_pressure_relax < 0 ) advance_pressure_relax = 0;

	     if (advance_lead_exit    < 0) advance_lead_exit = 0;
	     if (advance_lead_deprime < 0) advance_lead_deprime = 0;
#endif
	}
#ifdef SIMULATOR
        if ( (advance_lead_entry < 0) || (advance_lead_exit < 0) || (advance_pressure_relax < 0) ) {
	     char buf[1024];
	     snprintf(buf, sizeof(buf),
		      "*** calculate_trapezoid_for_block(): advance_lead_entry=%d, advance_lead_exit=%d, "
		      "advance_pressure_relax=%d; initial/nominal/maximum/final_rate=%d/%d/%d/%d; "
		      "accelerate_until/decelerate_after/step_events/plateau_steps=%d/%d/%d/%d; "
		      "i/n/f/a=%d/%d/%d/%d\n",
		      advance_lead_entry, advance_lead_exit, advance_pressure_relax,initial_rate, block->nominal_rate,
		      maximum_rate, final_rate, accelerate_steps, decelerate_after, block->step_event_count,
		      plateau_steps, initial_rate_sq, nominal_rate_sq, final_rate_sq, acceleration_doubled);
	     if (block->message[0] != '\0')
		  strncat(block->message, "\n", sizeof(block->message));
	     strncat(block->message, buf, sizeof(block->message));
	}
#endif
      }
  #endif
  
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if(block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = decelerate_after;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  #ifdef JKN_ADVANCE
	block->advance_lead_entry     = advance_lead_entry;
	block->advance_lead_exit      = advance_lead_exit;
	block->advance_lead_prime     = advance_lead_prime;
	block->advance_lead_deprime   = advance_lead_deprime;
	block->advance_pressure_relax = advance_pressure_relax;
  #endif
  }
  CRITICAL_SECTION_END;
#ifdef SIMULATOR
  block->planned += 1;
#endif
}                    

#ifndef FIXSIGN

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
FORCE_INLINE FPTYPE max_allowable_speed(FPTYPE acceleration, FPTYPE target_velocity, FPTYPE distance) {
#ifdef FIXED
  FPTYPE v2 = FPSQUARE(target_velocity) - (FPMULT2(acceleration, distance) << 1);
#else
  FPTYPE v2 = FPSQUARE(target_velocity) - 2.0 * acceleration * distance;
#endif
  if (v2 < 0) return (0);
  else	return FPSQRT(v2);
}

#else

// Calculates the speed you must start at in order to reach target_velocity using the 
// acceleration within the allotted distance.
//
// See final_speed() for a derivation of this code.  For initial_speed(), "-a" should
// be used in place of "a" for the acceleration.  And the target_velocity used in place
// of the initial_velocity.
//
// Note bene: if the distance or acceleration is sufficiently large, then there's
//   no initial speed which will work.  The acceleration is so large that the
//   the target velocity will be attained BEFORE the distance is covered.  As
//   such even an initial speed of zero won't work.  When this happens, the value
//   under the square root is negative.  In that case, we simply return a value
//   of zero.

FORCE_INLINE FPTYPE initial_speed(FPTYPE acceleration, FPTYPE target_velocity, FPTYPE distance) {
#ifdef FIXED
#ifdef SIMULATOR
  FPTYPE acceleration_original = acceleration;
  FPTYPE distance_original = distance;
  FPTYPE target_velocity_original = target_velocity;
  float  ftv = FPTOF(target_velocity);
  float  fac = FPTOF(acceleration);
  float   fd = FPTOF(distance);
  float fres = ftv * ftv - 2.0 * fac * fd;
  if (fres <= 0.0) fres = 0.0;
  else fres = sqrt(fres);
#endif

  // We wish to compute
  //
  //    sum2 = target_velocity * target_velocity - 2 * acceleration * distance
  //
  // without having any overflows.  We therefore divide everything in
  // site by 2^12.  After computing sqrt(sum2), we will then multiply
  // the result by 2^6 (the square root of 2^12).

  target_velocity >>= 12;
  acceleration >>= 6;
  distance >>= 5;  // 2 * (distance >> 6) 
  FPTYPE sum2 = FPSQUARE(target_velocity) - FPMULT2(distance, acceleration);

  // Now, comes the real speed up: use our fast 16 bit integer square
  // root (in assembler nonetheles). To pave the way for this, we shift
  // sum2 as much to the left as possible thereby maximizing the use of
  // the "whole" or "integral" part of the fixed point number.  We then
  // take the square root of the integer part of sum2 which has been
  // multiplied by 2^(2n) [n left shifts by 2].  After taking the square
  // root, we correct this scaling by dividing the result by 2^n (which
  // is the square root of 2^(2n)

  uint8_t n = 0;
  while ((sum2 != 0) && (sum2 & 0xe0000000) == 0) {
       sum2 <<= 2;
       n++;
  }

#ifndef SIMULATOR
  // Generate the final result.  We need to undo two sets of
  // scalings: our original division by 2^12 which we rectify
  // by multiplying by 2^6.  But also we need to divide by 2^n
  // so as to counter the 2^(2n) scaling we did.  This means
  // a net multiply by 2^(6-n).
  if (sum2 <= 0) return 0;
  else if (n > 6) return ITOFP(isqrt1(FPTOI16(sum2))) >> (n - 6);
  else return ITOFP(isqrt1(FPTOI16(sum2))) << (6 - n);
#else
  FPTYPE result;
  if (sum2 <= 0) result = 0;
  else result = ITOFP(isqrt1(FPTOI16(sum2)));
  if (n > 6) result >>= (n - 6);
  else result <<= (6 - n);
  if ((fres != 0.0) && ((fabsf(fres - FPTOF(result))/fres) > 0.05))
  {
       char buf[1024];
       snprintf(buf, sizeof(buf),
		"*** initial_speed(%f, %f, %f): fixed result = %f; "
		"float result = %f",
		FPTOF(acceleration_original),
		FPTOF(target_velocity_original),
		FPTOF(distance_original),
		FPTOF(result), fres);
       if (sblock)
       {
	    if (sblock->message[0] != '\0')
		 strncat(sblock->message, "\n", sizeof(sblock->message));
	    strncat(sblock->message, buf, sizeof(sblock->message));
       }
       else
	    printf("%s\n", buf);
  }
  return result;
#endif // SIMULATOR
#else
  FPTYPE v2 = FPSQUARE(target_velocity) - 2.0 * acceleration * distance;
  if (v2 <= 0) return 0;
  else return FPSQRT(v2);
#endif  // !FIXED
}

// Calculates the final speed (terminal speed) which will be attained if we start at
// speed initial_velocity and then accelerate at the given rate over the given distance
// From basic kinematics, we know that displacement d(t) at time t for an object moving
// initially at speed v0 and subject to constant acceleration a is given by
//
// [1]   d(t) = v0 t + 0.5 a t^2, t >= 0
//
// We also know that the speed v(t) at time t is governed by
//
// [2]   v(t) = v0 + a t
//
// Now, without reference to time, we desire to find the speed v_final we will
// find ourselves moving at after travelling a given distance d.  To find an answer
// to that question, we need to solve one of the two above equations for t and
// then substitute the result into the remaining equation.  As it turns out, the
// algebra is a little easier whne [1] is solved for t using the quadratic
// equation then first solving [2] for t and then plugging into [1] and then
// solving the result for v(t) with the quadratic equation.
//
// So, moving forward and solving [1] for t via the quadratic equation, gives
//
// [3]   t = - [ v0 +/- sqrt( v0^2 - 2ad(t) ) ] / a
//
// Substituting [3] into [2] then 
//
// [4]   v(t) = v0 - [ v0 +/- sqrt( v0^2 + 2ad(t) ) ]
//
// With some minor simplification and droping the (t) notation, we then have
//
// [5]  v = -/+ sqrt( v0^2 + 2ad )
//
// With equation [5], we then know the final speed v attained after accelerating
// from initial speed v0 over distance d.

FORCE_INLINE FPTYPE final_speed(FPTYPE acceleration, FPTYPE initial_velocity, FPTYPE distance) {
#ifdef FIXED
//  static int counts = 0;
#ifdef DEBUG_ZADVANCE
	//  DEBUG_TIMER_START;
#endif
#ifdef SIMULATOR
  FPTYPE acceleration_original = acceleration;
  FPTYPE distance_original = distance;
  FPTYPE initial_velocity_original = initial_velocity;
  float  ftv = FPTOF(initial_velocity);
  float  fac = FPTOF(acceleration);
  float   fd = FPTOF(distance);
  float fres = ftv * ftv + 2.0 * fac * fd;
  if (fres <= 0.0) fres = 0.0;
  else fres = sqrt(fres);
#endif

  // We wish to compute
  //
  //    sum2 = initial_velocity * initial_velocity + 2 * acceleration * distance
  //
  // without having any overflows.  We therefore judiciously divide
  // both summands by 2^12.  After computing sqrt(sum2), we will the
  // multiply the result by 2^6 which is he square root of 2^12.

  // Note that when initial_velocity < 1, we lose velocity resolution.
  // When acceleration or distance are < 1, we lose some resolution
  // in them as well.  We're here taking advantage of the fact that
  // the acceleration is usually pretty large as are the velocities.
  // Only the distances are sometimes small which is why we shift the
  // distance the least.  If this were to become a problem, we could
  // shift the acceleration more and the distance even less.  And,
  // when the velocity is tiny relative to the product 2 * a * d,
  // we really don't care as 2 * a * d will then dominate anyway.
  // That is, losing resolution in the velocity is not, in practice,
  // harmful since 2 * a * d will likely dominate in that case.

  initial_velocity >>= 12;
  acceleration >>= 6;
  distance >>= 5;  // 2 * (distance >> 6) 
  FPTYPE sum2 = FPSQUARE(initial_velocity) + FPMULT2(distance, acceleration);

  // Now, comes the real speed up: use our fast 16 bit integer square
  // root (in assembler nonetheles). To pave the way for this, we shift
  // sum2 as much to the left as possible thereby maximizing the use of
  // the "whole" or "integral" part of the fixed point number.  We then
  // take the square root of the integer part of sum2 which has been
  // multiplied by 2^(2n) [n left shifts by 2].  After taking the square
  // root, we correct this scaling by dividing the result by 2^n (which
  // is the square root of 2^(2n)

  uint8_t n = 0;
  while ((sum2 != 0) && (sum2 & 0xe0000000) == 0) {
       sum2 <<= 2;
       n++;
  }

#ifndef SIMULATOR

  // Generate the final result.  We need to undo two sets of
  // scalings: our original division by 2^12 which we rectify
  // by multiplying by 2^6.  But also we need to divide by 2^n
  // so as to counter the 2^(2n) scaling we did.  This means
  // a net multiply by 2^(6-n).

  if (sum2 <= 0) return 0;
  else if (n > 6) return ITOFP(isqrt1(FPTOI16(sum2))) >> (n - 6);
  else return ITOFP(isqrt1(FPTOI16(sum2))) << (6 - n);

#ifdef DEBUG_ZADVANCE
//  Timing code
//  FPTYPE result;
//  if (sum2 <= 0) result = 0;
//  else if (n > 6) result = ITOFP(isqrt1(FPTOI16(sum2))) >> (n - 6);
//  else result = ITOFP(isqrt1(FPTOI16(sum2))) << (6 - n);
//  DEBUG_TIMER_FINISH;
//  zadvance2 += DEBUG_TIMER_TCTIMER_US;
//  counts += 1;
//  zadvance = zadvance2 / counts;
//  return result;
#endif
#else
#define isqrt1(x) ((int32_t)sqrt((float)(x)))
  FPTYPE result;
  if (sum2 <= 0) result = 0;
  else result = ITOFP(isqrt1(FPTOI16(sum2)));
  if (n > 6) result >>= (n - 6);
  else result <<= (6 - n);
  if ((fres != 0.0) && ((fabsf(fres - FPTOF(result))/fres) > 0.05))
  {
       char buf[1024];
       snprintf(buf, sizeof(buf),
		"*** final_speed(%f, %f, %f): fixed result = %f; "
		"float result = %f",
		FPTOF(acceleration_original),
		FPTOF(initial_velocity_original),
		FPTOF(distance_original),
		FPTOF(result), fres);
       if (sblock)
       {
	    if (sblock->message[0] != '\0')
		 strncat(sblock->message, "\n", sizeof(sblock->message));
	    strncat(sblock->message, buf, sizeof(sblock->message));
       }
       else
	    printf("%s\n", buf);
  }
  return result;
#endif // SIMULATOR
#else
  // Just assume we're doing everything with floating point arithmetic
  // and do not need to worry about overflows or underflows
  FPTYPE v2 = FPSQUARE(initial_velocity) + 2.0 * acceleration * distance;
  if (v2 <= 0) return 0;
  else return FPSQRT(v2);
#endif  // !FIXED
}

#endif

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *current, block_t *next) {
  if(!current) { return; }
  
    if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
      if (VNEQ(current->entry_speed, current->max_entry_speed)) {
    
      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && next->speed_changed && (current->max_entry_speed > next->entry_speed)) {
#ifndef FIXSIGN
        current->entry_speed = min( current->max_entry_speed,
          max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
#else
        // We want to know what speed to start at so that if we decelerate -- negative acceleration --
        // over distance current->millimeters, we end up at speed next->entry_speed

	// This call produces the same result as !defined(FIXSIGN) case
#ifdef SIMULATOR
	sblock = current;
#endif
        current->entry_speed = min( current->max_entry_speed,
          initial_speed(-current->acceleration,next->entry_speed,current->millimeters));
#endif
      } else {
        current->entry_speed = current->max_entry_speed;
        current->speed_changed = true;
      }
      current->recalculate_flag = true;
    
    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;
  block_t *block[2] = { NULL, NULL};

  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END;

  while(block_index != tail) { 
      block_index = prev_block_index(block_index); 
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1]);
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current) {
  if(!previous || !current->use_accel) { return; }
  
  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag && previous->speed_changed) {
    if (VLT(previous->entry_speed, current->entry_speed)) {
#ifndef FIXSIGN
      FPTYPE entry_speed = min( current->entry_speed,
        max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );
#else
      // We want to know what the terminal speed from the prior block would be if
      // it accelerated over the entire block with starting speed prev->entry_speed

      // This call produces the same result as !defined(FIXSIGN) case
#ifdef SIMULATOR
      sblock = previous;
#endif
      FPTYPE entry_speed = min( current->entry_speed,
        final_speed(previous->acceleration,previous->entry_speed,previous->millimeters) );
#endif

      // Check for junction speed change
      if (VNEQ(current->entry_speed, entry_speed)) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
        current->speed_changed = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[2] = { NULL, NULL };

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1]);
    block_index = next_block_index(block_index);
  }
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids() {
  uint8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;
  
  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, FPDIV(current->entry_speed,current->nominal_speed),
          FPDIV(next->entry_speed,current->nominal_speed));
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with minimumPlannerSpeed. Always recalculated.
  if(next != NULL) {
    FPTYPE scaling = FPDIV(next->entry_speed,next->nominal_speed);
    calculate_trapezoid_for_block(next, scaling, scaling);
    // calculate_trapezoid_for_block(next,
    //    FPDIV(next->entry_speed,next->nominal_speed),
    //    FPDIV(minimumPlannerSpeed,next->nominal_speed));
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}


void plan_init(FPTYPE extruderAdvanceK, FPTYPE extruderAdvanceK2, bool zhold) {
#ifndef SIMULATOR
  stepperInterface = Motherboard::getBoard().getStepperAllInterfaces();
#else
  if ( (E_AXIS+1) != NUM_AXIS ) abort();
  if ( (X_AXIS >= NUM_AXIS) ||
       (Y_AXIS >= NUM_AXIS) ||
       (Z_AXIS >= NUM_AXIS) ||
       (E_AXIS >= NUM_AXIS) ) abort();
#endif
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_inverse_millimeters = 0;

#ifdef JKN_ADVANCE
  extruder_advance_k  = extruderAdvanceK;
  extruder_advance_k2 = extruderAdvanceK2;
#endif
  acceleration_zhold = zhold;

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
  current_move_index = 0;
#endif
}



// Add a new linear movement to the buffer. steps x, y and z is the absolute position in 
// steps. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
// Returns true if command was executed, otherwise false of the command was dopped (due to <dropsegments)
bool plan_buffer_line(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e,  FPTYPE feed_rate, const uint8_t &extruder, bool use_accel)
{
  //If we have an empty buffer, then disable slowdown until the buffer has become at least 1/2 full
  //This prevents slow start and gradual speedup at the beginning of a print, due to the SLOWDOWN algorithm
  if ( slowdown_limit && block_buffer_head == block_buffer_tail ) disable_slowdown = true;

  // Calculate the buffer head after we push this byte
  uint8_t next_buffer_head = next_block_index(block_buffer_head);

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  int32_t target[NUM_AXIS];
  target[X_AXIS] = x;
  target[Y_AXIS] = y;
  target[Z_AXIS] = z;
  target[E_AXIS] = e;
  
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Note whether block is accelerated or not
  block->use_accel = use_accel;

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

#ifndef CMD_SET_POSITION_CAUSES_DRAIN
  memcpy(block->starting_position, position, sizeof(block->starting_position)); // starting_position[] = position[]
#endif

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
  block->move_index = ++ current_move_index;
#endif

#ifdef SIMULATOR
  // Track how many times this block is worked on by the planner
  // Namely, how many times it is passed to calculate_trapezoid_for_block()
  block->planned = 0;
  block->message[0] = '\0';
  sblock = NULL;
  if (prior_dropped[0] != '\0')
  {
       strncat(block->message, prior_dropped, sizeof(block->message));
       prior_dropped[0] = '\0';
  }
#endif

  // Number of steps for each axis
  block->steps[X_AXIS] = planner_steps[X_AXIS];
  block->steps[Y_AXIS] = planner_steps[Y_AXIS];
  block->steps[Z_AXIS] = planner_steps[Z_AXIS];
  block->steps[E_AXIS] = planner_steps[E_AXIS];
  block->step_event_count = planner_master_steps;
	
  //Figure out which is the master axis
  for ( uint8_t i = 0; i < NUM_AXIS; i ++ ) {
  	if ( (uint32_t)labs(block->steps[i]) == block->step_event_count )
		block->dda_master_axis_index = i;
  }

  // Bail if this is a zero-length block
  if (block->step_event_count <=dropsegments)
#ifndef SIMULATOR
       return false;
#else
  {
       snprintf(prior_dropped, sizeof(prior_dropped),
		"*** Segment prior to this one was dropped: x/y/z/e steps = "
		"%d/%d/%d/%d, distance=%f",
		planner_steps[X_AXIS],
		planner_steps[Y_AXIS],
		planner_steps[Z_AXIS],
		planner_steps[E_AXIS],
		FPTOF(planner_distance));
       return false;
  }
#endif

  bool extruder_only_move = false;
  if ( block->steps[X_AXIS] == 0  &&  block->steps[Y_AXIS] == 0  &&  block->steps[Z_AXIS] == 0  &&  block->steps[E_AXIS] != 0 )
	extruder_only_move = true;

  if (block->steps[E_AXIS] == 0) {
        if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
  }
  else {
    	if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
  } 

#ifdef SIMULATOR
  // Save the original feed rate prior to modification by limits
  block->feed_rate = feed_rate;
#endif
  
  // Compute direction bits for this block 
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= (1<<X_AXIS); }
  if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= (1<<Y_AXIS); }
  if (target[Z_AXIS] < position[Z_AXIS]) { block->direction_bits |= (1<<Z_AXIS); }
  if (target[E_AXIS] < position[E_AXIS]) { block->direction_bits |= (1<<E_AXIS); }
  
  block->active_extruder = extruder;

#ifndef SIMULATOR
  //enable active axes
  if(block->steps[X_AXIS] != 0) stepperInterface[X_AXIS].setEnabled(true);
  if(block->steps[Y_AXIS] != 0) stepperInterface[Y_AXIS].setEnabled(true);
  if(block->steps[Z_AXIS] != 0) stepperInterface[Z_AXIS].setEnabled(true);

  // Enable all
  if(block->steps[E_AXIS] != 0) {
	stepperInterface[E_AXIS].setEnabled(true);
 }
#endif

  // slow down when de buffer starts to empty, rather than wait at the corner for a buffer refill
  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
#ifdef DEBUG_ZADVANCE
	//  if ( moves_queued < 2 )	zadvance += 1.0;
#endif

  //If we have an empty buffer, than anything "previous" should be wiped
  if ( moves_queued == 0 ) {
  	#ifndef YET_ANOTHER_JERK
		memset(previous_delta_mm, 0, sizeof(previous_delta_mm)); // clear position
  	#else
		memset(prev_speed, 0, sizeof(prev_speed));
  	#endif
  	previous_inverse_millimeters = 0;
  }

// SLOWDOWN
if ( slowdown_limit ) {
  //Renable slowdown if we have half filled up the buffer
  if (( disable_slowdown ) && ( moves_queued >= slowdown_limit ))	disable_slowdown = false;
  
  //If the buffer is less than half full, start slowing down the feed_rate
  //according to how little we have left in the buffer
  if ( moves_queued < slowdown_limit && (! disable_slowdown ) && moves_queued > 1)
	feed_rate = FPDIV( FPMULT2(feed_rate, ITOFP(moves_queued)), ITOFP((int32_t)slowdown_limit)); 
}
// END SLOWDOWN

  if ( extruder_only_move ) {
	block->millimeters = FPABS(delta_mm[E_AXIS]);
  } else {
  	block->millimeters = planner_distance;

	//If we have one item in the buffer, then control it's minimum time with minimumSegmentTime
	if ((moves_queued < 1 ) && (minimumSegmentTime > 0) && ( block->millimeters > 0 ) && 
	    ( feed_rate > 0 ) && (( FPDIV(block->millimeters, feed_rate) ) < minimumSegmentTime)) {
		 feed_rate = FPDIV(block->millimeters, minimumSegmentTime);
	}
  }
#ifdef FIXED
  FPTYPE inverse_millimeters = FPDIV(KCONSTANT_1, block->millimeters);  // Inverse millimeters to remove multiple divides 
#else
  FPTYPE inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 
#endif
  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  FPTYPE inverse_second = FPMULT2(feed_rate, inverse_millimeters);

#ifdef NO_CEIL_FLOOR
  block->nominal_speed = feed_rate; // (mm/sec) Always > 0
#ifdef FIXED
  block->nominal_rate = (uint32_t)FPTOI((KCONSTANT_0_5 + FPMULT2(ITOFP((int32_t)block->step_event_count), inverse_second))); // (step/sec) Always > 0
#else
  block->nominal_rate = (uint32_t)(0.5 + block->step_event_count * inverse_second); // (step/sec) Always > 0
#endif
#else
  block->nominal_speed = FPMULT2(block->millimeters, inverse_second); // (mm/sec) Always > 0
  block->nominal_rate = (uint32_t)FPTOI(FPCEIL(FPMULT2(ITOFP((int32_t)block->step_event_count), inverse_second))); // (step/sec) Always > 0
#endif


 // Calculate speed in mm/sec for each axis
  FPTYPE current_speed[NUM_AXIS];
  for(unsigned char i=0; i < NUM_AXIS; i++) {
    current_speed[i] = FPMULT2(delta_mm[i], inverse_second);
  }

  // Limit speed per axis
#ifdef FIXED
  FPTYPE speed_factor = KCONSTANT_1; //factor <=1 do decrease speed
#else
  FPTYPE speed_factor = 1.0; //factor <=1 do decrease speed
#endif
  if ( use_accel ) {
       if ( !extruder_only_move ) {
	    for(unsigned char i=0; i < NUM_AXIS; i++)
		 if(FPABS(current_speed[i]) > max_feedrate[i])
		      speed_factor = min(speed_factor, FPDIV(max_feedrate[i], FPABS(current_speed[i])));
       }
       else if(FPABS(current_speed[E_AXIS]) > extruder_only_max_feedrate)
	    speed_factor = FPDIV(extruder_only_max_feedrate, FPABS(current_speed[E_AXIS]));
  }
  else {
       for(unsigned char i=0; i < NUM_AXIS; i++)
	    if(FPABS(current_speed[i]) > max_speed_change[i])
		 speed_factor = min(speed_factor, FPDIV(max_speed_change[i], FPABS(current_speed[i])));
  }

  // Correct the speed  
#ifdef FIXED
  if( speed_factor < KCONSTANT_1 ) {
#else
  if( speed_factor < 1.0) {
#endif
    for(unsigned char i=0; i < NUM_AXIS; i++) {
        current_speed[i] = FPMULT2(current_speed[i], speed_factor);
    }
    block->nominal_speed = FPMULT2(block->nominal_speed, speed_factor);
    block->nominal_rate = (uint32_t)FPTOI(FPMULT2(ITOFP((int32_t)block->nominal_rate), speed_factor));
  }

  // Compute and limit the acceleration rate for the trapezoid generator.  
  FPTYPE steps_per_mm = FPDIV(ITOFP((int32_t)block->step_event_count), block->millimeters);
  if ( extruder_only_move ) {
#ifdef NO_CEIL_FLOOR
#ifdef FIXED
    //Assumptions made, due to the high value of acceleration_st / p_retract acceleration, dropped
    //ceil and floating point multiply
    block->acceleration_st = (uint32_t)(FPTOI(p_retract_acceleration) * FPTOI(steps_per_mm)); // convert to: acceleration steps/sec^2
#else
    block->acceleration_st = (uint32_t)(0.5 + p_retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
#endif
#else
    block->acceleration_st = (uint32_t)(FPTOI(FPCEIL(FPMULT2(p_retract_acceleration, steps_per_mm)))); // convert to: acceleration steps/sec^2
#endif
  }
  else {
#ifdef NO_CEIL_FLOOR
#ifdef FIXED
    //Assumptions made, due to the high value of acceleration_st / p_retract acceleration, dropped
    //ceil and floating point multiply
    block->acceleration_st = (uint32_t)(FPTOI(p_acceleration) * FPTOI(steps_per_mm)); // convert to: acceleration steps/sec^2
#else
    block->acceleration_st = (uint32_t)(0.5 + p_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
#endif
#else
    block->acceleration_st = (uint32_t)(FPTOI(FPCEIL(FPMULT2(p_acceleration, steps_per_mm)))); // convert to: acceleration steps/sec^2
#endif
    // Limit acceleration per axis
    for (uint8_t i = 0; i <= E_AXIS; i++)
      if((block->steps[i] != 0) &&
         ((block->acceleration_st * (uint32_t)block->steps[i]) > (axis_steps_per_sqr_second[i] * block->step_event_count)))
        block->acceleration_st = axis_steps_per_sqr_second[i];
  }
  block->acceleration = FPDIV(ITOFP((int32_t)block->acceleration_st), steps_per_mm);

  // The value 8.388608 derives from the timer frequency used for
  // st_interrupt().  That interrupt is driven by a timer counter which
  // ticks at a frequency of 2 MHz.  To convert counter values to seconds
  // the counter value needs to be divided by 2000000.  So that we
  // can do integer arithmetic (rather than floating point), we first
  // multiply the acceleration by the counter value and THEN divide the
  // result by 2000000.  However, the divide can be done by a shift and
  // it turns out that it is convenient to use >> 24 which is a divide
  // by approximately 16777216.  That's too large by about 8.388608.
  // Therefore, we pre-scale the acceleration here by 8.388608

  //This can potentially overflow in fixed point, due to a large block->acceleration_st,
  //so we don't use fixed point for this calculation
#ifdef FIXED
  block->acceleration_rate = (int32_t)(((int64_t)block->acceleration_st * 137439) >> 14);
#else
  block->acceleration_rate = (int32_t)((FPTYPE)block->acceleration_st * 8.388608);
#endif
  
#ifdef YET_ANOTHER_JERK
  FPTYPE scaling = KCONSTANT_1;
  bool docopy = true;
  if ( moves_queued == 0 ) 
  {
       vmax_junction = minimumPlannerSpeed;
       scaling = FPDIV(vmax_junction, block->nominal_speed);
  }
  else if ( (block->nominal_speed <= smallest_max_speed_change) || (!use_accel) )
  {
       vmax_junction = block->nominal_speed;
       // scaling remains KCONSTANT_1
  }
  else
  {
       FPTYPE delta_v;
       for (uint8_t i = 0; i < NUM_AXIS; i++)
       {
	    delta_v = FPABS(current_speed[i] - prev_speed[i]);
	    if ( delta_v > max_speed_change[i] )
	    {
		 // We wish to moderate max_entry_speed such that delta_v
		 // remains <= max_speed_change.  Moreover, any moderation we
		 // apply to the speed along this axis, we need to uniformly
		 // apply to all axes and, more importantly, to nominal_speed.
		 // As such, we need to determine a scaling factor, s.

		 // There's two approaches we can take without altering the
		 // prior block.
		 //
		 // Approach 1 -- approximate
		 //
		 //    s1 = scaling factor for approach 1
		 //    s1 * abs(current_speed - prev_speed) <= max_speed_change
		 //
		 // and thus
		 //
		 // [1]  s1 = max_speed_change / abs(current_speed - prev_speed)
		 //
		 // Note that if max_speed_change > 0 and [1] is only applied when
		 //
		 // [2]  abs(current_speed - prev_speed) <= max_speed_change
		 //
		 // then s1 always obeys
		 //
		 // [3]  0 < s1 < 1.
		 //
		 // This approach is "approximate" in that we will only scale
		 // current_speed (the present block).  We will not scale the
		 // prior block.  But, the planner will strive to scale the
		 // final speed of the prior block to match this scaled entry
		 // speed.
		 //
		 // Approach 2
		 //
		 //    s2 = scaling factor for approach 2
		 //    abs(s2 * current_speed - prev_speed) <= max_speed_change
		 //
		 // which leads to
		 //
		 // [4a]  s2 = abs( (prev_speed - max_speed_change) / current_speed )
		 //          WHEN (current_speed - prev_speed) < 0
		 //
		 // [4b]  s2 = abs( (prev_speed + max_speed_change) / current_speed )
		 //          WHEN (current_speed - prev_speed) > 0
		 //
		 // Unfortunately, s2 has the range
		 //
		 // [5]   0 <= s2 <= infinity.
		 //
		 // The difficulty with using [4a] and [4b] is that they can lead to
		 // very small scalings, s2.  For example, when the numerator is zero
		 // or close to zero.  Additionally, they lead to an infinite scaling
		 // when the current_speed is zero.
		 //
		 // In theory, we could pick and choose
		 //
		 //     s = max(s1, min(s2, 1))
		 //
		 // However, the benefit is slight and the additional code complexity
		 // (code space), and compute time doesn't make it worthwhile.  As such
		 // we stick to using the first approach, [2].

		 // Avoid using min(x,y) as it may generate duplicate
		 // computations if it is a macro
		 FPTYPE s = FPDIV(max_speed_change[i], delta_v);
		 if ( s < scaling ) scaling = s;
	    }
       }
       if (scaling != KCONSTANT_1)
       {
	    vmax_junction = FPMULT2(block->nominal_speed, scaling);
	    for (uint8_t i = 0; i < NUM_AXIS; i++)
		 prev_speed[i] = FPMULT2(current_speed[i], scaling);
	    docopy = false;
       }
       else
	    // scaling remains KCONSTANT_1
	    vmax_junction = block->nominal_speed;
  }
  if ( docopy ) memcpy(prev_speed, current_speed, sizeof(prev_speed));

#else
// START COSINE_JERK2

#ifdef DEBUG_ZADVANCE
	//	DEBUG_TIMER_START;
#endif

  if ( moves_queued == 0 ) 
  {
       vmax_junction = minimumPlannerSpeed;
  }
  else if ( (block->nominal_speed <= max_speed_change[0]) || (!use_accel) )
  {
	vmax_junction = block->nominal_speed;
  }
  // The cosine of the angle A between two vectors V1 and V2 is given by
  //
  //    cos(A) = dot-product(V1, V2) / [ norm(V1) norm(V2) ]
  //
  // When a vector V is a unit vector (normalized), then norm(V) = 1
  // Thus, we have
  //
  //    cos(A) = dot-product(V1, V2)
  //
  // Further, recall that if -90 < A < 90, then cos(A) > 0
  //                      if abs(A) == 90, then cos(A) = 0
  //                      if abs(A) > 90,  then cos(A) < 0
  //
  // Thus, the sign of cos(A) tells us if the two vectors are
  // at less than 90 degrees, 90 degrees, or more than 90 degrees
  // This is of interest as it tells us how we're turning between
  // each segment.
  else if (block->steps[X_AXIS] != 0 || block->steps[Y_AXIS] != 0)
  {
       cosine = FPMULT3((FPMULT2(previous_delta_mm[X_AXIS], delta_mm[X_AXIS])
		       + FPMULT2(previous_delta_mm[Y_AXIS], delta_mm[Y_AXIS])
		       + FPMULT2(previous_delta_mm[Z_AXIS], delta_mm[Z_AXIS])
		       + FPMULT2(previous_delta_mm[E_AXIS], delta_mm[E_AXIS])),
		       inverse_millimeters, previous_inverse_millimeters);

       // Convert the angle to an integer "turn" in the range -10 <= t <= 10
       // Note that cos(     0) = +1.000  ==> turn = +10
       //           cos(+/ -45) = +0.707  ==> turn =   7
       //           cos(+/ -90) = +0.000  ==> turn =   0
       //           cos(+/-180) = -1.000  ==> turn = -10

#ifdef FIXED
	if ( cosine <= 0 )
#else
       //The following code is a floating point trick to compare for < 
       //without having to do a floating point op.
       //http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
       //Saves upto 150uS
       //Replaces:
       //	if (cosine <= 0.0)
       if ( *(int32_t *)&cosine < *(int32_t *)&compareLessThanEqualZero )
#endif
       {
	    // Turn exceeds 90 degrees: apply serious breaking at the junction
	    //
	    // 1.0 + cos(+/-  90) = 1.0 for  90 degrees
	    // 1.0 + cos(+/- 180) = 0.0 for 180 degrees
	    vmax_junction = max_speed_change[0];
       }
       else
       {
	    // Turn < 90 degrees
	    // Apply breaking down to min_junction_speed based on the sharpness of the angle
	    vmax_junction = FPMULT2(block->nominal_speed - max_speed_change[0], cosine) + max_speed_change[0];
#ifdef FIXED
	    vmax_junction = max(vmax_junction, max_speed_change[0]);
#else
	    //The following code is a floating point trick to compare for < 
	    //without having to do a floating point op.
	    //http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
	    //Saves upto 100uS
	    //Replaces:
	    //	vmax_junction = max(vmax_junction, max_speed_change);
	    if ( *(int32_t *)&vmax_junction < *(int32_t *)&max_speed_change )
		 vmax_junction = max_speed_change[0];
#endif
       }
  }
  else
  {
       // No motion in the X or Y plane
       if (block->steps[E_AXIS] == 0)
	    // Z only move (or no move)
	    vmax_junction = max_feedrate[Z_AXIS];
       else if (block->steps[Z_AXIS] == 0)
	    // E only move (or no move)
	    vmax_junction = extruder_only_max_feedrate;
       else
	    // E and Z move
	    vmax_junction = min(max_feedrate[E_AXIS], max_feedrate[Z_AXIS]);

       vmax_junction = min(block->nominal_speed, vmax_junction);
  }

  // Save for next pass
  previous_inverse_millimeters = inverse_millimeters;
  memcpy(previous_delta_mm, delta_mm, sizeof(previous_delta_mm)); // previous_delta_mm[] = delta_mm[]

#ifdef DEBUG_ZADVANCE
	//	DEBUG_TIMER_FINISH;
	//	zadvance2 = DEBUG_TIMER_TCTIMER_US;
#endif

//END COSINE_JERK2

#endif

#ifdef DEBUG_ZADVANCE
	//  if ( block->steps[Z_AXIS] != 0 )
	// 	zadvance2 = FPTOF(vmax_junction);
#endif

  // Initialize block entry speed. Compute based on deceleration to user-defined minimumPlannerSpeed.
#ifndef FIXSIGN
  FPTYPE v_allowable = max_allowable_speed(-block->acceleration,minimumPlannerSpeed,block->millimeters);
  block->max_entry_speed = vmax_junction;
  if (use_accel) block->entry_speed = min(vmax_junction, v_allowable);
  else block->entry_speed = vmax_junction;
#else
  // We want our speed to be AT LEAST fast enough such that we hit minimumPlannerSpeed by the block's end
  // It's okay, however, if we go even faster so take the max of vmax_junction and v_allowable

  // This call produces the same result as the !FIXSIGN case
  // It's the max. speed we can achieve if we accelerate the entire length of the block
  //   starting with an initial speed of minimumPlannerSpeed
#ifdef SIMULATOR
  sblock = block;
#endif
  FPTYPE v_allowable = final_speed(block->acceleration,minimumPlannerSpeed,block->millimeters);

  // And this will typically produce a larger value than the !defined(FIXSIGN) case
  if (vmax_junction < minimumPlannerSpeed) {
       block->entry_speed = minimumPlannerSpeed;
       block->max_entry_speed = minimumPlannerSpeed;
  } else {
       block->entry_speed = vmax_junction;
       block->max_entry_speed = vmax_junction;
  }
#endif

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { block->nominal_length_flag = true; }
  else { block->nominal_length_flag = false; }
  block->recalculate_flag = true; // Always calculate trapezoid for new block
  block->speed_changed = false;

#ifdef JKN_ADVANCE
  block->advance_pressure_relax = 0;
  if((block->steps[E_AXIS] == 0) || ( extruder_only_move ) || (( extruder_advance_k == 0 ) && ( extruder_advance_k2 == 0)) || ( !use_accel )) {
	block->use_advance_lead = false;
	block->advance_lead_entry   = 0;
	block->advance_lead_exit    = 0;
	block->advance_lead_prime   = 0;
	block->advance_lead_deprime = 0;
	block->advance_pressure_relax = 0;
  } else {
	block->use_advance_lead = true;
  }
#endif

#ifndef YET_ANOTHER_JERK
  FPTYPE scaling = FPDIV(block->entry_speed,block->nominal_speed);
#endif
  calculate_trapezoid_for_block(block, scaling, scaling);

#ifdef JKN_ADVANCE
  //if ( block->advance_lead_entry < 0 )		zadvance = block->advance_lead_entry;
  //if ( block->advance_lead_exit < 0 )		zadvance = block->advance_lead_exit;
  //if ( block->advance_lead_prime < 0 )		zadvance = block->advance_lead_prime;
  //if ( block->advance_lead_deprime < 0 )	zadvance = block->advance_lead_deprime;
  //zadvance2 = block->advance_pressure_relax;
#endif
    
  // Move buffer head
  block_buffer_head = next_buffer_head;
  
  // Update position
  memcpy(position, target, sizeof(position)); // position[] = target[]


#ifdef DEBUG_ZADVANCE
	#ifdef FIXED
	//	zadvance = FPTOF(roundk(FTOFP(2.8934), 3));	//0 = 7 1,2 = 3.0 8 = 2.895
	//	zadvance2 = FPTOF(roundk(FTOFP(2.3846), 3));	//0 = 6 1,2 = 2.5 8 = 2.383
	#endif
#endif

  planner_recalculate();

  return true;
}

void plan_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e)
{
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
#endif
  position[X_AXIS] = x;
  position[Y_AXIS] = y;
  position[Z_AXIS] = z;
  position[E_AXIS] = e;
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
  CRITICAL_SECTION_END;  // Fill variables used by the stepper in a critical section
#endif

#ifdef CMD_SET_POSITION_CAUSES_DRAIN
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
#endif
}

void plan_set_e_position(const int32_t &e)
{
  position[E_AXIS] = (int32_t)e;
  st_set_e_position(position[E_AXIS]);
}

uint8_t movesplanned()
{
 return (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}  



//Figure out the acceleration stats by scanning through the command pipeline

void accelStatsGet(float *minSpeed, float *avgSpeed, float *maxSpeed) {
	block_t *block;
	int32_t count = 0;
	uint8_t block_index = block_buffer_tail;

	FPTYPE smax = 0, savg = 0;
#ifdef FIXED
	FPTYPE smin = KCONSTANT_1000;
#else
	FPTYPE smin = 1000.0;
#endif

	while(block_index != block_buffer_head) {
		block = &block_buffer[block_index];

		smin = min(smin, block->entry_speed);
		smax = max(smax, block->nominal_speed);
		savg += block->nominal_speed;
	
		block_index = next_block_index(block_index);
		count ++;
	}
	
	if ( count ) {
		//We have stats
		*minSpeed = FPTOF(smin);
		*maxSpeed = FPTOF(smax);
		*avgSpeed = FPTOF(FPDIV(savg, ITOFP(count)));
	} else {
		//We have no stats
		*minSpeed = 0.0;
		*maxSpeed = 0.0;
		*avgSpeed = 0.0;
	}
}

#endif
