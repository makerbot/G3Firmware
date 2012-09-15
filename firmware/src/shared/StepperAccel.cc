/*
  StepperAccel.cc - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Configuration.hh"

#ifdef HAS_STEPPER_ACCELERATION



#include "StepperAccel.hh"

#ifdef LOOKUP_TABLE_TIMER
#include "StepperAccelSpeedTable.hh"
#endif

#include "StepperInterface.hh"
#include "Motherboard.hh"

#include <avr/interrupt.h>
#include <string.h>
#include <math.h>




//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced
int32_t extruder_deprime_steps;	//Positive or negative depending on clockwise_extruder, when it's true, this is positive
bool clockwise_extruder;	//True if the extruder extrudes when rotating positive when viewed from the spindle end

//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
volatile static uint32_t step_events_completed; // The number of step events executed in the current block
#ifdef JKN_ADVANCE
  enum AdvanceState {
	ADVANCE_STATE_ACCEL = 0,
	ADVANCE_STATE_PLATEAU,
	ADVANCE_STATE_DECEL
  };
  static enum AdvanceState advance_state;
  static int32_t advance_pressure_relax_accumulator;
  static int32_t lastAdvanceDeprime;

  #define ADVANCE_INTERRUPT_FREQUENCY 10000	//10KHz

  static uint32_t st_advance_interrupt_rate = 0;
  static uint8_t advance_interrupt_steps_per_call = 0;
  static uint32_t st_advance_interrupt_rate_counter[EXTRUDERS];
  volatile int32_t starting_e_position = 0;
#endif
volatile static int32_t e_steps[EXTRUDERS];
static int32_t acceleration_time, deceleration_time;
//static uint32_t accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static uint16_t acc_step_rate, step_rate;
static char step_loops, step_loops_nominal;
static uint16_t OCR1A_nominal;

volatile int32_t count_position[NUM_AXIS] = { 0, 0, 0, 0};

static StepperInterface *stepperInterface;

static bool deprimed = true;
static unsigned char deprimeIndex = 0;

//STEP_TRUE, runs stepperInterface[axis].step(true)
//If CHECK_ENDSTOPS_ENABLED is defined, it checks the ends stops haven't been hit before moving
//and doesn't move if they have

#ifdef CHECK_ENDSTOPS_ENABLED
	#define STEP_TRUE(axis, direction)	if (( (direction) && (! stepperInterface[axis].isAtMaximum())) || \
						    ( (! (direction)) && (! stepperInterface[axis].isAtMinimum()))) \
								stepperInterface[axis].step(true)
#else
	#define STEP_TRUE(axis, direction)	stepperInterface[axis].step(true)
#endif

#if  defined(DEBUG_TIMER) && defined(TCNT3)
uint16_t debugTimer;
#endif

#ifdef OVERSAMPLED_DDA
    uint8_t oversampledCount = 0;
#endif

struct dda {
	bool	master;			//True if this is the master steps axis
	int32_t master_steps;		//The number of steps for the master axis	
	bool	eAxis;			//True if this is the e axis
	char	direction;		//Direction of the dda, 1 = forward, -1 = backwards
	bool	stepperInterfaceDir;	//The direction the stepper interface gets sent in

	int32_t	counter;		//Used for the dda counter
	int32_t steps_completed;	//Number of steps completed
	int32_t steps;			//Number of steps we need to execute for this axis
};

struct dda ddas[NUM_AXIS];


//===========================================================================
//=============================functions         ============================
//===========================================================================

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)


//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

FORCE_INLINE uint16_t calc_timer(uint16_t step_rate) {
  uint16_t timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  } 

  if(step_rate < 32) step_rate = 32;

#ifdef LOOKUP_TABLE_TIMER
  step_rate -= 32; // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate 
    uint16_t table_address = (uint16_t)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    uint16_t gain = (uint16_t)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (uint16_t)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    uint16_t table_address = (uint16_t)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (uint16_t)pgm_read_word_near(table_address);
    timer -= (((uint16_t)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  //if(timer < 100) { timer = 100; MSerial.print("Steprate to high : "); MSerial.println(step_rate); }//(20kHz this should never happen)
 
  return timer;
#else
  return (uint16_t)((uint32_t)2000000 / (uint32_t)step_rate);
#endif
}


#ifdef DEBUG_ZADVANCE
	volatile float zadvance = 0.0, zadvance2 = 0.0;
#endif


FORCE_INLINE void dda_create(uint8_t ind, bool eAxis)
{
	ddas[ind].eAxis			= eAxis;
	ddas[ind].counter		= 0;	
	ddas[ind].direction		= 1;
	ddas[ind].stepperInterfaceDir	= false;
}

FORCE_INLINE void dda_reset(uint8_t ind, bool master, int32_t master_steps, bool direction, int32_t steps, bool keepPhase)
{
	if ( keepPhase )	
	{
		//Calculate the phase of the new move, keeping the phase of the last move
		ddas[ind].counter += ddas[ind].master_steps >> 1;
		ddas[ind].counter = (ddas[ind].counter * master_steps) / (ddas[ind].master_steps << 1);
	}
	else	ddas[ind].counter  = master_steps >> 1;

#ifdef OVERSAMPLED_DDA
	ddas[ind].counter  = - (ddas[ind].counter << OVERSAMPLED_DDA);
#else
	ddas[ind].counter  = - ddas[ind].counter;
#endif

	ddas[ind].master		= master;
#ifdef OVERSAMPLED_DDA
	ddas[ind].master_steps		= master_steps << OVERSAMPLED_DDA;
#else
	ddas[ind].master_steps		= master_steps;
#endif
	ddas[ind].steps			= steps;
	ddas[ind].direction		= (direction) ? -1 : 1;
	ddas[ind].stepperInterfaceDir	= (direction) ? false : true;

	ddas[ind].steps_completed = 0;
}

FORCE_INLINE void dda_shift_phase(uint8_t ind, int32_t phase)
{
#ifdef OVERSAMPLED_DDA
	ddas[ind].counter += phase << OVERSAMPLED_DDA;
#else
	ddas[ind].counter += phase;
#endif
}

FORCE_INLINE void dda_step(uint8_t ind)
{
	ddas[ind].counter += ddas[ind].steps;
	if (( ddas[ind].counter > 0 ) && ( ddas[ind].steps_completed < ddas[ind].steps ))
	{
        	ddas[ind].counter -= ddas[ind].master_steps;

#ifdef JKN_ADVANCE
		if ( ddas[ind].eAxis )
          		e_steps[current_block->active_extruder] += ddas[ind].direction;
		else
		{
#endif
        		stepperInterface[ind].setDirection( ddas[ind].stepperInterfaceDir );
			STEP_TRUE(ind, ddas[ind].stepperInterfaceDir);
			stepperInterface[ind].step(false);
       		 	count_position[ind] += ddas[ind].direction;
#ifdef JKN_ADVANCE
		}
#endif

		ddas[ind].steps_completed ++;		
	}
}

// Sets up the next block from the buffer

FORCE_INLINE void setup_next_block() {
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
  memcpy((void *)count_position, current_block->starting_position, sizeof(count_position)); // count_position[] = current_block->starting_position[]
#endif

  #ifdef JKN_ADVANCE
  	starting_e_position = count_position[E_AXIS];

	//Something in the buffer, prime if we previously deprimed
	if (( deprimed ) && ( current_block->steps[E_AXIS] != 0 )) {
		deprimeIndex = current_block->active_extruder;
		if ( clockwise_extruder ) {
			e_steps[deprimeIndex] -= (int32_t)extruder_deprime_steps;
#ifdef JKN_ADVANCE_LEAD_DE_PRIME
			e_steps[deprimeIndex] -= current_block->advance_lead_prime;
#endif
		} else {
			e_steps[deprimeIndex] += (int32_t)extruder_deprime_steps;
#ifdef JKN_ADVANCE_LEAD_DE_PRIME
			e_steps[deprimeIndex] += current_block->advance_lead_prime;
#endif
		}
		deprimed = false;
	}
  #endif

  deceleration_time = 0;

  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  step_loops_nominal = step_loops;
  
  // step_rate to timer interval
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
#ifdef OVERSAMPLED_DDA
  OCR1A = acceleration_time >> OVERSAMPLED_DDA;
#else
  OCR1A = acceleration_time;
#endif
  #ifdef ACCELERATION_Z_HOLD_ENABLED
    stepperInterface[Z_AXIS].setEnabled(acceleration_zhold || (current_block->steps[Z_AXIS] > 0));
  #endif

  //Setup the next dda's
  out_bits = current_block->direction_bits;
  for ( uint8_t i = 0; i < NUM_AXIS; i ++ )
      	dda_reset(i, (current_block->dda_master_axis_index == i), current_block->step_event_count, (out_bits & (1<<i)), current_block->steps[i], DDA_KEEP_PHASE);

  #ifdef JKN_ADVANCE
      advance_state = ADVANCE_STATE_ACCEL;
  #endif
      step_events_completed = 0;

  #ifdef DEBUG_BLOCK_BY_MOVE_INDEX
      if ( current_block->move_index == 4 ) {
	zadvance = (float)current_block->initial_rate;
	zadvance2 = (float)current_block->final_rate;
      }
  #endif
}


// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
// Returns true if we deleted an item in the pipeline buffer 
bool st_interrupt()
{    
  bool block_deleted = false;

#ifdef OVERSAMPLED_DDA
  if ( current_block != NULL )
  {
      oversampledCount ++;
      if ( oversampledCount < (1 << OVERSAMPLED_DDA) )
      {
      	//Step the dda for each axis
     	for (uint8_t i = 0; i < NUM_AXIS; i ++ )
      		dda_step(i);
        return block_deleted;
      }
  }
#endif

//	DEBUG_TIMER_START;
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
	setup_next_block();
    } else {
        OCR1A=2000; // 1kHz.

#ifndef CMD_SET_POSITION_CAUSES_DRAIN
	//Buffer is empty, as the position may have since changed due to 
	//plan_set_position, we update our position here
  	memcpy((void *)count_position, position, sizeof(count_position)); // count_position[] = position
#endif
    }
  } 

#ifdef JKN_ADVANCE
  //Nothing in the buffer or we have no e steps, deprime
  if ((( current_block == NULL ) || ( current_block->steps[E_AXIS] == 0 )) && ( ! deprimed )) {
	if ( clockwise_extruder ) {
		e_steps[deprimeIndex] += (int32_t)extruder_deprime_steps;
#ifdef JKN_ADVANCE_LEAD_DE_PRIME
		e_steps[deprimeIndex] += lastAdvanceDeprime;
#endif
	} else {
		e_steps[deprimeIndex] -= (int32_t)extruder_deprime_steps;
#ifdef JKN_ADVANCE_LEAD_DE_PRIME
		e_steps[deprimeIndex] -= lastAdvanceDeprime;
#endif
	}
	deprimed = true;
  }    
#endif

  if (current_block != NULL) {

    for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 
#ifdef JKN_ADVANCE
     if ( advance_state == ADVANCE_STATE_ACCEL ) {
	dda_shift_phase(E_AXIS, current_block->advance_lead_entry);
     }
     if ( advance_state == ADVANCE_STATE_DECEL ) {
	dda_shift_phase(E_AXIS, - current_block->advance_lead_exit);
	dda_shift_phase(E_AXIS, - advance_pressure_relax_accumulator >> 8);
     }
#endif
      
      //Step the dda for each axis
      for (uint8_t i = 0; i < NUM_AXIS; i ++ )
      	dda_step(i);

#ifdef OVERSAMPLED_DDA
  oversampledCount = 0;
#endif

      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
    }

    // Calculate new timer value
    uint16_t timer;
    if (step_events_completed <= (uint32_t)current_block->accelerate_until) {
      
      // Note that we need to convert acceleration_time from units of
      // 2 MHz to seconds.  That is done by dividing acceleration_time
      // by 2000000.  But, that will make it 0 when we use integer
      // arithmetic.  So, we first multiply block->acceleration_rate by
      // acceleration_time and then do the divide.  However, it's
      // convenient to divide by 2^24 ( >> 24 ).  So, block->acceleration_rate
      // has been prescaled by a factor of 8.388608.

      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
#ifdef OVERSAMPLED_DDA
      OCR1A = timer >> OVERSAMPLED_DDA;
#else
      OCR1A = timer;
#endif
      acceleration_time += timer;
    } 
    else if (step_events_completed > (uint32_t)current_block->decelerate_after) {   
#ifdef JKN_ADVANCE
	if ( advance_state == ADVANCE_STATE_ACCEL ) {
		advance_state = ADVANCE_STATE_PLATEAU;
	}
	if ( advance_state == ADVANCE_STATE_PLATEAU ) {
		advance_state = ADVANCE_STATE_DECEL;
		advance_pressure_relax_accumulator = 0;
	}
      advance_pressure_relax_accumulator += current_block->advance_pressure_relax;
#endif

      // Note that we need to convert deceleration_time from units of
      // 2 MHz to seconds.  That is done by dividing deceleration_time
      // by 2000000.  But, that will make it 0 when we use integer
      // arithmetic.  So, we first multiply block->acceleration_rate by
      // deceleration_time and then do the divide.  However, it's
      // convenient to divide by 2^24 ( >> 24 ).  So, block->acceleration_rate
      // has been prescaled by a factor of 8.388608.

      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      
      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
        // lower limit
        if(step_rate < current_block->final_rate)
          step_rate = current_block->final_rate;
      }

      // step_rate to timer interval
      timer = calc_timer(step_rate);
#ifdef OVERSAMPLED_DDA
      OCR1A = timer >> OVERSAMPLED_DDA;
#else
      OCR1A = timer;
#endif
      deceleration_time += timer;
    }
    else {
#ifdef JKN_ADVANCE
	if ( advance_state == ADVANCE_STATE_ACCEL ) {
		advance_state = ADVANCE_STATE_PLATEAU;
	}
#endif
#ifdef OVERSAMPLED_DDA
      OCR1A = OCR1A_nominal >> OVERSAMPLED_DDA;
#else
      OCR1A = OCR1A_nominal;
#endif
      step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) {
#ifdef JKN_ADVANCE
      count_position[E_AXIS] = starting_e_position + current_block->steps[E_AXIS];
      starting_e_position = 0;
      lastAdvanceDeprime = current_block->advance_lead_deprime;
#endif
      current_block = NULL;
      plan_discard_current_block();
      block_deleted = true;
	
      //Preprocess the setup for the next block if have have one
      current_block = plan_get_current_block();
      if (current_block != NULL) {
	setup_next_block();
      } 
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
      else {
	//Buffer is empty, as the position may have since changed due to 
	//plan_set_position, we update our position here
  	memcpy((void *)count_position, position, sizeof(count_position)); // count_position[] = position
     }
#endif
    }   
  } 
  return block_deleted;
}

#ifdef JKN_ADVANCE
void st_advance_interrupt()
  {
    uint8_t i;

    //Increment the rate counters
#ifdef JKN_ADVANCE
    for ( i = 0; i < EXTRUDERS; i ++ )	st_advance_interrupt_rate_counter[i] ++;
#endif

    // Set E direction (Depends on E direction + advance)
    for ( i = 0; e_steps[0] &&
		 (st_advance_interrupt_rate_counter[0] >= st_advance_interrupt_rate) &&
		 (i < advance_interrupt_steps_per_call); i ++ ) {
	stepperInterface[E_AXIS].step(false);
        if (e_steps[0] < 0) {
      	  stepperInterface[E_AXIS].setDirection(false);
          e_steps[0]++;
	  STEP_TRUE(E_AXIS, false);
          count_position[E_AXIS]++;
        } 
        else if (e_steps[0] > 0) {
      	  stepperInterface[E_AXIS].setDirection(true);
          e_steps[0]--;
	  STEP_TRUE(E_AXIS, true);
          count_position[E_AXIS]--;
        }
	st_advance_interrupt_rate_counter[0] = 0;
    }
 #if EXTRUDERS > 1
    for ( i = 0; e_steps[1] &&
		 (st_advance_interrupt_rate_counter[?] >= st_advance_interrupt_rate) &&
		 (i < advance_interrupt_steps_per_call); i ++ ) {
	//stepperInterface[?].step(false);
        if (e_steps[1] < 0) {
      	  stepperInterface[?].setDirection(false);
          e_steps[1]++;
//	  STEP_TRUE(?, false);
//        count_position[?]++;
        } 
        else if (e_steps[1] > 0) {
      	  stepperInterface[?].setDirection(true);
          e_steps[1]--;
//	  STEP_TRUE(?, true);
//        count_position[?]--;
        }
	st_advance_interrupt_rate_counter[?] = 0;
    }
 #endif
 #if EXTRUDERS > 2
    for ( i = 0; e_steps[2] &&
		 (st_advance_interrupt_rate_counter[?] >= st_advance_interrupt_rate) &&
		 (i < advance_interrupt_steps_per_call); i ++ ) {
	//stepperInterface[?].step(false);
        if (e_steps[2] < 0) {
      	  //stepperInterface[?].setDirection(false);
          e_steps[2]++;
//	  STEP_TRUE(?, false);
//        count_position[?]++;
        } 
        else if (e_steps[2] > 0) {
      	  //stepperInterface[?].setDirection(true);
          e_steps[2]--;
//	  STEP_TRUE(?, true);
//        count_position[?]--;
        }
	st_advance_interrupt_rate_counter[?] = 0;
      }
 #endif
  }
#endif // JKN_ADVANCE

void st_init()
{
  //Create the dda's
  dda_create(X_AXIS, false);
  dda_create(Y_AXIS, false);
  dda_create(Z_AXIS, false);
  dda_create(E_AXIS, true);

#ifdef OVERSAMPLED_DDA
  oversampledCount = 0;
#endif

  //Grab the stepper interfaces
  stepperInterface = Motherboard::getBoard().getStepperAllInterfaces();

  Motherboard::getBoard().setupAccelStepperTimer();

  #ifdef JKN_ADVANCE
    deprimed = true;
    lastAdvanceDeprime = 0;
  #endif

#ifdef JKN_ADVANCE
  //Calculate the smallest number of st_advance_interrupt's between extruder steps based on the the
  //extruder_only_max_feedrate and an st_advance_interrupt of 10KHz (ADVANCE_INTERRUPT_FREQUENCY).
  float st_advance_interrupt_ratef = (float)ADVANCE_INTERRUPT_FREQUENCY / (FPTOF(extruder_only_max_feedrate) * axis_steps_per_unit[E_AXIS]);

  //Round up (slower), or if we need more steps than 1 in an interrupt (st_advance_interrupt_ratef < 1), then
  //we need to process more steps in the loop
  if	  ( st_advance_interrupt_ratef > 1.0 ) st_advance_interrupt_ratef = ceil(st_advance_interrupt_ratef);
  else if ( st_advance_interrupt_ratef < 1.0 )st_advance_interrupt_ratef = 0;
  st_advance_interrupt_rate = (uint32_t)st_advance_interrupt_ratef;

  advance_interrupt_steps_per_call = 0;
  if ( st_advance_interrupt_rate == 0 )
	advance_interrupt_steps_per_call = ceil(1.0 / st_advance_interrupt_ratef);	
  if ( advance_interrupt_steps_per_call < 1 ) advance_interrupt_steps_per_call = 1;

    for ( uint8_t i = 0; i < EXTRUDERS; i ++ ) {
	e_steps[i] = 0;
	st_advance_interrupt_rate_counter[i] = st_advance_interrupt_rate;
    }
#endif
}

// Block until all buffered steps are executed
bool st_empty()
{
    if (block_buffer_head == block_buffer_tail) return true;
    return false;
}

void st_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
#ifdef JKN_ADVANCE
  starting_e_position = count_position[E_AXIS]; 
#endif
  CRITICAL_SECTION_END;
}

void st_set_e_position(const int32_t &e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
#ifdef JKN_ADVANCE
  starting_e_position = count_position[E_AXIS]; 
#endif
  CRITICAL_SECTION_END;
}

int32_t st_get_position(uint8_t axis)
{
  int32_t count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
  	plan_discard_current_block();
  current_block = NULL;
  plan_set_position((const int32_t)count_position[X_AXIS],
		    (const int32_t)count_position[Y_AXIS],
		    (const int32_t)count_position[Z_AXIS],
		    (const int32_t)count_position[E_AXIS]);
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

#endif
