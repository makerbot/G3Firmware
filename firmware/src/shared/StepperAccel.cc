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
#include "StepperAccelSpeedTable.hh"
#include "StepperInterface.hh"
#include "Motherboard.hh"

#include  <avr/interrupt.h>




//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static int32_t counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
volatile static uint32_t step_events_completed; // The number of step events executed in the current block
#ifdef ADVANCE
  static int32_t advance_rate = 0, advance, final_advance = 0;
  static int32_t old_advance = 0;
#endif
volatile static int32_t e_steps[3];
volatile static unsigned char busy = false; // TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
static int32_t acceleration_time, deceleration_time;
//static uint32_t accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;

volatile int32_t count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

static StepperInterface *stepperInterface;

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

void st_wake_up() {
  //  TCNT1 = 0;
  if(busy == false) 
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
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
  step_rate -= 32; // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate 
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  //if(timer < 100) { timer = 100; MSerial.print("Steprate to high : "); MSerial.println(step_rate); }//(20kHz this should never happen)
 
  //TEMPORARY WORKAROUND FOR ZIT BUG
  if ( timer > 2000 )	timer = 2000;

  return timer;
}


//DEBUGGING
float zadvance;

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;  
    zadvance = advance;
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  #ifdef Z_LATE_ENABLE
    if(current_block->steps_z > 0) stepperInterface[Z_AXIS].setEnabled(true);
  #endif
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
void st_interrupt()
{    
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;
//      #ifdef ADVANCE
//      e_steps[current_block->active_extruder] = 0;
//      #endif
    } 
    else {
        OCR1A=2000; // 1kHz.
    }    
  } 

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction en check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
      stepperInterface[X_AXIS].setDirection(false);
      count_direction[X_AXIS]=-1;
    }
    else { // +direction 
      stepperInterface[X_AXIS].setDirection(true);
      count_direction[X_AXIS]=1;
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      stepperInterface[Y_AXIS].setDirection(false);
      count_direction[Y_AXIS]=-1;
    }
    else { // +direction
      stepperInterface[Y_AXIS].setDirection(true);
      count_direction[Y_AXIS]=1;
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      stepperInterface[Z_AXIS].setDirection(false);
      count_direction[Z_AXIS]=-1;
    }
    else { // +direction
      stepperInterface[Z_AXIS].setDirection(true);
      count_direction[Z_AXIS]=1;
    }

    #ifndef ADVANCE
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
	stepperInterface[E_AXIS].setDirection(false);
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
	stepperInterface[E_AXIS].setDirection(true);
        count_direction[E_AXIS]=-1;
      }
    #endif //!ADVANCE
    

    
    for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 
      #ifdef ADVANCE
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          e_steps[current_block->active_extruder]--;
        }
        else {
          e_steps[current_block->active_extruder]++;
        }
      }    
      #endif //ADVANCE
      
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
	stepperInterface[X_AXIS].step(true);
        counter_x -= current_block->step_event_count;
	stepperInterface[X_AXIS].step(false);
        count_position[X_AXIS]+=count_direction[X_AXIS];   
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
	stepperInterface[Y_AXIS].step(true);
        counter_y -= current_block->step_event_count;
	stepperInterface[Y_AXIS].step(false);
        count_position[Y_AXIS]+=count_direction[Y_AXIS];
      }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
	stepperInterface[Z_AXIS].step(true);
        counter_z -= current_block->step_event_count;
	stepperInterface[Z_AXIS].step(false);
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
      }

      #ifndef ADVANCE
        counter_e += current_block->steps_e;
        if (counter_e > 0) {
	  stepperInterface[E_AXIS].step(true);
          counter_e -= current_block->step_event_count;
	  stepperInterface[E_AXIS].step(false);
          count_position[E_AXIS]+=count_direction[E_AXIS];
        }
      #endif //!ADVANCE
      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
    }
    // Calculare new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (uint32_t)current_block->accelerate_until) {
      
      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance += advance_rate;
        }
        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
        
      #endif
    } 
    else if (step_events_completed > (uint32_t)current_block->decelerate_after) {   
      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      
      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance -= advance_rate;
        }
        if(advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
      #endif //ADVANCE
    }
    else {
      OCR1A = OCR1A_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }   
  } 
}

#ifdef ADVANCE
void st_advance_interrupt()
  {
    OCR4A = 100 * 16;
	
    // Set E direction (Depends on E direction + advance)
    for(unsigned char i=0; i<4;i++) {
      if (e_steps[0] != 0) {
	stepperInterface[E_AXIS].step(false);
        if (e_steps[0] < 0) {
      	  stepperInterface[E_AXIS].setDirection(false);
          e_steps[0]++;
	  stepperInterface[E_AXIS].step(true);
        } 
        else if (e_steps[0] > 0) {
      	  stepperInterface[E_AXIS].setDirection(true);
          e_steps[0]--;
	  stepperInterface[E_AXIS].step(true);
        }
      }
 #if EXTRUDERS > 1
      if (e_steps[1] != 0) {
	//stepperInterface[?].step(false);
        if (e_steps[1] < 0) {
      	  stepperInterface[?].setDirection(false);
          e_steps[1]++;
	  //stepperInterface[?].step(true);
        } 
        else if (e_steps[1] > 0) {
      	  stepperInterface[?].setDirection(true);
          e_steps[1]--;
	  //stepperInterface[?].step(true);
        }
      }
 #endif
 #if EXTRUDERS > 2
      if (e_steps[2] != 0) {
	//stepperInterface[?].step(false);
        if (e_steps[2] < 0) {
      	  //stepperInterface[?].setDirection(false);
          e_steps[2]++;
	  //stepperInterface[?].step(true);
        } 
        else if (e_steps[2] > 0) {
      	  //stepperInterface[?].setDirection(true);
          e_steps[2]--;
	  //stepperInterface[?].step(true);
        }
      }
 #endif
    }
  }
#endif // ADVANCE

void st_init()
{
  //Grab the stepper interfaces
  stepperInterface = Motherboard::getBoard().getStepperAllInterfaces();

  Motherboard::getBoard().setupAccelStepperTimer();

  #ifdef ADVANCE
    e_steps[0] = 0;
    e_steps[1] = 0;
    e_steps[2] = 0;
  #endif //ADVANCE
}

// Block until all buffered steps are executed
bool st_empty()
{
    if ( blocks_queued() )	return false;
    return true;
}

void st_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const int32_t &e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
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
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

#endif
