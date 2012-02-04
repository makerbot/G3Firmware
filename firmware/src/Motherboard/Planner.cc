/*
 *   Copyright 2011 by Rob Giseburt http://tinkerin.gs
 *   
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

/*
 *   This is heavily influenced by the Marlin RepRap firmware
 *   (https://github.com/ErikZalm/Marlin) which is derived from
 *   the Grbl firmware (https://github.com/simen/grbl/tree).
 */


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

	vt + (1/2)at^2=X for t
		x + vt + (1/2)at^2=X
		x = destination
		X = position
		v = current speed
		a = acceleration rate
		t = time
		See: http://www.wolframalpha.com/input/?i=vt+%2B+%281%2F2%29at%5E2%3DX+for+t
	
	Solved for time, simplified (with a few multiplications as possible) gives:
	(sqrt(v^2-2*a*(x-X)) - v)/a
	So, making x-X = D gives:
	(sqrt(v*v-2*a*D)-v)/a = time to accelerate from velocity v over D steps with acceleration a
	
*/


#include "Planner.hh"
#include <util/atomic.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memmove and memcpy

#include "Steppers.hh"
#include "Point.hh"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define A_AXIS 3
#define B_AXIS 4

#define  FORCE_INLINE __attribute__((always_inline)) inline

/* Setup some utilities */

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

template <typename T>
inline const T& min(const T& a, const T& b) { return (a)<(b)?(a):(b); }
// template inline const int32_t& min(const int32_t&, const int32_t&);
// template inline const uint32_t& min(const uint32_t&, const uint32_t&);

template <typename T>
inline const T& max(const T& a, const T& b) { return (a)>(b)?(a):(b); }
// template inline const int32_t& max(const int32_t&, const int32_t&);
// template inline const uint32_t& max(const uint32_t&, const uint32_t&);

// template <typename T>
// inline T abs(T x) { return (x)>0?(x):-(x); }

// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// #define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
// #define radians(deg) ((deg)*DEG_TO_RAD)
// #define degrees(rad) ((rad)*RAD_TO_DEG)
// #define sq(x) ((x)*(x))


namespace planner {
	
	
	// Super-simple circular buffer, where old nodes are reused
	// TODO: Move to a seperate file
	// WARNING WARNING WARNING: If the size of this buffer is not in the following list this WILL FAIL BADLY!
	// (2, 4, 8, 16, 32, 64, 128)
	template<typename T>
	class ReusingCircularBufferTempl
	{
	public:
		typedef T BufDataType;
		typedef uint8_t BufSizeType;
		
	private:
		volatile BufSizeType head, tail;
		BufSizeType size;
		BufSizeType size_mask;
		BufDataType* const data; /// Pointer to buffer data
	
	public:
		ReusingCircularBufferTempl(BufSizeType size_in, BufDataType* buffer_in) : head(0), tail(0), size(size_in), size_mask(size_in-1), data(buffer_in) {
			for (BufSizeType i = 0; i < size; i++) {
				data[i] = BufDataType();
			}
		};
		
		inline BufDataType *getHead() {
			return &data[head];
		}
		inline BufSizeType getHeadIndex() {
			return head;
		}
		
		inline BufDataType *getTail() {
			return &data[tail];
		}
		inline BufSizeType getTailIndex() {
			return tail;
		}
		
		inline BufSizeType getNextIndex(BufSizeType from) {
			return ((from + 1) & size_mask);
		}
		
		inline BufSizeType getPreviousIndex(BufSizeType from) {
			return (((from+size) - 1) & size_mask);
		}
		
		inline BufDataType *getNextHead() {
			return &data[getNextIndex(head)];
		}
		
		inline BufDataType &operator[] (BufSizeType index) {
			 // adding size should make negative indexes < size work ok
			// int16_t offset = index < 0 ? index : ((index + size) & size_mask);
			return data[index];
		}
		
		// bump the head with buffer++. cannot return anything useful, so it doesn't
		// WARNING: no sanity checks!
		inline void bumpHead() {
			head = getNextIndex(head);
		}

		// bump the tail with buffer--. cannot return anything useful, so it doesn't
		// WARNING: no sanity checks!
		inline void bumpTail() {
			tail = getNextIndex(tail);
		}
		
		inline bool isEmpty() {
			return head == tail;
		}
		
		inline bool isFull() {
			return getNextIndex(head) == tail;
		}
		
		inline BufSizeType getUsedCount() {
			return ((head-tail+size) & size_mask);
		}
		
		inline void clear() {
			head = 0;
			tail = 0;
		}
	};
	
	// this is very similar to the StepperAxis, but geared toward planning
	struct PlannerAxis
	{
		// how many steps does it take to go a mm (RepG should tell us this during init)
		float steps_per_mm;
		
		// how fast can we go, in mm/s (RepG should have already limited this, disabling)
		// float max_feedrate;
		
		// maximum acceleration for this axis in steps/s^2 (should be in EEPROM)
		uint32_t max_acceleration;
		
		// the maximum amount of speed change allowable for this axis
		// note that X+Y has it's own setting, and this if for all the rest
		float max_axis_jerk;
	};
	
	PlannerAxis axes[AXIS_COUNT];
	
	float default_acceleration;
	float default_junction_deviation;
	Point position; // the current position (planning-wise, not bot/stepper-wise) in steps
	float previous_speed[AXIS_COUNT]; // Speed of previous path line segment
	double previous_unit_vec[3];
	float previous_nominal_speed; // Nominal speed of previous path line segment
	static float max_xy_jerk;
	
	Block block_buffer_data[BLOCK_BUFFER_SIZE];
	ReusingCircularBufferTempl<Block> block_buffer(BLOCK_BUFFER_SIZE, block_buffer_data);
		
	
	void init()
	{
		for (int i = 0; i < AXIS_COUNT; i++) {
			// axes[i] = PlannerAxis(); // redundant, or a reset?
			previous_speed[i] = 0.0;
		}
		
		position = Point(0,0,0,0,0);
		previous_nominal_speed = 0.0;
		
		axes[0].max_acceleration = 900*axes[0].steps_per_mm;
		axes[1].max_acceleration = 900*axes[1].steps_per_mm;
		axes[2].max_acceleration = 200*axes[2].steps_per_mm;
		axes[3].max_acceleration = 20000*axes[3].steps_per_mm;
		axes[4].max_acceleration = 20000*axes[4].steps_per_mm;
		// axes[0].max_acceleration = 500*axes[0].steps_per_mm;
		// axes[1].max_acceleration = 500*axes[1].steps_per_mm;
		// axes[2].max_acceleration = 200*axes[2].steps_per_mm;
		// axes[3].max_acceleration = 20000*axes[3].steps_per_mm;
		// axes[4].max_acceleration = 20000*axes[4].steps_per_mm;
	}

	
	void setMaxAxisJerk(float jerk, uint8_t axis) {
		axes[axis].max_axis_jerk = jerk;
	}
	
	void setMaxXYJerk(float jerk) {
		max_xy_jerk = jerk;
	}
	
	void setAxisStepsPerMM(float steps_per_mm, uint8_t axis) {
		axes[axis].steps_per_mm = steps_per_mm;
	}

	void setAcceleration(float new_acceleration) {
		default_acceleration = new_acceleration;
		// for (int i = 0; i < AXIS_COUNT; i++) {
		// 	axes[i].max_acceleration = default_acceleration * axes[i].steps_per_mm;
		// }
	}

	void setJunctionDeviation(float new_junction_deviation) {
		default_junction_deviation = new_junction_deviation;
	}
	
	// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
	// acceleration within the allotted distance.
	FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
		return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
	}

	// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
	// given acceleration:
	FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
	{
		if (acceleration!=0) {
			return((target_rate*target_rate-initial_rate*initial_rate)/(2.0*acceleration));
		}
		else {
			return 0.0;  // acceleration was 0, set acceleration distance to 0
		}
	}

	// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
	// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
	// a total travel of distance. This can be used to compute the intersection point between acceleration and
	// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

	FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
	{
		if (acceleration!=0) {
			return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4.0*acceleration));
		}
		else {
			return 0.0;  // acceleration was 0, set intersection distance to 0
		}
	}

#if 0	
	// Calculates the time (not distance) in microseconds (S*1,000,000) it takes to go from initial_rate for distance at acceleration rate
	FORCE_INLINE uint32_t estimate_time_to_accelerate(float initial_rate, float acceleration, float distance) {
		/*
		if (acceleration!=0.0 && initial_rate == 0.0) {
					return (sqrt(-2*acceleration*distance)/acceleration) * 1000000;
				} else */
		if (acceleration!=0.0) {
			return abs((sqrt(2*acceleration*distance + initial_rate*initial_rate)-initial_rate)/acceleration) * 1000000;
		}
		else {
			return (distance/initial_rate) * 1000000; // no acceleration is just distance/rate
		}
	}
#endif

	// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
	// calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed, exit_factor_speed/block->nominal_speed);
	void Block::calculate_trapezoid(float exit_factor_speed) {
		float entry_factor = entry_speed/nominal_speed;
		float exit_factor = exit_factor_speed/nominal_speed;
		
		initial_rate = ceil((float)nominal_rate*entry_factor); // (step/min)
		final_rate = ceil((float)nominal_rate*exit_factor); // (step/min)
		
		// Limit minimal step rate (Otherwise the timer will overflow.)
		if(initial_rate < 120)
			initial_rate = 120;
		if(final_rate < 120)
			final_rate = 120;

		int32_t acceleration = acceleration_st;
		int32_t accelerate_steps =
			ceil(estimate_acceleration_distance(initial_rate, nominal_rate, acceleration));
		int32_t decelerate_steps =
			floor(estimate_acceleration_distance(nominal_rate, final_rate, -acceleration));

		// Calculate the size of Plateau of Nominal Rate.
		int32_t plateau_steps = step_event_count-accelerate_steps-decelerate_steps;

		// Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
		// have to use intersection_distance() to calculate when to abort acceleration and start braking
		// in order to reach the final_rate exactly at the end of this block.
		if (plateau_steps < 0) {
			accelerate_steps = ceil(
				intersection_distance(initial_rate, final_rate, acceleration, step_event_count));
			accelerate_steps = max(accelerate_steps, 0L); // Check limits due to numerical round-off
			accelerate_steps = min(accelerate_steps, (int32_t)step_event_count);
			plateau_steps = 0;
		}

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Fill variables used by the stepper in a critical section
			if(busy == false) { // Don't update variables if block is busy.
				accelerate_until = accelerate_steps;
				decelerate_after = accelerate_steps+plateau_steps;
				// initial_rate = initial_rate;
				// final_rate = final_rate;
			} // So, ummm, what if it IS busy?!
		} // ISR state will be automatically restored here
	}
	
	// forward declare, so we can order the code in a slightly more readable fashion
	void planner_reverse_pass_kernel(Block *previous, Block *current, Block *next);
	void planner_reverse_pass();
	void planner_forward_pass_kernel(Block *previous, Block *current, Block *next);
	void planner_forward_pass();
	void planner_recalculate_trapezoids();

	// Recalculates the motion plan according to the following algorithm:
	//
	//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. Block.entry_factor) 
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

	// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
	void planner_reverse_pass_kernel(Block *previous, Block *current, Block *next) {
		if(!current) { return; }

		if (next) {
			// If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
			// If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
			// check for maximum allowable speed reductions to ensure maximum possible planned speed.
			if (current->entry_speed != current->max_entry_speed) {

				// If nominal length true, max junction speed is guaranteed to be reached. Only compute
				// for max allowable speed if block is decelerating and nominal length is false.
				if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
					current->entry_speed = min( current->max_entry_speed,
						max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
				} else {
					current->entry_speed = current->max_entry_speed;
				}
				current->recalculate_flag = true;

			}
		} // Skip last block. Already initialized and set for recalculation.
	}

	// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
	// implements the reverse pass.
	void planner_reverse_pass() {
		if(block_buffer.getUsedCount() > 3) {
			uint8_t block_index = block_buffer.getHeadIndex();
			Block *block[3] = { NULL, NULL, NULL };
			while(block_index != block_buffer.getTailIndex()) { 
				block_index = block_buffer.getPreviousIndex(block_index); 
				block[2] = block[1];
				block[1] = block[0];
				// Move two blocks worth of ram, from [0] to [1], using the overlap-safe memmove
				//memmove(block[0], block[1], sizeof(Block)<<1);
				block[0] = &block_buffer[block_index];
				planner_reverse_pass_kernel(block[0], block[1], block[2]);
			}
			planner_reverse_pass_kernel(NULL, block[0], block[1]);
		}
	}

	// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
	void planner_forward_pass_kernel(Block *previous, Block *current, Block *next) {
		if(!previous) { return; }

		// If the previous block is an acceleration block, but it is not long enough to complete the
		// full speed change within the block, we need to adjust the entry speed accordingly. Entry
		// speeds have already been reset, maximized, and reverse planned by reverse planner.
		// If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
		if (!previous->nominal_length_flag) {
			if (previous->entry_speed < current->entry_speed) {
				double entry_speed = min( current->entry_speed,
					max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

				// Check for junction speed change
				if (current->entry_speed != entry_speed) {
					current->entry_speed = entry_speed;
					current->recalculate_flag = true;
				}
			}
		}
	}

	// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
	// implements the forward pass.
	void planner_forward_pass() {
		uint8_t block_index = block_buffer.getTailIndex();
		Block *block[3] = { NULL, NULL, NULL };

		while(block_index != block_buffer.getHeadIndex()) {
			block[0] = block[1];
			block[1] = block[2];
			// Move two blocks worth of ram, from [1] to [0], using the overlap-safe memmove
			//memmove(block[1], block[0], sizeof(Block)<<1);
			block[2] = &block_buffer[block_index];
			planner_forward_pass_kernel(block[0],block[1],block[2]);
			block_index = block_buffer.getNextIndex(block_index);
		}
		planner_forward_pass_kernel(block[1], block[2], NULL);
	}

	// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
	// entry_factor for each junction. Must be called by planner_recalculate() after 
	// updating the blocks.
	void planner_recalculate_trapezoids() {
		int8_t block_index = block_buffer.getTailIndex();
		Block *current;
		Block *next = NULL;

		while(block_index != block_buffer.getHeadIndex()) {
			current = next;
			next = &block_buffer[block_index];
			if (current) {
				// Recalculate if current block entry or exit junction speed has changed.
				if (current->recalculate_flag || next->recalculate_flag) {
					// NOTE: Entry and exit factors always > 0 by all previous logic operations.
					current->calculate_trapezoid(next->entry_speed);
					current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
				}
			}
			block_index = block_buffer.getNextIndex( block_index );
		}
	// Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
		if(next != NULL) {
			next->calculate_trapezoid(MINIMUM_PLANNER_SPEED);
			next->recalculate_flag = false;
		}
	}

	bool isBufferFull() {
		// return false;
		return block_buffer.isFull();
	}
	
	bool isBufferEmpty() {
		// return false;
		return block_buffer.isEmpty();
	}
	
	Block *getNextBlock() {
		Block *block = block_buffer.getTail();
		return block;
	}
	
	void doneWithNextBlock() {
		block_buffer.bumpTail();
	}
	

	
	// Buffer the move. IOW, add a new block, and recalculate the acceleration accordingly
	bool addMoveToBuffer(const Point& target, int32_t us_per_step)
	{
		if (block_buffer.isFull())
			return false;
		
		Block *block = block_buffer.getHead();
		// Mark block as not busy (Not executed by the stepper interrupt)
		block->busy = false;
		
		block->target = target;

		// // store the absolute number of steps in each direction, without direction
		Point steps = (target - position).abs();

		float delta_mm[AXIS_COUNT];
		block->millimeters = 0.0;
		block->step_event_count = 0;
		// // Compute direction bits for this block -- UNUSED FOR NOW
		// block->direction_bits = 0;
		for (int i = 0; i < AXIS_COUNT; i++) {
			if (steps[i] > block->step_event_count) {
				block->step_event_count = steps[i];
			}
			delta_mm[i] = ((float)steps[i])/axes[i].steps_per_mm;
			if (i < A_AXIS)
				block->millimeters += delta_mm[i] * delta_mm[i];
		// 	if (target[i] < position[i]) { block->direction_bits |= (1<<i); }
		}		
		block->millimeters = sqrt(block->millimeters);
		
		float inverse_millimeters = 1.0/block->millimeters; // Inverse millimeters to remove multiple divides
		// Calculate 1 second/(seconds for this movement)
		float inverse_second = 1000000.0/(float)(us_per_step * block->step_event_count);
		float steps_per_mm = (float)block->step_event_count * inverse_millimeters;
		
		// we are given microseconds/step, and we need steps/mm, and steps/second
		
		// Calculate speed in steps/sec
		uint32_t steps_per_second = 1000000/us_per_step;
		float mm_per_second = block->millimeters * inverse_second;
			  
		// Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
		block->nominal_speed = mm_per_second; // (mm/sec) Always > 0
		block->nominal_rate = steps_per_second; // (step/sec) Always > 0
		
		// TODO make sure we are going the minimum speed, at least
		// if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
		
		float current_speed[AXIS_COUNT];
		for(int i=0; i < AXIS_COUNT; i++) {
			current_speed[i] = delta_mm[i] * inverse_second;
		}

		// // Limit speed per axis (already done in RepG)
		// float speed_factor = 1.0; //factor <=1 do decrease speed
		// for(int i=0; i < AXIS_COUNT; i++) {
		// 	if(fabs(current_speed[i]) > max_feedrate[i])
		// 		speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
		// }
		
		// TODO fancy frequency checks
		
		
		// // Correct the speed  
		// if( speed_factor < 1.0) {
		// 	//    Serial.print("speed factor : "); Serial.println(speed_factor);
		// 	for(int i=0; i < 4; i++) {
		// 		if(fabs(current_speed[i]) > max_feedrate[i])
		// 			speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
		// 		/*     
		// 		if(speed_factor < 0.1) {
		// 			Serial.print("speed factor : "); Serial.println(speed_factor);
		// 			Serial.print("current_speed"); Serial.print(i); Serial.print(" : "); Serial.println(current_speed[i]);
		// 		}
		// 		*/
		// 	}
		// 	for(unsigned char i=0; i < 4; i++) {
		// 		current_speed[i] *= speed_factor;
		// 	}
		// 	block->nominal_speed *= speed_factor;
		// 	block->nominal_rate *= speed_factor;
		// }

		// Compute and limit the acceleration rate for the trapezoid generator.
		block->acceleration_st = ceil(default_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
		// Limit acceleration per axis
		for(int i=0; i < AXIS_COUNT; i++) {
			if((uint32_t)((float)block->acceleration_st * (float)steps[i] / (float)block->step_event_count) > axes[i].max_acceleration)
				block->acceleration_st = axes[i].max_acceleration;
		}
		block->acceleration = block->acceleration_st / steps_per_mm;
		// block->acceleration_rate = (int32_t)((float)block->acceleration_st * 8.388608); //WHOA! Where is this coming from?!?
		
		// Compute the speed trasitions, or "jerks"
		// Start with a safe speed
		float vmax_junction = max_xy_jerk/2.0;
		{
			float half_max_z_axis_jerk = axes[Z_AXIS].max_axis_jerk/2.0;
			if(fabs(current_speed[Z_AXIS]) > half_max_z_axis_jerk) 
				vmax_junction = half_max_z_axis_jerk;
		}

		vmax_junction = min(vmax_junction, block->nominal_speed);
		
		// Determine the stopping speed for this move, in case it's the last one
		// this is basically the same as the entry speed calcs (done next) except this move is
		//  the "previous" one and the "current" speed would be zero.
		float jerk = sqrt(pow((current_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]), 2));
		float stop_vmax_junction = vmax_junction; // we re-use vmax_junction below, so keep it
		if((current_speed[X_AXIS] != 0.0) || (current_speed[Y_AXIS] != 0.0)) {
			stop_vmax_junction = block->nominal_speed;
		}
		if (jerk > max_xy_jerk) {
			stop_vmax_junction *= (max_xy_jerk/jerk);
		}
		for(int i=Z_AXIS; i < AXIS_COUNT; i++) {
			float axis_jerk = fabs(current_speed[i]);
			if(axis_jerk > axes[i].max_axis_jerk) {
				stop_vmax_junction *= (axes[i].max_axis_jerk/axis_jerk);
		block->acceleration_rate = block->acceleration_st / ACCELERATION_TICKS_PER_SECOND;
		
		// Compute path unit vector
		double unit_vec[3];

		unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
		unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
		unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;
		
		// Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
		// Let a circle be tangent to both previous and current path line segments, where the junction 
		// deviation is defined as the distance from the junction to the closest edge of the circle, 
		// colinear with the circle center. The circular segment joining the two paths represents the 
		// path of centripetal acceleration. Solve for max velocity based on max acceleration about the
		// radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
		// path width or max_jerk in the previous grbl version. This approach does not actually deviate 
		// from path, but used as a robust way to compute cornering speeds, as it takes into account the
		// nonlinearities of both the junction angle and junction velocity.
		double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

		// Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
		if ((!block_buffer.isEmpty()) && (previous_nominal_speed > 0.0)) {
			// Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
			// NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
			double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] 
				- previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] 
				- previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;

			// Skip and use default max junction speed for 0 degree acute junction.
			if (cos_theta < 0.95) {
				vmax_junction = min(previous_nominal_speed,block->nominal_speed);
				// Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
				if (cos_theta > -0.95) {
				// Compute maximum junction velocity based on maximum acceleration and junction deviation
					double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
					vmax_junction = min(vmax_junction,
						sqrt(default_acceleration * default_junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
				}
			}
		}
		
		for (int i_axis = A_AXIS; i_axis < B_AXIS; i_axis++) {
			float jerk = abs(previous_speed[i_axis] - current_speed[i_axis]);
			if (jerk > axes[i_axis].max_axis_jerk) {
				vmax_junction *= (axes[i_axis].max_axis_jerk/jerk);
			}
		}
		block->max_entry_speed = vmax_junction;

		// Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
		double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
		block->entry_speed = min(vmax_junction, v_allowable);
		
		// Initialize planner efficiency flags
		// Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
		// If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
		// the current block and next block junction speeds are guaranteed to always be at their maximum
		// junction speeds in deceleration and acceleration, respectively. This is due to how the current
		// block nominal speed limits both the current and next maximum junction speeds. Hence, in both
		// the reverse and forward planners, the corresponding block junction speed will always be at the
		// the maximum junction speed and may always be ignored for any speed reduction checks.
		if (block->nominal_speed <= v_allowable)
			block->nominal_length_flag = true;
		else
			block->nominal_length_flag = false;
		block->recalculate_flag = true; // Always calculate trapezoid for new block

		// Update previous path unit_vector and nominal speed
		memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]

		// Update previous path unit_vector and nominal speed
		memcpy(previous_unit_vec, unit_vec, sizeof(unit_vec)); // previous_unit_vec[] = unit_vec[]
		previous_nominal_speed = block->nominal_speed;

		block->calculate_trapezoid(MINIMUM_PLANNER_SPEED);

		// Update position
		position = target;
		
		// Move buffer head -- should this move to after recalculate?
		block_buffer.bumpHead();

		planner_recalculate();
		
		steppers::startRunning();
		return true;
	}

	void startHoming(const bool maximums,
	                 const uint8_t axes_enabled,
	                 const uint32_t us_per_step)
	{
		// STUB
	}
	
	void abort() {
		steppers::abort();
		position = steppers::getPosition();
		
		// reset speed
		for (int i = 0; i < AXIS_COUNT; i++) {
			previous_speed[i] = 0.0;
		}
		
		block_buffer.clear();

		previous_unit_vec[0]= 0.0;
		previous_unit_vec[1]= 0.0;
		previous_unit_vec[2]= 0.0;
	}
	
	void definePosition(const Point& new_position)
	{
		position = new_position;
		steppers::definePosition(new_position);
		
		// reset speed
		for (int i = 0; i < AXIS_COUNT; i++) {
			previous_speed[i] = 0.0;
		}

		previous_unit_vec[0]= 0.0;
		previous_unit_vec[1]= 0.0;
		previous_unit_vec[2]= 0.0;
	}

	const Point getPosition()
	{
		return position;
	}
}