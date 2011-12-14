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

/* In this implenmentation, the motor control is handled by steppers, but this code does the planning. */

#ifndef PLANNER_HH
#define PLANNER_HH

#include "Configuration.hh"
#include <stdint.h>
#include "CircularBuffer.hh"
#include "Point.hh"

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 2.0 // (mm/sec)

namespace planner {
	// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
	// the source g-code and may never actually be reached if acceleration management is active.
	typedef struct {
	// Fields used by the bresenham algorithm for tracing the line
		Point steps;  // Step count and direction (may be negative) along each axis
		uint32_t step_event_count;           // The number of step events required to complete this block
		int32_t accelerate_until;                    // The index of the step event on which to stop acceleration
		int32_t decelerate_after;                    // The index of the step event on which to start decelerating
		int32_t acceleration_rate;                   // The acceleration rate used for acceleration calculation
		uint8_t direction_bits;             // The direction bit set for this block
		uint8_t active_extruder;            // Selects the active extruder
	#ifdef ADVANCE
		int32_t advance_rate;
		volatile int32_t initial_advance;
		volatile int32_t final_advance;
		float advance;
	#endif

	// Fields used by the motion planner to manage acceleration
	//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/minute for each axis
		float nominal_speed;                               // The nominal speed for this block in mm/min  
		float entry_speed;                                 // Entry speed at previous-current junction in mm/min
		float max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
		float millimeters;                                 // The total travel of this block in mm
		float acceleration;                                // acceleration mm/sec^2
		uint8_t recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
		uint8_t nominal_length_flag;                 // Planner flag for nominal speed always reached

	// Settings for the trapezoid generator
		uint32_t nominal_rate;                        // The nominal step rate for this block in step_events/sec 
		uint32_t initial_rate;                        // The jerk-adjusted step rate at start of block  
		uint32_t final_rate;                          // The minimal rate at exit
		uint32_t acceleration_st;                     // acceleration steps/sec^2
		uint8_t busy;
		
	// functions
		void calculate_trapezoid(float exit_factor_speed);
	} Block;

	/// Initilaize the planner data structures
	void init();
	
	/// Buffer a movement to the target point (in step-space), with us_per_step gaps between steps
	/// \param[in] target New position to move to, in step-space
	/// \param[in] us_per_step Homing speed, in us per step
	/// \return If the move was buffered
	bool addMoveToBuffer(const Point& target, int32_t us_per_step);

	/// Home one or more axes
	/// \param[in] maximums If true, home in the positive direction
	/// \param[in] axes_enabled Bitfield specifiying which axes to
	///                         home
	/// \param[in] us_per_step Homing speed, in us per step
	void startHoming(const bool maximums,
	                 const uint8_t axes_enabled,
	                 const uint32_t us_per_step);

	/// Reset the current system position to the given point
	/// \param[in] position New system position
	void definePosition(const Point& position);

	/// Get the current system position
	/// \return The current machine position.
	const Point getPosition();

	void setMaxXYJerk(float jerk);
	void setMaxAxisJerk(float jerk, uint8_t axis);

	void setAcceleration(float acceleration);
	
	void setAxisStepsPerMM(float steps_per_mm, uint8_t axis);
	
	// Super-simple circular buffer, where old nodes are reused
	// TODO: Move to a seperate file
	// WARNING WARNING WARNING: If the size of this buffer is not in the following list this WILL FAIL BADLY!
	// (2, 4, 8, 16, 32, 64, 128)
	template<typename T>
	class ReusingCircularBufferTempl
	{
	public:
		typedef T BufDataType;
		
	private:
		volatile int8_t head, tail;
		int8_t size;
		int8_t size_mask;
		BufDataType* const data; /// Pointer to buffer data
	
	public:
		ReusingCircularBufferTempl(int8_t size_in, BufDataType* buffer_in) : head(0), tail(0), size(size), size_mask(size-1), data(buffer_in) {};
		
		inline BufDataType *getHead() {
			return &data[head];
		}
		inline int8_t getHeadIndex() {
			return head;
		}
		
		inline BufDataType *getTail() {
			return &data[tail];
		}
		inline int8_t getTailIndex() {
			return tail;
		}
		
		inline int8_t getNextIndex(int8_t from) {
			return (from + 1) & size_mask;
		}
		
		inline int8_t getPreviousIndex(int8_t from) {
			return (from - 1) & size_mask;
		}
		
		inline BufDataType *getNextHead() {
			return &data[getNextIndex(head)];
		}
		
		inline BufDataType &operator[] (int8_t index) {
			 // adding size should make negative indexes < size work ok
			int8_t offset = (index + head + size) & size_mask;
			return data[offset];
		}
		
		// bump the head with buffer++. cannot return anything useful, so it doesn't
		// WARNING: no sanity checks!
		inline void operator++(int) {
			head = getNextIndex(head);
		}

		// bump the tail with buffer--. cannot return anything useful, so it doesn't
		// WARNING: no sanity checks!
		inline void operator--(int) {
			tail = getNextIndex(tail);
		}
		
		inline bool isEmpty() {
			return head == tail;
		}
		
		inline bool isFull() {
			return getNextIndex(head) == tail;
		}
		
		inline int8_t getUsedCount() {
			return ((head-tail+size) & size_mask);
		}
	};

	// Create a circular buffer of blocks to walk
	extern Block block_buffer_data[];
	extern ReusingCircularBufferTempl<Block> block_buffer;
}

#endif /* end of include guard: PLANNER_HH */
