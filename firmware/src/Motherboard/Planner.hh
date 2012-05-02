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
#include "Point.hh"

namespace planner {
	// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
	// the source g-code and may never actually be reached if acceleration management is active.
	class Block {
	public:
		typedef enum {
			Busy          = 1<<0,
			Recalculate   = 1<<1,
			NominalLength = 1<<2,
		} PlannerFlags;

		// Fields used by the bresenham algorithm for tracing the line
		Point target;                        // Final 5-axis target
		uint32_t step_event_count;           // The number of step events required to complete this block
		uint32_t accelerate_until;            // The index of the step event on which to stop acceleration
		uint32_t decelerate_after;            // The index of the step event on which to start decelerating
		uint32_t acceleration_rate;           // The acceleration rate used for acceleration calculation
		// uint8_t direction_bits;              // The direction bit set for this block
		// uint8_t active_extruder;             // Selects the active extruder
		

		// Fields used by the motion planner to manage acceleration
		//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/minute for each axis
		float nominal_speed;                               // The nominal speed for this block in mm/min  
		float entry_speed;                                 // Entry speed at previous-current junction in mm/min
		float max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
		float millimeters;                                 // The total travel of this block in mm
		float acceleration;                                // acceleration mm/sec^2
		// float stop_speed;                            // Speed to decelerate to if this is the last move
		// uint8_t recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
		// uint8_t nominal_length_flag;                 // Planner flag for nominal speed always reached

		// Settings for the trapezoid generator
		uint32_t nominal_rate;                        // The nominal step rate for this block in step_events/sec 
		uint32_t initial_rate;                        // The jerk-adjusted step rate at start of block  
		uint32_t final_rate;                          // The minimal rate at exit
		uint32_t acceleration_st;                     // acceleration steps/sec^2
		// uint8_t busy;
		uint8_t flags;
		
		Block() : target() {};
		
	// functions
		void calculate_trapezoid(float exit_factor_speed);
	};

	/// Initilaize the planner data structures
	void init();
	
	/// Buffer a movement to the target point (in step-space), with us_per_step gaps between steps
	/// \param[in] target New position to move to, in step-space
	/// \param[in] us_per_step Homing speed, in us per step
	/// \return If the move was buffered
	bool addMoveToBuffer(const Point& target, int32_t us_per_step);

	/// Buffer a movement to the target point (in step-space). We should avoid this, as it requires more calculation.
	/// \param[in] target New position to move to, in step-space
	/// \param[in] ms Duration of the move, in milliseconds
	/// \param[in] relative Bitfield specifying whether each axis should
	///                     interpret the new position as absolute or
	///                     relative.
	/// \return If the move was buffered
	bool addMoveToBufferRelative(const Point& target, const int32_t ms, const int8_t relative);

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

    /// Abort the current motion (and all planeed movments) and set the stepper subsystem to
    /// the not-running state.
    void abort();

	/// Get the current system position
	/// \return The current machine position.
	const Point getPosition();

	void setMaxXYJerk(float jerk);
	void setMaxAxisJerk(float jerk, uint8_t axis);

	void setMinimumPlannerSpeed(float speed);

	void setAcceleration(int32_t acceleration);
	void setAxisAcceleration(int32_t new_acceleration, uint8_t axis);
#ifdef CENTREPEDAL
	void setJunctionDeviation(float new_junction_deviation);
#endif	
	void setAxisStepsPerMM(float steps_per_mm, uint8_t axis);
	
	bool isBufferFull();
	bool isBufferEmpty();
	
	// Fetches the *tail*
	Block *getNextBlock();
	
	// pushes the tail forward, making it available
	void doneWithNextBlock();
}

#endif /* end of include guard: PLANNER_HH */
