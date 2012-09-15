// Stepperaccelplannerextrasquer.cc
//
// This module fills two roles
//   1. Provide no-op stubs for routines needed by StepperAccelPlanner, and
//   2. Provide additional utility routines needed by the simulated planner

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "Simulator.hh"
#include "StepperAccelPlanner.hh"
#include "StepperAccelPlannerExtras.hh"
#include "SqrtTable.hh"

// Which master_steps to use when calculating the feed rate
#define MASTER_STEPS planner_master_steps_cfr

//#define INCREMENTAL_TIME

#define min(a,b) (((a)<=(b))?(a):(b))
#define max(a,b) (((a)>=(b))?(a):(b))

uint32_t simulator_debug              = false;
bool     simulator_use_max_feed_rate  = false;
FPTYPE   simulator_max_feed_rate      = 0;
bool     simulator_dump_speeds        = false;
bool     simulator_show_alt_feed_rate = false;

uint32_t z1[10000];
uint32_t z2[10000];
int iz = 0;

// From time to time, StepperAccelPlanner.cc wants these for debugging
volatile float zadvance, zadvance2;

// These values are only needed for the "ADVANCE" feature of the planner
static FPTYPE advanceK = 0;
static FPTYPE advanceK2 = 0;
static FPTYPE noodleDiameter = 0;
//static float axis_steps_per_unit_e = 4.4;

extern float axis_steps_per_unit[NUM_AXIS];
FPTYPE axis_steps_per_unit_inverse[NUM_AXIS];

// Bins for tallying up how many blocks are planned once, twice, thrice, ...
// A block cannot be planned more time than there are blocks in the pipe line
static int planner_counts[BLOCK_BUFFER_SIZE+1];

// Track total time required to print
static float total_time = 0.0;

// Storage for the plan_record() counters
static int record_add    = 0;
static int record_mul    = 0;
static int record_div    = 0;
static int record_sqrt   = 0;
static int record_calc   = 0;
static int record_recalc = 0;

// Show calls to plan_buffer_()
static int show_calls = 0;

// Segment acceleration state
static bool segmentAccelState = true;

// We track the last absolute coordinate seen so that setTargetNew() can
// convert absolute coordinates to the relative coordinates needed by the planner
static Point lastTarget(0, 0, 0, 0, 0);
static Point droppedSegmentsRelative(0, 0, 0, 0, 0);
static int32_t droppedSegmentsUs = 0;

// Stubbs for routines used by StepperAccelPlanner
// While we could #ifdef the usage of these routines away in StepperAccelPlanner
// itself, it's just as easy to make our own versions of them.  That way we save
// some #ifdef clutter from occurring in StepperAccelPlanner

// Note, we draw the line at providing stubb classes such as Motherboard and
// StepperInterface.  For those, we #ifdef the usage away in StepperAccelPlanner

void st_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e) { }
void st_set_e_position(const int32_t &e) { }
int32_t st_get_position(uint8_t axis) { return 0; }
void st_wake_up(void) { }

// Yet another occurrence of this routine

// Given two coordinates and a time interval to traverse that
// distance between those two coordinates, compute the corresponding
// feed rate.  The time interval uses units of microseconds while
// the points are specified in stepper-space.

FPTYPE calcFeedRate(const Point& from, const Point& to, int32_t interval, bool includeEAxis, uint32_t us,
		    int32_t& zsteps)
{
	//Calculate the distance in mm's by:
	//Calculate the delta distances and convert to mm's
	//Then sqr of sum of squares of the deltas for the distance

	//We also calculate at the same time,  planner_master_steps in steps by finding the dominant axis (for all 5)
	//You would think it would be for X/Y/Z only, but they "mistakenly" use all 5 in rep g.
	//so we do the same here for consistancy

        // planner_distance_cfr  -- May or may not include the A & B axes
        // planner_distance      -- Includes ALL axes

#ifdef FIXED
	FPTYPE  master_delta ;      // Initialized later
	uint8_t master_delta_index; // Initialized later
	FPTYPE  master_delta_cfr       = 0;
	uint8_t master_delta_index_cfr = 0;
	float d = 0.0;
	float d_cfr = 0.0;
#endif

	// Handle the X, Y, and Z axes

        bool override_master_steps = false;
	uint32_t planner_master_steps_cfr = 0;
	FPTYPE   planner_distance_cfr     = 0;

	zsteps = 0;

#define MAX_STEPS 0x7000
// For testing on a ToM, try a MAX_STEPS which will trigger at 4 cm of Z axis travel @ 200 steps/mm
//#define MAX_STEPS ( 40 * 200 )

	// Recall that X_AXIS, Y_AXIS, Z_AXIS are the actual indices
	for ( uint8_t i = X_AXIS; i <= Z_AXIS; i++ )
	{
                planner_steps[i] = to[i] - from[i];
		if ( planner_steps[i] != 0 )
		{
		     if ( planner_steps[i] > MAX_STEPS )
		     {
			  if ( i == Z_AXIS )
			  {
			       // We are doing a large travel on a high resolution Z axis
			       // Let's break this move into several smaller moves.  We don't
			       // actually do this "correctly" as we take all the X, Y, and E
			       // steps in the first submove.  Thus this is a hack intended
			       // only for large Z-axis travel situations.

			       // Unexercised Z-axis steps which will remain once we truncate
			       // this move to MAX_STEPS steps

			       zsteps = planner_steps[Z_AXIS] - MAX_STEPS;
			  }
			  planner_steps[i] = MAX_STEPS;
		     }
		     else if ( planner_steps[i] < -(MAX_STEPS) )
		     {
			  // This is so infrequent, that we'll take the float hit
			  // Besides, we're dealing with a value to large for an _Accum
			  if ( i == Z_AXIS )
			  {
			       // We are doing a large travel on a high resolution Z axis
			       // Let's break this move into several smaller moves.  We don't
			       // actually do this "correctly" as we take all the X, Y, and E
			       // steps in the first submove.  Thus this is a hack intended
			       // only for large Z-axis travel situations.

			       // Unexercised Z-axis steps which will remain once we truncate
			       // this move to MAX_STEPS steps

			       zsteps = planner_steps[Z_AXIS] + MAX_STEPS;
			  }
			  planner_steps[i] = -(MAX_STEPS);
		     }

		     delta_mm[i] = FPMULT2(ITOFP(planner_steps[i]), axis_steps_per_unit_inverse[i]);
		     planner_steps[i] = labs(planner_steps[i]);
#ifdef FIXED
		     if ( FPABS(delta_mm[i]) > master_delta_cfr )
		     {
			  master_delta_index_cfr = i;
			  master_delta_cfr       = FPABS(delta_mm[i]);
		     }
#else
		     planner_distance_cfr += FPSQUARE(delta_mm[i]);
#endif
		     if ( (uint32_t)planner_steps[i] > planner_master_steps_cfr )
			  planner_master_steps_cfr = (uint32_t)planner_steps[i];
		     float dd = FPTOF(delta_mm[i]);
		     d_cfr += dd * dd;
		}
		else
		     delta_mm[i] = 0;
	}

	// For an extruder only move, force includeEAxis to be true
	if ( planner_master_steps_cfr == 0 )
	{
	     includeEAxis          = true;
	     override_master_steps = true;
	}

	// Handle the extruder axes
	planner_master_steps = planner_master_steps_cfr;
#ifdef FIXED
	master_delta_index   = master_delta_index_cfr;
	master_delta         = master_delta_cfr;
	d = d_cfr;
#else
	planner_distance     = planner_distance_cfr;
#endif

	for ( uint8_t i = Z_AXIS+1; i < NUM_AXIS; i++ )
	{
	     planner_steps[i] = to[i] - from[i];
	     if ( planner_steps[i] != 0 )
	     {
		  delta_mm[i] = FPMULT2(ITOFP(planner_steps[i]), axis_steps_per_unit_inverse[i]);
		  planner_steps[i] = labs(planner_steps[i]);
#ifdef FIXED
		  if ( FPABS(delta_mm[i]) > master_delta )
		  {
		       master_delta_index = i;
		       master_delta       = FPABS(delta_mm[i]);
		  }
#else
		  planner_distance += FPSQUARE(delta_mm[i]);
#endif
		  if ( (uint32_t)planner_steps[i] > planner_master_steps )
		       planner_master_steps = (uint32_t)planner_steps[i];
		  float dd = FPTOF(delta_mm[i]);
		  d += dd * dd;
	     }
	     else
		  delta_mm[i] = 0;
	}

	// If we forced includeEAxis on (because this is an extruder only move) then
	// also use planner_master_steps for planner_master_steps_cfr (which is zero)

	if ( override_master_steps ) planner_master_steps_cfr = planner_master_steps;

	// planner_distance is now a value between 0 and 3.  We want to know
	//
	//      sqrt(1+planner_distance)*delta_mm[master_delta_index]
	//
	// Our lookup table will give us table[(int)(x * SQRT_TABLE_RESOLUTION)] = sqrt(1 + x), 0 <= x <= 3

	if ( includeEAxis )
	{
	     // All distances include the A & B axes
	     // planner_distance_cfr == planner_distance
	     // Just compute planner_distance and then set planner_distance_cfr = planner_distance
#ifdef FIXED
	     FPTYPE planner_distance_sq = 0;
	     for ( uint8_t i = 0; i < NUM_AXIS; i++ )
	     {
		  if ( (i == master_delta_index) || (delta_mm[i] == 0) ) continue;
		  planner_distance_sq += FPSQUARE(FPDIV(delta_mm[i], master_delta));
	     }

	     uint8_t j = FPTOI(planner_distance_sq << SQRT_TABLE_SHIFT);
	     planner_distance = FPMULT2(sqrt_table[j], master_delta);
#else
	     planner_distance = FPSQRT(planner_distance);
#endif
	     planner_distance_cfr = planner_distance;
	     d_cfr = d;
	}
	else
	{
	     // We want planner_distance_cfr to NOT include the A or B axes (E_AXIS)
#ifdef FIXED
	     FPTYPE planner_distance_sq;

	     // First tackle planner_distance_cfr

	     // The (x,y,z) calc
	     planner_distance_sq = 0;
	     for ( uint8_t i = X_AXIS; i <= Z_AXIS; i++ )
	     {
		  if ( (i == master_delta_index_cfr) || (delta_mm[i] == 0) ) continue;
		  planner_distance_sq += FPSQUARE(FPDIV(delta_mm[i], master_delta_cfr));
	     }

	     uint8_t j = FPTOI(planner_distance_sq << SQRT_TABLE_SHIFT);
	     planner_distance_cfr = FPMULT2(sqrt_table[j], master_delta_cfr);

	     // Now include the A & B axes for planner_distance
	     // If master_index == master_index_cfr then we can use the prior calcs as a starting point
	     // Also master_delta == master_delta_cfr (which is significant when we get the the square root calc

	     if ( master_delta_index == master_delta_index_cfr )
	     {
		  for ( uint8_t i = Z_AXIS+1; i < NUM_AXIS; i++ )
		  {
		       if ( (i == master_delta_index_cfr) || (delta_mm[i] == 0) ) continue;
		       planner_distance_sq += FPSQUARE(FPDIV(delta_mm[i], master_delta_cfr));
		  }
	     }
	     else
	     {
		  // master_index != master_index_cfr
		  // master_delta != master_delta_cfr

		  // This means that the longest distance component was the A or B axis
		  // We need to recompute the sum of the squares with a different normalization

		  planner_distance_sq = 0;
		  for ( uint8_t i = 0; i < NUM_AXIS; i++ ) {
		       if ( (i == master_delta_index) || (delta_mm[i] == 0) ) continue;
		       planner_distance_sq += FPSQUARE(FPDIV(delta_mm[i], master_delta));
		  }
	     }

	     j = FPTOI(planner_distance_sq << SQRT_TABLE_SHIFT);
	     planner_distance = FPMULT2(sqrt_table[j], master_delta);
#else
	     planner_distance_cfr = FPSQRT(planner_distance_cfr);
	     planner_distance     = FPSQRT(planner_distance);
#endif
	}

// It's bad news if we get to here without MASTER_STEPS being defined
#ifndef MASTER_STEPS
#define MASTER_STEPS planner_master_steps_cfr
#endif

#ifdef FIXED
     d = sqrt(d);
     d_cfr = sqrt(d_cfr);

     if ((fabsf(FPTOF(planner_distance) - d)/d) > 0.01)
	  printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): planner_distance = %f; actual distance = %f\n",
		 from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		 FPTOF(planner_distance), d);

     if ((fabsf(FPTOF(planner_distance_cfr) - d_cfr)/d_cfr) > 0.01)
	  printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): planner_distance_cfr = %f; actual distance_cfr = %f\n",
		 from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		 FPTOF(planner_distance_cfr), d_cfr);

     float feed_rate = ((interval != 0) && (MASTER_STEPS != 0)) ?
	  (d_cfr * 1000000.0) / ((float)interval * (float)MASTER_STEPS) : 0;
     FPTYPE result;
     if ( (interval <= 0) || (MASTER_STEPS == 0) ) result = 0;
     // if (interval >= 0 && interval <= 0x7fff && MASTER_STEPS <= 0x7fff)
     else if ( (((uint32_t)interval | (uint32_t)MASTER_STEPS) & 0xffff8000) == 0 ) {

	  // We can convert interval to an _Accum and then shift it entirely to the right of the
	  // fixed decimal point without any loss of precision.   This ammounts to dividing
	  // interval by 2^16.  At the same time, we multiply planner_distance by 1000000 / 2^16,
	  // again with no loss of precision.  Since we've divided both the numerator and denominator
	  // by 2^16, we still get the correct result.

	  // This code case is expected to be the predominant case -- we want it to be fast

	  result = FPDIV(FPMULT2(planner_distance_cfr, KCONSTANT_1000000_LSR_16),
			 FPMULT2(ITOFP((int32_t)MASTER_STEPS), (ITOFP((int32_t)interval)>>16)));
     }
     // else if (interval >= 0 && interval <= 0x7fffff && MASTER_STEPS <= 0x7fffff)
     else if ( (((uint32_t)interval | (uint32_t)MASTER_STEPS) & 0xff800000) == 0 )
     {
	  // interval or MASTER_STEPS (or both) are too large to play the trick
	  // we did above.  Their product may still overflow an _Accum.  So, we shift each
	  // one over until it is <= 0x7fff, keeping track of how many bits we shifted.
	  // Then we do the same trick as above but shift by only >> (16-i) where i is the
	  // number of bits we pre-shifted interval and MASTER_STEPS by.

	  // For this case we do have loss of precision, but it should be pretty minor as these
	  // are large numbers.

	  // This case should be an infrequent case, likely only happening for large rafts or
	  // other very large, slow moves.  As such, the code being a little slower here should
	  // not be too much of a problem as the printer is moving slow which allows the pipeline
	  // planner to be slower as well.

	  // Of course, a machine with LOTS of steps/mm for a given axes might generate large
	  // MASTER_STEPS.

	  uint8_t n = 16;
	  while (interval > 0x7fff)
	  {
	       interval >>= 1;
	       n--;
	  }
	  while (MASTER_STEPS > 0x7fff)
	  {
	       MASTER_STEPS >>= 1;
	       n--;
	  }

	  // Since both interval and MASTER_STEPS were <= 0x7fffff
	  // when we started, we know that n >= 0.  In fact, it's >= 2.
	  // At any rate, we won't do ">> (negative-number)".

	  // So, yes we could have started with interval <= 0xffffff and MASTER_STEPS <= 0xffffff

	  result = FPDIV(FPMULT2(planner_distance_cfr, KCONSTANT_1000000_LSR_16),
			 FPMULT2(ITOFP((int32_t)MASTER_STEPS), (ITOFP((int32_t)interval)>>n)));
     }
     else
     {
	  // interval or MASTER_STEPS or both are really large numbers or interval < 0
	  // Just go ahead and pay the penalty of conversion to and from floats

	  // This case is not expected to happen in practice

	  result = FTOFP(((FPTOF(planner_distance_cfr) * 1000000.0) /
			  ((float)MASTER_STEPS * (float)interval)));

	  printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): FLOAT: using floating point math; "
		 "interval=%d, master_steps=%u\n",
		 from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		 interval, MASTER_STEPS);
     }
     if ((us != 0) && simulator_show_alt_feed_rate)
     {
	  float alt_feed_rate = d_cfr*1000.0*1000.0/(float)us;
	  if ((alt_feed_rate != 0.0) && ((fabsf(feed_rate - alt_feed_rate)/alt_feed_rate) > 0.01)) 
	       printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): us=%u, d(xyz)=%f, alt feed rate=%f, "
		      "float feed rate=%f\n",
		      from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		      us, d_cfr, alt_feed_rate, feed_rate);
     }
     if (simulator_debug & DEBUG_FEEDRATE)
	  printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): DEBUG: master_steps=%u, fixed d=%f, float d=%f, "
		 "fixed feed rate=%f, float feed rate=%f\n",
		 from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		 MASTER_STEPS, FPTOF(planner_distance), d, FPTOF(result), feed_rate);
     else if ((feed_rate != 0.0) && ((fabsf(feed_rate - FPTOF(result))/feed_rate) > 0.01))
	  // Different output from the above so as to not cause differences with old, saved output files
	  // which we may perform "diff's against
	  printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): error > 1%%: interval=%d, master_steps=%u, "
		 "fixed feed rate=%f; float feed rate=%f\n",
		 from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		 interval, MASTER_STEPS, FPTOF(result), feed_rate);
     if ( simulator_use_max_feed_rate && result > simulator_max_feed_rate )
	  printf("*** calcFeedRate([%d,%d,%d,%d],[%d,%d,%d,%d],%d,%d): rate > %4.1f: master_steps=%u, d=%f, d_cfr=%f, "
		 "fixed feed rate=%f; float feed rate=%f\n",
		 from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3], interval, includeEAxis ? 1 : 0,
		 FPTOF(simulator_max_feed_rate), MASTER_STEPS, d, d_cfr, FPTOF(result), feed_rate);

     return result;
#else
     planner_distance = FPSQRT(planner_distance);
     return ((planner_distance * 1000000.0) / ((FPTYPE)interval * (FPTYPE)MASTER_STEPS));
#endif
}


// And yet another occurrence of this routine

// The x, y, z, and e arguments are from a .s3g file or frame
// and are expressed in stepper space.  Whether they are absolute
// or relative coordinates is denoted by bits within the "relative"
// call argument.  The "us" argument specifies how many microseconds
// to take in travelling to (x,y,z,e) from the prior position.

void setTargetNew(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b, int32_t us, int8_t relative)
{
     Point target(x, y, z, a, b);
     Point newPosition = target;

     for (int i = 0; i < AXIS_COUNT; i++)
     {
	  if ((relative & (1 << i)) != 0)
	  {
	       newPosition[i] = lastTarget[i] + target[i];

	       //Added on any relative move we have saved from a previous dropped segments
	       //Only if we're relative, if we're absolute we don't care
	       newPosition[i] += droppedSegmentsRelative[i];
	  }
     }
     us += droppedSegmentsUs;

     int32_t max_delta = 0;
     for (int i = 0; i < AXIS_COUNT; i++)
     {
	  // Do not include extruder steps in max_delta unless this is an extruder-only move
	  // More correct code would allow for two extruders.  However, it's not as simple
	  // as "i >= E_AXIS" as that would only pick up the first extruder e-steps and not
	  // the larger of the two extruder e-steps.
	  if (i >= E_AXIS && max_delta != 0) continue;
	  int32_t delta = newPosition[i] - lastTarget[i];
	  if ( delta < 0 ) delta *= -1;
	  if ( delta > max_delta )
	       max_delta = delta;
     }

     if (max_delta != 0)
     {
	  int32_t dda_interval = us / max_delta;
	  int32_t zsteps;
	  do
	  {
	       int32_t ztarget;
	       FPTYPE feedRate = calcFeedRate(lastTarget, newPosition, dda_interval, false, us, zsteps);
	       if (show_calls)
		    printf("plan_buffer_line(%4d, %4d, %4d, %4d, %7.2f, 0)\n",
			   newPosition[0], newPosition[1], newPosition[2],
			   newPosition[3], FPTOF(feedRate));
	       if ( zsteps != 0 )
	       {
		    // In this case, plan_buffer_line() is assured to return True
		    // since the Z-axis motion exceeds 0x7000 steps
		    ztarget = newPosition[2] - zsteps;
	       }
	       else
		    ztarget = newPosition[2];
	       if (plan_buffer_line(newPosition[0], newPosition[1], ztarget,
				    newPosition[3], feedRate, 0, segmentAccelState))
	       {
		    lastTarget = newPosition;
		    droppedSegmentsRelative = Point(0, 0, 0, 0, 0);
		    droppedSegmentsUs = 0;
		    if ( zsteps != 0 )
			 // Actually it's okay to always do this step; however, it may be slower to always do it
			 // so we only do it when r.zsteps != 0
			 lastTarget[2] = ztarget;
	       }
	       else
	       {
		    //Because we've dropped segments, we need to add on a "relative" version
		    //of the dropped move onto droppedSegmentsRelative for possible later use, if we
		    //do another relative move for that axis, otherwise relative moves can be lost.
		    //Relative moves are added to the existing value, absolute moves have lastTarget
		    //subtracted.
		    //All information is stored in "relative" co-ords
#if 0
		    printf("*** setTargetNew(): plan_buffer_line(%4d, %4d, %4d, %4d, %7.2f, 0) "
			   "dropped segment\n",
			   newPosition[0], newPosition[1], newPosition[2],
			   newPosition[3], FPTOF(feedRate));
#endif
		    for (uint8_t i = 0; i < AXIS_COUNT; i ++)
		    {
			 if ((relative & (1 << i)) != 0)
			      droppedSegmentsRelative[i] += target[i];
			 else
			      droppedSegmentsRelative[i]  = target[i] - lastTarget[i];
		    }
		    droppedSegmentsUs += us;
	       }
	  } while ( zsteps != 0 );
     }
}

void setTarget(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b, int32_t dda_interval)
{
     Point target(x, y, z, a, b);
     int32_t zsteps, ztarget;

     do
     {
	  FPTYPE feedRate = calcFeedRate(lastTarget, target, dda_interval, true, 0, zsteps);
	  if ( zsteps != 0 )
	  {
	       // In this case, plan_buffer_line() is assured to return True
	       // since the Z-axis motion exceeds 0x7000 steps
	       ztarget = target[2] - zsteps;
	  }
	  else
	       ztarget = target[2];
	  if (show_calls)
	       printf("plan_buffer_line(%4d, %4d, %4d, %4d, %7.2f, 0)\n",
		      target[0], target[1], ztarget, target[3], FPTOF(feedRate));
	  if (plan_buffer_line(target[0], target[1], ztarget, target[3], feedRate, 0, segmentAccelState))
	       lastTarget = target;
	  if ( zsteps != 0 )
	       lastTarget[2] = ztarget;
     } while ( zsteps != 0 );
}

void definePosition(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b)
{
     Point target(x, y, z, a, 0);
     if (show_calls)
	  printf("plan_set_position(%4d, %4d, %4d, %4d, %4d)\n", x, y, z, a, b);
     plan_set_position(x, y, z, a);
     lastTarget = target;
}

void setSegmentAccelState(bool state)
{
     segmentAccelState = state;
}


// Set acceleration parameters used by the planner and normally
// stored in EEPROM.  These values should probably be moved to
// a header file at which time the Makefile should learn about
// header file dependencies.

// This routine also calls plan_init()

void plan_init_eeprom(void)
{
     int i;

     max_feedrate[X_AXIS] = FTOFP((float)160.0);
     max_feedrate[Y_AXIS] = FTOFP((float)160.0);
     max_feedrate[Z_AXIS] = FTOFP((float)16.0);
     max_feedrate[E_AXIS] = FTOFP((float)100.0);

     axis_steps_per_unit[X_AXIS] =  47.069852;
     axis_steps_per_unit[Y_AXIS] =  47.069852;
     axis_steps_per_unit[Z_AXIS] = 200.0;
     axis_steps_per_unit[E_AXIS] =   4.4;

     max_acceleration_units_per_sq_second[X_AXIS] =   500;
     max_acceleration_units_per_sq_second[Y_AXIS] =   500;
     max_acceleration_units_per_sq_second[Z_AXIS] =   150;
     max_acceleration_units_per_sq_second[E_AXIS] = 60000;

     for (i = 0; i < 5; i++)
     {
	  axis_steps_per_unit_inverse[i] = FTOFP(1.0 / axis_steps_per_unit[i]);
	  axis_steps_per_sqr_second[i]   = (uint32_t)((float)max_acceleration_units_per_sq_second[i]
						      * axis_steps_per_unit[i]);
     }

     p_acceleration           = FTOFP((float)2000.0);
     p_retract_acceleration   = FTOFP((float)4000.0);

     minimumfeedrate          = FTOFP((float)0.0);
     mintravelfeedrate        = FTOFP((float)0.0);

     minimumPlannerSpeed      = FTOFP((float)2.0);

     advanceK                 = FTOFP((float)0.00850);
#ifdef JKN_ADVANCE
     advanceK2                = FTOFP((float)0.00900);
#endif

     noodleDiameter             = FTOFP((float)0.58);
     minimumSegmentTime         = FTOFP((float)0.02);
     extruder_only_max_feedrate = FTOFP(200.0);

     // extruder_deprime_steps   = (int32_t)(axis_steps_per_unit[E_AXIS] * 0.0);
     slowdown_limit           = 4;
     // clockwise_extruder       = 1;

     max_speed_change[X_AXIS] = FTOFP((float)30.0);
     max_speed_change[Y_AXIS] = FTOFP((float)30.0);
     max_speed_change[Z_AXIS] = FTOFP((float)10.0);
     max_speed_change[E_AXIS] = FTOFP((float)30.0);

#ifdef YET_ANOTHER_JERK
     smallest_max_speed_change = max_speed_change[Z_AXIS];
     for (i = 0; i < NUM_AXIS; i++)
	  if (max_speed_change[i] < smallest_max_speed_change)
	       smallest_max_speed_change = max_speed_change[i];
#endif

     plan_init(advanceK, advanceK2, 0);

     lastTarget = Point(0, 0, 0, 0, 0);
     droppedSegmentsRelative = Point(0, 0, 0, 0, 0);
     droppedSegmentsUs = 0;

     memset(planner_counts, 0, sizeof(planner_counts));
     total_time = 0.0;

     setSegmentAccelState(true);
}


void plan_record(void *ctx, int item_code, ...)
{
     va_list ap;

     (void)ctx;

     va_start(ap, item_code);
     while (item_code != 0)
     {
	  switch(item_code)
	  {
	  case RECORD_CALC:
	       record_calc += va_arg(ap, int);
	       break;

	  case RECORD_ADD:
	       record_add += va_arg(ap, int);
	       break;

	  case RECORD_MUL:
	       record_mul += va_arg(ap, int);
	       break;

	  case RECORD_DIV:
	       record_div += va_arg(ap, int);
	       break;

	  case RECORD_SQRT:
	       record_sqrt += va_arg(ap, int);
	       break;

	  case RECORD_RECALC:
	       record_recalc += va_arg(ap, int);
	       break;

	  default :
	       goto badness;
	  }
	  item_code = va_arg(ap, int);
     }
badness:
     va_end(ap);
}

extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail;           // Index of the block to process now

#define MAX_STEP_FREQUENCY 40000
static uint16_t calc_timer(uint16_t step_rate, int *step_loops)
{
     if (step_rate > MAX_STEP_FREQUENCY)
	  step_rate = MAX_STEP_FREQUENCY;
  
     if (step_rate > 20000)
     {
          // If steprate > 20kHz >> step 4 times
	  step_rate = (step_rate >> 2)&0x3fff;
	  *step_loops = 4;
     }
     else if(step_rate > 10000)
     {
          // If steprate > 10kHz >> step 2 times
	  step_rate = (step_rate >> 1)&0x7fff;
	  *step_loops = 2;
     }
     else
	  *step_loops = 1;

     if (step_rate < 32)
	  step_rate = 32;

     return (uint16_t)((uint32_t)2000000 / (uint32_t)step_rate);
}

void plan_dump_current_block(int discard)
{
     int32_t acceleration_time, coast_time, deceleration_time;
     uint16_t acc_step_rate, dec_step_rate, intermed;
     char action[5];
     block_t *block;
     int count_direction[NUM_AXIS], step_loops;
     uint8_t out_bits;
     uint32_t step_events_completed;
     static int i = 0;
     static float z_height = 10.0;  // figure z-offset is around 10
#ifdef INCREMENTAL_TIME
     uint32_t acc_step_rate_32;
     uint16_t acc_step_rate_rem, dec_step_rate_rem;
     uint16_t atimer, dtimer;
#else
     uint16_t timer;
#endif

     block = plan_get_current_block();
     if (!block)
	  return;

     if (block->acceleration_rate == 0)
	  return;

     action[0] = (block->steps[X_AXIS] != 0) ?
	  (((uint32_t)(0x7fffffff & block->steps[X_AXIS]) == block->step_event_count) ? 'X' : 'x') : ' ';
     action[1] = (block->steps[Y_AXIS] != 0) ?
	  (((uint32_t)(0x7fffffff & block->steps[Y_AXIS]) == block->step_event_count) ? 'Y' : 'y') : ' ';
     action[2] = (block->steps[Z_AXIS] != 0) ?
	  (((uint32_t)(0x7fffffff & block->steps[Z_AXIS]) == block->step_event_count) ? 'Z' : 'z') : ' ';
     action[3] = (block->steps[E_AXIS] != 0) ?
	  (((uint32_t)(0x7fffffff & block->steps[E_AXIS]) == block->step_event_count) ? 'E' : 'e') : ' ';
     action[4] = '\0';

#ifndef INCREMENTAL_TIME
     acc_step_rate     = block->initial_rate;
     acceleration_time = calc_timer(acc_step_rate, &step_loops);

     dec_step_rate     = 0;
     deceleration_time = 0;

     coast_time        = 0;
     step_events_completed = 0;
     intermed          = 0;

     for (step_events_completed = step_loops;
	  step_events_completed <= block->step_event_count; )
     {
	  if (step_events_completed <= (uint32_t)(0x7fffffff & block->accelerate_until))
	  {
	       // speed(t) = speed(0) + acceleration * t
	       step_events_completed += step_loops;
	       uint16_t old_acc_step_rate = acc_step_rate;
	       uint16_t intermed_a;
	       acc_step_rate = intermed_a =
		    (uint16_t)((0xffffffffff & ((uint64_t)(0x00ffffff & acceleration_time) * 
						(uint64_t)(0x00ffffff & block->acceleration_rate))) >> 24);
	       acc_step_rate += block->initial_rate;
	       if (acc_step_rate < old_acc_step_rate)
		    printf("*** While accelerating, the step rate overflowed: "
			   "acc_step_rate = %u = %u + %u = %u + 0x%x * 0x%x\n",
			   acc_step_rate, block->initial_rate,
			   intermed_a, block->initial_rate,
			   block->acceleration_rate,
			   acceleration_time);
	       if (acc_step_rate > block->nominal_rate)
		    acc_step_rate = block->nominal_rate;
	       acceleration_time += timer = calc_timer(acc_step_rate, &step_loops);
	       dec_step_rate = acc_step_rate;
	  }
	  else if (step_events_completed > (uint32_t)(0x7fffffff & block->decelerate_after))
	  {
	       // speed(t) = speed(0) - deceleration * t
	       step_events_completed += step_loops;
	       uint16_t old_intermed = intermed;
	       intermed =
		    (uint16_t)((0xffffffffff & ((uint64_t)(0x00ffffff & deceleration_time) * 
						(uint64_t)(0x00ffffff & block->acceleration_rate))) >> 24);
	       if (intermed > acc_step_rate)
		    dec_step_rate = block->final_rate;
	       else
		    dec_step_rate = acc_step_rate - intermed;
	       if (dec_step_rate < block->final_rate)
		    dec_step_rate = block->final_rate;
	       if (intermed < old_intermed)
		    printf("*** While decelerating, the step rate overflowed: "
			   "%u = %u - %u = %u - 0x%x * 0x%x\n",
			   dec_step_rate, acc_step_rate, intermed,
			   acc_step_rate, block->acceleration_rate,
			   deceleration_time);
	       deceleration_time += calc_timer(dec_step_rate, &step_loops);
	  }
	  else
	  {
	       // Must make this call as it has side effects
	       step_events_completed += step_loops;
	       coast_time += calc_timer(acc_step_rate, &step_loops);
	       dec_step_rate = acc_step_rate;
	  }
     }

#else

     acc_step_rate     = block->initial_rate;
     acceleration_time = atimer = calc_timer(acc_step_rate, &step_loops);
     acc_step_rate_rem = 0;

     deceleration_time = dtimer = 0;
     dec_step_rate     = 0;
     dec_step_rate_rem = 0;

     coast_time = 0;

     for (step_events_completed = step_loops;
	  step_events_completed <= block->step_event_count; )
     {
	  if (step_events_completed <= (uint32_t)(0x7fffffff & block->accelerate_until))
	  {
	       step_events_completed += step_loops;

	       // speed(t) = speed(0) + acceleration * t
	       uint16_t old_acc_step_rate = acc_step_rate;
	       acc_step_rate_rem += (uint32_t)(((uint64_t)atimer * (uint64_t)(0xffffff & block->acceleration_rate)) >> 16);
	       acc_step_rate     += acc_step_rate_rem >> 8;
	       acc_step_rate_rem &= 0xff;
	       if (acc_step_rate < old_acc_step_rate)
		    printf("*** While accelerating, the step rate overflowed: "
			   "acc_step_rate = %u; acc_step_rate_rem = %u\n",
			   acc_step_rate, acc_step_rate_rem);
	       if (acc_step_rate > block->nominal_rate)
		    acc_step_rate = block->nominal_rate;
	       atimer = calc_timer(acc_step_rate, &step_loops);
	       acceleration_time += atimer;
	       dec_step_rate = acc_step_rate;
	  }
	  else if (step_events_completed > (uint32_t)(0x7fffffff & block->decelerate_after))
	  {
	       // speed(t) = speed(0) - deceleration * t
	       step_events_completed += step_loops;
	       uint16_t old_dec_step_rate = dec_step_rate;
	       dec_step_rate_rem += (uint32_t)(((uint64_t)dtimer * (uint64_t)(0xffffff & block->acceleration_rate)) >> 16);
	       uint16_t foo = dec_step_rate_rem >> 8;
	       if (dec_step_rate > foo)
	       {
		    dec_step_rate -= foo;
		    if (dec_step_rate > old_dec_step_rate)
			 printf("*** While decelerating, the step rate overflowed: "
				"dec_step_rate = %u; dec_step_rate_rem = %u\n",
				dec_step_rate, dec_step_rate_rem);

		    if (dec_step_rate < block->final_rate)
			 dec_step_rate = block->final_rate;
	       }
	       else
		    dec_step_rate = block->final_rate;
	       dec_step_rate_rem &= 0xff;
	       dtimer = calc_timer(dec_step_rate, &step_loops);
	       deceleration_time += dtimer;
	  }
	  else
	  {
	       // Must make this call as it has side effects
	       step_events_completed += step_loops;
	       coast_time += calc_timer(acc_step_rate, &step_loops);
	  }
     }

#endif

     if (block->message[0] != '\0')
	  printf("\n%s\n", block->message);

     out_bits = block->direction_bits;
     count_direction[X_AXIS] = ((out_bits & (1<<X_AXIS)) != 0) ? -1 : 1;
     count_direction[Y_AXIS] = ((out_bits & (1<<Y_AXIS)) != 0) ? -1 : 1;
     count_direction[Z_AXIS] = ((out_bits & (1<<Z_AXIS)) != 0) ? -1 : 1;
     count_direction[E_AXIS] = ((out_bits & (1<<E_AXIS)) != 0) ? -1 : 1;
     if (block->steps[Z_AXIS] != 0)
     {
	  float delta_z = block->steps[Z_AXIS] * FPTOF(axis_steps_per_unit_inverse[Z_AXIS]);
	  z_height += count_direction[Z_AXIS] * delta_z;
	  z1[iz] = block->initial_rate;
	  z2[iz] = acc_step_rate;
	  iz++;
     }

     i++;
     if (simulator_dump_speeds)
     {
	  float total_time = (float)(acceleration_time + coast_time + deceleration_time /*- last_time */) / 2000000.0;
	  float speed_xyze = FPTOF(block->millimeters)/total_time;
	  float dx = (float)block->steps[X_AXIS]/axis_steps_per_unit[0];
	  float dy = (float)block->steps[Y_AXIS]/axis_steps_per_unit[1];
	  float dz = (float)block->steps[Z_AXIS]/axis_steps_per_unit[2];
	  float speed_xyz = sqrt(dx*dx+dy*dy+dz*dz) / total_time;

	  printf("%d %s: z=%4.1f entry=%5d, peak=%5d, final=%5d steps/s; planned=%d; "
		 "feed_rate=%6.2f mm/s; xyze-dist/t=%6.2f, xyz-dist/t=%6.2f mm/s\n",
		 i, action, z_height, block->initial_rate, acc_step_rate, dec_step_rate,
		 block->planned, FPTOF(block->feed_rate), speed_xyze, speed_xyz);
     }
     else
	  printf("%d %s: z=%4.1f entry=%5d, peak=%5d, final=%5d steps/s; planned=%d; "
		 "feed_rate=%6.2f mm/s (x/y/z/e=%d/%d/%d/%d)\n",
		 i, action, z_height, block->initial_rate, acc_step_rate,
		 dec_step_rate, block->planned, FPTOF(block->feed_rate),
		 count_direction[X_AXIS]*block->steps[X_AXIS], count_direction[Y_AXIS]*block->steps[Y_AXIS],
		 count_direction[Z_AXIS]*block->steps[Z_AXIS], count_direction[E_AXIS]*block->steps[E_AXIS]);

     planner_counts[max(0, min(block->planned, BLOCK_BUFFER_SIZE))] += 1;
     total_time += (float)(acceleration_time + coast_time + deceleration_time) / 2000000.0;

     if (block->message[0] != '\0')
	  printf("\n");

     if (discard)
	  plan_discard_current_block();
}

void plan_dump_run_data(void)
{
     int cnt, i, ihours, imins, isecs, idsecs;
     float ttime = total_time;
     uint32_t ztot1, zavg_min1, zavg_max1;
     uint32_t ztot2, zavg_min2, zavg_max2;
     float zavg1, zavg2;

     ihours = (int)(ttime / (60.0 * 60.0));
     ttime -= (float)(ihours * 60 * 60);
     imins = (int)(ttime / 60.0);
     ttime -= (float)(imins * 60);
     isecs = (int)ttime;
     ttime -= (float)isecs;
     idsecs = (int)(0.5 + ttime * 100.0);
     printf("Total time print time is %02d:%02d:%02d.%02d (%f seconds)\n",
	    ihours, imins, isecs, idsecs, total_time);

     printf("Planner counts:\n");
     for (i = 0; i <= BLOCK_BUFFER_SIZE; i++)
	  if (planner_counts[i])
	       printf("    %d: %d\n", i, planner_counts[i]);

     memset(planner_counts, 0, sizeof(planner_counts));

     ztot1 = 0.0;
     ztot2 = 0.0;
     zavg_min1 = z1[2];
     zavg_max1 = z1[2];
     zavg_min2 = z2[2];
     zavg_max2 = z2[2];
     cnt = 0;
     for (i = 2; i < (iz - 2); i++)
     {
	  if (z1[i] < 1000) continue;
	  cnt++;
	  if (z1[i] < zavg_min1) zavg_min1 = z1[i];
	  if (z1[i] > zavg_max1) zavg_max1 = z1[i];
	  if (z2[i] < zavg_min2) zavg_min2 = z2[i];
	  if (z2[i] > zavg_max2) zavg_max2 = z2[i];
	  ztot1 += z1[i];
	  ztot2 += z2[i];
     }
     zavg1 = (float)ztot1 / (float)cnt;
     zavg2 = (float)ztot2 / (float)cnt;
     printf("Min/Max/Average entry speed = %d / %d / %f steps/s\n",
	    zavg_min1, zavg_max1, zavg1);
     printf("Min/Max/Average peak speed = %d / %d / %f steps/s\n",
	    zavg_min2, zavg_max2, zavg2);
}

#if 0
void plan_dump_current_block(int discard)
{
     block_t *block;
     int counter_x, counter_y, counter_z, counter_e, counter_enew, n,
	  pos, steps_x, steps_y, steps_z, steps_e, steps_enew,
	  step_events_completed, total_steps[5];
     int extra_e;
     static int i = 0;
#define BUFSIZE 1024
     char *x, xbuf[BUFSIZE+1], *y, ybuf[BUFSIZE+1], *e, ebuf[BUFSIZE+1];

#if 0
     printf("vmax calculations = %d\n"
	    "   add/subtracts  = %d\n"
	    "   multiplies     = %d\n"
	    "   divides        = %d\n"
	    "   square roots   = %d\n"
	    "recalculations    = %d\n",
	    record_calc, record_add, record_mul, record_div, record_sqrt,
	    record_recalc);
#endif

     block = plan_get_current_block();
     if (!block)
	  return;

     if ((block->step_event_count + 3) > BUFSIZE)
     {
	  if (NULL == (x = (char *)malloc(block->step_event_count+3+1))) goto vm_error;
	  if (NULL == (y = (char *)malloc(block->step_event_count+3+1))) goto vm_error;
	  if (NULL == (e = (char *)malloc(block->step_event_count+3+1))) goto vm_error;
     }
     else
     {
	  x = xbuf;
	  y = ybuf;
	  e = ebuf;
     }

     memset(x, ' ', block->step_event_count + 3);
     memset(y, ' ', block->step_event_count + 3);
     memset(e, ' ', block->step_event_count + 3);

     total_steps[0] = 0;
     total_steps[1] = 0;
     total_steps[2] = 0;
     total_steps[3] = 0;
     total_steps[4] = 0;

     pos = 0;
     step_events_completed = 0;

     extra_e = 0;
     if (block->steps[E_AXIS] != 0 && block->steps[E_AXIS] != block->step_event_count)
     {
	  int scale = (int)(40.0 * (1.0 - block->max_entry_speed / block->feed_rate));
	  if (scale > 2)
	       extra_e = (int)(((block->step_event_count >> 1) * scale)/10);
     }


     counter_x    = -(block->step_event_count >> 1);
     counter_y    = counter_x;
     counter_z    = counter_x;
     counter_e    = counter_x;
     counter_enew = counter_x - extra_e;

     steps_x    = 0;
     steps_y    = 0;
     steps_z    = 0;
     steps_e    = 0;
     steps_enew = 0;

     for(;; pos++)
     {

	  counter_x += block->steps[X_AXIS];
	  if (counter_x > 0)
	  {
	       steps_x++;
	       total_steps[0]++;
	       x[pos] = 'x';
	       counter_x -= block->step_event_count;
	  }

	  counter_y += block->steps[Y_AXIS];
	  if (counter_y > 0)
	  {
	       steps_y++;
	       total_steps[1]++;
	       y[pos] = 'y';
	       counter_y -= block->step_event_count;
	  }
  
	  counter_z += block->steps[Z_AXIS];
	  if (counter_z > 0)
	  {
	       steps_z++;
	       total_steps[2]++;
	       counter_z -= block->step_event_count;
	  }

	  counter_e += block->steps[E_AXIS];
	  if (counter_e > 0)
	  {
	       steps_e++;
	       total_steps[3]++;
	       e[pos] = 'e';
	       counter_e -= block->step_event_count;
	  }

	  counter_enew += block->steps[E_AXIS];
	  if (step_events_completed == block->accelerate_until)
	       counter_enew += extra_e;
	  else if (step_events_completed == block->decelerate_after)
	       counter_enew += extra_e;
	  if (counter_enew > 0)
	  {
	       steps_enew++;
	       total_steps[4]++;
	       e[pos] = (e[pos] == ' ') ? 'E' : '*';
	       counter_enew -= block->step_event_count;
	  }

	  if (step_events_completed == block->accelerate_until)
	  {
	       x[++pos] = '|';
	       y[pos] = '|';
	       e[pos] = '|';
	  }
	  step_events_completed += 1;
	  if (step_events_completed == block->decelerate_after)
	  {
	       x[++pos] = '|';
	       y[pos] = '|';
	       e[pos] = '|';
	  }
	  if (step_events_completed >= block->step_event_count)
	       break;
     }

     for (char *p = x + pos; (*p == ' ' || *p == '|') && p != x; )
	  *p-- = '\0';
     for (char *p = y + pos; (*p == ' ' || *p == '|') && p != y; )
	  *p-- = '\0';
     for (char *p = e + pos; (*p == ' ' || *p == '|') && p != e; )
	  *p-- = '\0';

     x[++pos] = '\0';
     y[pos]   = '\0';
     e[pos]   = '\0';

     i++;

     if (steps_e != steps_enew || true)
	  printf("\n%d: %d %d %d %d %d %5.2f\n"
		 "%s\n%s\n%s\n",
		 i, steps_x, steps_y, steps_z, steps_e, steps_enew,
		 block->max_entry_speed/FPTOF(block->feed_rate), x, y, e);

next:
     if (discard)
	  plan_discard_current_block();

     goto done;

vm_error:
     fprintf(stderr, "Insufficient virtual memory\n");

done:
     if (x != NULL && x != xbuf) free(x);
     if (y != NULL && y != ybuf) free(y);
     if (e != NULL && e != ebuf) free(e);

     return;
}
#endif
