// Simulator.hh
// Handle porting issues between avr-gcc and gcc

#ifndef SIMULATOR_HH_

#define SIMULATOR_HH

#include <inttypes.h>

// avr-gcc makes double the same as float
#define double float

// Maybe at some point in the future, we'll want to replace these
// with pthread mutices.  That, if it becomes desirable to simulate
// interrupts pulling information out of the pipeline: use a thread
// to simulate an interrupt and have the planner running in the
// primal thread, possibly at a lower thread priority.

#define CRITICAL_SECTION_START  {}
#define CRITICAL_SECTION_END    {}

// Seems like a good idea, eh?
#ifndef HAS_STEPPER_ACCELERATION
#define HAS_STEPPER_ACCELERATION
#endif

#endif
