// Declarations for various routines needed by StepperAccelPlanner.cc
// which we would prefer it not to get from elsewhere

#ifndef STEPPERACCELPLANNEREXTRAS_HH_

#define _STEPPERACCELPLANNEREXTRAS_HH_

#include "Point.hh"
#include "SimulatorRecord.hh"
#include "StepperAccelPlanner.hh"
#include "StepperAccel.hh"

// Debugging switch & bit flags
extern uint32_t simulator_debug;
#define DEBUG_FEEDRATE 0x01

extern bool   simulator_dump_speeds;
extern bool   simulator_use_max_feed_rate;
extern bool   simulator_show_alt_feed_rate;
extern FPTYPE simulator_max_feed_rate;

extern void plan_init_eeprom(void);
extern void st_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e);
extern void st_set_e_position(const int32_t &e);
extern int32_t st_get_position(uint8_t axis);
extern void st_wake_up(void);
extern FPTYPE calcFeedRate(const Point& from, const Point& to, int32_t interval, bool includeEAxis);
extern void setTargetNew(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b, int32_t us, int8_t relative);
extern void setTarget(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b, int32_t dda_interval);
extern void setSegmentAccelState(bool state);
extern void definePosition(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b);
extern void plan_dump(int chart);
extern void plan_dump_current_block(int discard);
extern void plan_dump_run_data(void);

#endif
