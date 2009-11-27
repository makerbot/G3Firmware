#ifndef VARIABLES_H
#define VARIABLES_H

#include "Configuration.h"
#include "Datatypes.h"
#include "WProgram.h"
#include "Heater.h"

#if HAS_I2C_LCD 
    #include "I2cLCD.h"
#else // We have to save memory in case of HAS_I2C_LCD
    #include <Servo.h>
#endif

//this is the version of our host software
extern unsigned int master_version;

//are we paused?
extern bool is_tool_paused;

// Two Temp Zones
extern Heater extruder_heater;
#if HAS_HEATED_BUILD_PLATFORM
extern Heater platform_heater;
#endif

extern int heater_low;
extern int heater_high;

extern MotorControlStyle motor1_control;
extern volatile MotorControlDirection motor1_dir;
extern volatile byte motor1_pwm;
extern volatile long motor1_target_rpm;
extern volatile long motor1_current_rpm;
extern bool motor1_reversal_state;
extern int motor1_reversal_count;

extern MotorControlStyle motor2_control;
extern MotorControlDirection motor2_dir;
extern byte motor2_pwm;
extern long motor2_target_rpm;
extern long motor2_current_rpm;

#if HAS_I2C_LCD
extern I2cLCD lcd;
#else // We have to save memory in case of HAS_I2C_LCD
extern Servo servo1;
extern Servo servo2;
#endif

//these are for the extruder PID
extern volatile int speed_error;        // extruder position / error variable.
extern volatile int pGain;              // Proportional gain
extern volatile int iGain;              // Integral gain
extern volatile int dGain;              // Derivative gain
extern volatile int iMax;               // Integrator max
extern volatile int iMin;               // Integrator min
extern volatile int iState;             // Integrator state
extern volatile int dState;             // Last position input

//variables to keep track of stepper state.
const byte coil_a_enabled   = B10011001;
const byte coil_a_direction = B11000011;
const byte coil_b_enabled   = B01100110;
const byte coil_b_direction = B11110000;

//what state are we in?
extern volatile byte stepper_index;
extern volatile long stepper_ticks;
extern volatile byte stepper_high_pwm;
extern volatile byte stepper_low_pwm;

#endif // VARIABLES_H
