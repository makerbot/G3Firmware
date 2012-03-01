/*
 * Copyright 2010 by Adam Mayer <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */


#ifndef EEPROMMAP_HH_
#define EEPROMMAP_HH_

#include <stdint.h>

namespace eeprom {

const static uint16_t EEPROM_SIZE				= 0x0200;

/// Version, low byte: 1 byte
const static uint16_t VERSION_LOW				= 0x0000;
/// Version, high byte: 1 byte
const static uint16_t VERSION_HIGH				= 0x0001;

/// Axis inversion flags: 1 byte.
/// Axis N (where X=0, Y=1, etc.) is inverted if the Nth bit is set.
/// Bit 7 is used for HoldZ OFF: 1 = off, 0 = on
const static uint16_t AXIS_INVERSION			= 0x0002;

/// Endstop inversion flags: 1 byte.
/// The endstops for axis N (where X=0, Y=1, etc.) are considered
/// to be logically inverted if the Nth bit is set.
/// Bit 7 is set to indicate endstops are present; it is zero to indicate
/// that endstops are not present.
/// Ordinary endstops (H21LOB et. al.) are inverted.
const static uint16_t ENDSTOP_INVERSION			= 0x0003;

/// Name of this machine: 32 bytes.
const static uint16_t MACHINE_NAME				= 0x0020;

/// Default locations for the axis: 5 x 32 bit = 20 bytes
const static uint16_t AXIS_HOME_POSITIONS		= 0x0060;

// Estop configuration byte: 1 byte.
const static uint16_t ESTOP_CONFIGURATION = 0x0074;

enum {
	ESTOP_CONF_NONE = 0x0,
	ESTOP_CONF_ACTIVE_HIGH = 0x1,
	ESTOP_CONF_ACTIVE_LOW = 0x2
};

const static uint16_t TOOL0_TEMP      		= 0x0080;
const static uint16_t TOOL1_TEMP      		= 0x0081;
const static uint16_t PLATFORM_TEMP   		= 0x0082;
const static uint16_t EXTRUDE_DURATION		= 0x0083;
const static uint16_t EXTRUDE_RPM     		= 0x0084;
const static uint16_t MOOD_LIGHT_SCRIPT		= 0x0085;
const static uint16_t MOOD_LIGHT_CUSTOM_RED	= 0x0086;
const static uint16_t MOOD_LIGHT_CUSTOM_GREEN	= 0x0087;
const static uint16_t MOOD_LIGHT_CUSTOM_BLUE	= 0x0088;

//Bit 1 is Model mode or user view mode (user view mode = bit set)
//Bit 2-4 are the jog mode distance 0 = short, 1 = long, 2 = cont
const static uint16_t JOG_MODE_SETTINGS		= 0x0089;

//0 = No system buzzing, >=1 = number of repeats to buzz for
const static uint16_t BUZZER_REPEATS		= 0x008A;

//Steps per mm, each one is 8 bytes long and are stored as int64_t
const static uint16_t STEPS_PER_MM_X		= 0x008B;
const static uint16_t STEPS_PER_MM_Y		= 0x0093;
const static uint16_t STEPS_PER_MM_Z		= 0x009B;
const static uint16_t STEPS_PER_MM_A		= 0x00A3;
const static uint16_t STEPS_PER_MM_B		= 0x00AB;

//int64_t (8 bytes) The filament used in steps
const static uint16_t FILAMENT_USED		= 0x00B3;
const static uint16_t FILAMENT_USED_TRIP	= 0x00BB;

//Number of ABP copies (1-254) when building from SDCard (1 byte)
const static uint16_t ABP_COPIES		= 0x00C3;

//Preheat during estimate 0 = Disable, 1 = Enabled
const static uint16_t PREHEAT_DURING_ESTIMATE	= 0x00C4;

//Override the temperature set in the gcode file at the start of the build
//0 = Disable, 1 = Enabled
const static uint16_t OVERRIDE_GCODE_TEMP	= 0x00C5;

//Profiles
#define PROFILE_NAME_LENGTH			8
#define PROFILE_HOME_OFFSETS_SIZE		(4 * 3)		//X, Y, Z (uint32_t)

#define PROFILE_NEXT_OFFSET			(PROFILE_NAME_LENGTH + \
						 PROFILE_HOME_OFFSETS_SIZE + \
						 4 )		//24 (0x18)    4=Bytes (Hbp, tool0, tool1, extruder)

//4 Profiles = 0x00C6 + PROFILE_NEXT_OFFSET * 4 
const static uint16_t PROFILE_BASE		= 0x00C6;

//1 = Accelerated Stepper Driver, 0 = Regular stepper driver (default)
//Bit 2 is planner enabled
const static uint16_t STEPPER_DRIVER	= 0x0126;

//uint32_t (4 bytes)
const static uint16_t ACCEL_MAX_FEEDRATE_X	= 0x0127;
const static uint16_t ACCEL_MAX_FEEDRATE_Y	= 0x012B;
const static uint16_t ACCEL_MAX_FEEDRATE_Z	= 0x012F;
const static uint16_t ACCEL_MAX_FEEDRATE_A	= 0x0133;
const static uint16_t ACCEL_MAX_FEEDRATE_B	= 0x0137;

//uint32_t (4 bytes)
const static uint16_t ACCEL_MAX_ACCELERATION_X	= 0x013B;
const static uint16_t ACCEL_MAX_ACCELERATION_Y	= 0x013F;
const static uint16_t ACCEL_MAX_ACCELERATION_Z	= 0x0143;
const static uint16_t ACCEL_MAX_ACCELERATION_A	= 0x0147;

//uint32_t (4 bytes)
const static uint16_t ACCEL_MAX_EXTRUDER_NORM	= 0x014B;
const static uint16_t ACCEL_MAX_EXTRUDER_RETRACT= 0x014F;

//uint32_t (4 bytes)
const static uint16_t ACCEL_E_STEPS_PER_MM	= 0x0153;

//uint32_t (4 bytes)
const static uint16_t ACCEL_MIN_FEED_RATE	= 0x0157;
const static uint16_t ACCEL_MIN_TRAVEL_FEED_RATE= 0x015B;
const static uint16_t ACCEL_MAX_XY_JERK		= 0x015F;
const static uint16_t ACCEL_MAX_Z_JERK		= 0x0163;
const static uint16_t ACCEL_ADVANCE_K		= 0x0167;
const static uint16_t ACCEL_FILAMENT_DIAMETER	= 0x016B;

/// Reset all data in the EEPROM to a default.
void setDefaults();

} // namespace eeprom

#endif // EEPROMMAP_HH_
