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

const static uint16_t EEPROM_SIZE                        = 0x0200;

/// Version, low byte: 1 byte
const static uint16_t VERSION_LOW                        = 0x0000;
/// Version, high byte: 1 byte
const static uint16_t VERSION_HIGH                       = 0x0001;

/// Axis inversion flags: 1 byte.
/// Axis N (where X = 0, Y=1, etc.) is inverted if the Nth bit is set.
/// Bit 7 is used for HoldZ OFF: 1 = off, 0 = on
const static uint16_t AXIS_INVERSION                     = 0x0002;

/// Endstop inversion flags: 1 byte.
/// The endstops for axis N (where X = 0, Y=1, etc.) are considered
/// to be logically inverted if the Nth bit is set.
/// Bit 7 is set to indicate endstops are present; it is zero to indicate
/// that endstops are not present.
/// Ordinary endstops (H21LOB et. al.) are inverted.
const static uint16_t ENDSTOP_INVERSION                  = 0x0003;

/// Name of this machine: 32 bytes.
const static uint16_t MACHINE_NAME                       = 0x0020;

/// Default locations for the axis: 5 x 32 bit = 20 bytes
const static uint16_t AXIS_HOME_POSITIONS                = 0x0060;

/// Default steps/mm for each axis: 32 bits = 4 bytes
const static uint16_t ESTOP_CONFIG                       = 0x0074; // .. 0x0077

/// Default steps/mm for each axis: 5 x 32 bit = 20 bytes
const static uint16_t STEPS_PER_MM                       = 0x0078; // .. 0x008B

/// Master acceleration rate for each axis: 32 bits = 4 bytes
const static uint16_t MASTER_ACCELERATION_RATE           = 0x008C; // .. 0x008F

/// Default acceleration rates for each axis: 5 x 32 bit = 20 bytes
const static uint16_t AXIS_ACCELERATION_RATES            = 0x0090; // .. 0x00A3

/// Default acceleration rates for each axis: 4 x 32 bit = 16 bytes
/// X+Y have an integrated value, and Z, A, and B have their own values.
const static uint16_t AXIS_JUNCTION_JERK                 = 0x00A4; // .. 0x00B3

/// Default minimum planner speed: 32 bits = 1 byte
const static uint16_t MINIMUM_PLANNER_SPEED              = 0x00B4; // .. 0x00B5

/// Reset all data in the EEPROM to a default.
void setDefaults();

} // namespace eeprom

#endif // EEPROMMAP_HH_
