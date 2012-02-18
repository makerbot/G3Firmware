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

#include "EepromMap.hh"
#include "Eeprom.hh"
#include <avr/eeprom.h>

namespace eeprom {

// TODO: Shouldn't this just reset everything to an uninitialized state?
void setDefaults() {
    // Initialize eeprom map
    // Default: enstops inverted, Y axis inverted
    uint8_t axis_invert = 1<<1; // Y axis = 1
    uint8_t endstop_invert = 0b00011111; // all endstops inverted
    eeprom_write_byte((uint8_t*)eeprom::AXIS_INVERSION,axis_invert);
    eeprom_write_byte((uint8_t*)eeprom::ENDSTOP_INVERSION,endstop_invert);
    eeprom_write_byte((uint8_t*)eeprom::MACHINE_NAME,0); // name is null
    eeprom_write_byte((uint8_t*)eeprom::TOOL0_TEMP,220);
    eeprom_write_byte((uint8_t*)eeprom::TOOL1_TEMP,220);
    eeprom_write_byte((uint8_t*)eeprom::PLATFORM_TEMP,110);
    eeprom_write_byte((uint8_t*)eeprom::EXTRUDE_DURATION,1);
    eeprom_write_byte((uint8_t*)eeprom::EXTRUDE_RPM,19);
    eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_SCRIPT,0);
    eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_RED,255);
    eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_GREEN,255);
    eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_BLUE,255);
    eeprom_write_byte((uint8_t*)eeprom::JOG_MODE_SETTINGS,0);
    eeprom_write_byte((uint8_t*)eeprom::BUZZER_REPEATS,3);
    putEepromInt64(eeprom::STEPS_PER_MM_X,STEPS_PER_MM_X_DEFAULT);
    putEepromInt64(eeprom::STEPS_PER_MM_Y,STEPS_PER_MM_Y_DEFAULT);
    putEepromInt64(eeprom::STEPS_PER_MM_Z,STEPS_PER_MM_Z_DEFAULT);
    putEepromInt64(eeprom::STEPS_PER_MM_A,STEPS_PER_MM_A_DEFAULT);
    putEepromInt64(eeprom::STEPS_PER_MM_B,STEPS_PER_MM_B_DEFAULT);
    putEepromInt64(eeprom::FILAMENT_USED,0);
    putEepromInt64(eeprom::FILAMENT_USED_TRIP,0);
    eeprom_write_byte((uint8_t*)eeprom::ABP_COPIES,1);
    eeprom_write_byte((uint8_t*)eeprom::PREHEAT_DURING_ESTIMATE,0);
    eeprom_write_byte((uint8_t*)eeprom::OVERRIDE_GCODE_TEMP,0);
    eeprom_write_byte((uint8_t*)eeprom::STEPPER_DRIVER,0);
    putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_X,160);
    putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Y,160);
    putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Z,10);
    putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_A,100);
    putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_B,100);
    putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_X,2000);
    putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Y,2000);
    putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Z,150);
    putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_A,60000);
    putEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_NORM,5000);
    putEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_RETRACT,3000);
    putEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM,44);		//Multiplied by 10
    putEepromUInt32(eeprom::ACCEL_MIN_FEED_RATE,0);		//Multiplied by 10
    putEepromUInt32(eeprom::ACCEL_MIN_TRAVEL_FEED_RATE,0);	//Multiplied by 10
    putEepromUInt32(eeprom::ACCEL_MAX_XY_JERK,2);		//30mm/s Multiplied by 10
    putEepromUInt32(eeprom::ACCEL_MAX_Z_JERK,100);		//10mm/s Multiplied by 10
    putEepromUInt32(eeprom::ACCEL_ADVANCE_K,50);		//0.00001 Multiplied by 100000
    putEepromUInt32(eeprom::ACCEL_FILAMENT_DIAMETER,175);	//1.75 Multiplied by 100
}

}
