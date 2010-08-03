/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
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

#ifndef STEPPERS_HH_
#define STEPPERS_HH_

#include "Configuration.hh"
#include <stdint.h>
#include "AvrPort.hh"
#include "Command.hh"

#include "Types.hh"
#include "Motherboard.hh"

namespace steppers {

/// Initialize the stepper subsystem.
void init(Motherboard& motherboard);
/// Returns true if the stepper subsystem is running.  If the
/// stepper subsystem is idle, returns false.  Will return true
/// if the system is running but paused.
bool isRunning();
/// Abort the current motion and set the stepper subsystem to
/// the not-running state.
void abort();
/// Enable/disable the given axis.
void enableAxis(uint8_t which, bool enable);
/// Set current target
void setTarget(const Point& target, int32_t dda_interval);
/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step);
/// Define current position as given point
void definePosition(const Point& position);
/// Handle interrupt.  Return true if still moving to target; false
/// if target has been reached.
bool doInterrupt();
/// Get current position
const Point getPosition();
/// Turn on in-build Z hold.  Defaults to off.
void setHoldZ(bool holdZ);
//move to a position carefully. (Move Z up first past destination, then move XY, then move Z to destination). Used to avoid crashing into BP at the end of homing scripts
void moveCarefully(const Point& target, int32_t Z_offset);
//home carefully. Checks to see if you are trying to home in the negative direction with Z and another axis. If so it will home the others before homing Z. It assumes that you know what you are doing if you are homing only Z. (Maybe it shouldn't?)
void homeCarefully(const bool direction, uint8_t flags, const uint32_t feedrate);

};

#endif // STEPPERS_HH_
