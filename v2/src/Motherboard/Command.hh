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

#ifndef COMMAND_HH_
#define COMMAND_HH_

#include <stdint.h>

/// The command namespace contains functions that handle the incoming command
/// queue, for both SD and serial jobs.
namespace command {

/// Reset the entire command queue.  Clears out any remaining queued
/// commands.
void reset();

/// Run the command thread slice.
void runCommandSlice();

/// Pause the command processor
/// \param[in] pause If true, disable the command processor. If false, enable it.
void pause(bool pause);

/// Check the state of the command processor
/// \return True if it is disabled, false if it is enabled.
bool isPaused();

/// Check the remaining capacity of the command buffer
/// \return Amount of space left in the buffer, in bytes
uint16_t getRemainingCapacity();

/// Check if the command buffer is empty
/// \return true if is empty
bool isEmpty();

/// Push a byte onto the command buffer. This is used by the host to add commands
/// to the buffer.
/// \param[in] byte Byte to add to the buffer.
void push(uint8_t byte);

}

#endif // COMMAND_HH_
