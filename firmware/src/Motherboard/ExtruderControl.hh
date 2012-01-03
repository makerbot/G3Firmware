/* 
 * Extruder Control Routine
 *
 * Copyright Dec, 2011 by Jetty840
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

#ifndef EXTRUDERCONTROL_HH_
#define EXTRUDERCONTROL_HH_

#include "Types.hh"
#include "Tool.hh"
#include "Timeout.hh"


enum extruderCommandType {
	EXTDR_CMD_GET,
	EXTDR_CMD_SET
};


bool extruderControl(uint8_t command, enum extruderCommandType cmdType,
		     OutPacket& responsePacket, uint16_t val);

#endif // EXTRUDERCONTROL_HH_
