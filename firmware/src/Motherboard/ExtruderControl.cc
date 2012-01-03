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

#include "ExtruderControl.hh"


#define HOST_TOOL_RESPONSE_TIMEOUT_MS 50


/// Send a packet to the extruder.  If cmdType == EXTDR_CMD_SET, then "val" should
/// contain the value to be written otherwise val is ignored.
/// responsePacket is filled with the returned value
bool extruderControl(uint8_t command, enum extruderCommandType cmdType,
		     OutPacket& responsePacket, uint16_t val) {

	Timeout acquire_lock_timeout;
	acquire_lock_timeout.start(HOST_TOOL_RESPONSE_TIMEOUT_MS);
	while (!tool::getLock()) {
		if (acquire_lock_timeout.hasElapsed()) {
			return false;
		}
	}
	OutPacket& out = tool::getOutPacket();
	InPacket& in = tool::getInPacket();
	out.reset();
	responsePacket.reset();

	// Fill the query packet. The first byte is the toolhead index, and the
	// second is the
	out.append8(0);
	out.append8(command);
	if ( cmdType == EXTDR_CMD_SET )	out.append16(val);	

	// Timeouts are handled inside the toolslice code; there's no need
	// to check for timeouts on this loop.
	tool::startTransaction();
	tool::releaseLock();
	// WHILE: bounded by tool timeout in runToolSlice
	while (!tool::isTransactionDone()) {
		tool::runToolSlice();
	}
	if (in.getErrorCode() == PacketError::PACKET_TIMEOUT) {
		return false;
	} else {
		// Copy payload back. Start from 0-- we need the response code.
		for (uint8_t i = 0; i < in.getLength(); i++) {
			responsePacket.append8(in.read8(i));
		}
	}

	// Check that the extruder was able to process the request
	if (!rcCompare(responsePacket.read8(0),RC_OK)) {
		return false;
	}

	return true;
}
