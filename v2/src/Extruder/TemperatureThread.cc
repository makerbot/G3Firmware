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

#include "ExtruderBoard.hh"
#include "TemperatureThread.hh"


/**
 * Return true if an open loop fault has been detected on the extruder heating
 * loop.
 */
bool hasExtruderOpenLoopFault() {
  ExtruderBoard& board = ExtruderBoard::getBoard();
  return board.getExtruderHeater().has_open_loop_fault();
}

/**
 * Return true if an open loop fault has been detected on the platform heating
 * loop.
 */
bool hasPlatformOpenLoopFault() {
  ExtruderBoard& board = ExtruderBoard::getBoard();
  return board.getPlatformHeater().has_open_loop_fault();
}

void runTempSlice() {
	ExtruderBoard& board = ExtruderBoard::getBoard();
	board.getExtruderHeater().manage_temperature();
	if (board.isUsingPlatform()) {
		board.getPlatformHeater().manage_temperature();
	}
}
