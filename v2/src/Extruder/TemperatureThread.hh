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

#ifndef TEMPERATURETHREAD_HH_
#define TEMPERATURETHREAD_HH_

// The "open loop" faults are the faults of last resort. Essentially, if the
// system is maintaining the duty cycle above a certain level continuously
// for more than a time period T, we should assume that there has been a
// failure in the software, hardware, or connections thereof and shut off
// the heating mosfet.

/**
 * Return true if an open loop fault has been detected on the extruder heating
 * loop.
 */
bool hasExtruderOpenLoopFault();

/**
 * Return true if an open loop fault has been detected on the platform heating
 * loop.
 */
bool hasPlatformOpenLoopFault();

void runTempSlice();

#endif /* TEMPERATURETHREAD_HH_ */
