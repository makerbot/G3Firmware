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

#ifndef LinearScripts_HH_
#define LinearScripts_HH_

#include "Configuration.hh"
#include <stdint.h>
#include "AvrPort.hh"
#include "Command.hh"

#include "Types.hh"
#include "Motherboard.hh"

namespace scripts {
void StartFirstAutoHome(uint8_t directionTemp[],uint32_t feedrateTemp,uint16_t timeout_sTemp);
void StartAutoHome(uint8_t flagsTemp,uint32_t feedrateTemp,uint16_t timeout_sTemp);
void StartMoveCarefully(const Point& targetT, int32_t Z_offsetT);
void StartHomeCarefully(uint8_t directionTemp[], uint32_t feedrateTemp);

void RunScripts();

bool isRunning();

}

#endif // LinearScripts_HH_
