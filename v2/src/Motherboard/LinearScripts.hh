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

void StartFirstAutoHome(uint8_t directionTemp[],uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp, uint16_t timeout_sTemp);
void StartAutoHome(uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp, uint16_t timeout_sTemp);
void StartMoveCarefully(const Point& targetT, int32_t Z_offsetT, uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp);
void StartHomeCarefully(uint8_t directionTemp[], uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp);

void RunScripts();

bool isRunning();

/**
**
Internal Subroutines
**
**/

//First time homing (calibration) subroutines.
void firstTimeHomeStep1();

void firstTimeHomeStep2();

void firstTimeHomeStep3();

void firstTimeHomeFinal();

void firstTimeHomeWithZProbeStep1();

void firstTimeHomeWithZProbeStep2();

void firstTimeHomeWithZProbeStep3();

//Auto homing script subroutines.

void autoHomeStep1();

void autoHomeStep2();

void autoHomeStep3();

void autoHomeZProbeStep1();

void autoHomeZProbeStep2();

void autoHomeZProbeStep3();

void autoHomeFinalEnd();

//Move carefully subroutines

void moveCarefullyStep1();

void moveCarefullyStep2();

void moveCarefullyStep3();

void moveCarefullyStep4();

void moveCarefullyFinalEnd();

//Home Carefully subroutines

void homeCarefullyStep1();

void homeCarefullyStep2();

void homeCarefullyStep3();

void homeCarefullyFinalEnd();
}

#endif // LinearScripts_HH_
