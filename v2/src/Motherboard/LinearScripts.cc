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

#define __STDC_LIMIT_MACROS
#include "LinearScripts.hh"
#include "Steppers.hh"
#include <stdint.h>

namespace scripts {

 enum {
	NONE,
	FIRSTAUTOHOME,
	AUTOHOME,
} ScriptRunning = NONE;

int currentStep = 0;

uint8_t flags; //varibles for the autohoming scripts.
bool direction;
uint32_t feedrate;
uint16_t timeout_s;

bool isRunning() {
if (ScriptRunning == NONE) {
return false;
} else {
return true;
}
}

void StartFirstAutoHome(uint8_t flagsTemp,bool directionTemp,uint32_t feedrateTemp,uint16_t timeout_sTemp) {
//start the FirstAutoHome script
currentStep = 1; //start at the begining.
ScriptRunning = FIRSTAUTOHOME;
flags = flagsTemp;
direction = directionTemp;
feedrate = feedrateTemp;
timeout_s = timeout_sTemp;

RunScripts();
}

void StartAutoHome() {
//start the FirstAutoHome script
currentStep = 1; //start at the begining.
ScriptRunning = AUTOHOME;
RunScripts();
}

void RunScripts() { //script that checks if the makerbot can continue a script.
if (ScriptRunning == FIRSTAUTOHOME) {
//check if we can advance.
switch (currentStep) {

case 1:
	int32_t x = 0; //set x
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps to 000
	
break;

//case 2:
//break;

//default:
//nothing to do.


}



}
}
	
	}
