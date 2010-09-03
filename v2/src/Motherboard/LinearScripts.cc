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
#include <avr/eeprom.h>
#define F_CPU 20000000UL  // 1 MHz
    //#define F_CPU 14.7456E6
#include <util/delay.h>


namespace scripts {

 enum {
	NONE,
	FIRSTAUTOHOME,
	AUTOHOME,
} ScriptRunning = NONE;

 enum {
	NOTRUNNING,
	MOVECAREFULLY,
	HOMECAREFULLY,
} LowScriptRunning = NOTRUNNING;

int currentStep = 0;
int lowScriptStep = 0;

uint8_t flags; //varibles for the autohoming scripts.
bool direction;
uint32_t feedrate;
uint16_t timeout_s;
uint8_t other_axis_flags;
int32_t zOffset; //varible to save the read amount for the Z Offset.s

//varibles for the Movecarefully script.
Point target;
int32_t Z_offset;


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

void StartMoveCarefully(const Point& targetT, int32_t Z_offsetT) {
lowScriptStep = 1;
LowScriptRunning = MOVECAREFULLY;
target = targetT;
Z_offset = Z_offsetT;
RunScripts();
}

void RunScripts() { //script that checks if the makerbot can continue a script.
if (LowScriptRunning == NOTRUNNING) {
if (ScriptRunning == FIRSTAUTOHOME) {
//check if we can advance.
switch (currentStep) {

case 1: { //step one.
	int32_t x = 0; //set x
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps to 000
	
	int32_t dataa; //temporary varible.
	int8_t data8; //temporary varible
	int16_t offset; //this is where in eeprom we should start saving/reading.
	
					
	//home carefully! We don't want to break anything!
	if (direction == false) { //if we are homing down, it would be a good idea to lift the zstage a bit before begining.
		offset = 0x10D; //offset of the saved Z offset amount.
		eeprom_read_block((void*)&zOffset, (const void*)offset, 4); //read it from eeprom
		steppers::setTarget(Point(0,0,zOffset), 1250); // move to zoffset
	currentStep = 2; //move on to the next step (that waits for the zoffset move.)
	}
break; }

case 2: { //start homing
if (!steppers::isRunning) {
//proceed with homing.

if (flags & (1<<2) != 0 && (flags - 4) != 0 && direction == false) { //if flags says home Z and something else (we don't care what). And it's also in the negative direction then home carefully.
					flags = flags - 4; //Don't home Z.
					steppers::startHoming(direction,
							flags,
							feedrate); //home the others
						currentStep = 3;
						} else if (flags & (1<<2) != 0 && (flags - 4) != 0 && direction == true) { 
						//home the z up before homing xy in the positive direction.
						other_axis_flags = flags - 4; //axis besides Z to home.
						
						flags = 4; // Home Z up.
					steppers::startHoming(direction,
							flags,
							feedrate); //home Z
						currentStep = 4;
						} else { //If it does not involve the Z axis, no special care is needed.
						steppers::startHoming(direction,
							flags,
							feedrate);
							currentStep = 5;
							}
				}
break; }

case 3: { //wait till Xy is homed
if (!steppers::isRunning()) {
	flags = 4; //Home Z.
	steppers::startHoming(direction, flags, feedrate);
	currentStep = 5;
}

break; }

case 4: { //wait till Z is homed
if (!steppers::isRunning()) {
	flags = other_axis_flags; //Home the rest of the axis (besides Z).
	steppers::startHoming(direction, flags, feedrate);
	currentStep = 5;
}
break; }

case 5: { //Homing is done.
if (!steppers::isRunning()) {

Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
						//Squirrel everything into EEPROM for the long Winter.		
	int16_t offset = 0x100;
	int8_t data8;
	int32_t dataa;
	if (direction == true) {
		data8 = 1;
	} else { 
		data8 = 0; 
	}
	eeprom_write_block((const void*)&data8, (void*)offset, 1); //save the set direction in eeprom.				
	for (int i = 0; i < 3; i++) { 
		offset = 0x101 + (i*0x4);
		dataa = currentPosition[i]; //Grab individual current position from current position X,Y,Z.
		if (dataa < 0) { //If the position is negative, make it positive!
			dataa = -dataa;
		}
			eeprom_write_block((const void*)&dataa, (void*)offset, 4); //save it!
			_delay_ms(50); //wait A little bit. EEPROM is not very fast. (Probably not needed but couldn't hurt.)
		}
	//next move back up to 0,0,0 (aka build platform height)
	if (direction == false) { //if we are homing down then move z before XY
	StartMoveCarefully(Point(0,0,0), zOffset); // move to 000 with a z offset of 400 steps.
	} else { //if positive then move XY before Z
	StartMoveCarefully(Point(0,0,0), -1); // move to 000 with a z offset of -1 (aka none.) Also tells the subroutine to move XY before Z..
						}
	
						}
						

break; }

//default:
//nothing to do.


}

}

} else {
//low script is running.

if (LowScriptRunning == MOVECAREFULLY) {
switch (lowScriptStep) {

case 1: {
if (Z_offset < 0) { //if the z offset is negative (this should not usually happen). assume we want to Center the XY before Z (as if we are centering the Z stage in the - direction)
	Point currentPosition = steppers::getPosition();
	int32_t x = target[0]; //Move XY
	int32_t y = target[1];
	int32_t z = currentPosition[2]; //Leave this alone.
	int32_t dda = 1017; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move XY
	lowScriptStep = 2;
} else { //we must be moving in the positive direction so move Z before XY.
	Point currentPosition = steppers::getPosition();
	int32_t x = currentPosition[0]; //leave these were they are
	int32_t y = currentPosition[1];
	int32_t z = target[2] + Z_offset; //move the Z stage back up to a bit above zero to avoid the BP hitting it.
	int32_t dda = 1250; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda);
	lowScriptStep = 3;
	}
						
break; }

case 2: {
if (!steppers::isRunning()){
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2]; 
	int32_t dda = 1250; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda); //move everything
	lowScriptStep = 5; //wait to say that you are finished.
	}
break; }

case 3: {
if (!steppers::isRunning()) {
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2] + Z_offset; //keep this where it was
	int32_t dda = 1017; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move everything back up
	lowScriptStep = 4;
	}
break; }

case 4: {
if (!steppers::isRunning()) {
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2]; 
	int32_t dda = 1250; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda); //move everything back up
	lowScriptStep = 5; //wait to say that this script is finished
}
break; }

case 5: { //script finished.
if (!steppers::isRunning()) {
lowScriptStep = 0;
LowScriptRunning = NOTRUNNING;
}
}


}
}
}
}



	
	}
