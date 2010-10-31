
/*
 * This file was created by Makerbot Industries intern Winter on September 2010.
 *
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
 
 //This file is for scripts that take a very long time to run (such as homing scripts and nozzle wipes). It contains two levels. The higher script level and the lower script level. The higher level is where the actual program is written, where as the lower level contains all of the subroutines. When a lower script is called, the higher script pauses and waits untill the lower script finishes.
 
 //The scripts in this file repeatedly poll themselves and move on, instead of polling themselves and then just waiting 'till they can move on. This way the motherboard can still attend to the other things it might have to do (like maintaining contact with a connected computer) and not too much time is wasted.

#define __STDC_LIMIT_MACROS
#include "LinearScripts.hh"
#include "Steppers.hh"
#include <stdint.h>
#include <avr/eeprom.h>

#define AUTOHOMEOFFSET 0x100



namespace scripts {

 enum { //possible states for the higher script.
	NONE,
	FIRSTAUTOHOME,
	AUTOHOME,
} ScriptRunning = NONE;

 enum { //possible states for the lower script. 
	NOTRUNNING,
	MOVECAREFULLY,
	HOMECAREFULLY,
} LowScriptRunning = NOTRUNNING;

volatile int currentStep = 0;
volatile int lowScriptStep = 0;

uint8_t flags; //varibles for the autohoming scripts.
uint8_t direction[STEPPER_COUNT];
uint32_t feedrateXY; //ReplicatorG defined feedrate for the XY stage moves
uint32_t feedrateZ; //Replicatorg defined feedrate for the Z stage moves
uint16_t timeout_s; //Time 'till abort
int32_t zOffset; //varible to save the read amount for the Z Offset.s
uint8_t EEPROM_direction[STEPPER_COUNT]; //direction setting saved in EEPROM
int32_t EEPROM_DATA[4] = {0,0,0,0}; //Array to hold the 32bit read values.

//varibles for the Movecarefully script.
Point target;
int32_t Z_offset;

//varibles for the homecarefully script
uint8_t homeDirection[STEPPER_COUNT];
uint8_t homeFlags;
uint32_t XYhomeFeedrate;
uint32_t ZhomeFeedrate;
uint8_t other_axis_flags[STEPPER_COUNT];


bool isRunning() { //program that can be called elsewhere in the firmware to check if the scripts are running or not. (False if not running)
if (ScriptRunning == NONE) {
return false;
} else {
return true;
}
}

void StartFirstAutoHome(uint8_t directionTemp[],uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp, uint16_t timeout_sTemp) {
//start the FirstAutoHome script
currentStep = 1; //start at the begining.
ScriptRunning = FIRSTAUTOHOME;
for (int i = 0; i < STEPPER_COUNT; i++) {
direction[i] = directionTemp[i];
}
feedrateXY = XYfeedrateTemp;
feedrateZ = ZfeedrateTemp;
timeout_s = timeout_sTemp;
}

void StartAutoHome(uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp, uint16_t timeout_sTemp) {
//start the AutoHome script
currentStep = 1; //start at the begining.
ScriptRunning = AUTOHOME;
feedrateXY = XYfeedrateTemp;
feedrateZ = ZfeedrateTemp;
timeout_s = timeout_sTemp;
}

void StartMoveCarefully(const Point& targetT, int32_t Z_offsetT, uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp) {
lowScriptStep = 1;
LowScriptRunning = MOVECAREFULLY;
target = targetT;
Z_offset = Z_offsetT;
XYhomeFeedrate = XYfeedrateTemp;
ZhomeFeedrate = ZfeedrateTemp;
}

void StartHomeCarefully(uint8_t directionTemp[], uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp) {
lowScriptStep = 1;
LowScriptRunning = HOMECAREFULLY;
for (int i = 0; i < STEPPER_COUNT; i++) {
homeDirection[i] = directionTemp[i];
}
XYhomeFeedrate = XYfeedrateTemp;
ZhomeFeedrate = ZfeedrateTemp;
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
	
	currentStep = 2;		
	//home carefully! We don't want to break anything!
	if (direction[2] == 1) { //if we are homing down, it would be a good idea to lift the zstage a bit before begining.
		offset = AUTOHOMEOFFSET + 0xF; //offset of the saved Z offset amount.
		eeprom_read_block((void*)&zOffset, (const void*)offset, 4); //read it from eeprom
		steppers::setTarget(Point(0,0,zOffset), feedrateZ); // move to zoffset
	}
break; }

case 2: { //start homing
if (!steppers::isRunning()) {
//proceed with homing.
StartHomeCarefully(direction, feedrateXY, feedrateZ);
currentStep = 3;
}
break; }

case 3: { //Homing is done.
if (LowScriptRunning == NOTRUNNING) {

Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
						//Squirrel everything into EEPROM for the long Winter.		
	int16_t offset;
	uint8_t data8;
	int32_t dataa;

	for (int i = 0; i < STEPPER_COUNT; i++) {
	offset = AUTOHOMEOFFSET + i;
	data8 = direction[i];
	eeprom_write_block((const void*)&data8, (void*)offset, 1); //save the set direction in eeprom.
	}
					
	for (int i = 0; i < 3; i++) { 
		offset = AUTOHOMEOFFSET + 0x3 + (i*0x4);
		dataa = currentPosition[i]; //Grab individual current position from current position X,Y,Z.
		if (dataa < 0) { //If the position is negative, make it positive!
			dataa = -dataa;
		}
			eeprom_write_block((const void*)&dataa, (void*)offset, 4); //save it!
		}
		
	//next move back up to 0,0,0 (aka build platform height)
	if (direction[2] == 1) { //if we are homing down then move z before XY
	StartMoveCarefully(Point(0,0,0), zOffset, feedrateXY, feedrateZ); // move to 000 with a saved z offset and a custom feedrate.
	} else if (direction[2] == 2) { //if positive then move XY before Z
	StartMoveCarefully(Point(0,0,0), -1, feedrateXY, feedrateZ); // move to 000 with a z offset of -1 (aka none.) Also tells the subroutine to move XY before Z..
						}
	currentStep = 4;
						}
						

break; }

case 4: {
//wait for the movecarefully script to finish before finishing the first autohome.
if (LowScriptRunning == NOTRUNNING) {
currentStep = 0;
ScriptRunning = NONE;
}
}

//default:
//nothing to do.


}

} else if (ScriptRunning == AUTOHOME) {
switch (currentStep) {
case 1: {
	int32_t x = 0; //set x. Zero current position
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps
	//Read values from EEPROM
	
	//EEPROM MAP
	//0x100, 0x101, 0x102 Directions of XYZ.
	//0x103-06, 0x107-0x10a, 0x10b-0x10e Steps to move.
	//0x10f-0x112 MM to move up on Z axis.
	
	
	int16_t offset; //where to start copying from. 
	uint8_t data8;
	for (int i = 0; i < STEPPER_COUNT; i++) {
	offset = AUTOHOMEOFFSET + i;
	eeprom_read_block((void*)&data8, (const void*)offset, 1); //read direction
	EEPROM_direction[i] = data8;
	}
	
	for (int i = 0; i < (STEPPER_COUNT + 1); i++) { //loop and copy all of the data for all of the axis.
	offset = AUTOHOMEOFFSET + 0x3 + (i*0x4);
	eeprom_read_block((void*)&EEPROM_DATA[i], (const void*)offset, 4);
	
	}
	//direction = (EEPROM_direction > 0);
	currentStep = 2;
	
	if (EEPROM_direction[2] == 1) { //if we are homing down, it would be a good idea to lift the zstage a bit before begining. (Just in case)
		steppers::setTarget(Point(0,0,EEPROM_DATA[3]), feedrateZ); // move to zoffset
	}
	
	//currentStep = 2;
break; }

case 2: {
if (!steppers::isRunning()) {
StartHomeCarefully(EEPROM_direction, feedrateXY, feedrateZ); //home carefully!
currentStep = 3;
}
break; }

case 3: {
	if (LowScriptRunning == NOTRUNNING) {
	int32_t x = 0; //set x at zero (we are at endstop)
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps
	//next move back up the read amount (aka build platform height)
	
	for (int i = 0; i < STEPPER_COUNT; i++) { 
	if (EEPROM_direction[i] == 2) { //If the direction homed was up, then make the direction to move negative.
	EEPROM_DATA[i] = -EEPROM_DATA[i];
	}
	}
	
	if (EEPROM_direction[2] == 1) { //If we homed down then raise the Z first.
	
		StartMoveCarefully(Point(EEPROM_DATA[0],EEPROM_DATA[1],EEPROM_DATA[2]), EEPROM_DATA[3], feedrateXY, feedrateZ); //move carefully (raise the Z stage first)	
	} else { //else move the XY first
		StartMoveCarefully(Point(EEPROM_DATA[0],EEPROM_DATA[1],EEPROM_DATA[2]), -1, feedrateXY, feedrateZ); //move carefully
}


currentStep = 4;
//eeprom_write_block((const void*)&data8, (void*)offset, 1); //save the set direction in eeprom.
}


break; }

case 4: {
if (LowScriptRunning == NOTRUNNING) {
steppers::definePosition(Point(0,0,0)); //set the position in steps to zero (we are at center of BP)
currentStep = 0;
ScriptRunning = NONE;
}
break; }

}

}
 
} else if (LowScriptRunning == MOVECAREFULLY) {
switch (lowScriptStep) {

case 1: {
if (Z_offset < 0) { //if the z offset is negative (this should not usually happen). assume we want to Center the XY before Z (as if we are centering the Z stage in the - direction) AKA from the top.
	Point currentPosition = steppers::getPosition();
	int32_t x = target[0]; //Move XY
	int32_t y = target[1];
	int32_t z = currentPosition[2]; //Leave this alone.
	int32_t dda = XYhomeFeedrate; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move XY
	lowScriptStep = 2;
} else { //we must be moving in the positive direction so move Z before XY.
	Point currentPosition = steppers::getPosition();
	int32_t x = currentPosition[0]; //leave these were they are
	int32_t y = currentPosition[1];
	int32_t z = target[2] + Z_offset; //move the Z stage back up to a bit above zero to avoid the BP hitting it.
	int32_t dda = ZhomeFeedrate; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda);
	lowScriptStep = 3;
	}
						
break; }

case 2: { //wait for XY stage
if (!steppers::isRunning()){
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2]; //move Z down 
	int32_t dda = ZhomeFeedrate; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda); //move everything
	lowScriptStep = 5; //wait to say that you are finished.
	}
break; }

case 3: { //wait for Z
if (!steppers::isRunning()) {
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2] + Z_offset; //keep this where it was
	int32_t dda = XYhomeFeedrate; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move everything back up
	lowScriptStep = 4;
	}
break; }

case 4: { //wait for XY
if (!steppers::isRunning()) {
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2]; //move back down to 000 
	int32_t dda = ZhomeFeedrate; // max feedrate for Z stage
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
} else if (LowScriptRunning == HOMECAREFULLY) { //AUTOHOME subroutine
switch (lowScriptStep) {

case 1: {
if (!steppers::isRunning()) {
//proceed with homing.

	if (homeDirection[2] == 1) { //if flags says home Z and something else (we don't care what). And it's also in the negative direction then home carefully.
					//Don't home Z.
					for (int i = 0; i < STEPPER_COUNT; i++) {
					other_axis_flags[i] = homeDirection[i]; //Save a snapshot of the directions
					}
					homeDirection[2] = 0; //don't home Z
					steppers::startHoming(homeDirection,XYhomeFeedrate); //home the others (XY)
						lowScriptStep = 2;
						
						
	} else if (homeDirection[2] == 2) { //else if we are homing the Z stage up then...
						//home the z up before homing xy in the positive direction.
						for (int i = 0; i < STEPPER_COUNT; i++) {
						other_axis_flags[i] = homeDirection[i]; // Save axis besides Z to home.
						homeDirection[i] = 0; //reset
						}
						
						homeDirection[2] = 2; // Home Z up.
					steppers::startHoming(homeDirection,ZhomeFeedrate); //home Z
						lowScriptStep = 3;
	} else { //If it does not involve the Z axis, no special care is needed.
						steppers::startHoming(homeDirection, XYhomeFeedrate);
							lowScriptStep = 4;
							}
				}
break; }

case 2: {
//wait till Xy is homed
if (!steppers::isRunning()) {
	for (int i = 0; i < STEPPER_COUNT; i++) {
	homeDirection[i] = 0;
	}
	homeDirection[2] = other_axis_flags[2];
	//Home Z.
	steppers::startHoming(homeDirection, ZhomeFeedrate);
	lowScriptStep = 4;
break; }

case 3: {//wait till Z is homed
if (!steppers::isRunning()) {
for (int i = 0; i < STEPPER_COUNT; i++) {
homeDirection[i] = other_axis_flags[i]; //Home the rest of the axis (besides Z).
}
	//Home the rest of the axis (besides Z).
	homeDirection[2] = 0;
	steppers::startHoming(homeDirection, XYhomeFeedrate);
	lowScriptStep = 4;
}
break; }

case 4: {
//wait till homing is done then reset low scripts.
if (!steppers::isRunning()) {
lowScriptStep = 0;
LowScriptRunning = NOTRUNNING;
}
break; }


}

}
}



	
	
	
	}
	}
