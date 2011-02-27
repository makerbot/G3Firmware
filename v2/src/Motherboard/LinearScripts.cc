
/*
 * This file was created by Makerbot Industries Intern Winter on September 2010.
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

/*
This file is for scripts that take a very long time to run (such as homing scripts and nozzle wipes).
It contains two levels. The higher script level and the lower script level.
The higher level is where the actual program is written, where as the lower level contains all of the subroutines.
When a lower script is called, the higher script pauses and waits until the lower script finishes.
*/

/*
The scripts in this file repeatedly poll themselves and move on,
instead of polling themselves and then just waiting 'till they can move on.
This way the motherboard can still attend to the other things it might have to do
(like maintaining contact with a connected computer) and not too much time is wasted.
*/


#define __STDC_LIMIT_MACROS
#include "LinearScripts.hh"
#include "Steppers.hh"
#include "Tool.hh"
#include "Commands.hh"
#include "Timeout.hh"
#include "Types.hh"
#include <stdint.h>
#include <avr/eeprom.h>

#define AUTOHOMEOFFSET 0x100    //Homing settings EEPROM starting block.
                                //Change this to move the save location for the data.

#define HOST_TOOL_RESPONSE_TIMEOUT_MS 50 //Timeouts for when the Z-Probe is communicating with the extruder.
#define HOST_TOOL_RESPONSE_TIMEOUT_MICROS (1000L*HOST_TOOL_RESPONSE_TIMEOUT_MS) //Micros version of the Z-Probe comm timeout


    /*
    **
    **EEPROM MAP (Based on a offset of 0x100)
	**0x100, 0x101, 0x102 Directions of XYZ.
	**0x103-06, 0x107-0x10a, 0x10b-0x10e Steps to move.
	**0x10f-0x112 MM to move up on Z axis.
	**0x113 tool index for Z-probe
	**0x114 lift angle for Z-Probe servo
	**0x115 lowered angle for Z-Probe servo
    **
    */



namespace scripts {

 enum { //possible states for the higher script.
	NONE,
	FIRSTAUTOHOME,
	AUTOHOME,
	MOVECAREFULLY,
	HOMECAREFULLY
} ScriptRunning = NONE, LowScriptRunning = NONE;

volatile int currentStep = -1;
volatile int lowScriptStep = 0;

uint8_t direction[AXIS_COUNT]; //homing directions from ReplicatorG
uint32_t feedrateXY; //ReplicatorG defined feedrate for the XY stage moves
uint32_t feedrateZ; //Replicatorg defined feedrate for the Z stage moves
uint16_t timeout_s; //TODO: Time 'till abort (not currently used)
int32_t zOffset; //varible to save the read amount for the Z Offset.
uint8_t EEPROM_direction[AXIS_COUNT]; //direction setting saved in EEPROM
int32_t EEPROM_DATA[4] = {0,0,0,0}; //Array to hold the 32bit read values.

//varibles for the Movecarefully script.
Point target;
int32_t Z_Offset; //z offset for the move carefully script.

//varibles for the homecarefully script
uint8_t homeDirection[AXIS_COUNT];
uint8_t other_axis_flags[AXIS_COUNT];

//Function Pointers
typedef void (*pointer2Subroutine)(); //function pointer type

pointer2Subroutine FirstTimeHomeSubroutineFunctionPointer[] = {&firstTimeHomeStep1,
&firstTimeHomeStep2, &firstTimeHomeStep3,&firstTimeHomeFinal, &firstTimeHomeWithZProbeStep1,
&firstTimeHomeWithZProbeStep2, &firstTimeHomeWithZProbeStep3 }; //pointer function array of all of the subroutines of FirstTimeHome
//Zero indexed.

pointer2Subroutine AutoHomeSubroutineFunctionPointer[] = {&autoHomeStep1, &autoHomeStep2, &autoHomeStep3, &autoHomeFinalEnd,
&autoHomeZProbeStep1, &autoHomeZProbeStep2, &autoHomeZProbeStep3}; //pointer function array of all of the subroutines of AutoHome

pointer2Subroutine MoveCarefullySubroutineFunctionPointer[] = {&moveCarefullyStep1, &moveCarefullyStep2, &moveCarefullyStep3,
&moveCarefullyStep4, &moveCarefullyFinalEnd}; //pointer function array of all of the subroutines of Move Carefully

pointer2Subroutine HomeCarefullySubroutineFunctionPointer[] = {&homeCarefullyStep1, &homeCarefullyStep2, &homeCarefullyStep3,
&homeCarefullyFinalEnd }; //pointer function array of all of the subroutines of Home Carefully



bool isRunning() { //program that can be called elsewhere in the firmware to check if the scripts are running or not. (False if not running)
if (ScriptRunning == NONE) {
return false;
} else {
return true;
}
}

void StartFirstAutoHome(uint8_t directionTemp[],uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp, uint16_t timeout_sTemp) {
//start the FirstAutoHome script
currentStep = 0; //start at the begining.
ScriptRunning = FIRSTAUTOHOME;
for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
direction[i] = directionTemp[i];
}
feedrateXY = XYfeedrateTemp;
feedrateZ = ZfeedrateTemp;
timeout_s = timeout_sTemp;
}

void StartAutoHome(uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp, uint16_t timeout_sTemp) {
//start the AutoHome script
currentStep = 0; //start at the begining.
ScriptRunning = AUTOHOME;
feedrateXY = XYfeedrateTemp;
feedrateZ = ZfeedrateTemp;
timeout_s = timeout_sTemp;
}

void StartMoveCarefully(const Point& targetT, int32_t Z_offsetT, uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp) {
lowScriptStep = 0;
LowScriptRunning = MOVECAREFULLY;
target = targetT;
Z_Offset = Z_offsetT;
feedrateXY = XYfeedrateTemp;
feedrateZ = ZfeedrateTemp;
}

void StartHomeCarefully(uint8_t directionTemp[], uint32_t XYfeedrateTemp, uint32_t ZfeedrateTemp) {
lowScriptStep = 0;
LowScriptRunning = HOMECAREFULLY;
for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
homeDirection[i] = directionTemp[i];
}
feedrateXY = XYfeedrateTemp;
feedrateZ = ZfeedrateTemp;
}

void RunScripts() { //script that checks if the makerbot can continue a script.
if (LowScriptRunning == NONE) {
if (ScriptRunning == FIRSTAUTOHOME) {
    FirstTimeHomeSubroutineFunctionPointer[currentStep](); //run the function based on current step

} else if (ScriptRunning == AUTOHOME) {
    //run the function based on current step. (using a array of function pointers)
    AutoHomeSubroutineFunctionPointer[currentStep]();
}

} else if (LowScriptRunning == MOVECAREFULLY) {
    //run current step by using a array of function pointers.
    MoveCarefullySubroutineFunctionPointer[lowScriptStep]();

} else if (LowScriptRunning == HOMECAREFULLY) { //AUTOHOME subroutine
    //run current step by using a array of function pointers.
    HomeCarefullySubroutineFunctionPointer[lowScriptStep]();
}
}


	/**
	**
	Start of the individual subroutines for each step.
	**
	**/

	/**
	**First Time Home (calibration) subroutines
	**/


void firstTimeHomeStep1() {
	//step one.
	int32_t x = 0; //set x
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps to 000

	int32_t dataa; //temporary varible.
	int8_t data8; //temporary varible
	int16_t offset; //this is where in eeprom we should start saving/reading.

	currentStep = 1;
	//home carefully! We don't want to break anything!
	if (direction[2] == 1 || direction[2] == 3) { //if we are homing down, it would be a good idea to lift the zstage a bit before begining.
		offset = AUTOHOMEOFFSET + 0xF; //offset of the saved Z offset amount.
		eeprom_read_block((void*)&zOffset, (const void*)offset, 4); //read it from eeprom
		steppers::setTarget(Point(0,0,zOffset), feedrateZ); // move to zoffset
	}
}

void firstTimeHomeStep2() {
//start homing
	if (!steppers::isRunning()) {
		//proceed with homing.
		if (direction[2] == 3) { //if using Z probe then...
		for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
			other_axis_flags[i] = direction[i]; //Save a snapshot of the direction
		}
		direction[2] = 0; //don't home Z
		steppers::startHoming(direction,feedrateXY); //home the others (XY)
		currentStep = 4; //skip to case 4

	} else { //if zprobe is not is use then proceed as normal
		StartHomeCarefully(direction, feedrateXY, feedrateZ);
		currentStep = 2;
	}
	}
	}

void firstTimeHomeStep3() {
 //Homing is done.
if (LowScriptRunning == NONE) {

Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
                                               //Squirrel everything into EEPROM for the long Winter.
       int16_t offset;
       uint8_t data8;
       int32_t dataa;

       for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
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
       currentStep = 3;
                                               }

}

void firstTimeHomeFinal() {
//wait for the script to finish before finishing the first autohome.
if (LowScriptRunning == NONE && !steppers::isRunning()) {
//Disable steppers.
for (uint8_t i = 0; i < COORDINATE_AXIS_COUNT; i++) {
	steppers::enableAxis(i, false);
}
currentStep = -1;
ScriptRunning = NONE;
}
}

void firstTimeHomeWithZProbeStep1() {
//wait for XY homing to finish
if (!steppers::isRunning()) {
//proceed with ZProbe
//save XY pos then center

Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
						//Squirrel everything into EEPROM for the long Winter.
	int16_t offset;
	uint8_t data8;
	int32_t dataa;
	for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
		direction[i] = other_axis_flags[i]; //Recover directions from snapshot
	}


	for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
	offset = AUTOHOMEOFFSET + i;
	data8 = direction[i];
	eeprom_write_block((const void*)&data8, (void*)offset, 1); //save the set direction in eeprom.
	}

	for (int i = 0; i < 2; i++) {
		offset = AUTOHOMEOFFSET + 0x3 + (i*0x4);
		dataa = currentPosition[i]; //Grab individual current position from current position X,Y, (Z is not ready yet)
		if (dataa < 0) { //If the position is negative, make it positive!
			dataa = -dataa;
		}
			eeprom_write_block((const void*)&dataa, (void*)offset, 4); //save it!
		}
//center BP
	int32_t x = 0;
	int32_t y = 0;
	int32_t z = currentPosition[2]; //keep this where it was
	int32_t dda = feedrateXY; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move everything back

//Lower servo (to give it time to lower)
//Read servo position values from EEPROM

	uint8_t LoweredPositiondata8;
	offset = AUTOHOMEOFFSET + 0x15;
	eeprom_read_block((void*)&LoweredPositiondata8, (const void*)offset, 1); //read direction


//To lower the servo you must send these commands:
    Timeout acquire_lock_timeout; //try to get a lock on the extruder.
	acquire_lock_timeout.start(HOST_TOOL_RESPONSE_TIMEOUT_MS);
	while (!tool::getLock()) {
		if (acquire_lock_timeout.hasElapsed()) {
			return;
		}
	} //locked!
	OutPacket& out = tool::getOutPacket();
	out.reset();
	out.append8(0); // TODO: tool index
	out.append8(SLAVE_CMD_SET_SERVO_1_POS); //send command
	out.append8(LoweredPositiondata8); //send pos (From EEPROM)
	tool::startTransaction(); //Send!
	tool::releaseLock(); //clean up after yourself.


	currentStep = 5;

}
}

void firstTimeHomeWithZProbeStep2() {
//xy is now centered. Home z downwards.
if (!steppers::isRunning()) {
    //Home down now....
	//maybe should wait till the servo is lowered all the way?... Nah....

	direction[0] = 0; //don't home X
	direction[1] = 0; //don't home Y
    steppers::startHoming(direction,feedrateZ); //home the Z axis.

currentStep = 6;
}
}

void firstTimeHomeWithZProbeStep3() {
//wait till done homing Z then save the current position in EEPROM and lift servo.
//Go back to 0,0,0 and end script.
if (!steppers::isRunning()) {

Point currentPosition = steppers::getPosition(); //get position and put in point currentPosition
                                               //Squirrel everything into EEPROM for the long Winter.
        int16_t offset;
        uint8_t data8;
        int32_t dataa;

               offset = AUTOHOMEOFFSET + 0x3 + (2*0x4);
               dataa = currentPosition[2]; //Grab individual current position from current position,Z.

               eeprom_write_block((const void*)&dataa, (void*)offset, 4); //save it!

    //Read servo position values from EEPROM

	uint8_t RaisedPositiondata8;
	offset = AUTOHOMEOFFSET + 0x14;
	eeprom_read_block((void*)&RaisedPositiondata8, (const void*)offset, 1); //read direction


    //To lower the servo you must send these commands:
    Timeout acquire_lock_timeout; //try to get a lock on the extruder.
	acquire_lock_timeout.start(HOST_TOOL_RESPONSE_TIMEOUT_MS);
	while (!tool::getLock()) {
		if (acquire_lock_timeout.hasElapsed()) {
			return;
		}
	} //locked!
	OutPacket& out = tool::getOutPacket();
	out.reset();
	out.append8(0); // TODO: tool index
	out.append8(SLAVE_CMD_SET_SERVO_1_POS); //send command
	out.append8(RaisedPositiondata8); //send pos (From EEPROM)
	tool::startTransaction(); //Send!
	tool::releaseLock(); //clean up after yourself.

    int32_t x = 0;
	int32_t y = 0;
	int32_t z = 0; //Zero all
	int32_t dda = feedrateZ; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda); //move everything to 000

	currentStep = 3; //end script


}
}


/**
**Auto Home Subroutines
**/


void autoHomeStep1() {
    int32_t x = 0; //set x. Zero current position
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps
	//Read values from EEPROM

	//EEPROM MAP
	//0x100, 0x101, 0x102 Directions of XYZ.
	//0x103-06, 0x107-0x10a, 0x10b-0x10e Steps to move.
	//0x10f-0x112 MM to move up on Z axis.
	//0x113 tool index for Z-probe
	//0x114 lift angle for Z-Probe servo
	//0x115 lowered angle for Z-Probe servo

	int16_t offset; //where to start copying from.
	uint8_t data8;
	for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
	offset = AUTOHOMEOFFSET + i;
	eeprom_read_block((void*)&data8, (const void*)offset, 1); //read direction
	EEPROM_direction[i] = data8;
	}

	for (int i = 0; i < (COORDINATE_AXIS_COUNT + 1); i++) { //loop and copy all of the data for all of the axis.
	offset = AUTOHOMEOFFSET + 0x3 + (i*0x4);
	eeprom_read_block((void*)&EEPROM_DATA[i], (const void*)offset, 4);

	}
	//direction = (EEPROM_direction > 0);
	currentStep = 1;

	if (EEPROM_direction[2] == 1 || EEPROM_direction[2] == 3) { //if we are homing down, it would be a good idea to lift the zstage a bit before begining. (Just in case)
		steppers::setTarget(Point(0,0,EEPROM_DATA[3]), feedrateZ); // move to zoffset
	}

}

void autoHomeStep2() {
    //wait 'till step 1 is done, before commencing the homing sequence.
    if (!steppers::isRunning()) {
        if (EEPROM_direction[2] == 3) { //if we are using Z-Probe hardware...
            //Home XY then center....
            //save a snapshot.
            for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
            other_axis_flags[i] = EEPROM_direction[i]; //Save a snapshot of the directions.
            }
            EEPROM_direction[2] = 0; //dont home Z

            steppers::startHoming(EEPROM_direction, feedrateXY); //home XY

            //need to redirect to other current step
            currentStep = 4; //next ZProbe step
        } else { //if Z-Probe is not installed then proceed as normal
    StartHomeCarefully(EEPROM_direction, feedrateXY, feedrateZ); //home carefully!
    currentStep = 2;
        }
    }
}

void autoHomeZProbeStep1() {
//wait till XY homing is done.
//center XY, lower servo.
if (!steppers::isRunning()) {

    //recover our original directions
    for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
            EEPROM_direction[i] = other_axis_flags[i]; //read a snapshot of the directions.
    }

    int32_t x = 0; //set x at zero (we are at endstop)
	int32_t y = 0; //set y
	int32_t z = 0; //set z (we are not at endstop but this is still ok.)
	steppers::definePosition(Point(x,y,z)); //set the position in steps
	//next move back up the read amount (aka build platform center)

	for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
	if (EEPROM_direction[i] == 2 || EEPROM_direction[i] == 3) { //If the direction homed was up, then make the direction to move negative.(this applies to Z_Probes)
	EEPROM_DATA[i] = -EEPROM_DATA[i];
	}
	}

    steppers::setTarget(Point(EEPROM_DATA[0],EEPROM_DATA[1],0), feedrateXY); // Center XY

    //Lower servo.
    //Read servo position values from EEPROM

	uint8_t LoweredPositiondata8;
	int16_t offset = AUTOHOMEOFFSET + 0x15;
	eeprom_read_block((void*)&LoweredPositiondata8, (const void*)offset, 1); //read direction


    //To lower the servo you must send these commands:
    Timeout acquire_lock_timeout; //try to get a lock on the extruder.
	acquire_lock_timeout.start(HOST_TOOL_RESPONSE_TIMEOUT_MS);
	while (!tool::getLock()) {
		if (acquire_lock_timeout.hasElapsed()) {
			return;
		}
	} //locked!
	OutPacket& out = tool::getOutPacket();
	out.reset();
	out.append8(0); // TODO: tool index
	out.append8(SLAVE_CMD_SET_SERVO_1_POS); //send command
	out.append8(LoweredPositiondata8); //send pos (From EEPROM)
	tool::startTransaction(); //Send!
	tool::releaseLock(); //clean up after yourself.
	//go to next command.
	currentStep = 5;


}
}

void autoHomeZProbeStep2() {
//wait till the platform is centered then home the ZAxis down
if (!steppers::isRunning()) {
    EEPROM_direction[0] = 0;
    EEPROM_direction[1] = 0;
    steppers::startHoming(EEPROM_direction, feedrateZ); //home XY

    //go to next step
    currentStep = 6;
}
}

void autoHomeZProbeStep3() {
    //ZProbe. after homing, lift servo and lower by amount saved in EEPROM (the probe is longer than the nozzle)
    //Center everything and go to last step (quit script)
    if (!steppers::isRunning()) {
    //Lift servo.
    //Read servo position values from EEPROM
	uint8_t RaisedPositiondata8;
	int16_t offset = AUTOHOMEOFFSET + 0x14;
	eeprom_read_block((void*)&RaisedPositiondata8, (const void*)offset, 1); //read direction


    //To lower the servo you must send these commands:
    Timeout acquire_lock_timeout; //try to get a lock on the extruder.
	acquire_lock_timeout.start(HOST_TOOL_RESPONSE_TIMEOUT_MS);
	while (!tool::getLock()) {
		if (acquire_lock_timeout.hasElapsed()) {
			return;
		}
	} //locked!
	OutPacket& out = tool::getOutPacket();
	out.reset();
	out.append8(0); // TODO: tool index
	out.append8(SLAVE_CMD_SET_SERVO_1_POS); //send command
	out.append8(RaisedPositiondata8); //send pos (From EEPROM)
	tool::startTransaction(); //Send!
	tool::releaseLock(); //clean up after yourself.

	//set z to zero (we are at endstop)
	int32_t x = EEPROM_DATA[0]; //set x at zero (we are at endstop)
	int32_t y = EEPROM_DATA[1]; //set y
	int32_t z = 0; //set z (we are not at endstop but this is still ok.)
	steppers::definePosition(Point(x,y,z)); //set the position in steps


	steppers::setTarget(Point(EEPROM_DATA[0],EEPROM_DATA[1],EEPROM_DATA[2]), feedrateZ); // Center all (mainly Z)
    //go to clean up step
    currentStep = 3;

    }

}

void autoHomeStep3() {
    //If the previous step is done, then move carefully to the 0,0,0 position.
if (LowScriptRunning == NONE) {
	int32_t x = 0; //set x at zero (we are at endstop)
	int32_t y = 0; //set y
	int32_t z = 0; //set z
	steppers::definePosition(Point(x,y,z)); //set the position in steps
	//next move back up the read amount (aka build platform height)

	for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
	if (EEPROM_direction[i] == 2) { //If the direction homed was up, then make the direction to move negative.
	EEPROM_DATA[i] = -EEPROM_DATA[i];
	}
	}

	if (EEPROM_direction[2] == 1) { //If we homed down then raise the Z first.

		StartMoveCarefully(Point(EEPROM_DATA[0],EEPROM_DATA[1],EEPROM_DATA[2]), EEPROM_DATA[3], feedrateXY, feedrateZ); //move carefully (raise the Z stage first)
	} else { //else move the XY first
		StartMoveCarefully(Point(EEPROM_DATA[0],EEPROM_DATA[1],EEPROM_DATA[2]), -1, feedrateXY, feedrateZ); //move carefully
}


currentStep = 3;

}
}

void autoHomeFinalEnd() {
    //check if the script has ended then end the script.
if (LowScriptRunning == NONE && !steppers::isRunning()) {
steppers::definePosition(Point(0,0,0)); //set the position in steps to zero (we are at center of BP)
//Disable steppers.
	for (uint8_t i = 0; i < COORDINATE_AXIS_COUNT; i++) {
		steppers::enableAxis(i, false);
	}
currentStep = -1;
ScriptRunning = NONE;
}

}



/**
**Move carefully subroutine steps
**/

void moveCarefullyStep1() {
//check all of the passed varibles and decide whether to center XY first or move Z first.

if (Z_Offset < 0) { //if the z offset is negative (this should not usually happen). assume we want to Center the XY before Z (as if we are centering the Z stage in the - direction) AKA from the top.
	Point currentPosition = steppers::getPosition();
	int32_t x = target[0]; //Move XY
	int32_t y = target[1];
	int32_t z = currentPosition[2]; //Leave this alone.
	int32_t dda = feedrateXY; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move XY
	lowScriptStep = 1;
} else { //we must be moving in the positive direction so move Z before XY.
	Point currentPosition = steppers::getPosition();
	int32_t x = currentPosition[0]; //leave these were they are
	int32_t y = currentPosition[1];
	int32_t z = target[2] + Z_Offset; //move the Z stage back up to a bit above zero to avoid the BP hitting it.
	int32_t dda = feedrateZ; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda);
	lowScriptStep = 2;
	}


}

void moveCarefullyStep2() {
//wait for XY stage to finish moving
if (!steppers::isRunning()){
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2]; //move Z down
	int32_t dda = feedrateZ; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda); //move everything
	lowScriptStep = 4; //wait to say that you are finished.
	}
}

void moveCarefullyStep3() {
//wait for Z
if (!steppers::isRunning()) {
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2] + Z_Offset; //keep this where it was
	int32_t dda = feedrateXY; // max feedrate for XY stage
	steppers::setTarget(Point(x,y,z),dda); //move everything back up
	lowScriptStep = 3;
	}
}

void moveCarefullyStep4() {
 //wait for XY
if (!steppers::isRunning()) {
	int32_t x = target[0];
	int32_t y = target[1];
	int32_t z = target[2]; //move back down to 000
	int32_t dda = feedrateZ; // max feedrate for Z stage
	steppers::setTarget(Point(x,y,z),dda); //move everything back up
	lowScriptStep = 4; //wait to say that this script is finished
}
}

void moveCarefullyFinalEnd() {
//script finished. Cleanup time!
if (!steppers::isRunning()) {
lowScriptStep = -1;
LowScriptRunning = NONE;
}
}


/**
**Home carefully subroutine steps
**/

void homeCarefullyStep1() {
//start home carefully
if (!steppers::isRunning()) {
//proceed with homing.

	if (homeDirection[2] == 1) { //if flags says home Z and something else (we don't care what). And it's also in the negative direction then home carefully.
					//Don't home Z.
					for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
					other_axis_flags[i] = homeDirection[i]; //Save a snapshot of the directions
					}
					homeDirection[2] = 0; //don't home Z
					steppers::startHoming(homeDirection,feedrateXY); //home the others (XY)
						lowScriptStep = 1;


	} else if (homeDirection[2] == 2) { //else if we are homing the Z stage up then...
						//home the z up before homing xy in the positive direction.
						for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
						other_axis_flags[i] = homeDirection[i]; // Save axis besides Z to home.
						homeDirection[i] = 0; //reset
						}

						homeDirection[2] = 2; // Home Z up.
					steppers::startHoming(homeDirection,feedrateZ); //home Z
						lowScriptStep = 2;
	} else { //If it does not involve the Z axis, no special care is needed.
						steppers::startHoming(homeDirection, feedrateXY);
							lowScriptStep = 3;
							}
				}

}

void homeCarefullyStep2() {
//wait till Xy is homed
if (!steppers::isRunning()) {
	for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
	homeDirection[i] = 0;
	}
	homeDirection[2] = other_axis_flags[2];
	//Home Z.
	steppers::startHoming(homeDirection, feedrateZ);
	lowScriptStep = 3;
	}
}

void homeCarefullyStep3() {
//wait till Z is homed
if (!steppers::isRunning()) {
for (int i = 0; i < COORDINATE_AXIS_COUNT; i++) {
homeDirection[i] = other_axis_flags[i]; //Home the rest of the axis (besides Z).
}
	//Home the rest of the axis (besides Z).
	homeDirection[2] = 0;
	steppers::startHoming(homeDirection, feedrateXY);
	lowScriptStep = 3;
}
}

void homeCarefullyFinalEnd() {
//wait till homing is done then reset low scripts.
if (!steppers::isRunning()) {
lowScriptStep = -1;
LowScriptRunning = NONE;
}
}

}
