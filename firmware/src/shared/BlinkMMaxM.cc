/*
 * BlinkM MaxM Support over Software I2C
 *
 * Reference:
 *      http://thingm.com/products/blinkm-maxm.html
 *
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

#include <util/delay.h>
#include <string.h>
#include "BlinkMMaxM.hh"

#define BLINKM_PRESENT_CHECK	if ( ! blinkMIsPresent ) return

void BlinkMMaxM::sendN(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6, uint8_t arg7, int countArgs) {
	softI2C.start( blinkMAddr, false );
	softI2C.txByte( cmd );

    	if ( countArgs >= 1 )	softI2C.txByte( arg1 );
    	if ( countArgs >= 2 )	softI2C.txByte( arg2 );
    	if ( countArgs >= 3 )	softI2C.txByte( arg3 );
    	if ( countArgs >= 4 )	softI2C.txByte( arg4 );
    	if ( countArgs >= 5 )	softI2C.txByte( arg5 );
    	if ( countArgs >= 6 )	softI2C.txByte( arg6 );
    	if ( countArgs >= 7 )	softI2C.txByte( arg7 );
	
    	softI2C.stop();
}

void BlinkMMaxM::send0(uint8_t cmd) {
	sendN(cmd, 0, 0, 0, 0, 0, 0, 0, 0);
}

void BlinkMMaxM::send1(uint8_t cmd, uint8_t arg1) {
	sendN(cmd, arg1, 0, 0, 0, 0, 0, 0, 1);
}

void BlinkMMaxM::send2(uint8_t cmd, uint8_t arg1, uint8_t arg2) {
	sendN(cmd, arg1, arg2, 0, 0, 0, 0, 0, 2);
}

void BlinkMMaxM::send3(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3) {
	sendN(cmd, arg1, arg2, arg3, 0, 0, 0, 0, 3);
}

void BlinkMMaxM::send4(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4) {
	sendN(cmd, arg1, arg2, arg3, arg4, 0, 0, 0, 4);
}

void BlinkMMaxM::send5(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5) {
	sendN(cmd, arg1, arg2, arg3, arg4, arg5, 0, 0, 5);
}

void BlinkMMaxM::send6(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6) {
	sendN(cmd, arg1, arg2, arg3, arg4, arg5, arg6, 0,  6);
}

void BlinkMMaxM::send7(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6, uint8_t arg7) {
	sendN(cmd, arg1, arg2, arg3, arg4, arg5, arg6, arg7, 7);
}

void BlinkMMaxM::recv1(uint8_t cmd, uint8_t *arg1) {
    softI2C.start( blinkMAddr, false);
    softI2C.txByte( cmd );
    softI2C.stop();
    softI2C.start( blinkMAddr, true );
    *arg1 = softI2C.rxByte(0);
    softI2C.stop();
}

void BlinkMMaxM::recv2(uint8_t cmd, uint8_t *arg1, uint8_t *arg2) {
    softI2C.start( blinkMAddr, false);
    softI2C.txByte( cmd );
    softI2C.stop();
    softI2C.start( blinkMAddr, true );
    *arg1 = softI2C.rxByte(1);
    *arg2 = softI2C.rxByte(0);
    softI2C.stop();
}

void BlinkMMaxM::recv3(uint8_t cmd, uint8_t *arg1, uint8_t *arg2, uint8_t *arg3) {
    softI2C.start( blinkMAddr, false);
    softI2C.txByte( cmd );
    softI2C.stop();
    softI2C.start( blinkMAddr, true );
    *arg1 = softI2C.rxByte(1);
    *arg2 = softI2C.rxByte(1);
    *arg3 = softI2C.rxByte(0);
    softI2C.stop();
}


BlinkMMaxM::BlinkMMaxM() {
}


BlinkMMaxM::BlinkMMaxM(Pin sda, Pin scl) {
	init(sda, scl);
}


void BlinkMMaxM::init(Pin sda, Pin scl) {
	blinkMIsPresent = false;
    	blinkMAddr = 0x09;
	softI2C.init(sda, scl);
	
	//Wait for BlinkM to come online
	_delay_us(500000);

        //Override and set the BlinkM address back to the default (9), as it 
	//save wasting time trying to locate it
	blinkMIsPresent = true;
        setAddress(blinkMAddr);

	//Get the version to see if we have a BlinkM Present
        int vers = getVersion();
	uint8_t major = vers >> 8;
	uint8_t minor = vers & 0x00FF;

	blinkMIsPresent= false;
	if ( major == 'a' ) {
		if (( minor >= 'a' ) && ( minor <= 'd' ))
			blinkMIsPresent = true;
	}

	factoryReset();
}


void BlinkMMaxM::stopScript() {
	BLINKM_PRESENT_CHECK;
	scriptPlaying = false;
	send0('o');
}


void BlinkMMaxM::setFadeSpeed( uint8_t f) {
	BLINKM_PRESENT_CHECK;

	if ( f == fadeSpeed )	return;

	send1('f', f);
	fadeSpeed = f;
	
	_delay_us(10000);
}


void BlinkMMaxM::setTimeAdjust(int8_t t) {
	BLINKM_PRESENT_CHECK;

	//Convert the signed byte to unsigned
	uint8_t v;
	memcpy((void *)v, (void *)t, sizeof(int8_t));

	send1('t', v);
	_delay_us(10000);
}
 

uint8_t BlinkMMaxM::getFadeSpeed() {
	if ( ! blinkMIsPresent )	return 0;
	return fadeSpeed;
}


void BlinkMMaxM::fadeToRGB( uint8_t r, uint8_t g, uint8_t b ) {
	BLINKM_PRESENT_CHECK;
	if ( scriptPlaying ) stopScript();
 	send3('c', r, g, b);
}


void BlinkMMaxM::setRGB( uint8_t r, uint8_t g, uint8_t b ) {
	BLINKM_PRESENT_CHECK;
	if ( scriptPlaying )	stopScript();
	send3('n', r, g, b);
}


void BlinkMMaxM::setAddress(uint8_t addr) {
	BLINKM_PRESENT_CHECK;

	//Send to 0x00, the broadcast address
	blinkMAddr = 0x00;
	send4('A', addr, 0xD0, 0x0D, addr );
	blinkMAddr = addr;
    	_delay_us(50000);
}


int BlinkMMaxM::getVersion() {
	if ( ! blinkMIsPresent )	return -1;

	uint8_t major, minor;

	recv2('Z', &major, &minor);

	return (major << 8) + minor;
}


void BlinkMMaxM::fadeToHSB( uint8_t h, uint8_t s, uint8_t b ) {
	BLINKM_PRESENT_CHECK;
	if ( scriptPlaying ) stopScript();
	send3('h', h, s, b);
}


void BlinkMMaxM::playScript(uint8_t scriptId, uint8_t reps) {
	BLINKM_PRESENT_CHECK;
	scriptPlaying = true;
	send3('p', scriptId, reps, 0);
}


void BlinkMMaxM::getRGBColor(uint8_t *r, uint8_t *g, uint8_t *b) {
	BLINKM_PRESENT_CHECK;
	recv3('g', r, g, b);
}


void BlinkMMaxM::writeScript(uint8_t script_id, uint8_t len, uint8_t reps, scriptLine *lines) {
	BLINKM_PRESENT_CHECK;

	for (uint8_t i = 0; i < len; i++ ) {
		scriptLine l = lines[i];
		send7('W', script_id, i, l.duration, l.cmd[0], l.cmd[1], l.cmd[2], l.cmd[3]);
    		_delay_us(20000);
	}

	send3('L', script_id, len, reps );
}


void BlinkMMaxM::factoryReset() {
	BLINKM_PRESENT_CHECK;

	//Set address to defaults
  	setAddress(0x09);

	//Write defaults
 	send5('B', 
	      0x01,	//Play Script (0x01)
	      0x00,	//Script 0
	      0x00,	//Repeat indefinately
	      0x08,	//Fade speed
	      0x00);	//Time adjust

	_delay_us(50000);

	scriptLine script[] = {
		{  1,  {'f',   10,  00,  00}},	//Fade Speed 10
		{ 100, {'c', 0xff,0xff,0xff}},  //White
		{  50, {'c', 0xff,0x00,0x00}},  //Red
		{  50, {'c', 0x00,0xff,0x00}},  //Green
		{  50, {'c', 0x00,0x00,0xff}},  //Blue
		{  50, {'c', 0x00,0x00,0x00}},  //Off
	};
	int len = 6;  // number of script lines above
  
	writeScript( 0, len, 0, script);

	_delay_us(50000);
}
