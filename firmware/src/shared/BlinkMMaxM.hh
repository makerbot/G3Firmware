/* 
 * BlinkM MaxM Support over Software I2C
 *
 * Reference:
 *	http://thingm.com/products/blinkm-maxm.html
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

#ifndef BLINKMMAXM_HH_
#define BLINKMMAXM_HH_

#include "SoftwareI2C.hh"


typedef struct _scriptLine {
	uint8_t duration;
  	uint8_t cmd[4];		//command, arg1, arg2, arg3
} scriptLine;


class BlinkMMaxM {
public:
	BlinkMMaxM();
	BlinkMMaxM(Pin sda, Pin sdl);
	void init(Pin sda, Pin sdl);
	
	bool blinkMIsPresent;

	bool scriptPlaying;

	void stopScript();
	void setFadeSpeed(uint8_t f);
	void setTimeAdjust(int8_t t);
	uint8_t getFadeSpeed();
	void fadeToRGB( uint8_t r, uint8_t g, uint8_t b );
	void setRGB(uint8_t r, uint8_t g, uint8_t b );
	void setAddress(uint8_t addr);
	int  getVersion();
	void fadeToHSB( uint8_t h, uint8_t s, uint8_t b );
        void playScript(uint8_t scriptId, uint8_t reps);
	void getRGBColor(uint8_t *r, uint8_t *g, uint8_t *b);
	void factoryReset();
	void writeScript(uint8_t script_id, uint8_t len, uint8_t reps, scriptLine *lines);

private:
	SoftwareI2C softI2C;
	uint8_t blinkMAddr;
	uint8_t fadeSpeed;

	void sendN(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6, uint8_t arg7, int countArgs);
	void send0(uint8_t cmd);
	void send1(uint8_t cmd, uint8_t arg1);
	void send2(uint8_t cmd, uint8_t arg1, uint8_t arg2);
	void send3(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3);
	void send4(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4);
	void send5(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5);
	void send6(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6);
	void send7(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6, uint8_t arg7);
	
	void recv1(uint8_t cmd, uint8_t *arg1);
	void recv2(uint8_t cmd, uint8_t *arg1, uint8_t *arg2);
	void recv3(uint8_t cmd, uint8_t *arg1, uint8_t *arg2, uint8_t *arg3);

};

#endif // BLINKMMAXM_HH_
