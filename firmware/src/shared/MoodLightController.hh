/* 
 * Mood Light Controller
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

#ifndef MOODLIGHTCONTROLLER_HH_
#define MOODLIGHTCONTROLLER_HH_

#include "BlinkMMaxM.hh"
#include <avr/pgmspace.h>


enum moodLightStatus {
	MOOD_LIGHT_STATUS_NOT_SET = 0,
	MOOD_LIGHT_STATUS_IDLE,
	MOOD_LIGHT_STATUS_HEATING,
	MOOD_LIGHT_STATUS_COOLING,
	MOOD_LIGHT_STATUS_PRINTING,
	MOOD_LIGHT_STATUS_ERROR
};


class MoodLightController {
public:
	BlinkMMaxM blinkM;

	MoodLightController(Pin sda, Pin sdl);
 
	//Used for Menu
	const PROGMEM prog_uchar *scriptIdToStr(uint8_t scriptId);
	uint8_t nextScriptId(uint8_t currentScriptId);
	uint8_t prevScriptId(uint8_t currentScriptId);


	bool start();
	void playScript(uint8_t scriptId);

	void debugLightSetValue(bool value);
	void displayStatus(enum moodLightStatus status, float percentHot);
	enum moodLightStatus getLastStatus();
	uint8_t getLastScriptPlayed();

private:
	bool	eStopTriggered;
	Pin	_sda, _scl;
};

#endif // MOODLIGHTCONTROLLER_HH_
