/*
 * Copyright 2010 by Adam Mayer <adam@makerbot.com>
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

#include "MotorController.hh"
#include "ExtruderBoard.hh"
#include "Eeprom.hh"
#include "EepromMap.hh"

MotorController::MotorController() {
	reset();
}

void MotorController::reset() {
	direction = true;
	paused = false;
	on = false;
	speed = 0;
	set_with_rpm = false;
	backoff_state = BO_INACTIVE;
	loadBackoffParameters();
}

#define DEFAULT_HALT_MS 5L
#define DEFAULT_REVERSE_MS 500L
#define DEFAULT_FORWARD_MS 300L
#define DEFAULT_TRIGGER_MS 300L

void MotorController::loadBackoffParameters()
{
	backoff_enabled = false; // TODO: fixme
	halt_ms = eeprom::getEeprom16(eeprom::BACKOFF_STOP_TIME,DEFAULT_HALT_MS);
	reverse_ms = eeprom::getEeprom16(eeprom::BACKOFF_REVERSE_TIME,DEFAULT_REVERSE_MS);
	forward_ms = eeprom::getEeprom16(eeprom::BACKOFF_FORWARD_TIME,DEFAULT_FORWARD_MS);
	trigger_ms = eeprom::getEeprom16(eeprom::BACKOFF_TRIGGER_TIME,DEFAULT_TRIGGER_MS);
}

void MotorController::update() {
	ExtruderBoard& board = ExtruderBoard::getBoard();
	if (backoff_enabled && backoff_state != BO_INACTIVE) {
		if (current_operation_timeout.hasElapsed()) {
			switch (backoff_state) {
			case BO_HALT_1:
				backoff_state = BO_REVERSE;
				current_operation_timeout.start(reverse_ms*1000L);
				board.setMotorSpeed(-speed);
				break;
			case BO_REVERSE:
				backoff_state = BO_HALT_2;
				current_operation_timeout.start(halt_ms*1000L);
				board.setMotorSpeed(0);
				break;
			case BO_HALT_2:
				backoff_state = BO_FORWARD;
				current_operation_timeout.start(forward_ms*1000L);
				board.setMotorSpeed(speed);
				break;
			case BO_FORWARD:
				board.setMotorSpeed(0);
				backoff_state = BO_INACTIVE;
				break;
			}
		}
	} else if (!set_with_rpm) {
		int new_speed = (!paused&&on)?(direction?speed:-speed):0;
		board.setMotorSpeed(new_speed);
	} else {
#ifdef DEFAULT_EXTERNAL_STEPPER
		board.setMotorSpeedRPM(rpm, direction);
		board.setMotorOn(!paused && on);
#else
		board.setMotorSpeedRPM((!paused&&on) ? rpm : 0, direction);
#endif
	}

}

void MotorController::setSpeed(int speed_in) {
	speed = speed_in;
	set_with_rpm = false;
}

void MotorController::setRPMSpeed(uint32_t speed_in) {
	rpm = speed_in;
	set_with_rpm = true;
}

uint32_t MotorController::getRPMSpeed() {
	return rpm;
}
int MotorController::getSpeed() {
	return speed;
}

void MotorController::pause() {
	paused = !paused;
	//ExtruderBoard::getBoard().indicateError(paused?1:0);

}

void MotorController::setDir(bool dir_in) {
	direction = dir_in;
}

void MotorController::setOn(bool on_in) {
	ExtruderBoard& board = ExtruderBoard::getBoard();
	if (!on_in && on && direction && backoff_enabled && forward_trigger_timeout.hasElapsed()) {
		backoff_state = BO_HALT_1;
   // Commented out since this is handled in MotorController::update(),
   // kept around for future reference
   // board.setMotorSpeed(0);
		current_operation_timeout.start(halt_ms*1000L);
	} else if (on_in) {
		if (!on && direction) {
			forward_trigger_timeout.start(trigger_ms*1000L);
		}
		backoff_state = BO_INACTIVE;
	}
	on = on_in;
}
