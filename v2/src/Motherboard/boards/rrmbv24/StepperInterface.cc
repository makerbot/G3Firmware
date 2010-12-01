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

#include "StepperInterface.hh"
#include "EepromMap.hh"
#include "Configuration.hh"

void StepperInterface::setDirection(bool forward) {
	if (invert_axis) forward = !forward;
	dir_pin.setValue(forward);
}

void StepperInterface::step(bool value) {
	step_pin.setValue(value);
}

void StepperInterface::setEnabled(bool enabled) {
	// The A3982 stepper driver chip has an inverted enable.
	enable_pin.setValue(!enabled);
}

bool StepperInterface::isAtMaximum() {
	if (max_pin.isNull()) return false;
	bool v = max_pin.getValue();
	if (invert_endstops) v = !v;
	return v;
}

bool StepperInterface::isAtMinimum() {
	if (min_pin.isNull()) return false;
	bool v = min_pin.getValue();
	if (invert_endstops) v = !v;
	return v;
}

void StepperInterface::init(uint8_t idx) {
	dir_pin.setDirection(true);
	step_pin.setDirection(true);
	enable_pin.setValue(true);
	enable_pin.setDirection(true);
	// get inversion characteristics
	uint8_t axes_invert = eeprom::getEeprom8(eeprom::AXIS_INVERSION, 1<<1);
#ifdef DEFAULT_INVERSIONS
	axes_invert ^= DEFAULT_INVERSIONS;
#endif
	uint8_t endstops_invert = eeprom::getEeprom8(eeprom::ENDSTOP_INVERSION, 0);
	bool endstops_present = (endstops_invert & (1<<7)) != 0;
	// If endstops are not present, then we consider them inverted, since they will
	// always register as high (pulled up).
	invert_endstops = !endstops_present || ((endstops_invert & (1<<idx)) != 0);
	invert_axis = (axes_invert & (1<<idx)) != 0;
	// pull pins up to avoid triggering when using inverted endstops
	if (!max_pin.isNull()) {
		max_pin.setDirection(false);
		max_pin.setValue(invert_endstops);
	}
	if (!min_pin.isNull()) {
		min_pin.setDirection(false);
		min_pin.setValue(invert_endstops);
	}
}
