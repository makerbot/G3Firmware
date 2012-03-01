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

#include "Main.hh"
#include "DebugPacketProcessor.hh"
#include "Host.hh"
#include "Tool.hh"
#include "Command.hh"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "Timeout.hh"
#include "Steppers.hh"
#include "Motherboard.hh"
#include "SDCard.hh"
#include "Eeprom.hh"
#include "EepromMap.hh"
#include "Errors.hh"


#ifdef HAS_ATX_POWER_GOOD
/// Workaround for hardware issue, where powering on with USB connected
/// cause blank LCD, or corrupted LCD, requiring reset.
volatile bool atxLastPowerGood;
#endif


void reset(bool hard_reset) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		Motherboard& board = Motherboard::getBoard();
		sdcard::reset();
		steppers::abort();
		steppers::reset();
		command::reset();
		eeprom::init();
		board.reset(hard_reset);

#ifdef HAS_ESTOP
		const uint8_t estop_conf = eeprom::getEeprom8(eeprom::ESTOP_CONFIGURATION, 0);
		if (estop_conf == eeprom::ESTOP_CONF_ACTIVE_HIGH) {
			ESTOP_ENABLE_RISING_INT;
		} else if (estop_conf == eeprom::ESTOP_CONF_ACTIVE_LOW) {
			ESTOP_ENABLE_FALLING_INT;
		}
#endif

#ifdef HAS_ATX_POWER_GOOD
		/// Workaround for hardware issue, where powering on with USB connected
		/// cause blank LCD, or corrupted LCD, requiring reset.
		ATX_POWER_GOOD.setDirection(false);
		atxLastPowerGood = ATX_POWER_GOOD.getValue();
#endif

		sei();
		// If we've just come from a hard reset, wait for 2.5 seconds before
		// trying to ping an extruder.  This gives the extruder time to boot
		// before we send it a packet.
		if (hard_reset) {
			Timeout t;
			t.start(1000L*2500L); // wait for 2500 ms
			while (!t.hasElapsed());
			tool::test(); // Run test
		}
		if (!tool::reset())
		{
			// Fail, but let it go; toggling the PSU is dangerous.
		}
	}
}

int main() {

	Motherboard& board = Motherboard::getBoard();
	steppers::init(Motherboard::getBoard());
	reset(true);
	sei();
	while (1) {
		// Toolhead interaction thread.
		tool::runToolSlice();
		// Host interaction thread.
		host::runHostSlice();
		// Command handling thread.
		command::runCommandSlice();
		// Motherboard slice
		board.runMotherboardSlice();

#ifdef HAS_ATX_POWER_GOOD
		/// Workaround for hardware issue, where powering on with USB connected
		/// cause blank LCD, or corrupted LCD, requiring reset.
		/// If we're running here and the last time we looped, power was not good,
		/// and now power is good, someone just hit the power button, so we force a
		/// reset
		bool powerGood = ATX_POWER_GOOD.getValue();
		if (( ! atxLastPowerGood ) && ( powerGood )) {
			host::resetBuild();
			reset(true);
		}
		atxLastPowerGood = powerGood;
#endif
	}
	return 0;
}

#ifdef HAS_ESTOP
ISR(ESTOP_vect, ISR_NOBLOCK) {
	// Emergency stop triggered; reset everything and kill the interface to RepG
	INTERFACE_BAR_PIN.setValue(true);
	INTERFACE_BAR_PIN.setDirection(true);
	tool::reset();
	steppers::abort();
	command::reset();
	UART::getHostUART().enable(false);
	Motherboard::getBoard().indicateError(ERR_ESTOP);
	Motherboard::getBoard().buzz(7, 10, eeprom::getEeprom8(eeprom::BUZZER_REPEATS, 3));
  
}
#endif

