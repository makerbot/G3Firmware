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

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "Motherboard.hh"
#include "Configuration.hh"
#include "Steppers.hh"
#include "Command.hh"
#include "Eeprom.hh"
#include "EepromMap.hh"
#include "EepromDefaults.hh"
#include "StepperAccelPlanner.hh"

/// Instantiate static motherboard instance
Motherboard Motherboard::motherboard(PSU_PIN);

/// Create motherboard object
Motherboard::Motherboard(const Pin& psu_pin) :
        psu(psu_pin)
{
	/// Set up the stepper pins on board creation
#if STEPPER_COUNT > 0
        stepper[0] = StepperInterface(X_DIR_PIN,
                                      X_STEP_PIN,
                                      X_ENABLE_PIN,
                                      X_MAX_PIN,
                                      X_MIN_PIN,
                                      eeprom::AXIS_INVERSION);
#endif
#if STEPPER_COUNT > 1
        stepper[1] = StepperInterface(Y_DIR_PIN,
                                      Y_STEP_PIN,
                                      Y_ENABLE_PIN,
                                      Y_MAX_PIN,
                                      Y_MIN_PIN,
                                      eeprom::AXIS_INVERSION);
#endif
#if STEPPER_COUNT > 2
        stepper[2] = StepperInterface(Z_DIR_PIN,
                                      Z_STEP_PIN,
                                      Z_ENABLE_PIN,
                                      Z_MAX_PIN,
                                      Z_MIN_PIN,
                                      eeprom::AXIS_INVERSION);
#endif
#if STEPPER_COUNT > 3
        stepper[3] = StepperInterface(A_DIR_PIN,
                                      A_STEP_PIN,
                                      A_ENABLE_PIN,
                                      Pin(),
                                      Pin(),
                                      eeprom::AXIS_INVERSION);
#endif
}

void Motherboard::setupFixedStepperTimer() {
        TCCR1A = 0x00;
        TCCR1B = 0x09;
        TCCR1C = 0x00;
        OCR1A = INTERVAL_IN_MICROSECONDS * 16;
        TIMSK1 = 0x02; // turn on OCR1A match interrupt
}

void Motherboard::setupAccelStepperTimer() {
	TCCR1A = 0x00;
	TCCR1B = 0x0A; //CTC1 + / 8 = 2Mhz.
	TCCR1C = 0x00;
  	OCR1A = 0x2000;	//1KHz
	TIMSK1 = 0x02; // turn on OCR1A match interrupt
}

void Motherboard::enableTimerInterrupts(bool enable) {
	if ( enable ) {
		TIMSK1 |= (1<<OCIE1A);
		TIMSK2 |= (1<<OCIE2A);
	}
	else {
		TIMSK1 &= ~(1<<OCIE1A);
		TIMSK2 &= ~(1<<OCIE2A);
	}
}

/// Reset the motherboard to its initial state.
/// This only resets the board, and does not send a reset
/// to any attached toolheads.
void Motherboard::reset(bool hard_reset) {
	indicateError(0); // turn off blinker

	// Init and turn on power supply
        //psu.init();
        //psu.turnOn(true);

	// Init steppers
	uint8_t axis_invert = eeprom::getEeprom8(eeprom::AXIS_INVERSION, EEPROM_DEFAULT_AXIS_INVERSION);
	// Z holding indicates that when the Z axis is not in
	// motion, the machine should continue to power the stepper
	// coil to ensure that the Z stage does not shift.
	// Bit 7 of the AXIS_INVERSION eeprom setting
	// indicates whether or not to use z holding; 
	// the bit is active low. (0 means use z holding,
	// 1 means turn it off.)
	bool hold_z = (axis_invert & (1<<7)) == 0;
	steppers::setHoldZ(hold_z);

	for (int i = 0; i < STEPPER_COUNT; i++) {
		stepper[i].init(i);
	}
	// Initialize the host and slave UARTs
        UART::getHostUART().enable(true);
        UART::getHostUART().in.reset();

        // TODO: These aren't done on other platforms, are they necessary?
        UART::getHostUART().reset();
        UART::getHostUART().out.reset();


        UART::getSlaveUART().enable(true);
        UART::getSlaveUART().in.reset();

        // TODO: These aren't done on other platforms, are they necessary?
        UART::getSlaveUART().reset();
        UART::getSlaveUART().out.reset();

        // Reset and configure timer 1, the stepper driver
        // interrupt timer.
        setupFixedStepperTimer();

	// Reset and configure timer 2, the debug LED flasher timer, Advance timer and microsecond timer
	// Timer 2 is a 8-bit
	TCCR2A = 0x02;	// CTC
	TCCR2B = 0x04;	// prescaler at 1/64
	OCR2A  = 25;	// Generate interrupts 16MHz / 64 / 25 = 10KHz
	TIMSK2 = 0x02;	// turn on OCR2A match interrupt

	// Configure the debug pin.
	DEBUG_PIN.setDirection(true);
}

/// Get the number of microseconds that have passed since
/// the board was booted.
micros_t Motherboard::getCurrentMicros() {
	micros_t micros_snapshot;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		micros_snapshot = micros;
	}
	return micros_snapshot;
}

void Motherboard::runMotherboardSlice() {
}

/// Do nothing, need this to stop compile error as it called from SDCard
/// for firmware mb24
void Motherboard::resetCurrentSeconds() {
}

#if defined(JKN_ADVANCE) && defined(HAS_STEPPER_ACCELERATION)

void Motherboard::doAdvanceInterrupt() {
	steppers::doAdvanceInterrupt();
}

#endif

/// Run the motherboard interrupt
void Motherboard::doStepperInterrupt() {
	enableTimerInterrupts(false);
	sei();

	// Do not move steppers if the board is in a paused state
	if (command::isPaused()) return;
	steppers::doInterrupt();

	cli();
	enableTimerInterrupts(true);
}

void Motherboard::updateMicros() {
	micros += 100;	//100 = (1.0 / ( 16MHz / 64 / 25 = 10KHz)) * 1000000;
}

/// Timer one comparator match interrupt
ISR(TIMER1_COMPA_vect) {
	Motherboard::getBoard().doStepperInterrupt();
}

/// Number of times to blink the debug LED on each cycle
volatile uint8_t blink_count = 0;

/// The current state of the debug LED
enum {
	BLINK_NONE,
	BLINK_ON,
	BLINK_OFF,
	BLINK_PAUSE
} blink_state = BLINK_NONE;

/// Write an error code to the debug pin.
void Motherboard::indicateError(int error_code) {
	if (error_code == 0) {
		blink_state = BLINK_NONE;
		DEBUG_PIN.setValue(false);
	}
	else if (blink_count != error_code) {
		blink_state = BLINK_OFF;
	}
	blink_count = error_code;
}

/// Get the current error code.
uint8_t Motherboard::getCurrentError() {
	return blink_count;
}

/// Timer2 overflow cycles that the LED remains on while blinking
#define OVFS_ON 18
/// Timer2 overflow cycles that the LED remains off while blinking
#define OVFS_OFF 18
/// Timer2 overflow cycles between flash cycles
#define OVFS_PAUSE 80

/// Number of overflows remaining on the current blink cycle
int blink_ovfs_remaining = 0;
/// Number of blinks performed in the current cycle
int blinked_so_far = 0;

int debug_light_interrupt_divisor = 0;
#define MAX_DEBUG_LIGHT_INTERRUPT_DIVISOR	164	//Timer interrupt frequency / (16MHz / 1026 / 256)

/// Timer 2 comparator match interrupt
ISR(TIMER2_COMPA_vect) {
	Motherboard::getBoard().updateMicros();

#if defined(JKN_ADVANCE) && defined(HAS_STEPPER_ACCELERATION)
	Motherboard::getBoard().doAdvanceInterrupt();
#endif

	debug_light_interrupt_divisor ++;
	if ( debug_light_interrupt_divisor < MAX_DEBUG_LIGHT_INTERRUPT_DIVISOR )
		return;

	debug_light_interrupt_divisor = 0;

	if (blink_ovfs_remaining > 0) {
		blink_ovfs_remaining--;
	} else {
		if (blink_state == BLINK_ON) {
			blinked_so_far++;
			blink_state = BLINK_OFF;
			blink_ovfs_remaining = OVFS_OFF;
			DEBUG_PIN.setValue(false);
		} else if (blink_state == BLINK_OFF) {
			if (blinked_so_far >= blink_count) {
				blink_state = BLINK_PAUSE;
				blink_ovfs_remaining = OVFS_PAUSE;
			} else {
				blink_state = BLINK_ON;
				blink_ovfs_remaining = OVFS_ON;
				DEBUG_PIN.setValue(true);
			}
		} else if (blink_state == BLINK_PAUSE) {
			blinked_so_far = 0;
			blink_state = BLINK_ON;
			blink_ovfs_remaining = OVFS_ON;
			DEBUG_PIN.setValue(true);
		}
	}
}
