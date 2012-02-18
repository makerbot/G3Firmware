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
#include "Interface.hh"
#include "Tool.hh"
#include "Commands.hh"
#include "Eeprom.hh"
#include "EepromMap.hh"
#include <avr/eeprom.h>

/// Instantiate static motherboard instance
Motherboard Motherboard::motherboard;

/// Create motherboard object
Motherboard::Motherboard() :
        lcd(LCD_RS_PIN,
            LCD_ENABLE_PIN,
            LCD_D0_PIN,
            LCD_D1_PIN,
            LCD_D2_PIN,
            LCD_D3_PIN),
	moodLightController(SOFTWARE_I2C_SDA_PIN,
		  	    SOFTWARE_I2C_SCL_PIN),
        interfaceBoard(buttonArray,
            lcd,
            INTERFACE_FOO_PIN,
            INTERFACE_BAR_PIN,
            &mainMenu,
            &monitorMode,
	    moodLightController)
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
#if STEPPER_COUNT > 4
        stepper[4] = StepperInterface(B_DIR_PIN,
                                      B_STEP_PIN,
                                      B_ENABLE_PIN,
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
  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11);
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10); // 2MHz timer

  OCR1A = 0x4000;
  TCNT1 = 0;
  TIMSK1 |= (1<<OCIE1A);	//Enable interrupt
}

/// Reset the motherboard to its initial state.
/// This only resets the board, and does not send a reset
/// to any attached toolheads.
void Motherboard::reset(bool hard_reset) {
	indicateError(0); // turn off blinker

	if ( hard_reset )	moodLightController.start();

	// Init steppers
	uint8_t axis_invert = eeprom::getEeprom8(eeprom::AXIS_INVERSION, 0);
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
        UART::getSlaveUART().enable(true);
        UART::getSlaveUART().in.reset();

	// Reset and configure timer 1, the stepper
	// interrupt timer.
	setupFixedStepperTimer();

	// Reset and configure timer 2, the debug LED flasher timer.
	TCCR2A = 0x00;
	TCCR2B = 0x07; // prescaler at 1/1024
	TIMSK2 = 0x01; // OVF flag on

	// Reset and configure timer 3, the microsecond and interface
	// interrupt timer.
	TCCR3A = 0x00;
	TCCR3B = 0x0B; //Prescaler = 64
	TCCR3C = 0x00;
	OCR3A = INTERVAL_IN_MICROSECONDS * 16;
	TIMSK3 = 0x02; // turn on OCR3A match interrupt

	// Reset and configure timer 4, the accelerated "ADVANCE" timer
	// interrupt timer.
	TCCR4A = 0x00;
	TCCR4B = 0x09;
	TCCR4C = 0x00;
	OCR4A = 100 * 16;
	TIMSK4 = 0x02; // turn on OCR4A match interrupt

        buzzerRepeats  = 0;
        buzzerDuration = 0.0;
        buzzerState    = BUZZ_STATE_NONE;
	BUZZER_PIN.setDirection(false);

	// Configure the debug pin.
	DEBUG_PIN.setDirection(true);

#if HAS_ESTOP
	// Configure the estop pin direction.
	ESTOP_PIN.setDirection(false);
#endif

	steppers::reset();

	// Check if the interface board is attached
        hasInterfaceBoard = interface::isConnected();

	if (hasInterfaceBoard) {
		// Make sure our interface board is initialized
                interfaceBoard.init();

                // Then add the splash screen to it.
                interfaceBoard.pushScreen(&splashScreen);

                // Finally, set up the *** interface
                interface::init(&interfaceBoard, &lcd);

                interface_update_timeout.start(interfaceBoard.getUpdateRate());
	}

        // Blindly try to reset the toolhead with index 0.
//        resetToolhead();
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


/// Get the number of seconds that have passed since
/// the board was booted or the timer reset.
float Motherboard::getCurrentSeconds() {
  micros_t seconds_snapshot;
  micros_t countupMicros_snapshot;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    countupMicros_snapshot  = countupMicros;
    seconds_snapshot	    = seconds;
  }
  return (float)seconds_snapshot + ((float)countupMicros_snapshot / (float)1000000);
}


/// Reset the seconds counter to 0.
void Motherboard::resetCurrentSeconds() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    seconds = 0L;
  }
}

/// Run the stepper interrupt

void Motherboard::doStepperInterrupt() {
	steppers::doInterrupt();
}

void Motherboard::doAdvanceInterrupt() {
	steppers::doAdvanceInterrupt();
}

/// Run the interface interrupt

void Motherboard::doInterfaceInterrupt() {
	if (hasInterfaceBoard) {
                interfaceBoard.doInterrupt();
	}
	micros += (INTERVAL_IN_MICROSECONDS * 64);
	countupMicros += (INTERVAL_IN_MICROSECONDS * 64);	//64 because we're using a 64 prescaler on timer 3
	while (countupMicros > 1000000L) {
		seconds += 1;
		countupMicros -= 1000000L;
	}
}

void Motherboard::runMotherboardSlice() {
	if (hasInterfaceBoard) {
		if (interface_update_timeout.hasElapsed()) {
                        interfaceBoard.doUpdate();
                        interface_update_timeout.start(interfaceBoard.getUpdateRate());
		}
	}

	serviceBuzzer();
}

MoodLightController Motherboard::getMoodLightController() {
	return moodLightController;
}


/// Timer one comparator match interrupt
ISR(TIMER1_COMPA_vect) {
	Motherboard::getBoard().doStepperInterrupt();
}

/// Timer one comparator match interrupt
ISR(TIMER3_COMPA_vect) {
	Motherboard::getBoard().doInterfaceInterrupt();
}

/// Timer one comparator match interrupt
ISR(TIMER4_COMPA_vect) {
	Motherboard::getBoard().doAdvanceInterrupt();
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

void Motherboard::MoodLightSetRGBColor(uint8_t r, uint8_t g, uint8_t b, uint8_t fadeSpeed, uint8_t writeToEeprom) {
	if ( writeToEeprom ) {
		eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_RED,  r);
		eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_GREEN,g);
		eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_BLUE, b);
	} else {
		moodLightController.blinkM.setFadeSpeed(fadeSpeed);
		moodLightController.blinkM.fadeToRGB(r,g,b);
	}
}

void Motherboard::MoodLightSetHSBColor(uint8_t r, uint8_t g, uint8_t b, uint8_t fadeSpeed) {
	moodLightController.blinkM.setFadeSpeed(fadeSpeed);
	moodLightController.blinkM.fadeToHSB(r,g,b);
}

void Motherboard::MoodLightPlayScript(uint8_t scriptId, uint8_t writeToEeprom) {
	if ( writeToEeprom ) eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_SCRIPT,scriptId);
	moodLightController.playScript(scriptId);
}

//Duration is the length of each buzz in 1/10secs
//Issue "repeats = 0" to kill a current buzzing

void Motherboard::buzz(uint8_t buzzes, uint8_t duration, uint8_t repeats) {
	if ( repeats == 0 ) {
		buzzerState = BUZZ_STATE_NONE;
		return;
	}

	buzzerBuzzes	  = buzzes;
	buzzerBuzzesReset = buzzes;
	buzzerDuration	  = (float)duration / 10.0;	
	buzzerRepeats	  = repeats;

	BUZZER_PIN.setDirection(true);
	buzzerState = BUZZ_STATE_MOVE_TO_ON;
}

void Motherboard::stopBuzzer() {
	buzzerState = BUZZ_STATE_NONE;

	BUZZER_PIN.setValue(false);
	BUZZER_PIN.setDirection(false);
}

void Motherboard::serviceBuzzer() {
	if ( buzzerState == BUZZ_STATE_NONE )	return;

	float currentSeconds = getCurrentSeconds();

	switch (buzzerState)
	{
		case BUZZ_STATE_BUZZ_ON:
			if ( currentSeconds >= buzzerSecondsTarget )
				buzzerState = BUZZ_STATE_MOVE_TO_OFF;
			break;
		case BUZZ_STATE_MOVE_TO_OFF:
			buzzerBuzzes --;
			BUZZER_PIN.setValue(false);
			buzzerSecondsTarget = currentSeconds + buzzerDuration;
			buzzerState = BUZZ_STATE_BUZZ_OFF;
			break;
		case BUZZ_STATE_BUZZ_OFF:
			if ( currentSeconds >= buzzerSecondsTarget ) {
				if ( buzzerBuzzes == 0 ) {
					buzzerRepeats --;
					if ( buzzerRepeats == 0 )	stopBuzzer();
					else				buzzerState = BUZZ_STATE_MOVE_TO_DELAY;
				} else	buzzerState = BUZZ_STATE_MOVE_TO_ON;
			}
			break;
		case BUZZ_STATE_MOVE_TO_ON:
			BUZZER_PIN.setValue(true);
			buzzerSecondsTarget = currentSeconds + buzzerDuration;
			buzzerState = BUZZ_STATE_BUZZ_ON;
			break;
		case BUZZ_STATE_MOVE_TO_DELAY:
			BUZZER_PIN.setValue(false);
			buzzerSecondsTarget = currentSeconds + buzzerDuration * 3;
			buzzerState = BUZZ_STATE_BUZZ_DELAY;
			break;
		case BUZZ_STATE_BUZZ_DELAY:
			if ( currentSeconds >= buzzerSecondsTarget ) {
				buzzerBuzzes = buzzerBuzzesReset;
				buzzerSecondsTarget = currentSeconds + buzzerDuration;
				BUZZER_PIN.setValue(true);
				buzzerState = BUZZ_STATE_BUZZ_ON;
			}
			break;
	}
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

/// Timer 2 overflow interrupt
ISR(TIMER2_OVF_vect) {
	if (blink_ovfs_remaining > 0) {
		blink_ovfs_remaining--;
	} else {
		if (blink_state == BLINK_ON) {
			blinked_so_far++;
			blink_state = BLINK_OFF;
			blink_ovfs_remaining = OVFS_OFF;
			DEBUG_PIN.setValue(false);
			if ( blink_count == ERR_ESTOP )
				Motherboard::getBoard().getMoodLightController().debugLightSetValue(false);
		} else if (blink_state == BLINK_OFF) {
			if (blinked_so_far >= blink_count) {
				blink_state = BLINK_PAUSE;
				blink_ovfs_remaining = OVFS_PAUSE;
			} else {
				blink_state = BLINK_ON;
				blink_ovfs_remaining = OVFS_ON;
				DEBUG_PIN.setValue(true);
				if ( blink_count == ERR_ESTOP )
					Motherboard::getBoard().getMoodLightController().debugLightSetValue(true);
			}
		} else if (blink_state == BLINK_PAUSE) {
			blinked_so_far = 0;
			blink_state = BLINK_ON;
			blink_ovfs_remaining = OVFS_ON;
			DEBUG_PIN.setValue(true);
			if ( blink_count == ERR_ESTOP )
				Motherboard::getBoard().getMoodLightController().debugLightSetValue(true);
		}
	}
}
