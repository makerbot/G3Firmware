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

#include "ExtruderBoard.hh"
#include "HeatingElement.hh"
#include "ExtruderMotor.hh"
#include "MotorController.hh"
#include "Configuration.hh"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include "EepromMap.hh"

ExtruderBoard ExtruderBoard::extruderBoard;

Pin channel_a(PortC,1);
Pin channel_b(PortB,3);
Pin channel_c = FAN_ENABLE_PIN;

volatile bool using_relays = false;

ExtruderBoard::ExtruderBoard() :
		micros(0L),
		extruder_thermistor(THERMISTOR_PIN,0),
		platform_thermistor(PLATFORM_PIN,1),
		extruder_heater(extruder_thermistor,extruder_element,SAMPLE_INTERVAL_MICROS_THERMISTOR,eeprom::EXTRUDER_PID_P_TERM),
		platform_heater(platform_thermistor,platform_element,SAMPLE_INTERVAL_MICROS_THERMISTOR,eeprom::HBP_PID_P_TERM),
		using_platform(true)
{
}

// Turn on/off PWM for channel A.
void pwmAOn(bool on) {
	if (on) {
		TIMSK2 = 0b00000101;
	} else {
		TIMSK2 = 0b00000000;
	}
}

// Turn on/off PWM for channel B.
void pwmBOn(bool on) {
	if (on) {
		TCCR2A = 0b10000011;
	} else {
		TCCR2A = 0b00000011;
	}
}

#define SERVO_COUNT 2

volatile int servoPos[SERVO_COUNT];

// Index 0 = D9, Index 1 = D10.  Value = -1 to turn off, 0-255 to set position.
void ExtruderBoard::setServo(uint8_t index, int value) {
	servoPos[0] = value;
}

void ExtruderBoard::reset() {
	for (uint8_t i = 0; i < SERVO_COUNT; i++) {
		servoPos[i] = -1;
	}
	initExtruderMotor();
	// Timer 1 is for microsecond-level timing and servo pulses.
	// CTC mode, interrupt on ICR1, top at ICR1, no prescaler
	TCCR1A = _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
	TCCR1C = 0x00;
	ICR1 = INTERVAL_IN_MICROSECONDS * 16;
	TIMSK1 = _BV(ICIE1) | _BV(OCIE1A) | _BV(OCIE1B); // turn on ICR1 match interrupt
	TIMSK2 = 0x00; // turn off channel A PWM by default
	// TIMER2 is used to PWM mosfet channel B on OC2A, and channel A on
	// PC1 (using the OC2B register).
	DEBUG_LED.setDirection(true);
	channel_a.setValue(false);
	channel_a.setDirection(true); // set channel A as output
	channel_b.setValue(false);
	channel_b.setDirection(true); // set channel B as output
	channel_c.setValue(false);
	channel_c.setDirection(true); // set channel C as output
	TCCR2A = 0b10000011;
	TCCR2B = 0b00000110; // prescaler 1/256
	OCR2A = 0;
	OCR2B = 0;
	// We use interrupts on OC2B and OVF to control channel A.
	TIMSK2 = 0b00000101;
	extruder_thermistor.init();
	platform_thermistor.init();
	extruder_heater.reset();
	platform_heater.reset();
	setMotorSpeed(0);
	getHostUART().enable(true);
	getHostUART().in.reset();
	// These are disabled until the newer replicatorg with eeprom path
	// support has been out for a while.
//	uint16_t features = getEeprom16(eeprom::FEATURES);
//	setUsingRelays((features & eeprom::RELAY_BOARD) != 0);
//	setStepperMode((features & eeprom::HBRIDGE_STEPPER) != 0);
	// Init servo ports: OC1A and OC1B as outputs when not linked to counter.
	PORTB &= ~_BV(1) & ~_BV(2);
	DDRB |= _BV(1) | _BV(2);
#ifdef DEFAULT_STEPPER
	setStepperMode(true);
#else
	setStepperMode(false);
#endif
#ifdef DEFAULT_RELAYS
	setUsingRelays(true);
#else
	setUsingRelays(false);
#endif
}

void ExtruderBoard::setMotorSpeed(int16_t speed) {
	setExtruderMotor(speed);
}

micros_t ExtruderBoard::getCurrentMicros() {
	micros_t micros_snapshot;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		micros_snapshot = micros;
	}
	return micros_snapshot;
}

// ms between servo updates; conservative to avoid 7805 overheating
#define SERVO_CYCLE_LENGTH 8
volatile uint8_t servo_cycle = 0;

/// Run the extruder board interrupt
void ExtruderBoard::doInterrupt() {
	// update microseconds
	micros += INTERVAL_IN_MICROSECONDS;
	// update servos
	ExtruderBoard::getBoard().indicateError(0);
	if (servo_cycle == 0) {
		if (servoPos[0] != -1) {
			PORTB |= _BV(1);
			OCR1A = (600*16) + (servoPos[0]*160);
		}
		if (servoPos[1] != -1) {
			PORTB |= _BV(2);
			OCR1B = (600*16) + (servoPos[1] * 160);
		}
	}
	servo_cycle++;
	if (servo_cycle > SERVO_CYCLE_LENGTH) { servo_cycle = 0; }
}

void ExtruderBoard::setFan(bool on) {
	channel_c.setValue(on);
}

void ExtruderBoard::setValve(bool on) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		setUsingPlatform(false);
		pwmAOn(false);
		channel_a.setValue(on);
	}
}

void ExtruderBoard::indicateError(int errorCode) {
	DEBUG_LED.setValue(errorCode != 0);
}

void ExtruderBoard::setUsingPlatform(bool is_using) {
	using_platform = is_using;
}

void ExtruderBoard::setUsingRelays(bool is_using) {
	using_relays = is_using;
}

/// Timer one ICR1 match interrupt
ISR(TIMER1_CAPT_vect) {
	ExtruderBoard::getBoard().doInterrupt();
}

ISR(TIMER1_COMPA_vect) {
	PORTB &= ~_BV(1);
}

ISR(TIMER1_COMPB_vect) {
	PORTB &= ~_BV(2);
}

void ExtruderHeatingElement::setHeatingElement(uint8_t value) {
//	if (value > 128) {
//		value = 255;
//	} else if (value > 0) {
//		value = 128;
//	}
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (using_relays) {
			pwmBOn(false);
			channel_b.setValue(value != 0);
		} else if (value == 0 || value == 255) {
			pwmBOn(false);
			channel_b.setValue(value == 255);
		} else {
			OCR2A = value;
			pwmBOn(true);
		}
	}
}

void BuildPlatformHeatingElement::setHeatingElement(uint8_t value) {
	// This is a bit of a hack to get the temperatures right until we fix our
	// PWM'd PID implementation.  We reduce the MV to one bit, essentially.
	// It works relatively well.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		pwmAOn(false);
		channel_a.setValue(value != 0);
	}
	/*
	if (value > 128) {
		value = 255;
	} else if (value > 0) {
		value = 128;
	}
	if (value == 0 || value == 255) {
		pwmAOn(false);
		channel_a.setValue(value == 255);
	} else {
		OCR2B = value;
		pwmAOn(true);
	}
	*/
}

ISR(TIMER2_OVF_vect) {
	if (OCR2B != 0) {
		channel_a.setValue(true);
	}
}

ISR(TIMER2_COMPB_vect) {
	channel_a.setValue(false);
}
