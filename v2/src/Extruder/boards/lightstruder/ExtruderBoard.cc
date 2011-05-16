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


//Sin table values
// from: http://www.meraman.com/uploads/files/public/sinTable.htm
#define SIN_STEPS 128
uint8_t sin_vector[] = {127, 133, 139, 146, 152, 158, 164, 170, 176, 181, 187, 192, 198, 203, 208, 212, 217, 221, 225, 229, 233, 236, 239, 242, 244, 247, 249, 250, 252, 253, 253, 254, 254, 254, 253, 253, 252, 250, 249, 247, 244, 242, 239, 236, 233, 229, 225, 221, 217, 212, 208, 203, 198, 192, 187, 181, 176, 170, 164, 158, 152, 146, 139, 133, 127, 121, 115, 108, 102, 96, 90, 84, 78, 73, 67, 62, 56, 51, 46, 42, 37, 33, 29, 25, 21, 18, 15, 12, 10, 7, 5, 4, 2, 1, 1, 0, 0, 0, 1, 1, 2, 4, 5, 7, 10, 12, 15, 18, 21, 25, 29, 33, 37, 42, 46, 51, 56, 62, 67, 73, 78, 84, 90, 96, 102, 108, 115, 121 };

ExtruderBoard ExtruderBoard::extruder_board;
uint8_t led_current_channel;
uint16_t led_values[LED_CHANNELS];
float phasor;		// current angle of the led sine wheel
float phasor_speed;	// speed to increment the angle

ExtruderBoard::ExtruderBoard() :
		micros(0L),
		extruder_thermocouple(THERMOCOUPLE_CS,THERMOCOUPLE_SCK,THERMOCOUPLE_SO),
		platform_thermistor(PLATFORM_PIN,1),
		extruder_heater(extruder_thermocouple,extruder_element,SAMPLE_INTERVAL_MICROS_THERMOCOUPLE,eeprom::EXTRUDER_PID_P_TERM),
		platform_heater(platform_thermistor,platform_element,SAMPLE_INTERVAL_MICROS_THERMISTOR,eeprom::HBP_PID_P_TERM),
		using_platform(true),
		servoA(SERVO0),
		servoB(SERVO1)
{
}

// Get the reset flags from the processor, as a bitfield
// return: The bitfield looks like this: 0 0 0 0 WDRF BORF EXTRF PORF
uint8_t ExtruderBoard::getResetFlags() {
	return resetFlags;
}

// Turn on/off PWM for channel A on OC1B
void pwmAOn(bool on) {
}

// Turn on/off PWM for channel B on OC1A
void pwmBOn(bool on) {
}

// Turn on/off PWM for channel C on OC0A
void pwmCOn(bool on) {
}

void ExtruderBoard::reset(uint8_t resetFlags) {
	this->resetFlags = resetFlags;

	Tlc.init();

	// Clear the LED channel data
	for(uint8_t i = 0; i < LED_CHANNELS; i++) {
		led_values[i] = 0;
	}
	led_current_channel = 0;
	phasor = 0;
	phasor_speed = .2;

	initExtruderMotor();

	servoA.disable();
	servoB.disable();


	// Timer 0:
	//  Mode: CTC (WGM2:0 = 010), cycle freq=
	//  Prescaler: 1/32 (500 KHz)
	//  used as a provider for microsecond-level counting
	//  - Generates interrupt every 32uS
	//  used also to run servos in software
	TCCR0A = 0x02; // CTC is mode 2 on timer 0
	TCCR0B = 0x03; // prescaler: 1/64
	OCR0A = INTERVAL_IN_MICROSECONDS / 2; // 2uS/tick at 1/32 prescaler
	TIMSK0 = 0x02; // turn on OCR2A match interrupt


	extruder_thermocouple.init();
	platform_thermistor.init();
	extruder_heater.reset();
	platform_heater.reset();
	setMotorSpeed(0);
	getHostUART().enable(true);
	getHostUART().in.reset();
}

void ExtruderBoard::setMotorSpeed(int16_t speed) {
}

void ExtruderBoard::setServo(uint8_t index, int value) {
	if (index == 0) {		
		if(value < LED_CHANNELS + 1) {
			led_current_channel = value;
		}
	}
	else {
		if (led_current_channel < LED_CHANNELS) {
			led_values[led_current_channel] = value;
		}
		else {
			phasor_speed = (float)value/100;
		}	
	}
/*
	SoftwareServo* servo;
	if (index == 0) {
		servo = &servoA;
	}
	else if (index == 1) {
		servo = &servoB;
	}
	else {
		return;
	}

	if (value == -1) {
		servo->disable();
	}
	else {
		if (!(servo->isEnabled())) {
			servo->enable();
		}
		servo->setPosition(value);
	}
*/
}

micros_t ExtruderBoard::getCurrentMicros() {
	micros_t micros_snapshot;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		micros_snapshot = micros;
	}
	return micros_snapshot;
}

/// Run the extruder board interrupt
void ExtruderBoard::doInterrupt() {
	static micros_t servo_counter = 0;
	static micros_t led_counter = 0;

	micros += INTERVAL_IN_MICROSECONDS;

	// Check if the servos need servicing
	servo_counter += INTERVAL_IN_MICROSECONDS;

	// Overflow, so turn both servos on
	if (servo_counter > 16000) {
		servo_counter = 0;

		if (servoA.isEnabled()) {
			servoA.pin.setValue(true);
		}
		if (servoB.isEnabled()) {
			servoB.pin.setValue(true);
		}
	}

	if ((servoA.isEnabled()) && (servo_counter > servoA.getCounts())) {
		servoA.pin.setValue(false);
	}
	if ((servoB.isEnabled()) && (servo_counter > servoB.getCounts())) {
		servoB.pin.setValue(false);
	}


	// Check if the servos need servicing
	led_counter += INTERVAL_IN_MICROSECONDS;

	// Overflow, so refresh the LEDs
	// TODO: fading, etc...
	if (led_counter > 1000) {
		led_counter = 0;
		uint8_t value; 
		for (uint8_t channel = 0; channel < LED_CHANNELS; channel++) {
			value = led_values[channel];			
			// 0...100 means constant color
			if (value < 101) {
				Tlc.set(channel, value*40);
			}
			// 101...165 means sine wave with a phase offset
			else {
				Tlc.set(channel,
					16*sin_vector[((uint8_t)(phasor + value - 101)) % SIN_STEPS]);
			}
		}
		Tlc.update();

		// TODO: this close enough?
		phasor = phasor + phasor_speed;
		if (phasor > SIN_STEPS) {
			phasor -= SIN_STEPS;
		}
	}
}

void ExtruderBoard::setFan(bool on) {
}

void ExtruderBoard::setValve(bool on) {
}

void ExtruderBoard::indicateError(int errorCode) {
}

void ExtruderBoard::setUsingPlatform(bool is_using) {
	using_platform = is_using;
}

/// Timer two comparator A match interrupt
ISR(TIMER0_COMPA_vect) {
	ExtruderBoard::getBoard().doInterrupt();
}

void ExtruderHeatingElement::setHeatingElement(uint8_t value) {
}

void BuildPlatformHeatingElement::setHeatingElement(uint8_t value) {
}
