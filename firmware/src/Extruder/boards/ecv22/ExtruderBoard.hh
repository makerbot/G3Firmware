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

#ifndef BOARDS_ECV22_EXTRUDER_BOARD_HH_
#define BOARDS_ECV22_EXTRUDER_BOARD_HH_

#include "UART.hh"
#include "ExtruderMotor.hh"
#include "Thermistor.hh"
#include "HeatingElement.hh"
#include "Heater.hh"
#include "MotorController.hh"


/// \defgroup ECv22
/// Code specific to the Extruder Controller version 2.2 (gen3 hardware)

/// Extruder heating element on v22 Extruder controller
/// \ingroup ECv22
class ExtruderHeatingElement : public HeatingElement {
public:
	void setHeatingElement(uint8_t value);
};

/// Build platform heating element on v22 Extruder controller
/// \ingroup ECv22
class BuildPlatformHeatingElement : public HeatingElement {
public:
	void setHeatingElement(uint8_t value);
};

/// Main class for Extruder controller version 2.2
/// \ingroup ECv22
class ExtruderBoard {
private:
        static ExtruderBoard extruder_board;

public:
        static ExtruderBoard& getBoard() { return extruder_board; }

private:
        MotorController motor_controller;
        Thermistor extruder_thermistor;
        Thermistor platform_thermistor;
        ExtruderHeatingElement extruder_element;
        BuildPlatformHeatingElement platform_element;
        Heater extruder_heater;
        Heater platform_heater;
        bool using_platform;

        /// Microseconds since board initialization
        volatile micros_t micros;
        ExtruderBoard();

        uint8_t resetFlags;

        uint8_t slave_id;





public:
	void reset(uint8_t resetFlags);

        void runExtruderSlice();

	// Return the processor's reset status flags.  These are useful
	// for diagnosing what might have triggered the last processor
	// reset.
	uint8_t getResetFlags();


	int get_current_temperature();
	void set_target_temperature(int);

	Heater& getExtruderHeater() { return extruder_heater; }
	Heater& getPlatformHeater() { return platform_heater; }

        MotorController& getMotorController() { return motor_controller; }

	void setMotorSpeed(int16_t speed);
	void setMotorSpeedRPM(uint32_t speed, bool direction);
#ifdef DEFAULT_EXTERNAL_STEPPER
	// Hack to decouple holding torque from RPM speed
	// Stops/starts the motor while holding torque
	void setMotorOn(bool on);
#endif
	void setFanRunning(bool state);
	void setAutomatedBuildPlatformRunning(bool state);
	void setValve(bool on);
	UART& getHostUART() { return UART::getHostUART(); }

	/// Get the number of microseconds that have passed since
	/// the board was initialized.  This value will wrap after
	/// 2**16 microseconds; callers should compensate for this.
	micros_t getCurrentMicros();
	/// Perform the timer interrupt routine.
	void doInterrupt();
	/// Indicate an error by manipulating the debug LED.
	void indicateError(int errorCode);

        void lightIndicatorLED() {}

	bool isUsingPlatform() { return using_platform; }
	void setUsingPlatform(bool is_using);
	void setUsingRelays(bool is_using);
	// Index 0 = D9, Index 1 = D10.  Value = -1 to turn off, 0-180 to set position.
	void setServo(uint8_t index, int value);

        uint8_t getSlaveID() { return slave_id; }
};

#endif // BOARDS_ECV22_EXTRUDER_BOARD_HH_
