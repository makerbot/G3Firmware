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

#ifndef BOARDS_MB24_MOTHERBOARD_HH_
#define BOARDS_MB24_MOTHERBOARD_HH_

#include "UART.hh"
#include "StepperInterface.hh"
#include "Types.hh"
#include "PSU.hh"
#include "Configuration.hh"
#include "Timeout.hh"
#include "Menu.hh"
#include "InterfaceBoard.hh"
#include "LiquidCrystal.hh"
#include "ButtonArray.hh"
#include "MoodLightController.hh"
#include "Errors.hh"


/// Main class for Motherboard version 2.4+ (Gen4 electronics)
/// \ingroup HardwareLibraries
/// \ingroup MBv24
class Motherboard {
private:
        // TODO: Declare this in main, drop the singleton.
        /// Static instance of the motherboard
        static Motherboard motherboard;

public:
        /// Get the motherboard instance.
        static Motherboard& getBoard() { return motherboard; }

private:
        /// Collection of stepper controllers that are on this board
        StepperInterface stepper[STEPPER_COUNT];

	/// Microseconds since board initialization
	volatile micros_t micros;
	volatile micros_t countupMicros;
	volatile seconds_t seconds;

	/// Private constructor; use the singleton
	Motherboard();

        // TODO: Move this to an interface board slice.
	Timeout interface_update_timeout;

        /// True if we have an interface board attached
	bool hasInterfaceBoard;

        ButtonArray buttonArray;
        LiquidCrystal lcd;
        InterfaceBoard interfaceBoard;
	MoodLightController   moodLightController;

        MainMenu mainMenu;              ///< Main system menu
        SplashScreen splashScreen;      ///< Displayed at startup
        MonitorMode monitorMode;        ///< Displayed during build


	bool buzzOn;
	uint8_t buzzerRepeats;
	uint8_t buzzerBuzzes;
	uint8_t buzzerBuzzesReset;
	float buzzerDuration;
	float buzzerSecondsTarget;

	enum BuzzerState {
		BUZZ_STATE_NONE = 0,
		BUZZ_STATE_MOVE_TO_ON,
		BUZZ_STATE_BUZZ_ON,
		BUZZ_STATE_MOVE_TO_OFF,
		BUZZ_STATE_BUZZ_OFF,
		BUZZ_STATE_MOVE_TO_DELAY,
		BUZZ_STATE_BUZZ_DELAY
	};

	enum BuzzerState buzzerState;

	void serviceBuzzer();

public:
	//2 types of stepper timers depending on if we're using accelerated or not
	void setupFixedStepperTimer();
	void setupAccelStepperTimer();

	/// Reset the motherboard to its initial state.
	/// This only resets the board, and does not send a reset
	/// to any attached toolheads.
	void reset(bool hard_reset);

	void runMotherboardSlice();

	/// Count the number of steppers available on this board.
        const int getStepperCount() const { return STEPPER_COUNT; }
	/// Get the stepper interface for the nth stepper.
	StepperInterface& getStepperInterface(int n)
	{
		return stepper[n];
	}

        StepperInterface *getStepperAllInterfaces()
	{
		return stepper;
	}

	/// Get the number of microseconds that have passed since
	/// the board was initialized.  This value will wrap after
	/// 2**32 microseconds (ca. 70 minutes); callers should compensate for this.
	micros_t getCurrentMicros();
	float getCurrentSeconds();
	void resetCurrentSeconds();

	/// Write an error code to the debug pin.
	void indicateError(int errorCode);
	/// Get the current error being displayed.
	uint8_t getCurrentError();

	/// Perform the stepper timer interrupt routine.
	void doStepperInterrupt();

	void doAdvanceInterrupt();

	/// Perform the interface timer interrupt routine.
	void doInterfaceInterrupt();
 
	MoodLightController getMoodLightController();
	void MoodLightSetRGBColor(uint8_t r, uint8_t g, uint8_t b, uint8_t fadeSpeed, uint8_t writeToEeprom);
	void MoodLightSetHSBColor(uint8_t r, uint8_t g, uint8_t b, uint8_t fadeSpeed);
	void MoodLightPlayScript(uint8_t scriptId, uint8_t writeToEeprom);

	void buzz(uint8_t buzzes, uint8_t duration, uint8_t repeats);
	void stopBuzzer();
};

#endif // BOARDS_MB24_MOTHERBOARD_HH_
