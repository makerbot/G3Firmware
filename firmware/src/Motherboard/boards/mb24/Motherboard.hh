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

	/// Private constructor; use the singleton
	Motherboard();

        // TODO: Move this to an interface board slice.
	Timeout interface_update_timeout;

        /// True if we have an interface board attached
	bool hasInterfaceBoard;

        ButtonArray buttonArray;
        LiquidCrystal lcd;
        InterfaceBoard interfaceBoard;

        MainMenu mainMenu;              ///< Main system menu
        SplashScreen splashScreen;      ///< Displayed at startup
        MonitorMode monitorMode;        ///< Displayed during build

public:
	/// Reset the motherboard to its initial state.
	/// This only resets the board, and does not send a reset
	/// to any attached toolheads.
	void reset();

	void runMotherboardSlice();

	/// Count the number of steppers available on this board.
        const int getStepperCount() const { return STEPPER_COUNT; }
	/// Get the stepper interface for the nth stepper.
	StepperInterface& getStepperInterface(int n)
	{
		return stepper[n];
	}

	/// Get the number of microseconds that have passed since
	/// the board was initialized.  This value will wrap after
	/// 2**32 microseconds (ca. 70 minutes); callers should compensate for this.
	micros_t getCurrentMicros();

	/// Write an error code to the debug pin.
	void indicateError(int errorCode);
	/// Get the current error being displayed.
	uint8_t getCurrentError();

	/// Perform the timer interrupt routine.
	void doInterrupt();
};

#endif // BOARDS_MB24_MOTHERBOARD_HH_
