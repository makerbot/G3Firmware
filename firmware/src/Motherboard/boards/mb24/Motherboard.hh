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
//#include "PSU.hh"
#include "Configuration.hh"
#include "Timeout.hh"
#include "InterfaceBoard.hh"


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
#if STEPPER_COUNT > 0
	static StepperTmpltEndstops<0, X_DIR_PIN,X_STEP_PIN,X_ENABLE_PIN,X_MAX_PIN,X_MIN_PIN> stepperX;
#endif
#if STEPPER_COUNT > 1
	static StepperTmpltEndstops<1, Y_DIR_PIN,Y_STEP_PIN,Y_ENABLE_PIN,Y_MAX_PIN,Y_MIN_PIN> stepperY;
#endif
#if STEPPER_COUNT > 2
	static StepperTmpltEndstops<2, Z_DIR_PIN,Z_STEP_PIN,Z_ENABLE_PIN,Z_MAX_PIN,Z_MIN_PIN> stepperZ;
#endif
#if STEPPER_COUNT > 3
    // we're using B over A because of the assigned AVR Port which gets us a performance win using it over A
	static StepperTmplt<3, B_DIR_PIN,B_STEP_PIN,B_ENABLE_PIN> stepperB;
#endif
#if STEPPER_COUNT > 4
	static StepperTmplt<4, A_DIR_PIN,A_STEP_PIN,A_ENABLE_PIN> stepperA;
#endif

    /// Collection of stepper controllers that are on this board
	static const StepperInterface* stepper[STEPPER_COUNT];

	/// Microseconds since board initialization
	volatile micros_t micros;

	/// Private constructor; use the singleton
	Motherboard();

#if HAS_INTERFACE_BOARD > 0
        // TODO: Move this to an interface board slice.
	Timeout interface_update_timeout;

        /// True if we have an interface board attached
	bool hasInterfaceBoard;

    InterfaceBoard interfaceBoard;

    MainMenu mainMenu;              ///< Main system menu
    SplashScreen splashScreen;      ///< Displayed at startup
    MonitorMode monitorMode;        ///< Displayed during build
#endif // HAS_INTERFACE_BOARD > 0

public:
	/// Reset the motherboard to its initial state.
	/// This only resets the board, and does not send a reset
	/// to any attached toolheads.
	void reset();

	void runMotherboardSlice();

	/// Count the number of steppers available on this board.
        const int getStepperCount() const { return STEPPER_COUNT; }
	/// Get the stepper interface for the nth stepper.
	const StepperInterface* getStepperInterface(int n)
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
