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

#ifndef BOARDS_RRMBV12_MOTHERBOARD_HH_
#define BOARDS_RRMBV12_MOTHERBOARD_HH_


#include "UART.hh"
#include "StepperInterface.hh"
#include "Types.hh"
#include "PSU.hh"
#include "Configuration.hh"

/// Main class for Motherboard version 1.2 (Gen3 electronics)
/// \ingroup HardwareLibraries
/// \ingroup MBv12
class Motherboard {
private:
	const static int STEPPERS = STEPPER_COUNT;

	StepperInterface stepper[STEPPERS];

	PSU psu;

	/// Microseconds since board initialization
	volatile micros_t micros;

        /// Private constructor; use the singleton
        Motherboard(const Pin& psu_pin);

	static Motherboard motherboard;
public:
        //2 types of stepper timers depending on if we're using accelerated or not
        void setupFixedStepperTimer();
        void setupAccelStepperTimer();

	//Enable / Disable Timer Interrupts
	void enableTimerInterrupts(bool enable);

	/// Reset the motherboard to its initial state.
	/// This only resets the board, and does not send a reset
	/// to any attached toolheads.
	void reset(bool hard_reset);

	void runMotherboardSlice();

	/// Count the number of steppers available on this board.
	const int getStepperCount() const { return STEPPERS; }

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
	/// 2**16 microseconds; callers should compensate for this.
	micros_t getCurrentMicros();
	void resetCurrentSeconds();

	/// Write an error code to the debug pin.
	void indicateError(int errorCode);

        /// Get the current error being displayed.
	uint8_t getCurrentError();

	/// Get the motherboard instance.
	static Motherboard& getBoard() { return motherboard; }

        /// Perform the stepper timer interrupt routine.
        void doStepperInterrupt();

        void doAdvanceInterrupt();
	
	void updateMicros();
};

#endif // BOARDS_RRMBV12_MOTHERBOARD_HH_
