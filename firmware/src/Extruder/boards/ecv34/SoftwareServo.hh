/*
 * Copyright 2011 by Matt Mets <matt.mets@makerbot.com>
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

#ifndef SOFTWARE_SERVO_HH_
#define SOFTWARE_SERVO_HH_

#include "PinTmplt.hh"

/// Software implementation of a hobby servo driver. Though module is implemented
/// purely in software, it does require periodic servicing from a microsecond
/// capable hardware interrupt in order to function properly.
///
/// TODO: rewrite this module so that it is more self-contained.
///
/// Porting notes:
/// A large portion of the logic that makes this module work is in
/// src\Extruder\boards\ecv34\ExtruderBoard.cc. This will have to be replicated
/// (or perhaps somehow refactored) before use in another board.
/// \ingroup SoftwareLibraries
template <class pin> class SoftwareServo {
public:
        /// Create a new sofware serial instance.
        /// \param [in] pin Digital output #Pin that this servo should be attached to.
	SoftwareServo() : enabled(false)
    {
	    pin::setDirection(true);
	    pin::setValue(false);
    }

        /// Set the servo position
        /// \param[in] position Servo position in degrees, from 0 - 180
	void setPosition(uint8_t position){
	    // Program the timer match value so that we generate a pulse width per:
	    //  http://www.servocity.com/html/hs-311_standard.html
	    //  600us + (value * 10)
	    //  so 0deg = 600us, 90deg = 1500us, 180deg = 2400us
	    if (position > 180) {
		    position = 180;
	    }

	    counts = 600 + 10*position;
    }
        /// Enable the software servo module. The servo output will not be modified
        /// automatically; it is dependant on the code in:
        /// src\Extruder\boards\ecv34\ExtruderBoard.cc
    inline void enable() { enabled = true; }

        /// Disable the software servo module. The servo output will be turned off
        /// immediately (which may cause a glitch if the output was already on).
	inline void disable()
    { 
        enabled = false; 
        pin::setValue(false); 
    }

    inline void setValue(bool val) 
    { 
        pin::setValue(val); 
    }

        /// Determine if this software servo module is enabled.
        /// \return true if the software servo module is enabled.
	bool isEnabled() { return enabled; }

        // TODO: Change this to be 'pulse width' instead
        /// Get the number of counts that the servo output should be held high for
        /// \return Pulse width of the servo control signal, in microseconds.
        uint16_t getCounts() { return counts; }

private:
        bool enabled;       ///< True if the servo is enabled.
        uint16_t counts;    ///< Length of servo on-time, in microseconds.
};

#endif
