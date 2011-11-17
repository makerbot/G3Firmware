/*
 * Copyright 2011 by Craig Link craig@moonrock.com
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

#ifndef STEPPERINTERFACE_HH_
#define STEPPERINTERFACE_HH_

#include <PinTmplt.hh>
#include "Eeprom.hh"
#include "EepromMap.hh"
#include "Configuration.hh"

/// StepperInterface instances encapsulate the low-level communication
/// with a stepper board.
class StepperInterface {
public:
	/// Set the direction for the stepper to move
	virtual void setDirection(bool forward) const = 0;
	
    /// Set the value of the step line
    virtual void step(bool value) const = 0;

	/// Enable or disable this axis
	virtual void setEnabled(bool enabled) const = 0;

	/// True if the axis has triggered its maximum endstop
	virtual bool isAtMaximum() const = 0;
	/// True if the axis has triggered its minimum endstop
	virtual bool isAtMinimum() const = 0;

};

template < uint8_t offset, class dir_pin, class step_pin, class enable_pin > class StepperTmplt : public StepperInterface {

public:

	/// Set the direction for the stepper to move
	virtual void setDirection(bool forward) const { dir_pin::setValue(invert_axis ? !forward : forward, false); }
	
    /// Set the value of the step line
    virtual void step(bool value) const {step_pin::setValue(value, false);}

	/// Enable or disable this axis
	virtual void setEnabled(bool enabled) const
    {
    	// The A3982 stepper driver chip has an inverted enable.
    	enable_pin::setValue(!enabled, false);
    }


	/// True if the axis has triggered its maximum endstop
	virtual bool isAtMaximum() const 
    {
        return false;
    }
	/// True if the axis has triggered its minimum endstop
	virtual bool isAtMinimum() const
    {
        return false;
    }

protected:

	friend class Motherboard;

	bool invert_axis;
	/// Default constructor
	//StepperInterface() {}
	StepperTmplt() : invert_axis(false) {}

    void init() 
    {
	    dir_pin::setDirection(true);
	    step_pin::setDirection(true);
	    enable_pin::setDirection(true);

	    enable_pin::setValue(true);
	    // get inversion characteristics
	    uint8_t axes_invert = eeprom::getEeprom8(eeprom::AXIS_INVERSION, 1<<1);
	    invert_axis = (axes_invert & (1<<offset)) != 0;
    }
};

template < uint8_t offset, class dir_pin, class step_pin, class enable_pin, class max_pin, class min_pin  > class StepperTmpltEndstops
    : public  StepperTmplt< offset, dir_pin, step_pin, enable_pin >
{

public:

	/// True if the axis has triggered its maximum endstop
	virtual bool isAtMaximum() const 
    {
	    bool v = max_pin::getValue();
	    if (invert_endstops) v = !v;
	    return v;
    }
	/// True if the axis has triggered its minimum endstop
	virtual bool isAtMinimum() const
    {
	    bool v = min_pin::getValue();
	    if (invert_endstops) v = !v;
	    return v;
    }

protected:

	friend class Motherboard;

	bool invert_endstops;

	StepperTmpltEndstops() : invert_endstops(true) {}

    void init() 
    {
        StepperTmplt< offset, dir_pin, step_pin, enable_pin >::init();
	    uint8_t endstops_invert = eeprom::getEeprom8(eeprom::ENDSTOP_INVERSION, 0);
	    bool endstops_present = (endstops_invert & (1<<7)) != 0;
    	// If endstops are not present, then we consider them inverted, since they will
    	// always register as high (pulled up).
	    invert_endstops = !endstops_present || ((endstops_invert & (1<<offset)) != 0);
	    // pull pins up to avoid triggering when using inverted endstops

		max_pin::setDirection(false);
		max_pin::setValue(invert_endstops);

		min_pin::setDirection(false);
		min_pin::setValue(invert_endstops);
    }

};

#endif // STEPPERINTERFACE_HH_
