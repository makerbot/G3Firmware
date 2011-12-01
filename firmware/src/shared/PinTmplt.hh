/*
 * Copyright 2012 by Craig Link
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

#ifndef PIN_TMPLT_HH_
#define PIN_TMPLT_HH_

#include <avr/io.h>
#include <util/atomic.h>

// The AVR port and pin mapping is based on a convention that has held true for all ATMega chips
// released so far: that the ports begin in sequence from register 0x00 from A onwards, and are
// arranged:
// 0 PINx
// 1 DDRx
// 2 PORTx
// This is verified true for the 168/328/644p/1280/2560.

// We support three platforms: Atmega168 (1 UART), Atmega644, and Atmega1280/2560
#if defined (__AVR_ATmega168__)     \
    || defined (__AVR_ATmega328__)  \
    || defined (__AVR_ATmega644P__) \
    || defined (__AVR_ATmega1280__) \
    || defined (__AVR_ATmega2560__)
#else
    #error UART not implemented on this processor type!
#endif

// The AVR port and pin mapping is based on a convention that has held true for all ATMega chips
// released so far: that the ports begin in sequence from register 0x00 from A onwards, and are
// arranged:
// 0 PINx
// 1 DDRx
// 2 PORTx
// This is verified true for the 168/644p/1280.

#define PINx _SFR_MEM8(port_base+0)
#define DDRx _SFR_MEM8(port_base+1)
#define PORTx _SFR_MEM8(port_base+2)


template < uint16_t port_base, uint8_t pin_index > class PinTmplt {
public:

    // ports with 2 byte base address need to be wrapped in an atomic
    // block.  We provide the option for the caller to wrap several calls in a single
    // atomic block for code optimizing.
    // see http://code.google.com/p/arduino/issues/detail?id=146
    // and http://code.google.com/p/digitalwritefast/

    static  inline void setDirection(bool output, bool externalAtomicBlock = false) 
    { 
        if (port_base != 0)
        {
            if( (port_base < 0x40) || externalAtomicBlock )
            {
                output ? DDRx |= _BV(pin_index) :  DDRx &= ~_BV(pin_index); 
            }
            else
            {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
                { 
                    output ? DDRx |= _BV(pin_index) :  DDRx &= ~_BV(pin_index); 
                }
            }
        }
    }

	static inline void setValue(bool on, bool externalAtomicBlock = false) 
    { 
        if (port_base != 0)
        {
            if( (port_base < 0x40) || externalAtomicBlock )
            {
                on ? PORTx |= _BV(pin_index) :  PORTx &= ~_BV(pin_index);
            }
            else
            {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
                { 
                    on ? PORTx |= _BV(pin_index) :  PORTx &= ~_BV(pin_index);
                }
            }
        }
    }

	static inline bool getValue()  
    { 
        if (port_base != 0)
        {
            return !!(PINx & _BV(pin_index)); 
        }
        else
        {
            return 0;
        }
    }

};

#define Pin(port_id,pin_id) PinTmplt<port_id,pin_id>

#define NULL_PIN PinTmplt<0,0>

#if defined(__AVR_ATmega644P__) || \
	defined(__AVR_ATmega1280__) || \
	defined(__AVR_ATmega2560__)
const uint8_t PortA = 0x20;
#endif // __AVR_ATmega644P__
const uint8_t PortB = 0x23;
const uint8_t PortC = 0x26;
const uint8_t PortD = 0x29;
#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
const uint8_t PortE = 0x2C;
const uint8_t PortF = 0x2F;
const uint8_t PortG = 0x32;
const uint16_t PortH = 0x100;
const uint16_t PortJ = 0x103;
const uint16_t PortK = 0x106;
const uint16_t PortL = 0x109;
#endif //__AVR_ATmega1280__
#endif // SHARED_AVR_PORT_HH_

