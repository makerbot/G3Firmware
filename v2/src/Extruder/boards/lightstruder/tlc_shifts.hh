/*  Copyright (c) 2009 by Alex Leone <acleone ~AT~ gmail.com>

    This file is part of the Arduino TLC5940 Library.

    The Arduino TLC5940 Library is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    The Arduino TLC5940 Library is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with The Arduino TLC5940 Library.  If not, see
    <http://www.gnu.org/licenses/>. */

#ifndef TLC_SHIFTS_H
#define TLC_SHIFTS_H

/** \file
    TLC channel shifting functions. */

#include "Tlc5940.hh"

uint16_t tlc_shiftUp(uint16_t zeroValue = 0);
uint16_t tlc_shiftDown(uint16_t topValue = 0);

/** \addtogroup ExtendedFunctions
    \code #include "tlc_shifts.h" \endcode
    - uint16_t tlc_shiftUp(uint16_t zeroValue = 0) - shifts all channel data
        up (OUT0 becomes OUT1 ...) and returns OUT15
    - uint16_t tlc_shiftDown(uint16_t topValue = 0) - shifts all channel data
        down (OUT15 becomes OUT14 ...) and returns OUT0 */
/* @{ */

/** Shifts all the channel data up (OUT0 becomes OUT1 ...).  Needs a
    Tlc.update() after.
    \param zeroValue the value of channel 0.
    \returns the value that was shifted off the end (OUT15) */
uint16_t tlc_shiftUp(uint16_t zeroValue)
{
    uint16_t topValue = ((uint16_t)(*tlc_GSData) << 4)
                      | (*(tlc_GSData + 1) >> 4);
    uint8_t *p = tlc_GSData + 1;
    while (p < tlc_GSData + NUM_TLCS * 24 - 1) {
        *(p - 1) = (*p << 4) | (*(p + 1) >> 4);
        *p = (*(p + 1) << 4) | (*(p + 2) >> 4);
        p += 2;
    }
    *(tlc_GSData + NUM_TLCS * 24 - 2) = (*(tlc_GSData + NUM_TLCS * 24 - 1) << 4)
                                      | ((zeroValue & 0x0F00) >> 8);
    *(tlc_GSData + NUM_TLCS * 24 - 1) = (uint8_t)zeroValue;
    return topValue;
}

/** Shifts all the channel data down (OUT 15 -> OUT 14 ...). Needs a
    Tlc.update() after.
    \param topValue the value of Tlc (n) channel 15.
    \returns the value that was shifted off the bottom (OUT0) */
uint16_t tlc_shiftDown(uint16_t topValue)
{
    uint8_t *p = tlc_GSData + NUM_TLCS * 24 - 2;
    uint16_t zeroValue =
            ((uint16_t)(*(tlc_GSData + NUM_TLCS * 24 - 2) & 0x0F) << 8)
                      | *(tlc_GSData + NUM_TLCS * 24 - 1);
    while (p > tlc_GSData) {
        *(p + 1) = (*p >> 4) | (*(p - 1) << 4);
        *p = (*(p - 1) >> 4) | (*(p - 2) << 4);
        p -= 2;
    }
    *(tlc_GSData + 1) = (*tlc_GSData >> 4) | ((uint8_t)topValue << 4);
    *tlc_GSData = topValue >> 4;
    return zeroValue;
}

/* @} */

#endif

