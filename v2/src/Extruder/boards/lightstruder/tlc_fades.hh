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

#ifndef TLC_FADES_H
#define TLC_FADES_H

/** \file
    TLC fading functions. */

#include <avr/interrupt.h>

#include "Tlc5940.hh"
#include "WProgram.h"

#ifndef TLC_FADE_BUFFER_LENGTH
/** The default fade buffer length (24).  Uses 24*13 = 312 bytes of ram. */
#define TLC_FADE_BUFFER_LENGTH    24
#endif

/** Data for a single fade */
struct Tlc_Fade {
    TLC_CHANNEL_TYPE channel; /**< channel this fade is on */
    int16_t startValue;       /**< value when the fade starts (0 - 4095) */
    int16_t changeValue;      /**< start + changeValue = endValue (0 - 4095) */
    uint32_t startMillis;     /**< millis() when to start */
    uint32_t endMillis;       /**< millis() when to end */
} tlc_fadeBuffer[TLC_FADE_BUFFER_LENGTH];

/** The current fade buffer size */
uint8_t tlc_fadeBufferSize;

uint8_t tlc_updateFades();
uint8_t tlc_updateFades(uint32_t currentMillis);
uint8_t tlc_addFade(struct Tlc_Fade *fade);
uint8_t tlc_addFade(TLC_CHANNEL_TYPE channel, int16_t startValue,
                    int16_t endValue, uint32_t startMillis, uint32_t endMillis);
uint8_t tlc_isFading(TLC_CHANNEL_TYPE channel);
uint8_t tlc_removeFades(TLC_CHANNEL_TYPE channel);
static void tlc_removeFadeFromBuffer(Tlc_Fade *current, Tlc_Fade *end);

/** \addtogroup ExtendedFunctions
    \code #include "tlc_fades.h" \endcode
     - uint8_t tlc_updateFades() - updates all fades
     - uint8_t tlc_updateFades(uint32_t currentMillis) - updates fades using
            currentMillis as the current time
     - uint8_t tlc_addFade(struct Tlc_Fade *fade) - copies fade into the
            fade buffer
     - uint8_t tlc_addFade(TLC_CHANNEL_TYPE channel, int16_t startValue,
            int16_t endValue, uint32_t startMillis, uint32_t endMillis) - adds
            a fade to the fade buffer
     - uint8_t tlc_isFading(TLC_CHANNEL_TYPE channel) - returns 1 if there's
            a fade on this channel in the buffer
     - uint8_t tlc_removeFades(TLC_CHANNEL_TYPE channel) - removes all fades
            on channel */
/* @{ */

/** Adds a fade to the buffer.
    \param fade the fade to be copied into the buffer
    \returns 0 if the fade buffer is full, fadeBufferSize if added successfully
*/
uint8_t tlc_addFade(struct Tlc_Fade *fade)
{
    if (tlc_fadeBufferSize == TLC_FADE_BUFFER_LENGTH) {
        return 0; // fade buffer full
    }
    struct Tlc_Fade *p = tlc_fadeBuffer + tlc_fadeBufferSize++;
    p->channel = fade->channel;
    p->startValue = fade->startValue;
    p->changeValue = fade->changeValue;
    p->startMillis = fade->startMillis;
    p->endMillis = fade->endMillis;
    return tlc_fadeBufferSize;
}

/** Adds a fade to the fade buffer.
    \param channel the ouput channel this fade is on
    \param startValue the value at the start of the fade
    \param endValue the value at the end of the fade
    \param startMillis the millis() when to start the fade
    \param endMillis the millis() when to end the fade
    \returns 0 if the fade buffer is full, fadeBufferSize if added successfully
*/
uint8_t tlc_addFade(TLC_CHANNEL_TYPE channel, int16_t startValue,
                    int16_t endValue, uint32_t startMillis, uint32_t endMillis)
{
    if (tlc_fadeBufferSize == TLC_FADE_BUFFER_LENGTH) {
        return 0; // fade buffer full
    }
    struct Tlc_Fade *p = tlc_fadeBuffer + tlc_fadeBufferSize++;
    p->channel = channel;
    p->startValue = startValue;
    p->changeValue = endValue - startValue;
    p->startMillis = startMillis;
    p->endMillis = endMillis;
    return tlc_fadeBufferSize;
}

/** Checks to see if any fades are happening on channel
    \param channel the channel to check
    \returns 1 if there is a fade in the buffer on this channel, 0 otherwise */
uint8_t tlc_isFading(TLC_CHANNEL_TYPE channel)
{
    struct Tlc_Fade *end = tlc_fadeBuffer + tlc_fadeBufferSize;
    for (struct Tlc_Fade *p = tlc_fadeBuffer; p < end; p++) {
        if (p->channel == channel) {
            return 1;
        }
    }
    return 0;
}

/** Removes any fades from the fade buffer on this channel.
    \param channel which channel the fades are on
    \returns how many fades were removed */
uint8_t tlc_removeFades(TLC_CHANNEL_TYPE channel)
{
    uint8_t removed = 0;
    struct Tlc_Fade *end = tlc_fadeBuffer + tlc_fadeBufferSize;
    for (struct Tlc_Fade *p = tlc_fadeBuffer; p < end; p++) {
        if (p->channel == channel) {
            removed++;
            tlc_removeFadeFromBuffer(p, --end);
        }
    }
    return removed;
}

/** Copies the end of the buffer to the current and decrements
    tlc_fadeBufferSize.  This will change the end of the buffer (pass by
    reference)
    \param current the fade to be removed
    \param endp the end of the fade buffer (pointer to pointer) */
static void tlc_removeFadeFromBuffer(struct Tlc_Fade *current,
                                     struct Tlc_Fade *endp)
{
    if (endp != current) { // if this is not the last fade
        current->channel = endp->channel;
        current->startValue = endp->startValue;
        current->changeValue = endp->changeValue;
        current->startMillis = endp->startMillis;
        current->endMillis = endp->endMillis;
    }
    tlc_fadeBufferSize--;
}

/** Updates fades using millis()
    \returns 0 if there are no fades left in the buffer. */
uint8_t tlc_updateFades()
{
    return tlc_updateFades(millis());
}

/** Updates any running fades.
    \param currentMillis the current millis() time.
    \returns 0 if there are no fades left in the buffer. */
uint8_t tlc_updateFades(uint32_t currentMillis)
{
    struct Tlc_Fade *end = tlc_fadeBuffer + tlc_fadeBufferSize;
    uint8_t needsUpdate = 0;
    for (struct Tlc_Fade *p = tlc_fadeBuffer; p < end;){
        if (currentMillis >= p->endMillis) { // fade done
            Tlc.set(p->channel, p->startValue + p->changeValue);
            needsUpdate = 1;
            tlc_removeFadeFromBuffer(p, --end);
            continue;
        } else {
            uint32_t startMillis = p->startMillis;
            if (currentMillis >= startMillis) {
                Tlc.set(p->channel, p->startValue + p->changeValue
                        * (int32_t)(currentMillis - startMillis)
                        / (int32_t)(p->endMillis - startMillis));
                needsUpdate = 1;
            }
        }
        p++;
    }
    if (needsUpdate) {
        Tlc.update();
        if (tlc_fadeBufferSize == 0) {
            while (tlc_needXLAT)
                ;
        }
    }
    return tlc_fadeBufferSize;
}

/* @} */

#endif

