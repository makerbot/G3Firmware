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

#include "Packet.hh"
#include "Configuration.hh"
#include <util/crc16.h>
#include <util/atomic.h>

void InPacket::appendByte(uint8_t data) {
	if (length < MAX_PACKET_PAYLOAD) {
		crc = _crc_ibutton_update(crc, data);
		payload[length] = data;
		length++;
	}
}
/// Append a byte and update the CRC
void OutPacket::appendByte(uint8_t data) {
	if (length < MAX_PACKET_PAYLOAD) {
		crc = _crc_ibutton_update(crc, data);
		payload[length] = data;
		length++;
	}
}
/// Reset this packet to an empty state
void Packet::reset() {
	crc = 0;
	length = 0;
#ifdef PARANOID
	for (uint8_t i = 0; i < MAX_PACKET_PAYLOAD; i++) {
		payload[i] = 0;
	}
#endif // PARANOID
	error_code = PacketError::NO_ERROR;
	state = PS_START;
}

void InPacket::computeCRC()
{
    if ( state == PS_COMPUTE_CRC )
    {
        state = PS_LAST;
        uint8_t _crc = 0;
        for( uint8_t ndx = 0; ndx < length; ndx++ )
        {
		    _crc = _crc_ibutton_update(_crc, payload[ndx]);
        }
	    if (crc != _crc) {
		    error(PacketError::BAD_CRC);
	    }
    }
}


//process a byte for our packet.
void InPacket::processByte(uint8_t b) {
    switch(state){
        case PS_START:
		    if (b == START_BYTE) {
			    state = PS_LEN;
		    } else {
			    error(PacketError::NOISE_BYTE);
		    }
           break;
        case PS_LEN:
		    if (b < MAX_PACKET_PAYLOAD) {
			    expected_length = b;
			    state = (expected_length == 0) ? PS_CRC : PS_PAYLOAD;
		    } else {
			    error(PacketError::EXCEEDED_MAX_LENGTH);
		    }
           break;
        case PS_PAYLOAD:
		    appendByte(b);
		    if (length >= expected_length) {
			    state = PS_CRC;
		    }
           break;
        case PS_CRC:
            crc = b;
            state = PS_COMPUTE_CRC;
/*
            if ( crc == b )
            {
			    state = PS_LAST;
            }
            else
            {
		        error(PacketError::BAD_CRC);
            }
*/
           break;
        default:
            //DEBUG_PACKET_PIN::setValue(true);
           break;
    }

    
}

// Reads an 8-bit byte from the specified index of the payload
/*
uint32_t Packet::read32(uint8_t index) const {
	union {
		// AVR is little-endian
		int32_t a;
		struct {
			uint8_t data[4];
		} b;
	} shared;
    return *((uint32_t*)(payload+index));
	shared.b.data[0] = p[index];
	shared.b.data[1] = payload[index+1];
	shared.b.data[2] = payload[index+2];
	shared.b.data[3] = payload[index+3];

	return shared.a;
}
*/

void OutPacket::prepareForResend() {
	error_code = PacketError::NO_ERROR;
	state = PS_START;
	send_payload_index = 0;
}
uint8_t OutPacket::getNextByteToSend() {
	switch(state){
        case PS_START:
		    state = PS_LEN;
            return START_BYTE;
        case PS_LEN:
		    state = (length==0)?PS_CRC:PS_PAYLOAD;
            return length;
        case PS_PAYLOAD:
            {
                uint8_t next_byte = payload[send_payload_index++];
		        if (send_payload_index >= length) {
			        state = PS_CRC;
		        }
                return next_byte;
            }
        case PS_CRC:
		    state = PS_LAST;
            return crc;
        default:
            // we should never get here
            return 0;
	}
}

void OutPacket::append16(uint16_t value) {
	appendByte(value&0xff);
	appendByte((value>>8));  // don't need to bit-wise AND the last byte
}
void OutPacket::append32(uint32_t value) {
	appendByte(value&0xff);
	appendByte((value>>8)&0xff);
	appendByte((value>>16)&0xff);
	appendByte((value>>24)); // don't need to bit-wise AND the last byte
}
