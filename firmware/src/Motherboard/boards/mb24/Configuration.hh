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

#ifndef BOARDS_MB24_CONFIGURATION_HH_
#define BOARDS_MB24_CONFIGURATION_HH_


/// This file details the pin assignments and features of the
/// Makerbot Motherboard v2.x

/// Interval for the stepper update in microseconds.  This interval is the minimum
/// possible time between steps; in practical terms, your time between steps should
/// be at least eight times this large.  Reducing the interval can cause resource
/// starvation; leave this at 64uS or greater unless you know what you're doing.
#define INTERVAL_IN_MICROSECONDS 128

// --- Secure Digital Card configuration ---
// NOTE: If SD support is enabled, it is implicitly assumed that the
// following pins are connected:
//  AVR    |   SD header
//---------|--------------
//  MISO   |   DATA_OUT
//  MOSI   |   DATA_IN
//  SCK    |   CLK

// Define as 1 if and SD card slot is present; 0 if not.
#define HAS_SD                  1
// The pin that connects to the write protect line on the SD header.
#define SD_WRITE_PIN            Pin(PortD,0)
// The pin that connects to the card detect line on the SD header.
#define SD_DETECT_PIN           Pin(PortD,1)
// The pin that connects to the chip select line on the SD header.
#define SD_SELECT_PIN           Pin(PortB,0)


// --- Slave UART configuration ---
// The slave UART is presumed to be an RS485 connection through a sn75176 chip.
// Define as 1 if the slave UART is present; 0 if not.
#define HAS_SLAVE_UART          1
// The pin that connects to the driver enable line on the RS485 chip.
#define TX_ENABLE_PIN           Pin(PortC,5)
// The pin that connects to the active-low recieve enable line on the RS485 chip.
#define RX_ENABLE_PIN           Pin(PortC,7)


// --- Host UART configuration ---
// The host UART is presumed to always be present on the RX/TX lines.


// --- Piezo Buzzer configuration ---
// Define as 1 if the piezo buzzer is present, 0 if not.
#define HAS_BUZZER              1
// The pin that drives the buzzer
#define BUZZER_PIN              Pin(PortC,6)


// --- Emergency Stop configuration ---
// Define as 1 if the estop is present, 0 if not.
#define HAS_ESTOP               1
// The pin connected to the emergency stop
#define ESTOP_PIN               Pin(PortE,4)


// --- Axis configuration ---
// Define the number of stepper axes supported by the board.  The axes are
// denoted by X, Y, Z, A and B.
#define STEPPER_COUNT           5


// --- Stepper and endstop configuration ---
// Pins should be defined for each axis present on the board.  They are denoted
// X, Y, Z, A and B respectively.

// This indicates the default interpretation of the endstop values.
// If your endstops are based on the H21LOB, they are inverted;
// if they are based on the H21LOI, they are not.
#define DEFAULT_INVERTED_ENDSTOPS 1

// The X stepper step pin (active on rising edge)
#define X_STEP_PIN              Pin(PortA,6)
// The X direction pin (forward on logic high)
#define X_DIR_PIN               Pin(PortA,5)
// The X stepper enable pin (active low)
#define X_ENABLE_PIN            Pin(PortA,4)
// The X minimum endstop pin (active high)
#define X_MIN_PIN               Pin(PortB,6)
// The X maximum endstop pin (active high)
#define X_MAX_PIN               Pin(PortB,5)

// The Y stepper step pin (active on rising edge)
#define Y_STEP_PIN              Pin(PortA,3)
// The Y direction pin (forward on logic high)
#define Y_DIR_PIN               Pin(PortA,2)
// The Y stepper enable pin (active low)
#define Y_ENABLE_PIN            Pin(PortA,1)
// The Y minimum endstop pin (active high)
#define Y_MIN_PIN               Pin(PortB,4)
// The Y maximum endstop pin (active high)
#define Y_MAX_PIN               Pin(PortH,6)

// The Z stepper step pin (active on rising edge)
#define Z_STEP_PIN              Pin(PortA,0)
// The Z direction pin (forward on logic high)
#define Z_DIR_PIN               Pin(PortH,0)
// The Z stepper enable pin (active low)
#define Z_ENABLE_PIN            Pin(PortH,1)
// The Z minimum endstop pin (active high)
#define Z_MIN_PIN               Pin(PortH,5)
// The Z maximum endstop pin (active high)
#define Z_MAX_PIN               Pin(PortH,4)

// The A stepper step pin (active on rising edge)
#define A_STEP_PIN              Pin(PortJ,0)
// The A direction pin (forward on logic high)
#define A_DIR_PIN               Pin(PortJ,1)
// The A stepper enable pin (active low)
#define A_ENABLE_PIN            Pin(PortE,5)

// The B stepper step pin (active on rising edge)
#define B_STEP_PIN              Pin(PortG,5)
// The B direction pin (forward on logic high)
#define B_DIR_PIN               Pin(PortE,3)
// The B stepper enable pin (active low)
#define B_ENABLE_PIN            Pin(PortH,3)


// --- Debugging configuration ---
// The pin which controls the debug LED (active high)
#define DEBUG_PIN               Pin(PortB,7)
// By default, debugging packets should be honored; this is made
// configurable if we're short on cycles or EEPROM.
// Define as 1 if debugging packets are honored; 0 if not.
#define HONOR_DEBUG_PACKETS     1

#define HAS_INTERFACE_BOARD     1


/// Pin mappings for the LCD connection.
#define LCD_RS_PIN		Pin(PortC,4)
#define LCD_ENABLE_PIN          Pin(PortC,3)
#define LCD_D0_PIN		Pin(PortD,7)
#define LCD_D1_PIN		Pin(PortG,2)
#define LCD_D2_PIN		Pin(PortG,1)
#define LCD_D3_PIN		Pin(PortG,0)

/// This is the pin mapping for the interface board. Because of the relatively
/// high cost of using the pins in a direct manner, we will instead read the
/// buttons directly by scanning their ports. If any of these definitions are
/// modified, the #scanButtons() function _must_ be updated to reflect this.
///
/// TLDR: These are here for decoration only, actual pins defined in #scanButtons()
#define INTERFACE_X+_PIN        Pin(PortL,7)
#define INTERFACE_X-_PIN        Pin(PortL,6)
#define INTERFACE_Y+_PIN        Pin(PortL,5)
#define INTERFACE_Y-_PIN        Pin(PortL,4)
#define INTERFACE_Z+_PIN        Pin(PortL,3)
#define INTERFACE_Z-_PIN        Pin(PortL,2)
#define INTERFACE_ZERO_PIN      Pin(PortL,1)

#define INTERFACE_OK_PIN        Pin(PortC,2)
#define INTERFACE_CANCEL_PIN    Pin(PortC,1)

#define INTERFACE_FOO_PIN       Pin(PortC,0)
#define INTERFACE_BAR_PIN       Pin(PortL,0)
#define INTERFACE_DEBUG_PIN     Pin(PortB,7)

#endif // BOARDS_RRMBV12_CONFIGURATION_HH_
