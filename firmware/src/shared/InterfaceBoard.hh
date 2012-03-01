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


#ifndef INTERFACE_BOARD_HH_
#define INTERFACE_BOARD_HH_

#include "Configuration.hh"
#include "ButtonArray.hh"
#if HAS_INTERFACE_BOARD > 0
#include "Display.hh"
#include "Menu.hh"

/// Maximum number of screens that can be active at once.
#define SCREEN_STACK_DEPTH      5

/// Character LCD screen geometry
///
/// Porting Note: Screens may need to be rewritten to support different sizes.


/// The InterfaceBoard module provides support for the MakerBot Industries
/// Gen4 Interface Board. It could very likely be adopted to support other
/// LCD/button setups as well.
/// \ingroup HardwareLibraries
class InterfaceBoard {
public:
        Display& display;              ///< LCD to write to
private:

        ButtonArray buttons;            ///< Button array to read from

        // TODO: Drop this?
        Screen* buildScreen;            ///< Screen to display while building

        // TODO: Drop this?
        Screen* mainScreen;            ///< Root menu screen

        /// Stack of screens to display; the topmost one will actually
        /// be drawn to the screen, while the other will remain resident
        /// but not active.
        Screen* screenStack[SCREEN_STACK_DEPTH];
        int8_t screenIndex;             ///< Stack index of the current screen.

        /// TODO: Delete this.
        bool building;                  ///< True if the bot is building

public:
        /// Construct an interface board.
        /// \param[in] Main screen, shown as root display
        /// \param[in] Screen to display while building
        InterfaceBoard(Screen* mainScreen_in,
                       Screen* buildScreen_in);

        /// Initialze the interface board. This needs to be called once
        /// at system startup (or reset).
	void init();

        /// This should be called periodically by a high-speed interrupt to
        /// service the button input pad.
	inline void doInterrupt() { buttons.scanButtons(); }

        /// Add a new screen to the stack. This automatically calls reset()
        /// and then update() on the screen, to ensure that it displays
        /// properly. If there are more than SCREEN_STACK_DEPTH screens already
        /// in the stack, than this function does nothing.
        /// \param[in] newScreen Screen to display.
	void pushScreen(Screen* newScreen);

        /// Remove the current screen from the stack. If there is only one screen
        /// being displayed, then this function does nothing.
	void popScreen();

    inline micros_t getUpdateRate() const { return screenStack[screenIndex]->getUpdateRate(); }

	void doUpdate();

	void showMonitorMode();

    static void Debug(const char message[]);
};
#endif
#endif
