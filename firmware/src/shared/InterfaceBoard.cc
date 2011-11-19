#include "InterfaceBoard.hh"
#if HAS_INTERFACE_BOARD > 0
#include "Host.hh"

#if DISPLAY_TYPE == DISPLAY_TYPE_LIQUIDCRYSTAL
#include "LiquidCrystal.hh"
static LiquidCrystal globalDisplay;

#elif DISPLAY_TYPE == DISPLAY_TYPE_MODTRONIXLCD2S
#include "ModtronixLCD2S.hh"
static ModtronixLCD2S globalDisplay;

#elif DISPLAY_TYPE == DISPLAY_TYPE_DUAL
#include "DualDisplay.hh"
static DualDisplay globalDisplay;
#endif


#define foo_pin INTERFACE_FOO_PIN
#define bar_pin INTERFACE_BAR_PIN



InterfaceBoard::InterfaceBoard(Screen* mainScreen_in,
                               Screen* buildScreen_in) :
    display(globalDisplay)
{
        buildScreen = buildScreen_in;
        mainScreen = mainScreen_in;
}

void InterfaceBoard::init() {
        buttons.init();

        display.init();

#if HAS_INTERFACE_BUTTONS > 0
        foo_pin::setValue(false);
        foo_pin::setDirection(true);
        bar_pin::setValue(false);
        bar_pin::setDirection(true);
#endif HAS_INTERFACE_BUTTONS > 0

        building = false;

        screenIndex = -1;

        pushScreen(mainScreen);
}

void InterfaceBoard::doInterrupt() {
	buttons.scanButtons();
}

micros_t InterfaceBoard::getUpdateRate() {
	return screenStack[screenIndex]->getUpdateRate();
}

void InterfaceBoard::doUpdate() {

	// If we are building, make sure we show a build menu; otherwise,
	// turn it off.
	switch(host::getHostState()) {
	case host::HOST_STATE_BUILDING:
	case host::HOST_STATE_BUILDING_FROM_SD:
		if (!building) {
            pushScreen(buildScreen);
			building = true;
		}
		break;
	default:
		if (building) {
			popScreen();
			building = false;
		}
		break;
	}


    static ButtonArray::ButtonName button;


	if (buttons.getButton(button)) {
		screenStack[screenIndex]->notifyButtonPressed(button);
	}

	screenStack[screenIndex]->update(display, false);
}

void InterfaceBoard::pushScreen(Screen* newScreen) {
	if (screenIndex < SCREEN_STACK_DEPTH - 1) {
		screenIndex++;
		screenStack[screenIndex] = newScreen;
	}
	screenStack[screenIndex]->reset();
	screenStack[screenIndex]->update(display, true);
}

void InterfaceBoard::popScreen() {
	// Don't allow the root menu to be removed.
	if (screenIndex > 0) {
		screenIndex--;
	}

	screenStack[screenIndex]->update(display, true);
}

#endif // HAS_INTERFACE_BOARD > 0
