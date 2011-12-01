
#include "Interface.hh"
#include "InterfaceBoard.hh"
#if HAS_INTERFACE_BOARD > 0


// TODO: Make this a proper module.

namespace interface {


InterfaceBoard* board;

bool isConnected() {

#if HAS_INTERFACE_BUTTONS > 0
	// Strategy: Set up the foo pin as an input, turn on pull up resistor,
	// then measure it. If low, then we probably have an interface board.
	// If high, we probably don't.

	INTERFACE_FOO_PIN::setValue(true);
	INTERFACE_FOO_PIN::setDirection(false);

	// if we are pulled down, then we have an led attached??
	if (!INTERFACE_FOO_PIN::getValue()) {
		INTERFACE_FOO_PIN::setDirection(true);
		INTERFACE_FOO_PIN::setValue(true);

		return true;
	}
	else {
		INTERFACE_FOO_PIN::setDirection(true);
		INTERFACE_FOO_PIN::setValue(false);

		return false;
	}

	return (!INTERFACE_FOO_PIN::getValue());
#else
    return true;
#endif HAS_INTERFACE_BUTTONS > 0

}

void init(InterfaceBoard* board_in) {
    board = board_in;
}

}

#endif // HAS_INTERFACE_BOARD > 0
