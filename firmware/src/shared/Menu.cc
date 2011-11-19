#include "InterfaceBoard.hh"
#if HAS_INTERFACE_BOARD > 0
#include "Menu.hh"

#include "Steppers.hh"
#include "Commands.hh"
#include "Errors.hh"
#include "Tool.hh"
#include "Host.hh"
#include "Timeout.hh"
#include "Interface.hh"
#include <util/delay.h>
#include <stdlib.h>
#include "SDCard.hh"


#define HOST_PACKET_TIMEOUT_MS 20
#define HOST_PACKET_TIMEOUT_MICROS (1000L*HOST_PACKET_TIMEOUT_MS)

#define HOST_TOOL_RESPONSE_TIMEOUT_MS 50
#define HOST_TOOL_RESPONSE_TIMEOUT_MICROS (1000L*HOST_TOOL_RESPONSE_TIMEOUT_MS)

/// Send a query packet to the extruder
bool queryExtruderParameter(uint8_t parameter, OutPacket& responsePacket) {

	Timeout acquire_lock_timeout;
	acquire_lock_timeout.start(HOST_TOOL_RESPONSE_TIMEOUT_MS);
	while (!tool::getLock()) {
		if (acquire_lock_timeout.hasElapsed()) {
			return false;
		}
	}
	OutPacket& out = tool::getOutPacket();
	InPacket& in = tool::getInPacket();
	out.reset();
	responsePacket.reset();

	// Fill the query packet. The first byte is the toolhead index, and the
	// second is the
	out.append8(0);
	out.append8(parameter);

	// Timeouts are handled inside the toolslice code; there's no need
	// to check for timeouts on this loop.
	tool::startTransaction();
	tool::releaseLock();
	// WHILE: bounded by tool timeout in runToolSlice
	while (!tool::isTransactionDone()) {
		tool::runToolSlice();
	}
	if (in.getErrorCode() == PacketError::PACKET_TIMEOUT) {
		return false;
	} else {
		// Copy payload back. Start from 0-- we need the response code.
		for (uint8_t i = 0; i < in.getLength(); i++) {
			responsePacket.append8(in.read8(i));
		}
	}

	// Check that the extruder was able to process the request
	if (!rcCompare(responsePacket.read8(0),RC_OK)) {
		return false;
	}

	return true;
}

void SplashScreen::update(Display& display, bool forceRedraw) {
	static PROGMEM prog_char splash1[] = "    CraigBot    ";
	static PROGMEM prog_char splash2[] = "   ----------   ";
	static PROGMEM prog_char splash3[] = "  Cupcake  CNC  ";
    static PROGMEM prog_char splash4[] = "      #545      ";


	if (forceRedraw) {
		display.setCursor(0,0);
		display.writeFromPgmspace(splash1);

		display.setCursor(0,1);
		display.writeFromPgmspace(splash2);

		display.setCursor(0,2);
		display.writeFromPgmspace(splash3);

		display.setCursor(0,3);
		display.writeFromPgmspace(splash4);
	}
	else {
		// The machine has started, so we're done!
            interface::popScreen();
        }
}

void SplashScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	// We can't really do anything, since the machine is still loading, so ignore.
}

void SplashScreen::reset() {
}

void JogMode::reset() {
	jogDistance = DISTANCE_SHORT;
	distanceChanged = false;
}

void JogMode::update(Display& display, bool forceRedraw) {
	static PROGMEM prog_char jog1[] = "Jog mode: ";
	static PROGMEM prog_char jog2[] = "  Y+          Z+";
	static PROGMEM prog_char jog3[] = "X-  X+    (mode)";
	static PROGMEM prog_char jog4[] = "  Y-          Z-";

	static PROGMEM prog_char distanceShort[] = "SHORT";
	static PROGMEM prog_char distanceLong[] = "LONG";

	if (forceRedraw || distanceChanged) {
		display.clear();
		display.setCursor(0,0);
		display.writeFromPgmspace(jog1);

		switch (jogDistance) {
		case DISTANCE_SHORT:
			display.writeFromPgmspace(distanceShort);
			break;
		case DISTANCE_LONG:
			display.writeFromPgmspace(distanceLong);
			break;
		}

		display.setCursor(0,1);
		display.writeFromPgmspace(jog2);

		display.setCursor(0,2);
		display.writeFromPgmspace(jog3);

		display.setCursor(0,3);
		display.writeFromPgmspace(jog4);

		distanceChanged = false;
	}
}

void JogMode::jog(ButtonArray::ButtonName direction) {
	Point position = steppers::getPosition();

	int32_t interval = 2000;
	uint8_t steps;

	switch(jogDistance) {
	case DISTANCE_SHORT:
		steps = 20;
		break;
	case DISTANCE_LONG:
		steps = 200;
		break;
	}

	switch(direction) {
        case ButtonArray::XMINUS:
		position[0] -= steps;
		break;
        case ButtonArray::XPLUS:
		position[0] += steps;
		break;
        case ButtonArray::YMINUS:
		position[1] -= steps;
		break;
        case ButtonArray::YPLUS:
		position[1] += steps;
		break;
        case ButtonArray::ZMINUS:
		position[2] -= steps;
		break;
        case ButtonArray::ZPLUS:
		position[2] += steps;
		break;
	}

	steppers::setTarget(position, interval);
}

void JogMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::ZERO:
        case ButtonArray::OK:
		if (jogDistance == DISTANCE_SHORT) {
			jogDistance = DISTANCE_LONG;
		}
		else {
			jogDistance = DISTANCE_SHORT;
		}
		distanceChanged = true;
		break;
        case ButtonArray::YMINUS:
        case ButtonArray::ZMINUS:
        case ButtonArray::YPLUS:
        case ButtonArray::ZPLUS:
        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		jog(button);
		break;
        case ButtonArray::CANCEL:
                interface::popScreen();
		break;
	}
}


void SnakeMode::update(Display& display, bool forceRedraw) {
	static PROGMEM prog_char gameOver[] =  "GAME OVER!";

	// If we are dead, restart the game.
	if (!snakeAlive) {
		reset();
		forceRedraw = true;
	}

    if (appleReset)
    {
	    // Put the apple in an initial position (this could collide with the snake!)    
	    applePosition.x = rand()%display.width();
	    applePosition.y = rand()%display.height();
        appleReset = false;
    }

	if (forceRedraw) {
		display.clear();

		for (uint8_t i = 0; i < snakeLength; i++) {
			display.setCursor(snakeBody[i].x, snakeBody[i].y);
			display.write('O');
		}
	}

	// Always redraw the apple, just in case.
	display.setCursor(applePosition.x, applePosition.y);
	display.write('*');

	// First, undraw the snake's tail
	display.setCursor(snakeBody[snakeLength-1].x, snakeBody[snakeLength-1].y);
	display.write(' ');

	// Then, shift the snakes body parts back, deleting the tail
	for(int8_t i = snakeLength-1; i >= 0; i--) {
		snakeBody[i+1] = snakeBody[i];
	}

	// Create a new head for the snake (this causes it to move forward)
	switch(snakeDirection)
	{
	case DIR_EAST:
		snakeBody[0].x = (snakeBody[0].x + 1) % display.width();
		break;
	case DIR_WEST:
		snakeBody[0].x = (snakeBody[0].x +  display.width() - 1) % display.width();
		break;
	case DIR_NORTH:
		snakeBody[0].y = (snakeBody[0].y + display.height() - 1) % display.height();
		break;
	case DIR_SOUTH:
		snakeBody[0].y = (snakeBody[0].y + 1) % display.height();
		break;
	}

	// Now, draw the snakes new head
	display.setCursor(snakeBody[0].x, snakeBody[0].y);
	display.write('O');

	// Check if the snake has run into itself
	for (uint8_t i = 1; i < snakeLength; i++) {
		if (snakeBody[i].x == snakeBody[0].x
			&& snakeBody[i].y == snakeBody[0].y) {
			snakeAlive = false;

			display.setCursor(1,1);
			display.writeFromPgmspace(gameOver);
			updateRate = 5000L * 1000L;
		}
	}

	// If the snake just ate an apple, increment count and make new apple
	if (snakeBody[0].x == applePosition.x
			&& snakeBody[0].y == applePosition.y) {
		applesEaten++;

		if(applesEaten % APPLES_BEFORE_GROW == 0) {
			snakeLength++;
			updateRate -= 5L * 1000L;
		}

		applePosition.x = rand()%display.width();
		applePosition.y = rand()%display.height();

		display.setCursor(applePosition.x, applePosition.y);
		display.write('*');
	}
}

void SnakeMode::reset() {
	updateRate = 150L * 1000L;
	snakeDirection = DIR_EAST;
	snakeLength = 3;
	applesEaten = 0;
	snakeAlive = true;

	// Put the snake in an initial position
	snakeBody[0].x = 2; snakeBody[0].y = 1;
	snakeBody[1].x = 1; snakeBody[1].y = 1;
	snakeBody[2].x = 0; snakeBody[2].y = 1;

    appleReset = true;
}


void SnakeMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::YMINUS:
		    snakeDirection = DIR_SOUTH;
		    break;
        case ButtonArray::YPLUS:
		    snakeDirection = DIR_NORTH;
		    break;
        case ButtonArray::XMINUS:
		    snakeDirection = DIR_WEST;
		    break;
        case ButtonArray::XPLUS:
		    snakeDirection = DIR_EAST;
		    break;
        case ButtonArray::CANCEL:
            interface::popScreen();
    		break;
	}
}


void MonitorMode::reset() {
	updatePhase = 0;
}

void MonitorMode::update(Display& display, bool forceRedraw) {
	static PROGMEM prog_char extruder_temp[] =   "Tool: ---/---C";
	static PROGMEM prog_char platform_temp[] =   "Bed:  ---/---C";

	if (forceRedraw) {
		display.clear();
		display.setCursor(0,0);
		switch(host::getHostState()) {
		case host::HOST_STATE_READY:
			display.writeString(host::getMachineName());
			break;
		case host::HOST_STATE_BUILDING:
		case host::HOST_STATE_BUILDING_FROM_SD:
			display.writeString(host::getBuildName());
			break;
		case host::HOST_STATE_ERROR:
			display.writeString("error!");
			break;
		}

		display.setCursor(0,2);
		display.writeFromPgmspace(extruder_temp);

		display.setCursor(0,3);
		display.writeFromPgmspace(platform_temp);

	} else {
	}


	OutPacket responsePacket;

	// Redraw tool info
	switch (updatePhase) {
	case 0:
		display.setCursor(6,2);
		if (queryExtruderParameter(SLAVE_CMD_GET_TEMP, responsePacket)) {
			uint16_t data = responsePacket.read16(1);
			display.writeInt(data,3);
		} else {
			display.writeString("XXX");
		}
		break;

	case 1:
		display.setCursor(10,2);
		if (queryExtruderParameter(SLAVE_CMD_GET_SP, responsePacket)) {
			uint16_t data = responsePacket.read16(1);
			display.writeInt(data,3);
		} else {
			display.writeString("XXX");
		}
		break;

	case 2:
		display.setCursor(6,3);
		if (queryExtruderParameter(SLAVE_CMD_GET_PLATFORM_TEMP, responsePacket)) {
			uint16_t data = responsePacket.read16(1);
			display.writeInt(data,3);
		} else {
			display.writeString("XXX");
		}
		break;

	case 3:
		display.setCursor(10,3);
		if (queryExtruderParameter(SLAVE_CMD_GET_PLATFORM_SP, responsePacket)) {
			uint16_t data = responsePacket.read16(1);
			display.writeInt(data,3);
		} else {
			display.writeString("XXX");
		}
		break;
	}

	updatePhase++;
	if (updatePhase > 3) {
		updatePhase = 0;
	}
}

void MonitorMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
		switch(host::getHostState()) {
		case host::HOST_STATE_BUILDING:
		case host::HOST_STATE_BUILDING_FROM_SD:
                        interface::pushScreen(&cancelBuildMenu);
			break;
		default:
                        interface::popScreen();
			break;
		}
	}
}


void Menu::update(Display& display, bool forceRedraw) {
	static PROGMEM prog_char blankLine[] =  "                ";

	// Do we need to redraw the whole menu?
	if ((itemIndex/display.height()) != (lastDrawIndex/display.height())
			|| forceRedraw ) {
		// Redraw the whole menu
		display.clear();

		for (uint8_t i = 0; i < display.height(); i++) {
			// Instead of using display.clear(), clear one line at a time so there
			// is less screen flickr.

			if (i+(itemIndex/display.height())*display.height() +1 > itemCount) {
				break;
			}

			display.setCursor(1,i);
			// Draw one page of items at a time
			drawItem(i+(itemIndex/display.height())*display.height(), display);
		}
	}
	else {
		// Only need to clear the previous cursor
		display.setCursor(0,(lastDrawIndex%display.height()));
		display.write(' ');
	}

	display.setCursor(0,(itemIndex%display.height()));
	display.write('>');
	lastDrawIndex = itemIndex;
}

void Menu::reset() {
	firstItemIndex = 0;
	itemIndex = 0;
	lastDrawIndex = 255;

	resetState();
}

void Menu::resetState() {
}

void Menu::handleSelect(uint8_t index) {
}

void Menu::handleCancel() {
	// Remove ourselves from the menu list
        interface::popScreen();
}

void Menu::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::ZERO:
        case ButtonArray::OK:
		handleSelect(itemIndex);
		break;
        case ButtonArray::CANCEL:
		handleCancel();
		break;
        case ButtonArray::YMINUS:
        case ButtonArray::ZMINUS:
		// increment index
		if (itemIndex < itemCount - 1) {
			itemIndex++;
		}
		break;
        case ButtonArray::YPLUS:
        case ButtonArray::ZPLUS:
		// decrement index
		if (itemIndex > firstItemIndex) {
			itemIndex--;
		}
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}
}


CancelBuildMenu::CancelBuildMenu() {
	itemCount = 4;
	reset();
}

void CancelBuildMenu::resetState() {
	itemIndex = 2;
	firstItemIndex = 2;
}

void CancelBuildMenu::drawItem(uint8_t index, Display& display) {
	static PROGMEM prog_char cancel[] = "Cancel Build?";
	static PROGMEM prog_char yes[] =   "Yes";
	static PROGMEM prog_char no[] =   "No";

	switch (index) {
	case 0:
		display.writeFromPgmspace(cancel);
		break;
	case 1:
		break;
	case 2:
		display.writeFromPgmspace(yes);
		break;
	case 3:
		display.writeFromPgmspace(no);
		break;
	}
}

void CancelBuildMenu::handleSelect(uint8_t index) {
	switch (index) {
	case 2:
		// Cancel build, returning to whatever menu came before monitor mode.
		// TODO: Cancel build.
		interface::popScreen();
		host::stopBuild();
		break;
	case 3:
		// Don't cancel, just close dialog.
                interface::popScreen();
		break;
	}
}


MainMenu::MainMenu() {
	itemCount = 5;
	reset();
}

void MainMenu::drawItem(uint8_t index, Display& display) {
	static PROGMEM prog_char monitor[] = "Monitor Mode";
	static PROGMEM prog_char build[] =   "Build from SD";
	static PROGMEM prog_char jog[] =   "Jog Mode";
	static PROGMEM prog_char snake[] =   "Snake Game";

	switch (index) {
	case 0:
		display.writeFromPgmspace(monitor);
		break;
	case 1:
		display.writeFromPgmspace(build);
		break;
	case 2:
		display.writeFromPgmspace(jog);
		break;
	case 3:
		// blank
		break;
	case 4:
		display.writeFromPgmspace(snake);
		break;
	}
}

void MainMenu::handleSelect(uint8_t index) {
	switch (index) {
		case 0:
			// Show monitor build screen
                        interface::pushScreen(&monitorMode);
			break;
		case 1:
			// Show build from SD screen
                        interface::pushScreen(&sdMenu);
			break;
		case 2:
			// Show build from SD screen
                        interface::pushScreen(&jogger);
			break;
		case 4:
			// Show build from SD screen
                        interface::pushScreen(&snake);
			break;
		}
}

SDMenu::SDMenu() {
	reset();
}

void SDMenu::resetState() {
	itemCount = countFiles();
}

// Count the number of files on the SD card
uint8_t SDMenu::countFiles() {
	uint8_t count = 0;

	sdcard::SdErrorCode e;

	// First, reset the directory index
	e = sdcard::directoryReset();
	if (e != sdcard::SD_SUCCESS) {
		// TODO: Report error
		return 6;
	}

	const int MAX_FILE_LEN = 2;
	char fnbuf[MAX_FILE_LEN];

	// Count the files
	do {
		e = sdcard::directoryNextEntry(fnbuf,MAX_FILE_LEN);
		if (fnbuf[0] == '\0') {
			break;
		}

		// If it's a dot file, don't count it.
		if (fnbuf[0] == '.') {
		}
		else {
			count++;
		}
	} while (e == sdcard::SD_SUCCESS);

	// TODO: Check for error again?

	return count;
}

bool SDMenu::getFilename(uint8_t index, char buffer[], uint8_t buffer_size) {
	sdcard::SdErrorCode e;

	// First, reset the directory list
	e = sdcard::directoryReset();
	if (e != sdcard::SD_SUCCESS) {
                return false;
	}


	for(uint8_t i = 0; i < index+1; i++) {
		// Ignore dot-files
		do {
			e = sdcard::directoryNextEntry(buffer,buffer_size);
			if (buffer[0] == '\0') {
                                return false;
			}
		} while (e == sdcard::SD_SUCCESS && buffer[0] == '.');

		if (e != sdcard::SD_SUCCESS) {
                        return false;
		}
	}

        return true;
}

void SDMenu::drawItem(uint8_t index, Display& display) {
	if (index > itemCount - 1) {
		// TODO: report error
		return;
	}

	const uint8_t MAX_FILE_LEN = display.width();
	char fnbuf[MAX_FILE_LEN];

        if ( !getFilename(index, fnbuf, MAX_FILE_LEN) ) {
                // TODO: report error
		return;
	}

	uint8_t idx;
	for (idx = 0; (idx < MAX_FILE_LEN) && (fnbuf[idx] != 0); idx++) {
		display.write(fnbuf[idx]);
	}
}

void SDMenu::handleSelect(uint8_t index) {
	if (host::getHostState() != host::HOST_STATE_READY) {
		// TODO: report error
		return;
	}

	char* buildName = host::getBuildName();

        if ( !getFilename(index, buildName, host::MAX_FILE_LEN) ) {
		// TODO: report error
		return;
	}

        sdcard::SdErrorCode e;
	e = host::startBuildFromSD();
	if (e != sdcard::SD_SUCCESS) {
		// TODO: report error
		return;
	}
}

#endif
