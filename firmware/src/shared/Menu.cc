#include "Menu.hh"
#include "Configuration.hh"

// TODO: Kill this, should be hanlded by build system.
#ifdef HAS_INTERFACE_BOARD

#include "Steppers.hh"
#include "Commands.hh"
#include "Errors.hh"
#include "Tool.hh"
#include "Host.hh"
#include "Timeout.hh"
#include "InterfaceBoard.hh"
#include "Interface.hh"
#include "Motherboard.hh"
#include "Version.hh"
#include <util/delay.h>
#include <stdlib.h>
#include "SDCard.hh"
#include "EepromMap.hh"
#include "Eeprom.hh"
#include <avr/eeprom.h>
#include "ExtruderControl.hh"


#define HOST_PACKET_TIMEOUT_MS 20
#define HOST_PACKET_TIMEOUT_MICROS (1000L*HOST_PACKET_TIMEOUT_MS)

#define HOST_TOOL_RESPONSE_TIMEOUT_MS 50
#define HOST_TOOL_RESPONSE_TIMEOUT_MICROS (1000L*HOST_TOOL_RESPONSE_TIMEOUT_MS)

#define MAX_ITEMS_PER_SCREEN 4

int16_t overrideExtrudeSeconds = 0;

bool estimatingBuild = false;

Point pausedPosition, homePosition;

//Stored using STEPS_PER_MM_PRECISION
int64_t axisStepsPerMM[5];


enum Axis {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
	AXIS_A,
	AXIS_B
};


//Stack checking
//http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=52249
extern uint8_t _end; 
extern uint8_t __stack;

#define STACK_CANARY 0xc5

void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init1"))); 

void StackPaint(void) 
{ 
#if 0 
    uint8_t *p = &_end; 

    while(p <= &__stack) 
    { 
        *p = STACK_CANARY; 
        p++; 
    } 
#else 
    __asm volatile ("    ldi r30,lo8(_end)\n" 
                    "    ldi r31,hi8(_end)\n" 
                    "    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */ 
                    "    ldi r25,hi8(__stack)\n" 
                    "    rjmp .cmp\n" 
                    ".loop:\n" 
                    "    st Z+,r24\n" 
                    ".cmp:\n" 
                    "    cpi r30,lo8(__stack)\n" 
                    "    cpc r31,r25\n" 
                    "    brlo .loop\n" 
                    "    breq .loop"::); 
#endif 
}


uint16_t StackCount(void) 
{ 
    const uint8_t *p = &_end; 
    uint16_t       c = 0; 

    while(*p == STACK_CANARY && p <= &__stack) 
    { 
        p++; 
        c++; 
    } 

    return c; 
}


void VersionMode::reset() {
}


//Convert mm's to steps for the given axis
//Accurate to 1/1000 mm

int32_t mmToSteps(float mm, enum Axis axis) {
	//Multiply mm by 1000 to avoid floating point errors
	int64_t intmm = (int64_t)(mm * 1000.0);

	//Calculate the number of steps
	int64_t ret = intmm * axisStepsPerMM[axis];

	//Divide the number of steps by the fixed precision and
	//mm 1000;
	for (uint8_t i=0 ; i < STEPS_PER_MM_PRECISION; i ++ )
		ret /= 10;
	ret /= 1000;
	
	return (int32_t)ret;
}

//Convert steps to mm's
//As accurate as floating point is

float stepsToMM(int32_t steps, enum Axis axis) {
	//Convert axisStepsPerMM to a float	
	float aspmf = (float)axisStepsPerMM[axis];
	for (uint8_t i=0 ; i < STEPS_PER_MM_PRECISION; i ++ )
		aspmf /= 10.0;
	return (float)steps / aspmf;
}

void strcat(char *buf, const char* str)
{
	char *ptr = buf;
	while (*ptr) ptr++;
	while (*str) *ptr++ = *str++;
	*ptr++ = '\0';
}


int appendTime(char *buf, uint8_t buflen, uint32_t val)
{
	bool hasdigit = false;
	uint8_t idx = 0;
	uint8_t written = 0;

	if (buflen < 1) {
		return written;
	}

	while (idx < buflen && buf[idx]) idx++;
	if (idx >= buflen-1) {
		buf[buflen-1] = '\0';
		return written;
	}

	uint8_t radidx = 0;
	const uint8_t radixcount = 5;
	const uint8_t houridx = 2;
	const uint8_t minuteidx = 4;
	uint32_t radixes[radixcount] = {360000, 36000, 3600, 600, 60};
	if (val >= 3600000) {
		val %= 3600000;
	}
	for (radidx = 0; radidx < radixcount; radidx++) {
		char digit = '0';
		uint8_t bit = 8;
		uint32_t radshift = radixes[radidx] << 3;
		for (; bit > 0; bit >>= 1, radshift >>= 1) {
			if (val > radshift) {
				val -= radshift;
				digit += bit;
			}
		}
		if (hasdigit || digit != '0' || radidx >= houridx) {
			buf[idx++] = digit;
			hasdigit = true;
		} else {
			buf[idx++] = ' ';
		}
		if (idx >= buflen) {
			buf[buflen-1] = '\0';
			return written;
		}
		written++;
		if (radidx == houridx) {
			buf[idx++] = 'h';
			if (idx >= buflen) {
				buf[buflen-1] = '\0';
				return written;
			}
			written++;
		}
		if (radidx == minuteidx) {
			buf[idx++] = 'm';
			if (idx >= buflen) {
				buf[buflen-1] = '\0';
				return written;
			}
			written++;
		}
	}

	if (idx < buflen) {
		buf[idx] = '\0';
	} else {
		buf[buflen-1] = '\0';
	}

	return written;
}



int appendUint8(char *buf, uint8_t buflen, uint8_t val)
{
	bool hasdigit = false;
	uint8_t written = 0;
	uint8_t idx = 0;

	if (buflen < 1) {
		return written;
	}

	while (idx < buflen && buf[idx]) idx++;
	if (idx >= buflen-1) {
		buf[buflen-1] = '\0';
		return written;
	}

	if (val >= 100) {
		uint8_t res = val / 100;
		val -= res * 100;
		buf[idx++] = '0' + res;
		if (idx >= buflen) {
			buf[buflen-1] = '\0';
			return written;
		}
		hasdigit = true;
		written++;
	} else {
		buf[idx++] = ' ';
		if (idx >= buflen) {
			buf[buflen-1] = '\0';
			return written;
		}
		written++;
	}

	if (val >= 10 || hasdigit) {
		uint8_t res = val / 10;
		val -= res * 10;
		buf[idx++] = '0' + res;
		if (idx >= buflen) {
			buf[buflen-1] = '\0';
			return written;
		}
		hasdigit = true;
		written++;
	} else {
		buf[idx++] = ' ';
		if (idx >= buflen) {
			buf[buflen-1] = '\0';
			return written;
		}
		written++;
	}

	buf[idx++] = '0' + val;
	if (idx >= buflen) {
		buf[buflen-1] = '\0';
		return written;
	}
	written++;

	if (idx < buflen) {
		buf[idx] = '\0';
	} else {
		buf[buflen-1] = '\0';
	}

	return written;
}



void SplashScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar splash1[] = " Thing-O-Matic  ";
	const static PROGMEM prog_uchar splash2[] = "   ---------    ";
	const static PROGMEM prog_uchar splash3[] = " Jetty Firmware ";
	const static PROGMEM prog_uchar splash4[] = "Thing 15380 3.0b";


	if (forceRedraw) {
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(splash1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(splash2);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(splash3);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(splash4);
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

UserViewMenu::UserViewMenu() {
	itemCount = 4;
	reset();
}

void UserViewMenu::resetState() {
        uint8_t jogModeSettings = eeprom::getEeprom8(eeprom::JOG_MODE_SETTINGS, 0);

	if ( jogModeSettings & 0x01 )	itemIndex = 3;
	else				itemIndex = 2;

	firstItemIndex = 2;
}

void UserViewMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg[]  = "X/Y Direction:";
	const static PROGMEM prog_uchar model[]= "Model View";
	const static PROGMEM prog_uchar user[] = "User View";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg);
		break;
	case 1:
		break;
	case 2:
		lcd.writeFromPgmspace(model);
		break;
	case 3:
		lcd.writeFromPgmspace(user);
		break;
	}
}

void UserViewMenu::handleSelect(uint8_t index) {
	uint8_t jogModeSettings = eeprom::getEeprom8(eeprom::JOG_MODE_SETTINGS, 0);

	switch (index) {
	case 2:
		// Model View
		eeprom_write_byte((uint8_t *)eeprom::JOG_MODE_SETTINGS, (jogModeSettings & (uint8_t)0xFE));
		interface::popScreen();
		break;
	case 3:
		// User View
		eeprom_write_byte((uint8_t *)eeprom::JOG_MODE_SETTINGS, (jogModeSettings | (uint8_t)0x01));
                interface::popScreen();
		break;
	}
}

void JogMode::reset() {
	uint8_t jogModeSettings = eeprom::getEeprom8(eeprom::JOG_MODE_SETTINGS, 0);

	jogDistance = (enum distance_t)((jogModeSettings >> 1 ) & 0x07);
	if ( jogDistance > DISTANCE_CONT ) jogDistance = DISTANCE_0_1MM;

	distanceChanged = false;
	lastDirectionButtonPressed = (ButtonArray::ButtonName)0;

	userViewMode = jogModeSettings & 0x01;
	userViewModeChanged = false;

	steppers::switchToRegularDriver();
}

void JogMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar jog1[]      = "Jog mode: ";
	const static PROGMEM prog_uchar jog2[] 	    = "   Y+         Z+";
	const static PROGMEM prog_uchar jog3[]      = "X- V  X+  (mode)";
	const static PROGMEM prog_uchar jog4[]      = "   Y-         Z-";
	const static PROGMEM prog_uchar jog2_user[] = "  Y           Z+";
	const static PROGMEM prog_uchar jog3_user[] = "X V X     (mode)";
	const static PROGMEM prog_uchar jog4_user[] = "  Y           Z-";

	const static PROGMEM prog_uchar distance0_1mm[] = ".1mm";
	const static PROGMEM prog_uchar distance1mm[] = "1mm";
	const static PROGMEM prog_uchar distanceCont[] = "Cont..";

	if ( userViewModeChanged ) userViewMode = eeprom::getEeprom8(eeprom::JOG_MODE_SETTINGS, 0) & 0x01;

	if (forceRedraw || distanceChanged || userViewModeChanged) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(jog1);

		switch (jogDistance) {
		case DISTANCE_0_1MM:
			lcd.write(0xF3);	//Write tilde
			lcd.writeFromPgmspace(distance0_1mm);
			break;
		case DISTANCE_1MM:
			lcd.write(0xF3);	//Write tilde
			lcd.writeFromPgmspace(distance1mm);
			break;
		case DISTANCE_CONT:
			lcd.writeFromPgmspace(distanceCont);
			break;
		}

		lcd.setCursor(0,1);
		if ( userViewMode )	lcd.writeFromPgmspace(jog2_user);
		else			lcd.writeFromPgmspace(jog2);

		lcd.setCursor(0,2);
		if ( userViewMode )	lcd.writeFromPgmspace(jog3_user);
		else			lcd.writeFromPgmspace(jog3);

		lcd.setCursor(0,3);
		if ( userViewMode )	lcd.writeFromPgmspace(jog4_user);
		else			lcd.writeFromPgmspace(jog4);

		distanceChanged = false;
		userViewModeChanged    = false;
	}

	if ( jogDistance == DISTANCE_CONT ) {
		if ( lastDirectionButtonPressed ) {
			if (interface::isButtonPressed(lastDirectionButtonPressed))
				JogMode::notifyButtonPressed(lastDirectionButtonPressed);
			else {
				lastDirectionButtonPressed = (ButtonArray::ButtonName)0;
				steppers::abort();
			}
		}
	}
}

void JogMode::jog(ButtonArray::ButtonName direction) {
	Point position = steppers::getPosition();

	int32_t interval = 2000;
	float	speed;	//In mm's

	if ( jogDistance == DISTANCE_CONT )	interval = 1000;

	switch(jogDistance) {
	case DISTANCE_0_1MM:
		speed = 0.1;   //0.1mm
		break;
	case DISTANCE_1MM:
		speed = 1.0;   //1mm
		break;
	case DISTANCE_CONT:
		speed = 1.5;   //1.5mm
		break;
	}


	//Reverse direction of X and Y if we're in User View Mode and
	//not model mode
	int32_t vMode = 1;
	if ( userViewMode ) vMode = -1;

	switch(direction) {
        case ButtonArray::XMINUS:
		position[0] -= vMode * mmToSteps(speed,AXIS_X);
		break;
        case ButtonArray::XPLUS:
		position[0] += vMode * mmToSteps(speed,AXIS_X);
		break;
        case ButtonArray::YMINUS:
		position[1] -= vMode * mmToSteps(speed,AXIS_Y);
		break;
        case ButtonArray::YPLUS:
		position[1] += vMode * mmToSteps(speed,AXIS_Y);
		break;
        case ButtonArray::ZMINUS:
		position[2] -= mmToSteps(speed,AXIS_Z);
		break;
        case ButtonArray::ZPLUS:
		position[2] += mmToSteps(speed,AXIS_Z);
		break;
	}

	if ( jogDistance == DISTANCE_CONT )	lastDirectionButtonPressed = direction;
	else					lastDirectionButtonPressed = (ButtonArray::ButtonName)0;

	steppers::setTarget(position, interval);
}

void JogMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::ZERO:
		userViewModeChanged = true;
		interface::pushScreen(&userViewMenu);
		break;
        case ButtonArray::OK:
		switch(jogDistance)
		{
			case DISTANCE_0_1MM:
				jogDistance = DISTANCE_1MM;
				break;
			case DISTANCE_1MM:
				jogDistance = DISTANCE_CONT;
				break;
			case DISTANCE_CONT:
				jogDistance = DISTANCE_0_1MM;
				break;
		}
		distanceChanged = true;
		eeprom_write_byte((uint8_t *)eeprom::JOG_MODE_SETTINGS, userViewMode | (jogDistance << 1));
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
		steppers::abort();
		steppers::enableAxis(0, false);
		steppers::enableAxis(1, false);
		steppers::enableAxis(2, false);
                interface::popScreen();
		steppers::switchToAcceleratedDriver();
		break;
	}
}

void ExtruderMode::reset() {
	extrudeSeconds = (enum extrudeSeconds)eeprom::getEeprom8(eeprom::EXTRUDE_DURATION, 1);
	updatePhase = 0;
	timeChanged = false;
	lastDirection = 1;
	overrideExtrudeSeconds = 0;
	steppers::switchToRegularDriver();
}

void ExtruderMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar extrude1[] = "Extrude: ";
	const static PROGMEM prog_uchar extrude2[] = "(set rpm)    Fwd";
	const static PROGMEM prog_uchar extrude3[] = " (stop)    (dur)";
	const static PROGMEM prog_uchar extrude4[] = "---/---C     Rev";
	const static PROGMEM prog_uchar secs[]     = "SECS";
	const static PROGMEM prog_uchar blank[]    = "       ";

	if (overrideExtrudeSeconds)	extrude(overrideExtrudeSeconds, true);

	if (forceRedraw) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(extrude1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(extrude2);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(extrude3);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(extrude4);
	}

	if ((forceRedraw) || (timeChanged)) {
		lcd.setCursor(9,0);
		lcd.writeFromPgmspace(blank);
		lcd.setCursor(9,0);
		lcd.writeFloat((float)extrudeSeconds, 0);
		lcd.writeFromPgmspace(secs);
		timeChanged = false;
	}

	OutPacket responsePacket;
	Point position;

	// Redraw tool info
	switch (updatePhase) {
	case 0:
		lcd.setCursor(0,3);
		if (extruderControl(SLAVE_CMD_GET_TEMP, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t data = responsePacket.read16(1);
			lcd.writeInt(data,3);
		} else {
			lcd.writeString("XXX");
		}
		break;

	case 1:
		lcd.setCursor(4,3);
		if (extruderControl(SLAVE_CMD_GET_SP, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t data = responsePacket.read16(1);
			lcd.writeInt(data,3);
		} else {
			lcd.writeString("XXX");
		}
		break;
	}

	updatePhase++;
	if (updatePhase > 1) {
		updatePhase = 0;
	}
}

void ExtruderMode::extrude(seconds_t seconds, bool overrideTempCheck) {
	//Check we're hot enough
	if ( ! overrideTempCheck )
	{
		OutPacket responsePacket;
		if (extruderControl(SLAVE_CMD_IS_TOOL_READY, EXTDR_CMD_GET, responsePacket, 0)) {
			uint8_t data = responsePacket.read8(1);
		
			if ( ! data )
			{
				overrideExtrudeSeconds = seconds;
				interface::pushScreen(&extruderTooColdMenu);
				return;
			}
		}
	}

	Point position = steppers::getPosition();

	float rpm = (float)eeprom::getEeprom8(eeprom::EXTRUDE_RPM, 19) / 10.0;

	//60 * 1000000 = # uS in a minute
	//200 * 8 = 200 steps per revolution * 1/8 stepping
	int32_t interval = (int32_t)(60L * 1000000L) / (int32_t)((float)(200 * 8) * rpm);
	int16_t stepsPerSecond = (int16_t)((200.0 * 8.0 * rpm) / 60.0);

	//50.235479 is ToM stepper extruder speed, we use this as a baseline
	stepsPerSecond = (int16_t)((float)stepsPerSecond * stepsToMM((int32_t)50.235479, AXIS_A));

	if ( seconds == 0 )	steppers::abort();
	else {
		position[3] += seconds * stepsPerSecond;
		steppers::setTarget(position, interval);
	}

	if (overrideTempCheck)	overrideExtrudeSeconds = 0;
}

void ExtruderMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	int16_t zReverse = -1;

	switch (button) {
        	case ButtonArray::OK:
			switch(extrudeSeconds) {
                		case EXTRUDE_SECS_1S:
					extrudeSeconds = EXTRUDE_SECS_2S;
					break;
                		case EXTRUDE_SECS_2S:
					extrudeSeconds = EXTRUDE_SECS_5S;
					break;
                		case EXTRUDE_SECS_5S:
					extrudeSeconds = EXTRUDE_SECS_10S;
					break;
				case EXTRUDE_SECS_10S:
					extrudeSeconds = EXTRUDE_SECS_30S;
					break;
				case EXTRUDE_SECS_30S:
					extrudeSeconds = EXTRUDE_SECS_60S;
					break;
				case EXTRUDE_SECS_60S:
					extrudeSeconds = EXTRUDE_SECS_90S;
					break;
				case EXTRUDE_SECS_90S:
					extrudeSeconds = EXTRUDE_SECS_120S;
					break;
                		case EXTRUDE_SECS_120S:
					extrudeSeconds = EXTRUDE_SECS_240S;
					break;
                		case EXTRUDE_SECS_240S:
					extrudeSeconds = EXTRUDE_SECS_1S;
					break;
				default:
					extrudeSeconds = EXTRUDE_SECS_1S;
					break;
			}

			eeprom_write_byte((uint8_t *)eeprom::EXTRUDE_DURATION, (uint8_t)extrudeSeconds);

			//If we're already extruding, change the time running
			if (steppers::isRunning())
				extrude((seconds_t)(zReverse * lastDirection * extrudeSeconds), false);

			timeChanged = true;
			break;
        	case ButtonArray::YPLUS:
			// Show Extruder RPM Setting Screen
                        interface::pushScreen(&extruderSetRpmScreen);
			break;
        	case ButtonArray::ZERO:
        	case ButtonArray::YMINUS:
        	case ButtonArray::XMINUS:
        	case ButtonArray::XPLUS:
			extrude((seconds_t)EXTRUDE_SECS_CANCEL, true);
        		break;
        	case ButtonArray::ZMINUS:
        	case ButtonArray::ZPLUS:
			if ( button == ButtonArray::ZPLUS )	lastDirection = 1;
			else					lastDirection = -1;
			
			extrude((seconds_t)(zReverse * lastDirection * extrudeSeconds), false);
			break;
       	 	case ButtonArray::CANCEL:
			steppers::abort();
			steppers::enableAxis(3, false);
               		interface::popScreen();
    			steppers::switchToAcceleratedDriver();
			break;
	}
}



ExtruderTooColdMenu::ExtruderTooColdMenu() {
	itemCount = 4;
	reset();
}

void ExtruderTooColdMenu::resetState() {
	itemIndex = 2;
	firstItemIndex = 2;
}

void ExtruderTooColdMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar warning[]  = "Tool0 too cold!";
	const static PROGMEM prog_uchar cancel[]   =  "Cancel";
	const static PROGMEM prog_uchar override[] =  "Override";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(warning);
		break;
	case 1:
		break;
	case 2:
		lcd.writeFromPgmspace(cancel);
		break;
	case 3:
		lcd.writeFromPgmspace(override);
		break;
	}
}

void ExtruderTooColdMenu::handleSelect(uint8_t index) {
	switch (index) {
	case 2:
		// Cancel extrude
		overrideExtrudeSeconds = 0;
		interface::popScreen();
		break;
	case 3:
		// Override and extrude
                interface::popScreen();
		break;
	}
}

void ExtruderSetRpmScreen::reset() {
	rpm = eeprom::getEeprom8(eeprom::EXTRUDE_RPM, 19);
}

void ExtruderSetRpmScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Extruder RPM:";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]    = " ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(0,1);
	lcd.writeFloat((float)rpm / 10.0, 1);
	lcd.writeFromPgmspace(blank);
}

void ExtruderSetRpmScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
		case ButtonArray::CANCEL:
			interface::popScreen();
			break;
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			eeprom_write_byte((uint8_t *)eeprom::EXTRUDE_RPM, rpm);
			interface::popScreen();
			break;
		case ButtonArray::ZPLUS:
			// increment more
			if (rpm <= 250) rpm += 5;
			break;
		case ButtonArray::ZMINUS:
			// decrement more
			if (rpm >= 8) rpm -= 5;
			break;
		case ButtonArray::YPLUS:
			// increment less
			if (rpm <= 254) rpm += 1;
			break;
		case ButtonArray::YMINUS:
			// decrement less
			if (rpm >= 4) rpm -= 1;
			break;
		case ButtonArray::XMINUS:
		case ButtonArray::XPLUS:
			break;
	}
}


void MoodLightMode::reset() {
	updatePhase = 0;
	scriptId = eeprom_read_byte((uint8_t *)eeprom::MOOD_LIGHT_SCRIPT);
}

void MoodLightMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar mood1[] = "Mood: ";
	const static PROGMEM prog_uchar mood3_1[] = "(set RGB)";
	const static PROGMEM prog_uchar msg4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]   = "          ";
	const static PROGMEM prog_uchar moodNotPresent1[] = "Mood Light not";
	const static PROGMEM prog_uchar moodNotPresent2[] = "present!!";
	const static PROGMEM prog_uchar moodNotPresent3[] = "See Thingiverse";
	const static PROGMEM prog_uchar moodNotPresent4[] = "   thing:15347";

	//If we have no mood light, point to thingiverse to make one
	if ( ! interface::moodLightController().blinkM.blinkMIsPresent ) {
		//Try once more to restart the mood light controller
		if ( ! interface::moodLightController().start() ) {
			if ( forceRedraw ) {
				lcd.clear();
				lcd.setCursor(0,0);
				lcd.writeFromPgmspace(moodNotPresent1);
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(moodNotPresent2);
				lcd.setCursor(0,2);
				lcd.writeFromPgmspace(moodNotPresent3);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(moodNotPresent4);
			}
		
			return;
		}
	}

	if (forceRedraw) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(mood1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(msg4);
	}

 	//Redraw tool info

	switch (updatePhase) {
	case 0:
		lcd.setCursor(6, 0);
		lcd.writeFromPgmspace(blank);	
		lcd.setCursor(6, 0);
		lcd.writeFromPgmspace(interface::moodLightController().scriptIdToStr(scriptId));	
		break;

	case 1:
		lcd.setCursor(0, 2);
		if ( scriptId == 1 )	lcd.writeFromPgmspace(mood3_1);
		else			lcd.writeFromPgmspace(blank);	
		break;
	}

	updatePhase++;
	if (updatePhase > 1) {
		updatePhase = 0;
	}
}



void MoodLightMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	if ( ! interface::moodLightController().blinkM.blinkMIsPresent )	interface::popScreen();

	uint8_t i;

	switch (button) {
        	case ButtonArray::OK:
			eeprom_write_byte((uint8_t *)eeprom::MOOD_LIGHT_SCRIPT, scriptId);
               		interface::popScreen();
			break;

        	case ButtonArray::ZERO:
			if ( scriptId == 1 )
			{
				//Set RGB Values
                        	interface::pushScreen(&moodLightSetRGBScreen);
			}
			break;

	        case ButtonArray::ZPLUS:
			// increment more
			for ( i = 0; i < 5; i ++ )
				scriptId = interface::moodLightController().nextScriptId(scriptId);
			interface::moodLightController().playScript(scriptId);
			break;

        	case ButtonArray::ZMINUS:
			// decrement more
			for ( i = 0; i < 5; i ++ )
				scriptId = interface::moodLightController().prevScriptId(scriptId);
			interface::moodLightController().playScript(scriptId);
			break;

        	case ButtonArray::YPLUS:
			// increment less
			scriptId = interface::moodLightController().nextScriptId(scriptId);
			interface::moodLightController().playScript(scriptId);
			break;

        	case ButtonArray::YMINUS:
			// decrement less
			scriptId = interface::moodLightController().prevScriptId(scriptId);
			interface::moodLightController().playScript(scriptId);
			break;

        	case ButtonArray::XMINUS:
        	case ButtonArray::XPLUS:
			break;

       	 	case ButtonArray::CANCEL:
			scriptId = eeprom_read_byte((uint8_t *)eeprom::MOOD_LIGHT_SCRIPT);
			interface::moodLightController().playScript(scriptId);
               		interface::popScreen();
			break;
	}
}


void MoodLightSetRGBScreen::reset() {
	inputMode = 0;	//Red
	redrawScreen = false;

	red   = eeprom::getEeprom8(eeprom::MOOD_LIGHT_CUSTOM_RED,   255);;
	green = eeprom::getEeprom8(eeprom::MOOD_LIGHT_CUSTOM_GREEN, 255);;
	blue  = eeprom::getEeprom8(eeprom::MOOD_LIGHT_CUSTOM_BLUE,  255);;
}

void MoodLightSetRGBScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1_red[]   = "Red:";
	const static PROGMEM prog_uchar message1_green[] = "Green:";
	const static PROGMEM prog_uchar message1_blue[]  = "Blue:";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";

	if ((forceRedraw) || (redrawScreen)) {
		lcd.clear();

		lcd.setCursor(0,0);
		if      ( inputMode == 0 ) lcd.writeFromPgmspace(message1_red);
		else if ( inputMode == 1 ) lcd.writeFromPgmspace(message1_green);
		else if ( inputMode == 2 ) lcd.writeFromPgmspace(message1_blue);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);

		redrawScreen = false;
	}


	// Redraw tool info
	lcd.setCursor(0,1);
	if      ( inputMode == 0 ) lcd.writeInt(red,  3);
	else if ( inputMode == 1 ) lcd.writeInt(green,3);
	else if ( inputMode == 2 ) lcd.writeInt(blue, 3);
}

void MoodLightSetRGBScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	uint8_t *value = &red;

	if 	( inputMode == 1 )	value = &green;
	else if ( inputMode == 2 )	value = &blue;

	switch (button) {
        case ButtonArray::CANCEL:
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
		if ( inputMode < 2 ) {
			inputMode ++;
			redrawScreen = true;
		} else {
			eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_RED,  red);
			eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_GREEN,green);
			eeprom_write_byte((uint8_t*)eeprom::MOOD_LIGHT_CUSTOM_BLUE, blue);

			//Set the color
			interface::moodLightController().playScript(1);

			interface::popScreen();
		}
		break;
        case ButtonArray::ZPLUS:
		// increment more
		if (*value <= 245) *value += 10;
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		if (*value >= 10) *value -= 10;
		break;
        case ButtonArray::YPLUS:
		// increment less
		if (*value <= 254) *value += 1;
		break;
        case ButtonArray::YMINUS:
		// decrement less
		if (*value >= 1) *value -= 1;
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}
}


void SnakeMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar gameOver[] =  "GAME OVER!";

	// If we are dead, restart the game.
	if (!snakeAlive) {
		reset();
		forceRedraw = true;
	}

	if (forceRedraw) {
		lcd.clear();

		for (uint8_t i = 0; i < snakeLength; i++) {
			lcd.setCursor(snakeBody[i].x, snakeBody[i].y);
			lcd.write('O');
		}
	}

	// Always redraw the apple, just in case.
	lcd.setCursor(applePosition.x, applePosition.y);
	lcd.write('*');

	// First, undraw the snake's tail
	lcd.setCursor(snakeBody[snakeLength-1].x, snakeBody[snakeLength-1].y);
	lcd.write(' ');

	// Then, shift the snakes body parts back, deleting the tail
	for(int8_t i = snakeLength-1; i >= 0; i--) {
		snakeBody[i+1] = snakeBody[i];
	}

	// Create a new head for the snake (this causes it to move forward)
	switch(snakeDirection)
	{
	case DIR_EAST:
		snakeBody[0].x = (snakeBody[0].x + 1) % LCD_SCREEN_WIDTH;
		break;
	case DIR_WEST:
		snakeBody[0].x = (snakeBody[0].x +  LCD_SCREEN_WIDTH - 1) % LCD_SCREEN_WIDTH;
		break;
	case DIR_NORTH:
		snakeBody[0].y = (snakeBody[0].y + LCD_SCREEN_HEIGHT - 1) % LCD_SCREEN_HEIGHT;
		break;
	case DIR_SOUTH:
		snakeBody[0].y = (snakeBody[0].y + 1) % LCD_SCREEN_HEIGHT;
		break;
	}

	// Now, draw the snakes new head
	lcd.setCursor(snakeBody[0].x, snakeBody[0].y);
	lcd.write('O');

	// Check if the snake has run into itself
	for (uint8_t i = 1; i < snakeLength; i++) {
		if (snakeBody[i].x == snakeBody[0].x
			&& snakeBody[i].y == snakeBody[0].y) {
			snakeAlive = false;

			lcd.setCursor(1,1);
			lcd.writeFromPgmspace(gameOver);
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

		applePosition.x = rand()%LCD_SCREEN_WIDTH;
		applePosition.y = rand()%LCD_SCREEN_HEIGHT;

		lcd.setCursor(applePosition.x, applePosition.y);
		lcd.write('*');
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

	// Put the apple in an initial position (this could collide with the snake!)
	applePosition.x = rand()%LCD_SCREEN_WIDTH;
	applePosition.y = rand()%LCD_SCREEN_HEIGHT;
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
	updatePhase =  UPDATE_PHASE_FIRST;
	buildTimePhase = BUILD_TIME_PHASE_FIRST;
	lastBuildTimePhase = BUILD_TIME_PHASE_FIRST;
	lastElapsedSeconds = 0.0;
	pausePushLockout = false;
	pauseMode.autoPause = false;
	buildCompleteBuzzPlayed = false;
	overrideForceRedraw = false;
	copiesPrinted = 0;
	timeLeftDisplayed = false;
}


void MonitorMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar extruder_temp[]      =   "Tool: ---/---C";
	const static PROGMEM prog_uchar platform_temp[]      =   "Bed:  ---/---C";
	const static PROGMEM prog_uchar elapsed_time[]       =   "Elapsed:   0h00m";
	const static PROGMEM prog_uchar completed_percent[]  =   "Completed:   0% ";
	const static PROGMEM prog_uchar time_left[]          =   "TimeLeft:  0h00m";
	const static PROGMEM prog_uchar duration[]           =   "Duration:  0h00m";
	const static PROGMEM prog_uchar time_left_secs[]     =   "secs";
	const static PROGMEM prog_uchar time_left_none[]     =   "   none";
	const static PROGMEM prog_uchar zpos[] 		     =   "ZPos:           ";
	const static PROGMEM prog_uchar zpos_mm[] 	     =   "mm";
	const static PROGMEM prog_uchar estimate2[]          =   "Estimating:   0%";
	const static PROGMEM prog_uchar estimate3[]          =   "          (skip)";
	const static PROGMEM prog_uchar filament[]           =   "Filament:0.00m  ";
	const static PROGMEM prog_uchar copies[]	     =   "Copy:           ";
	const static PROGMEM prog_uchar of[]		     =   " of ";
	char buf[17];

	if ( command::isPaused() ) {
		if ( ! pausePushLockout ) {
			pausePushLockout = true;
			pauseMode.autoPause = true;
			interface::pushScreen(&pauseMode);
			return;
		}
	} else pausePushLockout = false;

	if ( host::getHostState() != host::HOST_STATE_ESTIMATING_FROM_SD )
	estimatingBuild = false;

	//Check for a build complete, and if we have more than one copy
	//to print, setup another one
	if (( ! estimatingBuild ) && ( host::isBuildComplete() )) {
		uint8_t copiesToPrint = eeprom::getEeprom8(eeprom::ABP_COPIES, 1);
		if ( copiesToPrint > 1 ) {
			if ( copiesPrinted < (copiesToPrint - 1)) {
				copiesPrinted ++;
				overrideForceRedraw = true;
				command::buildAnotherCopy();
			}
		}
	}

	if ((forceRedraw) || (overrideForceRedraw)) {
		lcd.clear();
		lcd.setCursor(0,0);
		switch(host::getHostState()) {
		case host::HOST_STATE_READY:
			lcd.writeString(host::getMachineName());
			break;
		case host::HOST_STATE_BUILDING:
		case host::HOST_STATE_BUILDING_FROM_SD:
		case host::HOST_STATE_ESTIMATING_FROM_SD:
			lcd.writeString(host::getBuildName());
			lcd.setCursor(0,1);
			if ( estimatingBuild ) {
				lcd.writeFromPgmspace(estimate2);
				lcd.setCursor(0,2);
				lcd.writeFromPgmspace(estimate3);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(duration);
			} else {
				lcd.writeFromPgmspace(completed_percent);
			}
			break;
		case host::HOST_STATE_ERROR:
			lcd.writeString("error!");
			break;
		}

		if ( ! estimatingBuild ) {
			lcd.setCursor(0,2);
			lcd.writeFromPgmspace(extruder_temp);

			lcd.setCursor(0,3);
			lcd.writeFromPgmspace(platform_temp);

			lcd.setCursor(15,3);
			if ( command::getPauseAtZPos() == 0 )	lcd.write(' ');
			else					lcd.write('*');
		}
	}

	overrideForceRedraw = false;

	//Display estimating stats
	if ( estimatingBuild ) {
		//If preheat_during_estimate is set, then preheat
		OutPacket responsePacket;
		if ( eeprom::getEeprom8(eeprom::PREHEAT_DURING_ESTIMATE, 0) ) {
			uint8_t value = eeprom::getEeprom8(eeprom::TOOL0_TEMP, 220);
			extruderControl(SLAVE_CMD_SET_TEMP, EXTDR_CMD_SET, responsePacket, (uint16_t)value);
			value = eeprom::getEeprom8(eeprom::PLATFORM_TEMP, 110);
			extruderControl(SLAVE_CMD_SET_PLATFORM_TEMP, EXTDR_CMD_SET, responsePacket, value);
		}

		//Write out the % estimated
		lcd.setCursor(12,1);
		buf[0] = '\0';
		appendUint8(buf, sizeof(buf), (uint8_t)sdcard::getPercentPlayed());
		strcat(buf, "%");
		lcd.writeString(buf);

		//Write out the time calculated
		buf[0] = '\0';
		lcd.setCursor(9,3);
		appendTime(buf, sizeof(buf), (uint32_t)command::estimateSeconds());
		lcd.writeString(buf);

		//Check for estimate finished, and switch states to building
		if ( host::isBuildComplete() ) {
			//Store the estimate seconds
			buildDuration = command::estimateSeconds();
			host::setHostStateBuildingFromSD();
			command::setEstimation(false);
			overrideForceRedraw = true;
			estimatingBuild = false;
		}
	
		return;
	}

	OutPacket responsePacket;

	// Redraw tool info
	switch (updatePhase) {
	case UPDATE_PHASE_TOOL_TEMP:
		lcd.setCursor(6,2);
		if (extruderControl(SLAVE_CMD_GET_TEMP, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t data = responsePacket.read16(1);
			lcd.writeInt(data,3);
		} else {
			lcd.writeString("XXX");
		}
		break;

	case UPDATE_PHASE_TOOL_TEMP_SET_POINT:
		lcd.setCursor(10,2);
		if (extruderControl(SLAVE_CMD_GET_SP, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t data = responsePacket.read16(1);
			lcd.writeInt(data,3);
		} else {
			lcd.writeString("XXX");
		}
		break;

	case UPDATE_PHASE_PLATFORM_TEMP:
		lcd.setCursor(6,3);
		if (extruderControl(SLAVE_CMD_GET_PLATFORM_TEMP, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t data = responsePacket.read16(1);
			lcd.writeInt(data,3);
		} else {
			lcd.writeString("XXX");
		}
		break;

	case UPDATE_PHASE_PLATFORM_SET_POINT:
		lcd.setCursor(10,3);
		if (extruderControl(SLAVE_CMD_GET_PLATFORM_SP, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t data = responsePacket.read16(1);
			lcd.writeInt(data,3);
		} else {
			lcd.writeString("XXX");
		}

		lcd.setCursor(15,3);
		if ( command::getPauseAtZPos() == 0 )	lcd.write(' ');
		else					lcd.write('*');
		break;
	case UPDATE_PHASE_BUILD_PHASE_SCROLLER:
		enum host::HostState hostState = host::getHostState();
		
		if ( (hostState != host::HOST_STATE_BUILDING ) && ( hostState != host::HOST_STATE_BUILDING_FROM_SD )) break;

		//Signal buzzer if we're complete
		if (( ! buildCompleteBuzzPlayed ) && ( sdcard::getPercentPlayed() >= 100.0 )) {
			buildCompleteBuzzPlayed = true;
       			Motherboard::getBoard().buzz(2, 3, eeprom::getEeprom8(eeprom::BUZZER_REPEATS, 3));
		}

		bool okButtonHeld = interface::isButtonPressed(ButtonArray::OK);

		//Holding the ok button stops rotation
        	if ( okButtonHeld )	buildTimePhase = lastBuildTimePhase;

		float secs;
		int32_t tsecs;
		Point position;
		uint8_t precision;
		float completedPercent;
		float filamentUsed, lastFilamentUsed;

		switch (buildTimePhase) {
			case BUILD_TIME_PHASE_COMPLETED_PERCENT:
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(completed_percent);
				lcd.setCursor(11,1);
				buf[0] = '\0';

				if ( buildDuration == 0 ) completedPercent = sdcard::getPercentPlayed();
				else			  completedPercent = ((float)command::estimateSeconds() / (float)buildDuration) * 100.0;

				appendUint8(buf, sizeof(buf), (uint8_t)completedPercent);
				strcat(buf, "% ");
				lcd.writeString(buf);
				break;
			case BUILD_TIME_PHASE_ELAPSED_TIME:
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(elapsed_time);
				lcd.setCursor(9,1);
				buf[0] = '\0';

				if ( host::isBuildComplete() ) secs = lastElapsedSeconds; //We stop counting elapsed seconds when we are done
				else {
					lastElapsedSeconds = Motherboard::getBoard().getCurrentSeconds();
					secs = lastElapsedSeconds;
				}
				appendTime(buf, sizeof(buf), (uint32_t)secs);
				lcd.writeString(buf);
				break;
			case BUILD_TIME_PHASE_TIME_LEFT:
				lcd.setCursor(0,1);
				if (( timeLeftDisplayed ) || ( command::getFilamentLength() >= 1 )) {
					lcd.writeFromPgmspace(time_left);
					timeLeftDisplayed = true;
				}
				else					 lcd.writeFromPgmspace(duration);
				lcd.setCursor(9,1);

				tsecs = buildDuration - command::estimateSeconds();
				
				buf[0] = '\0';
				if 	  ((tsecs > 0 ) && (tsecs < 60) && ( host::isBuildComplete() ) ) {
					appendUint8(buf, sizeof(buf), (uint8_t)tsecs);
					lcd.writeString(buf);
					lcd.writeFromPgmspace(time_left_secs);	
				} else if (( tsecs <= 0) || ( host::isBuildComplete()) ) {
					command::addFilamentUsed();
					lcd.writeFromPgmspace(time_left_none);
				} else {
					appendTime(buf, sizeof(buf), (uint32_t)tsecs);
					lcd.writeString(buf);
				}
				break;
			case BUILD_TIME_PHASE_ZPOS:
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(zpos);
				lcd.setCursor(6,1);

				position = steppers::getPosition();
			
				//Divide by the axis steps to mm's
				lcd.writeFloat(stepsToMM(position[2], AXIS_Z), 3);

				lcd.writeFromPgmspace(zpos_mm);
				break;
			case BUILD_TIME_PHASE_FILAMENT:
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(filament);
				lcd.setCursor(9,1);
				lastFilamentUsed = stepsToMM(command::getLastFilamentLength(), AXIS_A);
				if ( lastFilamentUsed != 0.0 )	filamentUsed = lastFilamentUsed;
				else				filamentUsed = stepsToMM(command::getFilamentLength(), AXIS_A);
				filamentUsed /= 1000.0;	//convert to meters
				if	( filamentUsed < 0.1 )	{
					 filamentUsed *= 1000.0;	//Back to mm's
					precision = 1;
				}
				else if ( filamentUsed < 10.0 )	 precision = 4;
				else if ( filamentUsed < 100.0 ) precision = 3;
				else				 precision = 2;
				lcd.writeFloat(filamentUsed, precision);
				if ( precision == 1 ) lcd.write('m');
				lcd.write('m');
				break;
			case BUILD_TIME_PHASE_COPIES_PRINTED:
				uint8_t totalCopies = eeprom::getEeprom8(eeprom::ABP_COPIES, 1);
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(copies);
				lcd.setCursor(7,1);
				lcd.writeFloat((float)(copiesPrinted + 1), 0);
				lcd.writeFromPgmspace(of);
				lcd.writeFloat((float)totalCopies, 0);
				break;
		}

        	if ( ! okButtonHeld ) {
			//Advance buildTimePhase and wrap around
			lastBuildTimePhase = buildTimePhase;
			buildTimePhase = (enum BuildTimePhase)((uint8_t)buildTimePhase + 1);

			//Skip Time left if we skipped the estimation
			if (( buildTimePhase == BUILD_TIME_PHASE_TIME_LEFT ) && ( buildDuration == 0 ))
				buildTimePhase = (enum BuildTimePhase)((uint8_t)buildTimePhase + 1);

			//If we're setup to print more than one copy, then show that build phase,
			//otherwise skip it
			if ( buildTimePhase == BUILD_TIME_PHASE_COPIES_PRINTED ) {
				uint8_t totalCopies = eeprom::getEeprom8(eeprom::ABP_COPIES, 1);
				if ( totalCopies <= 1 )	
					buildTimePhase = (enum BuildTimePhase)((uint8_t)buildTimePhase + 1);
			}

			if ( buildTimePhase >= BUILD_TIME_PHASE_LAST )
				buildTimePhase = BUILD_TIME_PHASE_FIRST;
		}
		break;
	}

	//Advance updatePhase and wrap around
	updatePhase = (enum UpdatePhase)((uint8_t)updatePhase + 1);
	if (updatePhase >= UPDATE_PHASE_LAST)
		updatePhase = UPDATE_PHASE_FIRST;

	steppers::doLcd();
}

void MonitorMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
		switch(host::getHostState()) {
		case host::HOST_STATE_BUILDING:
		case host::HOST_STATE_BUILDING_FROM_SD:
		case host::HOST_STATE_ESTIMATING_FROM_SD:
                        interface::pushScreen(&cancelBuildMenu);
			break;
		default:
                        interface::popScreen();
			break;
		}
        case ButtonArray::ZPLUS:
		if ( host::getHostState() == host::HOST_STATE_BUILDING_FROM_SD )
			updatePhase = UPDATE_PHASE_BUILD_PHASE_SCROLLER;
		break;
	case ButtonArray::OK:
		//Skip build if user hit the button during the estimation phase
		if (( host::getHostState() == host::HOST_STATE_ESTIMATING_FROM_SD ) && ( estimatingBuild )) {
			buildDuration = 0;
			host::setHostStateBuildingFromSD();
			command::setEstimation(false);
			overrideForceRedraw = true;
			estimatingBuild = false;
		}
		break;
	}
}



void VersionMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar version1[] = "Motherboard: _._";
	const static PROGMEM prog_uchar version2[] = "   Extruder: _._";
	const static PROGMEM prog_uchar version4[] = "FreeSram: ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(version1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(version2);

		//Display the motherboard version
		lcd.setCursor(13, 0);
		lcd.writeInt(firmware_version / 100, 1);

		lcd.setCursor(15, 0);
		lcd.writeInt(firmware_version % 100, 1);

		//Display the extruder version
		OutPacket responsePacket;

		if (extruderControl(SLAVE_CMD_VERSION, EXTDR_CMD_GET, responsePacket, 0)) {
			uint16_t extruderVersion = responsePacket.read16(1);

			lcd.setCursor(13, 1);
			lcd.writeInt(extruderVersion / 100, 1);

			lcd.setCursor(15, 1);
			lcd.writeInt(extruderVersion % 100, 1);
		} else {
			lcd.setCursor(13, 1);
			lcd.writeString("X.X");
		}

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(version4);
		lcd.writeFloat((float)StackCount(),0);
	} else {
	}
}

void VersionMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	interface::popScreen();
}

void Menu::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar blankLine[] =  "                ";

	// Do we need to redraw the whole menu?
	if ((itemIndex/LCD_SCREEN_HEIGHT) != (lastDrawIndex/LCD_SCREEN_HEIGHT)
			|| forceRedraw ) {
		// Redraw the whole menu
		lcd.clear();

		for (uint8_t i = 0; i < LCD_SCREEN_HEIGHT; i++) {
			// Instead of using lcd.clear(), clear one line at a time so there
			// is less screen flickr.

			if (i+(itemIndex/LCD_SCREEN_HEIGHT)*LCD_SCREEN_HEIGHT +1 > itemCount) {
				break;
			}

			lcd.setCursor(1,i);
			// Draw one page of items at a time
			drawItem(i+(itemIndex/LCD_SCREEN_HEIGHT)*LCD_SCREEN_HEIGHT, lcd);
		}
	}
	else {
		// Only need to clear the previous cursor
		lcd.setCursor(0,(lastDrawIndex%LCD_SCREEN_HEIGHT));
		lcd.write(' ');
	}

	lcd.setCursor(0,(itemIndex%LCD_SCREEN_HEIGHT));
	lcd.write('>');
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
	uint8_t steps = MAX_ITEMS_PER_SCREEN;
	switch (button) {
        case ButtonArray::ZERO:
        case ButtonArray::OK:
		handleSelect(itemIndex);
		break;
        case ButtonArray::CANCEL:
		handleCancel();
		break;
        case ButtonArray::YMINUS:
		steps = 1;
        case ButtonArray::ZMINUS:
		// increment index
		if      (itemIndex < itemCount - steps) 
			itemIndex+=steps;
		else if (itemIndex==itemCount-1)
			itemIndex=firstItemIndex;
		else	itemIndex=itemCount-1;
		break;
        case ButtonArray::YPLUS:
		steps = 1;
        case ButtonArray::ZPLUS:
		// decrement index
		if      (itemIndex-steps > firstItemIndex)
			itemIndex-=steps;
		else if (itemIndex==firstItemIndex)
			itemIndex=itemCount - 1;
		else	itemIndex=firstItemIndex;
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}
}


CancelBuildMenu::CancelBuildMenu() {
	pauseMode.autoPause = false;
	itemCount = 5;
	reset();
	pauseDisabled = false;
	if ( ( estimatingBuild ) || ( steppers::isHoming() ) ||
	     (sdcard::getPercentPlayed() >= 100.0))	pauseDisabled = true;

	if (( ! estimatingBuild ) && ( host::isBuildComplete() ))
		printAnotherEnabled = true;
	else	printAnotherEnabled = false;

}

void CancelBuildMenu::resetState() {
	pauseMode.autoPause = false;
	pauseDisabled = false;	
	if ( ( estimatingBuild ) || ( steppers::isHoming() ) ||
	     (sdcard::getPercentPlayed() >= 100.0))	pauseDisabled = true;

	if (( ! estimatingBuild ) && ( host::isBuildComplete() ))
		printAnotherEnabled = true;
	else	printAnotherEnabled = false;

	if ( pauseDisabled )	{
		itemIndex = 2;
		itemCount = 4;
	} else {
		itemIndex = 1;
		itemCount = 5;
	}

	if ( printAnotherEnabled ) {
		itemIndex = 1;
	}

	firstItemIndex = itemIndex;
}

void CancelBuildMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar choose[]	= "Please Choose:";
	const static PROGMEM prog_uchar abort[]		= "Abort Print   ";
	const static PROGMEM prog_uchar printAnother[]	= "Print Another";
	const static PROGMEM prog_uchar pauseZ[]	= "Pause at ZPos ";
	const static PROGMEM prog_uchar pause[]		= "Pause         ";
	const static PROGMEM prog_uchar back[]		= "Continue Build";

	if ( ( estimatingBuild ) || ( steppers::isHoming() ) ||
	     (sdcard::getPercentPlayed() >= 100.0))	pauseDisabled = true;

	//Implement variable length menu
	uint8_t lind = 0;

	if ( index == lind )	lcd.writeFromPgmspace(choose);
	lind ++;

	if (( pauseDisabled ) && ( ! printAnotherEnabled )) lind ++;

	if ( index == lind)	lcd.writeFromPgmspace(abort);
	lind ++;

	if ( printAnotherEnabled ) {
		if ( index == lind ) lcd.writeFromPgmspace(printAnother);
		lind ++;
	}

	if ( ! pauseDisabled ) {
		if ( index == lind )	lcd.writeFromPgmspace(pauseZ);
		lind ++;
	}

	if ( ! pauseDisabled ) {
		if ( index == lind )	lcd.writeFromPgmspace(pause);
		lind ++;
	}

	if ( index == lind )	lcd.writeFromPgmspace(back);
	lind ++;
}

void CancelBuildMenu::handleSelect(uint8_t index) {
	int32_t interval = 2000;

	//Implement variable length menu
	uint8_t lind = 0;

	if (( pauseDisabled ) && ( ! printAnotherEnabled )) lind ++;

	lind ++;

	if ( index == lind) {
		command::addFilamentUsed();
		// Cancel build, returning to whatever menu came before monitor mode.
		// TODO: Cancel build.
		interface::popScreen();
		host::stopBuild();
	}
	lind ++;

	if ( printAnotherEnabled ) {
		if ( index == lind ) {
			command::buildAnotherCopy();
			interface::popScreen();
		}
		lind ++;
	}

	if ( ! pauseDisabled ) {
		if ( index == lind )	interface::pushScreen(&pauseAtZPosScreen);
		lind ++;
	}

	if ( ! pauseDisabled ) {
		if ( index == lind ) {
			command::pause(true);
			pauseMode.autoPause = false;
			interface::pushScreen(&pauseMode);
		}
		lind ++;
	}

	if ( index == lind ) {
		// Don't cancel print, just close dialog.
                interface::popScreen();
	}
	lind ++;
}

MainMenu::MainMenu() {
	itemCount = 21;
	reset();

	//Read in the axisStepsPerMM, we'll need these for various firmware functions later on
        cli();
        axisStepsPerMM[AXIS_X] = eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_X, STEPS_PER_MM_X_DEFAULT);
        axisStepsPerMM[AXIS_Y] = eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_Y, STEPS_PER_MM_Y_DEFAULT);
        axisStepsPerMM[AXIS_Z] = eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_Z, STEPS_PER_MM_Z_DEFAULT);
        axisStepsPerMM[AXIS_A] = eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_A, STEPS_PER_MM_A_DEFAULT);
        axisStepsPerMM[AXIS_B] = eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_B, STEPS_PER_MM_B_DEFAULT);
        sei();
}

void MainMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar monitor[]	= "Monitor";
	const static PROGMEM prog_uchar build[]		= "Build from SD";
	const static PROGMEM prog_uchar jog[]		= "Jog";
	const static PROGMEM prog_uchar preheat[]	= "Preheat";
	const static PROGMEM prog_uchar extruder[]	= "Extrude";
	const static PROGMEM prog_uchar homeAxis[]	= "Home Axis";
	const static PROGMEM prog_uchar advanceABP[]	= "Advance ABP";
	const static PROGMEM prog_uchar steppersS[]	= "Steppers";
	const static PROGMEM prog_uchar moodlight[]	= "Mood Light";
	const static PROGMEM prog_uchar buzzer[]	= "Buzzer";
	const static PROGMEM prog_uchar buildSettings[]	= "Build Settings";
	const static PROGMEM prog_uchar profiles[]	= "Profiles";
	const static PROGMEM prog_uchar extruderFan[]	= "Extruder Fan";
	const static PROGMEM prog_uchar calibrate[]	= "Calibrate";
	const static PROGMEM prog_uchar homeOffsets[]	= "Home Offsets";
	const static PROGMEM prog_uchar filamentUsed[]	= "Filament Used";
	const static PROGMEM prog_uchar currentPosition[]= "Position";
	const static PROGMEM prog_uchar endStops[]	= "Test End Stops";
	const static PROGMEM prog_uchar stepsPerMm[]	= "Axis Steps:mm";
	const static PROGMEM prog_uchar versions[]	= "Version";
	const static PROGMEM prog_uchar snake[]		= "Snake Game";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(monitor);
		break;
	case 1:
		lcd.writeFromPgmspace(build);
		break;
	case 2:
		lcd.writeFromPgmspace(jog);
		break;
	case 3:
		lcd.writeFromPgmspace(preheat);
		break;
	case 4:
		lcd.writeFromPgmspace(extruder);
		break;
	case 5:
		lcd.writeFromPgmspace(homeAxis);
		break;
	case 6:
		lcd.writeFromPgmspace(advanceABP);
		break;
	case 7:
		lcd.writeFromPgmspace(steppersS);
		break;
	case 8:
		lcd.writeFromPgmspace(moodlight);
		break;
	case 9:
		lcd.writeFromPgmspace(buzzer);
		break;
	case 10:
		lcd.writeFromPgmspace(buildSettings);
		break;
	case 11:
		lcd.writeFromPgmspace(profiles);
		break;
	case 12:
		lcd.writeFromPgmspace(extruderFan);
		break;
	case 13:
		lcd.writeFromPgmspace(calibrate);
		break;
	case 14:
		lcd.writeFromPgmspace(homeOffsets);
		break;
	case 15:
		lcd.writeFromPgmspace(filamentUsed);
		break;
	case 16:
		lcd.writeFromPgmspace(currentPosition);
		break;
	case 17:
		lcd.writeFromPgmspace(endStops);
		break;
	case 18:
		lcd.writeFromPgmspace(stepsPerMm);
		break;
	case 19:
		lcd.writeFromPgmspace(versions);
		break;
	case 20:
		lcd.writeFromPgmspace(snake);
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
		case 3:
			// Show preheat menu
			interface::pushScreen(&preheatMenu);
			preheatMenu.fetchTargetTemps();
			break;
		case 4:
			// Show extruder menu
			interface::pushScreen(&extruderMenu);
			break;
		case 5:
			// Show home axis
			interface::pushScreen(&homeAxisMode);
			break;
		case 6:
			// Show advance ABP
			interface::pushScreen(&advanceABPMode);
			break;
		case 7:
			// Show steppers menu
			interface::pushScreen(&steppersMenu);
			break;
		case 8:
			// Show Mood Light Mode
                        interface::pushScreen(&moodLightMode);
			break;
		case 9: 
			// Show Buzzer Mode
			interface::pushScreen(&buzzerSetRepeats);
			break;
		case 10: 
			// Show Build Settings Mode
			interface::pushScreen(&buildSettingsMenu);
			break;
		case 11: 
			// Show Profiles Menu
			interface::pushScreen(&profilesMenu);
			break;
		case 12: 
			// Show Extruder Fan Mode
			interface::pushScreen(&extruderFanMenu);
			break;
		case 13:
			// Show Calibrate Mode
                        interface::pushScreen(&calibrateMode);
			break;
		case 14:
			// Show Home Offsets Mode
                        interface::pushScreen(&homeOffsetsMode);
			break;
		case 15:
			// Show Filament Used Mode
                        interface::pushScreen(&filamentUsedMode);
			break;
		case 16:
			// Show Current Position Mode
                        interface::pushScreen(&currentPositionMode);
			break;
		case 17:
			// Show test end stops menu
			interface::pushScreen(&testEndStopsMode);
			break;
		case 18:
			// Show steps per mm menu
			interface::pushScreen(&stepsPerMMMode);
			break;
		case 19:
			// Show build from SD screen
                        interface::pushScreen(&versionMode);
			break;
		case 20:
			// Show build from SD screen
                        interface::pushScreen(&snake);
			break;
		}
}

SDMenu::SDMenu() {
	reset();
	updatePhase = 0;
	drawItemLockout = false;
}

void SDMenu::resetState() {
	itemCount = countFiles();
	updatePhase = 0;
	lastItemIndex = 0;
	drawItemLockout = false;
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

void SDMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	if (index > itemCount - 1) {
		// TODO: report error
		return;
	}

	const uint8_t MAX_FILE_LEN = host::MAX_FILE_LEN;
	char fnbuf[MAX_FILE_LEN];

        if ( !getFilename(index, fnbuf, MAX_FILE_LEN) ) {
                // TODO: report error
		return;
	}

	//Figure out length of filename
	uint8_t filenameLength;
	for (filenameLength = 0; (filenameLength < MAX_FILE_LEN) && (fnbuf[filenameLength] != 0); filenameLength++) ;

	uint8_t idx;
	uint8_t longFilenameOffset = 0;
	uint8_t displayWidth = LCD_SCREEN_WIDTH - 1;

	//Support scrolling filenames that are longer than the lcd screen
	if (filenameLength >= displayWidth) longFilenameOffset = updatePhase % (filenameLength - displayWidth + 1);

	for (idx = 0; (idx < displayWidth) && (fnbuf[longFilenameOffset + idx] != 0) &&
		      ((longFilenameOffset + idx) < MAX_FILE_LEN); idx++)
		lcd.write(fnbuf[longFilenameOffset + idx]);

	//Clear out the rest of the line
	while ( idx < displayWidth ) {
		lcd.write(' ');
		idx ++;
	}
}

void SDMenu::update(LiquidCrystal& lcd, bool forceRedraw) {
	
	if (( ! forceRedraw ) && ( ! drawItemLockout )) {
		//Redraw the last item if we have changed
		if (((itemIndex/LCD_SCREEN_HEIGHT) == (lastDrawIndex/LCD_SCREEN_HEIGHT)) &&
		     ( itemIndex != lastItemIndex ))  {
			lcd.setCursor(1,lastItemIndex % LCD_SCREEN_HEIGHT);
			drawItem(lastItemIndex, lcd);
		}
		lastItemIndex = itemIndex;

		lcd.setCursor(1,itemIndex % LCD_SCREEN_HEIGHT);
		drawItem(itemIndex, lcd);
	}

	Menu::update(lcd, forceRedraw);

	updatePhase ++;
}

void SDMenu::notifyButtonPressed(ButtonArray::ButtonName button) {
	updatePhase = 0;
	Menu::notifyButtonPressed(button);
}

void SDMenu::handleSelect(uint8_t index) {
	if (host::getHostState() != host::HOST_STATE_READY) {
		// TODO: report error
		return;
	}

	drawItemLockout = true;

	char* buildName = host::getBuildName();

        if ( !getFilename(index, buildName, host::MAX_FILE_LEN) ) {
		// TODO: report error
		return;
	}

	estimatingBuild = true;
	command::setEstimation(true);
        sdcard::SdErrorCode e;
	e = host::startBuildFromSD(true);
	if (e != sdcard::SD_SUCCESS) {
		// TODO: report error
		interface::pushScreen(&unableToOpenFileMenu);
		return;
	}
}


void Tool0TempSetScreen::reset() {
	value = eeprom::getEeprom8(eeprom::TOOL0_TEMP, 220);;
}

void Tool0TempSetScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Tool0 Targ Temp:";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}


	// Redraw tool info
	lcd.setCursor(0,1);
	lcd.writeInt(value,3);
}

void Tool0TempSetScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
		eeprom_write_byte((uint8_t*)eeprom::TOOL0_TEMP,value);
		interface::popScreen();
		break;
        case ButtonArray::ZPLUS:
		// increment more
		if (value <= 250) {
			value += 5;
		}
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		if (value >= 5) {
			value -= 5;
		}
		break;
        case ButtonArray::YPLUS:
		// increment less
		if (value <= 254) {
			value += 1;
		}
		break;
        case ButtonArray::YMINUS:
		// decrement less
		if (value >= 1) {
			value -= 1;
		}
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}
}


void PlatformTempSetScreen::reset() {
	value = eeprom::getEeprom8(eeprom::PLATFORM_TEMP, 110);;
}

void PlatformTempSetScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Bed Target Temp:";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}


	// Redraw tool info
	lcd.setCursor(0,1);
	lcd.writeInt(value,3);
}

void PlatformTempSetScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
		eeprom_write_byte((uint8_t*)eeprom::PLATFORM_TEMP,value);
		interface::popScreen();
		break;
        case ButtonArray::ZPLUS:
		// increment more
		if (value <= 250) {
			value += 5;
		}
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		if (value >= 5) {
			value -= 5;
		}
		break;
        case ButtonArray::YPLUS:
		// increment less
		if (value <= 254) {
			value += 1;
		}
		break;
        case ButtonArray::YMINUS:
		// decrement less
		if (value >= 1) {
			value -= 1;
		}
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}
}


PreheatMenu::PreheatMenu() {
	itemCount = 4;
	reset();
}

void PreheatMenu::fetchTargetTemps() {
	OutPacket responsePacket;
	if (extruderControl(SLAVE_CMD_GET_SP, EXTDR_CMD_GET, responsePacket, 0)) {
		tool0Temp = responsePacket.read16(1);
	}
	if (extruderControl(SLAVE_CMD_GET_PLATFORM_SP, EXTDR_CMD_GET, responsePacket, 0)) {
		platformTemp = responsePacket.read16(1);
	}
}

void PreheatMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar heat[]     = "Heat ";
	const static PROGMEM prog_uchar cool[]     = "Cool ";
	const static PROGMEM prog_uchar tool0[]    = "Tool0";
	const static PROGMEM prog_uchar platform[] = "Bed";
	const static PROGMEM prog_uchar tool0set[] = "Set Tool0 Temp";
	const static PROGMEM prog_uchar platset[]  = "Set Bed Temp";

	switch (index) {
	case 0:
		fetchTargetTemps();
		if (tool0Temp > 0) {
			lcd.writeFromPgmspace(cool);
		} else {
			lcd.writeFromPgmspace(heat);
		}
		lcd.writeFromPgmspace(tool0);
		break;
	case 1:
		if (platformTemp > 0) {
			lcd.writeFromPgmspace(cool);
		} else {
			lcd.writeFromPgmspace(heat);
		}
		lcd.writeFromPgmspace(platform);
		break;
	case 2:
		lcd.writeFromPgmspace(tool0set);
		break;
	case 3:
		lcd.writeFromPgmspace(platset);
		break;
	}
}

void PreheatMenu::handleSelect(uint8_t index) {
	OutPacket responsePacket;
	switch (index) {
		case 0:
			// Toggle Extruder heater on/off
			if (tool0Temp > 0) {
				extruderControl(SLAVE_CMD_SET_TEMP, EXTDR_CMD_SET, responsePacket, 0);
			} else {
				uint8_t value = eeprom::getEeprom8(eeprom::TOOL0_TEMP, 220);
				extruderControl(SLAVE_CMD_SET_TEMP, EXTDR_CMD_SET, responsePacket, (uint16_t)value);
			}
			fetchTargetTemps();
			lastDrawIndex = 255; // forces redraw.
			break;
		case 1:
			// Toggle Platform heater on/off
			if (platformTemp > 0) {
				extruderControl(SLAVE_CMD_SET_PLATFORM_TEMP, EXTDR_CMD_SET, responsePacket, 0);
			} else {
				uint8_t value = eeprom::getEeprom8(eeprom::PLATFORM_TEMP, 110);
				extruderControl(SLAVE_CMD_SET_PLATFORM_TEMP, EXTDR_CMD_SET, responsePacket, value);
			}
			fetchTargetTemps();
			lastDrawIndex = 255; // forces redraw.
			break;
		case 2:
			// Show Extruder Temperature Setting Screen
                        interface::pushScreen(&tool0TempSetScreen);
			break;
		case 3:
			// Show Platform Temperature Setting Screen
                        interface::pushScreen(&platTempSetScreen);
			break;
		}
}

void HomeAxisMode::reset() {
}

void HomeAxisMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar home1[] = "Home Axis: ";
	const static PROGMEM prog_uchar home2[] = "  Y            Z";
	const static PROGMEM prog_uchar home3[] = "X   X           ";
	const static PROGMEM prog_uchar home4[] = "  Y            Z";

	if (forceRedraw) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(home1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(home2);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(home3);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(home4);
	}
}

void HomeAxisMode::home(ButtonArray::ButtonName direction) {
	uint8_t axis = 0;
	bool 	maximums;
	float	interval = 2000.0;

	switch(direction) {
	        case ButtonArray::XMINUS:
      		case ButtonArray::XPLUS:
			axis 	 = 0x01;
			maximums = false;
			interval *= stepsToMM((int32_t)47.06, AXIS_X); //Use ToM as baseline
			break;
        	case ButtonArray::YMINUS:
        	case ButtonArray::YPLUS:
			axis 	 = 0x02;
			maximums = false;
			interval *= stepsToMM((int32_t)47.06, AXIS_Y); //Use ToM as baseline
			break;
        	case ButtonArray::ZMINUS:
        	case ButtonArray::ZPLUS:
			axis 	 = 0x04;
			maximums = true;
			interval /= 4.0;	//Speed up Z
			interval *= stepsToMM((int32_t)200.0, AXIS_Z); //Use ToM as baseline
			break;
	}

	steppers::startHoming(maximums, axis, (uint32_t)interval);
}

void HomeAxisMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        	case ButtonArray::YMINUS:
        	case ButtonArray::ZMINUS:
        	case ButtonArray::YPLUS:
        	case ButtonArray::ZPLUS:
        	case ButtonArray::XMINUS:
        	case ButtonArray::XPLUS:
			home(button);
			break;
        	case ButtonArray::ZERO:
        	case ButtonArray::OK:
        	case ButtonArray::CANCEL:
			steppers::abort();
			steppers::enableAxis(0, false);
			steppers::enableAxis(1, false);
			steppers::enableAxis(2, false);
               		interface::popScreen();
			break;
	}
}

SteppersMenu::SteppersMenu() {
	itemCount = 4;
	reset();
}

void SteppersMenu::resetState() {
	if (( steppers::isEnabledAxis(0) ) ||
	    ( steppers::isEnabledAxis(1) ) ||
	    ( steppers::isEnabledAxis(2) ) ||
	    ( steppers::isEnabledAxis(3) ))	itemIndex = 3;
	else					itemIndex = 2;
	firstItemIndex = 2;
}

void SteppersMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar title[]  = "Stepper Motors:";
	const static PROGMEM prog_uchar disable[]   =  "Disable";
	const static PROGMEM prog_uchar enable[] =  "Enable";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(title);
		break;
	case 1:
		break;
	case 2:
		lcd.writeFromPgmspace(disable);
		break;
	case 3:
		lcd.writeFromPgmspace(enable);
		break;
	}
}

void SteppersMenu::handleSelect(uint8_t index) {
	switch (index) {
		case 2:
			//Disable Steppers
			steppers::enableAxis(0, false);
			steppers::enableAxis(1, false);
			steppers::enableAxis(2, false);
			steppers::enableAxis(3, false);
			interface::popScreen();
			break;
		case 3:
			//Enable Steppers
			steppers::enableAxis(0, true);
			steppers::enableAxis(1, true);
			steppers::enableAxis(2, true);
			steppers::enableAxis(3, true);
                	interface::popScreen();
			break;
	}
}

void TestEndStopsMode::reset() {
}

void TestEndStopsMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar test1[] = "Test End Stops: ";
	const static PROGMEM prog_uchar test2[] = "(press end stop)";
	const static PROGMEM prog_uchar test3[] = "XMin:N    YMin:N";
	const static PROGMEM prog_uchar test4[] = "ZMax:N";
	const static PROGMEM prog_uchar strY[]  = "Y";
	const static PROGMEM prog_uchar strN[]  = "N";

	if (forceRedraw) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(test1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(test2);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(test3);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(test4);
	}

	lcd.setCursor(5, 2);
	if ( steppers::isAtMinimum(0) ) lcd.writeFromPgmspace(strY);
	else				lcd.writeFromPgmspace(strN);

	lcd.setCursor(15, 2);
	if ( steppers::isAtMinimum(1) ) lcd.writeFromPgmspace(strY);
	else				lcd.writeFromPgmspace(strN);

	lcd.setCursor(5, 3);
	if ( steppers::isAtMaximum(2) ) lcd.writeFromPgmspace(strY);
	else				lcd.writeFromPgmspace(strN);
}

void TestEndStopsMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        	case ButtonArray::YMINUS:
        	case ButtonArray::ZMINUS:
        	case ButtonArray::YPLUS:
        	case ButtonArray::ZPLUS:
        	case ButtonArray::XMINUS:
        	case ButtonArray::XPLUS:
        	case ButtonArray::ZERO:
        	case ButtonArray::OK:
        	case ButtonArray::CANCEL:
               		interface::popScreen();
			break;
	}
}

void PauseMode::reset() {
	pauseState = 0;
	lastDirectionButtonPressed = (ButtonArray::ButtonName)0;
}

void PauseMode::jog(ButtonArray::ButtonName direction) {
	bool extrude = false;
	int32_t interval = 1000;
	float	speed = 1.5;	//In mm's
	Point position = steppers::getPosition();

	switch(direction) {
       		case ButtonArray::XMINUS:
			position[0] -= mmToSteps(speed, AXIS_X);
			break;
        	case ButtonArray::XPLUS:
			position[0] += mmToSteps(speed, AXIS_X);
			break;
        	case ButtonArray::YMINUS:
			position[1] -= mmToSteps(speed, AXIS_Y);
			break;
       	 	case ButtonArray::YPLUS:
			position[1] += mmToSteps(speed, AXIS_Y);
			break;
        	case ButtonArray::ZMINUS:
			position[2] -= mmToSteps(speed, AXIS_Z);
			break;
       		case ButtonArray::ZPLUS:
			position[2] += mmToSteps(speed, AXIS_Z);
			break;
		case ButtonArray::OK:
		case ButtonArray::ZERO:
			float rpm = (float)eeprom::getEeprom8(eeprom::EXTRUDE_RPM, 19) / 10.0;

			//60 * 1000000 = # uS in a minute
			//200 * 8 = 200 steps per revolution * 1/8 stepping
			interval = (int32_t)(60L * 1000000L) / (int32_t)((float)(200 * 8) * rpm);
			int16_t stepsPerSecond = (int16_t)((200.0 * 8.0 * rpm) / 60.0);

			//50.235479 is ToM stepper extruder speed, we
			//use this as a baseline
			stepsPerSecond = (int16_t)((float)stepsPerSecond * stepsToMM((int32_t)50.235479, AXIS_A));

			//Handle reverse
			if ( direction == ButtonArray::OK )	stepsPerSecond *= -1;

			//Extrude for 0.5 seconds
			position[3] += 0.5 * stepsPerSecond;
			break;
	}

	lastDirectionButtonPressed = direction;

	steppers::setTarget(position, interval);
}


void PauseMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar waitForCurrentCommand[] = "Entering pause..";
	const static PROGMEM prog_uchar retractFilament[]	= "Retract Filament";
	const static PROGMEM prog_uchar movingZ[] 		= "Moving Z up 2mm ";
	const static PROGMEM prog_uchar movingY[]		= "Moving Y 2mm    ";
	const static PROGMEM prog_uchar leavingPaused[]		= "Leaving pause.. ";
	const static PROGMEM prog_uchar paused1[] 		= "Paused(";
	const static PROGMEM prog_uchar paused2[] 		= "   Y+         Z+";
	const static PROGMEM prog_uchar paused3[] 		= "X- Rev X+  (Fwd)";
	const static PROGMEM prog_uchar paused4[] 		= "   Y-         Z-";

	int32_t interval = 1000;
	Point newPosition = pausedPosition;

	if (forceRedraw)	lcd.clear();

	lcd.setCursor(0,0);

	switch (pauseState) {
		case 0:	//Entered pause, waiting for steppers to finish last command
			lcd.writeFromPgmspace(waitForCurrentCommand);

			steppers::switchToRegularDriver();

			if ( ! steppers::isRunning()) pauseState ++;
			break;

		case 1:	//Last command finished, record current position and
			//retract filament
			lcd.writeFromPgmspace(retractFilament);

			pausedPosition = steppers::getPosition();
			newPosition = pausedPosition;
			newPosition[3] += mmToSteps(1.0, AXIS_A);	//Retract the filament so we don't get blobs
			steppers::setTarget(newPosition, interval / 2);
			
			pauseState ++;
			break;

		case 2: //Wait for the retract to complete
			lcd.writeFromPgmspace(retractFilament);
			if ( ! steppers::isRunning()) {
				pauseState ++;
			}
			break;

		case 3: //Last command finished, record position and move Y to dislodge filament
			lcd.writeFromPgmspace(movingY);
			pausedPosition = steppers::getPosition();
			newPosition = pausedPosition;
			newPosition[1] += mmToSteps(4.0, AXIS_Y);
			steppers::setTarget(newPosition, interval / 2);

			pauseState ++;
			break;
		
		case 4: //Wait for the Y move to complete
			if ( ! steppers::isRunning()) {
				pauseState ++;
			}
			break;

		case 5: //Last command finished, move Z away from build
			lcd.writeFromPgmspace(movingZ);

			newPosition = steppers::getPosition();
			newPosition[2] += mmToSteps(2.0, AXIS_Z);
			steppers::setTarget(newPosition, interval / 2);
			
			pauseState ++;
			break;

		case 6: //Wait for the Z move up to complete
			lcd.writeFromPgmspace(movingZ);
			if ( ! steppers::isRunning()) {
				pauseState ++;

				//We write this here to avoid tieing up the processor
				//in the next state
				lcd.clear();

				lcd.writeFromPgmspace(paused1);
				lcd.writeFloat(stepsToMM(pausedPosition[2], AXIS_Z), 3);
				lcd.writeString("):");

				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(paused2);
				lcd.setCursor(0,2);
				lcd.writeFromPgmspace(paused3);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(paused4);
			}
			break;
	
		case 7: //Buzz if we're a Pause@ZPos
			if ( autoPause ) Motherboard::getBoard().buzz(4, 3, eeprom::getEeprom8(eeprom::BUZZER_REPEATS, 3));
			pauseState ++;
			break;
		
		case 8: //We're now paused
			break;

		case 9: //Leaving paused, wait for any steppers to finish
			if ( autoPause ) command::pauseAtZPos(0);
			lcd.clear();
			lcd.writeFromPgmspace(leavingPaused);
			if ( ! steppers::isRunning()) pauseState ++;
			break;

		case 10://Return to original position
			lcd.writeFromPgmspace(leavingPaused);

			//The extruders may have moved, so it doesn't make sense
			//to go back to the old position, or we'll eject the filament
			newPosition = steppers::getPosition();
			pausedPosition[3] = newPosition[3];
			pausedPosition[4] = newPosition[4];

			steppers::setTarget(pausedPosition, interval);
			pauseState ++;
			break;

		case 11://Wait for return to original position
			lcd.writeFromPgmspace(leavingPaused);
			if ( ! steppers::isRunning()) {
				pauseState = 0;
                		interface::popScreen();
				command::pause(false);
				if ( ! autoPause ) interface::popScreen();
				steppers::switchToAcceleratedDriver();
			}
			break;
	}

	if ( lastDirectionButtonPressed ) {
		if (interface::isButtonPressed(lastDirectionButtonPressed))
			jog(lastDirectionButtonPressed);
		else {
			lastDirectionButtonPressed = (ButtonArray::ButtonName)0;
			steppers::abort();
		}
	}
}

void PauseMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	if ( pauseState == 8 ) {
		if ( button == ButtonArray::CANCEL )	pauseState ++;
		else					jog(button);
	}
}

void PauseAtZPosScreen::reset() {
	int32_t currentPause = command::getPauseAtZPos();
	if ( currentPause == 0 ) {
		Point position = steppers::getPosition();
		pauseAtZPos = stepsToMM(position[2], AXIS_Z);
	} else  pauseAtZPos = stepsToMM(currentPause, AXIS_Z);
}

void PauseAtZPosScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Pause at ZPos:";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar mm[]    = "mm   ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(0,1);
	lcd.writeFloat((float)pauseAtZPos, 3);
	lcd.writeFromPgmspace(mm);
}

void PauseAtZPosScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			//Set the pause
			command::pauseAtZPos(mmToSteps(pauseAtZPos, AXIS_Z));
		case ButtonArray::CANCEL:
			interface::popScreen();
			interface::popScreen();
			break;
		case ButtonArray::ZPLUS:
			// increment more
			if (pauseAtZPos <= 250) pauseAtZPos += 1.0;
			break;
		case ButtonArray::ZMINUS:
			// decrement more
			if (pauseAtZPos >= 1.0) pauseAtZPos -= 1.0;
			else			pauseAtZPos = 0.0;
			break;
		case ButtonArray::YPLUS:
			// increment less
			if (pauseAtZPos <= 254) pauseAtZPos += 0.05;
			break;
		case ButtonArray::YMINUS:
			// decrement less
			if (pauseAtZPos >= 0.05) pauseAtZPos -= 0.05;
			else			 pauseAtZPos = 0.0;
			break;
		case ButtonArray::XMINUS:
		case ButtonArray::XPLUS:
			break;
	}

	if ( pauseAtZPos < 0.001 )	pauseAtZPos = 0.0;
}

void AdvanceABPMode::reset() {
	abpForwarding = false;
}

void AdvanceABPMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar abp1[] = "Advance ABP:";
	const static PROGMEM prog_uchar abp2[] = "hold key...";
	const static PROGMEM prog_uchar abp3[] = "           (fwd)";

	if (forceRedraw) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(abp1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(abp2);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(abp3);
	}

	if (( abpForwarding ) && ( ! interface::isButtonPressed(ButtonArray::OK) )) {
		OutPacket responsePacket;

		abpForwarding = false;
		extruderControl(SLAVE_CMD_TOGGLE_ABP, EXTDR_CMD_SET8, responsePacket, (uint16_t)0);
	}
}

void AdvanceABPMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	OutPacket responsePacket;

	switch (button) {
        	case ButtonArray::OK:
			abpForwarding = true;
			extruderControl(SLAVE_CMD_TOGGLE_ABP, EXTDR_CMD_SET8, responsePacket, (uint16_t)1);
			break;
        	case ButtonArray::YMINUS:
        	case ButtonArray::ZMINUS:
        	case ButtonArray::YPLUS:
        	case ButtonArray::ZPLUS:
        	case ButtonArray::XMINUS:
        	case ButtonArray::XPLUS:
        	case ButtonArray::ZERO:
        	case ButtonArray::CANCEL:
               		interface::popScreen();
			break;
	}
}

void CalibrateMode::reset() {
	//Disable stepps on axis 0, 1, 2, 3, 4
	steppers::enableAxis(0, false);
	steppers::enableAxis(1, false);
	steppers::enableAxis(2, false);
	steppers::enableAxis(3, false);
	steppers::enableAxis(4, false);

	lastCalibrationState = CS_NONE;
	calibrationState = CS_START1;
}

void CalibrateMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar calib1[] = "Calibrate: Move ";
	const static PROGMEM prog_uchar calib2[] = "build platform";
	const static PROGMEM prog_uchar calib3[] = "until nozzle...";
	const static PROGMEM prog_uchar calib4[] = "          (cont)";
	const static PROGMEM prog_uchar calib5[] = "lies in center,";
	const static PROGMEM prog_uchar calib6[] = "turn threaded";
	const static PROGMEM prog_uchar calib7[] = "rod until...";
	const static PROGMEM prog_uchar calib8[] = "nozzle just";
	const static PROGMEM prog_uchar calib9[] = "touches.";
	const static PROGMEM prog_uchar homeZ[]  = "Homing Z...";
	const static PROGMEM prog_uchar homeY[]  = "Homing Y...";
	const static PROGMEM prog_uchar homeX[]  = "Homing X...";
	const static PROGMEM prog_uchar done[]   = "! Calibrated !";
	const static PROGMEM prog_uchar regen[]  = "Regenerate gcode";
	const static PROGMEM prog_uchar reset[]  = "         (reset)";

	if ((forceRedraw) || (calibrationState != lastCalibrationState)) {
		lcd.clear();
		lcd.setCursor(0,0);
		switch(calibrationState) {
			case CS_START1:
				lcd.writeFromPgmspace(calib1);
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(calib2);
				lcd.setCursor(0,2);
				lcd.writeFromPgmspace(calib3);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(calib4);
				break;
			case CS_START2:
				lcd.writeFromPgmspace(calib5);
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(calib6);
				lcd.setCursor(0,2);
				lcd.writeFromPgmspace(calib7);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(calib4);
				break;
			case CS_PROMPT_MOVE:
				lcd.writeFromPgmspace(calib8);
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(calib9);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(calib4);
				break;
			case CS_HOME_Z:
			case CS_HOME_Z_WAIT:
				lcd.writeFromPgmspace(homeZ);
				break;
			case CS_HOME_Y:
			case CS_HOME_Y_WAIT:
				lcd.writeFromPgmspace(homeY);
				break;
			case CS_HOME_X:
			case CS_HOME_X_WAIT:
				lcd.writeFromPgmspace(homeX);
				break;
			case CS_PROMPT_CALIBRATED:
				lcd.writeFromPgmspace(done);
				lcd.setCursor(0,1);
				lcd.writeFromPgmspace(regen);
				lcd.setCursor(0,3);
				lcd.writeFromPgmspace(reset);
				break;
		}
	}

	lastCalibrationState = calibrationState;

	//Change the state
	//Some states are changed when a button is pressed via notifyButton
	//Some states are changed when something completes, in which case we do it here
	uint8_t axes;

	float interval = 2000.0;

	switch(calibrationState) {
		case CS_HOME_Z:
			//Declare current position to be x=0, y=0, z=0, a=0, b=0
			steppers::definePosition(Point(0,0,0,0,0));
			interval *= stepsToMM((int32_t)200.0, AXIS_Z); //Use ToM as baseline
			steppers::startHoming(true, 0x04, (uint32_t)interval);
			calibrationState = CS_HOME_Z_WAIT;
			break;
		case CS_HOME_Z_WAIT:
			if ( ! steppers::isHoming() )	calibrationState = CS_HOME_Y;
			break;
		case CS_HOME_Y:
			interval *= stepsToMM((int32_t)47.06, AXIS_Y); //Use ToM as baseline
			steppers::startHoming(false, 0x02, (uint32_t)interval);
			calibrationState = CS_HOME_Y_WAIT;
			break;
		case CS_HOME_Y_WAIT:
			if ( ! steppers::isHoming() )	calibrationState = CS_HOME_X;
			break;
		case CS_HOME_X:
			interval *= stepsToMM((int32_t)47.06, AXIS_X); //Use ToM as baseline
			steppers::startHoming(false, 0x01, (uint32_t)interval);
			calibrationState = CS_HOME_X_WAIT;
			break;
		case CS_HOME_X_WAIT:
			if ( ! steppers::isHoming() ) {
				//Record current X, Y, Z, A, B co-ordinates to the motherboard
				for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
					uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*i;
					uint32_t position = steppers::getPosition()[i];
					cli();
					eeprom_write_block(&position, (void*) offset, 4);
					sei();
				}

				//Disable stepps on axis 0, 1, 2, 3, 4
				steppers::enableAxis(0, false);
				steppers::enableAxis(1, false);
				steppers::enableAxis(2, false);
				steppers::enableAxis(3, false);
				steppers::enableAxis(4, false);

				calibrationState = CS_PROMPT_CALIBRATED;
			}
			break;
	}
}

void CalibrateMode::notifyButtonPressed(ButtonArray::ButtonName button) {

	if ( calibrationState == CS_PROMPT_CALIBRATED ) {
		host::stopBuild();
		return;
	}

	switch (button) {
        	case ButtonArray::OK:
        	case ButtonArray::YMINUS:
        	case ButtonArray::ZMINUS:
        	case ButtonArray::YPLUS:
        	case ButtonArray::ZPLUS:
        	case ButtonArray::XMINUS:
        	case ButtonArray::XPLUS:
        	case ButtonArray::ZERO:
			if (( calibrationState == CS_START1 ) || ( calibrationState == CS_START2 ) ||
			    (calibrationState == CS_PROMPT_MOVE ))	calibrationState = (enum calibrateState)((uint8_t)calibrationState + 1);
			break;
        	case ButtonArray::CANCEL:
               		interface::popScreen();
			break;
	}
}

void HomeOffsetsMode::reset() {
	homePosition = steppers::getPosition();

	for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
		uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*i;
		cli();
		eeprom_read_block(&(homePosition[i]), (void*) offset, 4);
		sei();
	}

	lastHomeOffsetState = HOS_NONE;
	homeOffsetState	    = HOS_OFFSET_X;
}

void HomeOffsetsMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1x[] = "X Offset:";
	const static PROGMEM prog_uchar message1y[] = "Y Offset:";
	const static PROGMEM prog_uchar message1z[] = "Z Offset:";
	const static PROGMEM prog_uchar message4[]  = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]     = " ";
	const static PROGMEM prog_uchar mm[]        = "mm";

	if ( homeOffsetState != lastHomeOffsetState )	forceRedraw = true;

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		switch(homeOffsetState) {
			case HOS_OFFSET_X:
				lcd.writeFromPgmspace(message1x);
				break;
                	case HOS_OFFSET_Y:
				lcd.writeFromPgmspace(message1y);
				break;
                	case HOS_OFFSET_Z:
				lcd.writeFromPgmspace(message1z);
				break;
		}

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	float position = 0.0;

	switch(homeOffsetState) {
		case HOS_OFFSET_X:
			position = stepsToMM(homePosition[0], AXIS_X);
			break;
		case HOS_OFFSET_Y:
			position = stepsToMM(homePosition[1], AXIS_Y);
			break;
		case HOS_OFFSET_Z:
			position = stepsToMM(homePosition[2], AXIS_Z);
			break;
	}

	lcd.setCursor(0,1);
	lcd.writeFloat((float)position, 3);
	lcd.writeFromPgmspace(mm);

	lastHomeOffsetState = homeOffsetState;
}

void HomeOffsetsMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	if (( homeOffsetState == HOS_OFFSET_Z ) && (button == ButtonArray::OK )) {
		//Write the new home positions
		for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
			uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*i;
			uint32_t position = homePosition[i];
			cli();
			eeprom_write_block(&position, (void*) offset, 4);
			sei();
		}

		host::stopBuild();
		return;
	}

	uint8_t currentIndex = homeOffsetState - HOS_OFFSET_X;

	switch (button) {
		case ButtonArray::CANCEL:
			interface::popScreen();
			break;
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			if 	( homeOffsetState == HOS_OFFSET_X )	homeOffsetState = HOS_OFFSET_Y;
			else if ( homeOffsetState == HOS_OFFSET_Y )	homeOffsetState = HOS_OFFSET_Z;
			break;
		case ButtonArray::ZPLUS:
			// increment more
			homePosition[currentIndex] += 20;
			break;
		case ButtonArray::ZMINUS:
			// decrement more
			homePosition[currentIndex] -= 20;
			break;
		case ButtonArray::YPLUS:
			// increment less
			homePosition[currentIndex] += 1;
			break;
		case ButtonArray::YMINUS:
			// decrement less
			homePosition[currentIndex] -= 1;
			break;
		case ButtonArray::XMINUS:
		case ButtonArray::XPLUS:
			break;
	}
}

void BuzzerSetRepeatsMode::reset() {
	repeats = eeprom::getEeprom8(eeprom::BUZZER_REPEATS, 3);
}

void BuzzerSetRepeatsMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Repeat Buzzer:";
	const static PROGMEM prog_uchar message2[] = "(0=Buzzer Off)";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar times[]    = " times ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(message2);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(0,2);
	lcd.writeInt(repeats, 3);
	lcd.writeFromPgmspace(times);
}

void BuzzerSetRepeatsMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
		case ButtonArray::CANCEL:
			interface::popScreen();
			break;
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			eeprom_write_byte((uint8_t *)eeprom::BUZZER_REPEATS, repeats);
			interface::popScreen();
			break;
		case ButtonArray::ZPLUS:
			// increment more
			if (repeats <= 249) repeats += 5;
			break;
		case ButtonArray::ZMINUS:
			// decrement more
			if (repeats >= 5) repeats -= 5;
			break;
		case ButtonArray::YPLUS:
			// increment less
			if (repeats <= 253) repeats += 1;
			break;
		case ButtonArray::YMINUS:
			// decrement less
			if (repeats >= 1) repeats -= 1;
			break;
		case ButtonArray::XMINUS:
		case ButtonArray::XPLUS:
			break;
	}
}

ExtruderFanMenu::ExtruderFanMenu() {
	itemCount = 4;
	reset();
}

void ExtruderFanMenu::resetState() {
	itemIndex = 2;
	firstItemIndex = 2;
}

void ExtruderFanMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar title[] = "Extruder Fan:";
	const static PROGMEM prog_uchar off[]   =  "Off";
	const static PROGMEM prog_uchar on[]    =  "On";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(title);
		break;
	case 1:
		break;
	case 2:
		lcd.writeFromPgmspace(off);
		break;
	case 3:
		lcd.writeFromPgmspace(on);
		break;
	}
}

void ExtruderFanMenu::handleSelect(uint8_t index) {
	OutPacket responsePacket;

	switch (index) {
		case 2:
			//Disable Cooling Fan
			extruderControl(SLAVE_CMD_TOGGLE_FAN, EXTDR_CMD_SET, responsePacket, 0);
			interface::popScreen();
			break;
		case 3:
			//Enable Cooling Fan
			extruderControl(SLAVE_CMD_TOGGLE_FAN, EXTDR_CMD_SET, responsePacket, 1);
                	interface::popScreen();
			break;
	}
}

void StepsPerMMMode::reset() {
	lastStepsPerMMState = SPM_NONE;
	stepsPerMMState	    = SPM_SET_X;
	cursorLocation	    = 0;
	originalStepsPerMM  = axisStepsPerMM[AXIS_X];
}

#define STEPS_PER_MM_INCREMENT	0.000001

void StepsPerMMMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1x[] = "X Steps per mm:";
	const static PROGMEM prog_uchar message1y[] = "Y Steps per mm:";
	const static PROGMEM prog_uchar message1z[] = "Z Steps per mm:";
	const static PROGMEM prog_uchar message1a[] = "A Steps per mm:";
	const static PROGMEM prog_uchar message4[]  = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]     = " ";

	if ( stepsPerMMState != lastStepsPerMMState )	forceRedraw = true;

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		switch(stepsPerMMState) {
			case SPM_SET_X:
				lcd.writeFromPgmspace(message1x);
				break;
                	case SPM_SET_Y:
				lcd.writeFromPgmspace(message1y);
				break;
                	case SPM_SET_Z:
				lcd.writeFromPgmspace(message1z);
				break;
                	case SPM_SET_A:
				lcd.writeFromPgmspace(message1a);
				break;
		}

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	int64_t spm = 0;

	switch(stepsPerMMState) {
		case SPM_SET_X:
			spm = axisStepsPerMM[AXIS_X];
			break;
		case SPM_SET_Y:
			spm = axisStepsPerMM[AXIS_Y];
			break;
		case SPM_SET_Z:
			spm = axisStepsPerMM[AXIS_Z];
			break;
		case SPM_SET_A:
			spm = axisStepsPerMM[AXIS_A];
			break;
	}

	//Write the number
	lcd.setCursor(0,1);
	lcd.writeFixedPoint(spm, STEPS_PER_MM_PADDING, STEPS_PER_MM_PRECISION);

	//Draw the cursor
	lcd.setCursor(cursorLocation,2);
	lcd.write('^');

	//Write a blank before and after the cursor if we're not at the ends
	if ( cursorLocation >= 1 ) {
		lcd.setCursor(cursorLocation-1, 2);
		lcd.writeFromPgmspace(blank);
	}
	if ( cursorLocation < 15 ) {
		lcd.setCursor(cursorLocation+1, 2);
		lcd.writeFromPgmspace(blank);
	}

	lastStepsPerMMState = stepsPerMMState;
}

void StepsPerMMMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	int64_t spm;
	uint16_t offset;
	uint8_t currentIndex = stepsPerMMState - SPM_SET_X;

	spm = axisStepsPerMM[currentIndex];

	//Calculate the increment based on the cursor location, allowing
	//for the decimal point
	int64_t increment = 1;
	for (uint8_t i = (STEPS_PER_MM_PADDING + STEPS_PER_MM_PRECISION); i >= 0; i -- ) {
		if ( i == cursorLocation ) break;
		if ( i != STEPS_PER_MM_PADDING ) increment *= 10;
	}
	
	//Don't increment if we're sitting on the decimcal point
	if ( cursorLocation == STEPS_PER_MM_PADDING )	increment = 0;

	switch (button) {
		case ButtonArray::CANCEL:
			axisStepsPerMM[currentIndex] = originalStepsPerMM;
			interface::popScreen();
			return;
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			//Write the new steps per mm positions
			offset = eeprom::STEPS_PER_MM_X + sizeof(int64_t) * currentIndex;
			cli();
			eeprom::putEepromInt64(offset,axisStepsPerMM[currentIndex]);
		
			//Read it back in, because we could have floating point rounding happening
			axisStepsPerMM[currentIndex] = eeprom::getEepromInt64(offset, 1);
			sei();

			if ( stepsPerMMState == SPM_SET_A ) {
				interface::popScreen();
			}
			else {
				//Increment to the next index
				stepsPerMMState = (enum StepsPerMMState)((uint8_t)stepsPerMMState + 1);
				cursorLocation	    = 0;
				originalStepsPerMM = axisStepsPerMM[currentIndex + 1];
			}
			return;
		case ButtonArray::YPLUS:
		case ButtonArray::ZPLUS:
			// increment
			spm += increment;
			break;
		case ButtonArray::YMINUS:
		case ButtonArray::ZMINUS:
			// decrement
			spm -= increment;
			break;
		case ButtonArray::XMINUS:
			if ( cursorLocation > 0 )	cursorLocation --;
			break;
		case ButtonArray::XPLUS:
			if ( cursorLocation < 15 ) 	cursorLocation ++;
			break;
	}

	//Hard limits
	if ( spm >= STEPS_PER_MM_UPPER_LIMIT ) spm = STEPS_PER_MM_UPPER_LIMIT;
        if ( spm <= STEPS_PER_MM_LOWER_LIMIT ) spm = STEPS_PER_MM_LOWER_LIMIT;

	axisStepsPerMM[currentIndex] = spm;
}

FilamentUsedResetMenu::FilamentUsedResetMenu() {
	itemCount = 4;
	reset();
}

void FilamentUsedResetMenu::resetState() {
	itemIndex = 2;
	firstItemIndex = 2;
}

void FilamentUsedResetMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg[]  = "Reset To Zero?";
	const static PROGMEM prog_uchar no[] = "No";
	const static PROGMEM prog_uchar yes[]= "Yes";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg);
		break;
	case 1:
		break;
	case 2:
		lcd.writeFromPgmspace(no);
		break;
	case 3:
		lcd.writeFromPgmspace(yes);
		break;
	}
}

void FilamentUsedResetMenu::handleSelect(uint8_t index) {
	switch (index) {
	case 3:
		//Reset to zero
                eeprom::putEepromInt64(eeprom::FILAMENT_USED, 0);
                eeprom::putEepromInt64(eeprom::FILAMENT_USED_TRIP, 0);
	case 2:
		interface::popScreen();
                interface::popScreen();
		break;
	}
}

void FilamentUsedMode::reset() {
	lifetimeDisplay = true;
	overrideForceRedraw = false;
}

void FilamentUsedMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar lifetime[] = "Lifetime Odo.:";
	const static PROGMEM prog_uchar trip[]	   = "Trip Odometer:";
	const static PROGMEM prog_uchar but_life[] = "(trip)   (reset)";
	const static PROGMEM prog_uchar but_trip[] = "(life)   (reset)";

	if ((forceRedraw) || (overrideForceRedraw)) {
		lcd.clear();

		lcd.setCursor(0,0);
		if ( lifetimeDisplay )	lcd.writeFromPgmspace(lifetime);
		else			lcd.writeFromPgmspace(trip);

	        int64_t filamentUsed = eeprom::getEepromInt64(eeprom::FILAMENT_USED, 0);

		if ( ! lifetimeDisplay ) {
			int64_t trip = eeprom::getEepromInt64(eeprom::FILAMENT_USED_TRIP, 0);
			filamentUsed = filamentUsed - trip;	
		}

		float filamentUsedMM = stepsToMM(filamentUsed, AXIS_A);

		lcd.setCursor(0,1);
		lcd.writeFloat(filamentUsedMM / 1000.0, 4);
		lcd.write('m');

		lcd.setCursor(0,2);
		if ( lifetimeDisplay )	lcd.writeFromPgmspace(but_life);
		else			lcd.writeFromPgmspace(but_trip);

		lcd.setCursor(0,3);
		lcd.writeFloat(((filamentUsedMM / 25.4) / 12.0), 4);
		lcd.writeString("ft");

		overrideForceRedraw = false;
	}
}

void FilamentUsedMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
		case ButtonArray::CANCEL:
			interface::popScreen();
			break;
		case ButtonArray::ZERO:
			lifetimeDisplay ^= true;
			overrideForceRedraw = true;
			break;
		case ButtonArray::OK:
			if ( lifetimeDisplay )
				interface::pushScreen(&filamentUsedResetMenu);
			else {
                		eeprom::putEepromInt64(eeprom::FILAMENT_USED_TRIP, eeprom::getEepromInt64(eeprom::FILAMENT_USED, 0));
				interface::popScreen();
			}
			break;
		case ButtonArray::ZPLUS:
		case ButtonArray::ZMINUS:
		case ButtonArray::YPLUS:
		case ButtonArray::YMINUS:
		case ButtonArray::XMINUS:
		case ButtonArray::XPLUS:
			break;
	}
}

BuildSettingsMenu::BuildSettingsMenu() {
	itemCount = 4;
	reset();
}

void BuildSettingsMenu::resetState() {
	itemIndex = 0;
	firstItemIndex = 0;
}

void BuildSettingsMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar item1[] = "EstimatePreheat";
	const static PROGMEM prog_uchar item2[] = "Override Temp";
	const static PROGMEM prog_uchar item3[] = "ABP Copies (SD)";
	const static PROGMEM prog_uchar item4[] = "Acceleration";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(item1);
		break;
	case 1:
		lcd.writeFromPgmspace(item2);
		break;
	case 2:
		lcd.writeFromPgmspace(item3);
		break;
	case 3:
		lcd.writeFromPgmspace(item4);
		break;
	}
}

void BuildSettingsMenu::handleSelect(uint8_t index) {
	OutPacket responsePacket;

	switch (index) {
		case 0:
			//Preheat during estimation
			interface::pushScreen(&preheatDuringEstimateMenu);
			break;
		case 1:
			//Override the gcode temperature
			interface::pushScreen(&overrideGCodeTempMenu);
			break;
		case 2:
			//Change number of ABP copies
			interface::pushScreen(&abpCopiesSetScreen);
			break;
		case 3:
			//Acceleration menu
			interface::pushScreen(&accelerationMenu);
			break;
	}
}

void ABPCopiesSetScreen::reset() {
	value = eeprom::getEeprom8(eeprom::ABP_COPIES, 1);
	if ( value < 1 ) {
		eeprom_write_byte((uint8_t*)eeprom::ABP_COPIES,1);
		value = eeprom::getEeprom8(eeprom::ABP_COPIES, 1); //Just in case
	}
}

void ABPCopiesSetScreen::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "ABP Copies (SD):";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(0,1);
	lcd.writeInt(value,3);
}

void ABPCopiesSetScreen::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
		eeprom_write_byte((uint8_t*)eeprom::ABP_COPIES,value);
		interface::popScreen();
		interface::popScreen();
		break;
        case ButtonArray::ZPLUS:
		// increment more
		if (value <= 249) {
			value += 5;
		}
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		if (value >= 6) {
			value -= 5;
		}
		break;
        case ButtonArray::YPLUS:
		// increment less
		if (value <= 253) {
			value += 1;
		}
		break;
        case ButtonArray::YMINUS:
		// decrement less
		if (value >= 2) {
			value -= 1;
		}
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}
}

PreheatDuringEstimateMenu::PreheatDuringEstimateMenu() {
	itemCount = 4;
	reset();
}

void PreheatDuringEstimateMenu::resetState() {
	if ( eeprom::getEeprom8(eeprom::PREHEAT_DURING_ESTIMATE, 0) ) 	itemIndex = 3;
	else						 		itemIndex = 2;
	firstItemIndex = 2;
}

void PreheatDuringEstimateMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg1[]    = "Preheat during";
	const static PROGMEM prog_uchar msg2[]    = "estimate phase:";
	const static PROGMEM prog_uchar disable[] = "Disable";
	const static PROGMEM prog_uchar enable[]  = "Enable";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg1);
		break;
	case 1:
		lcd.writeFromPgmspace(msg2);
		break;
	case 2:
		lcd.writeFromPgmspace(disable);
		break;
	case 3:
		lcd.writeFromPgmspace(enable);
		break;
	}
}

void PreheatDuringEstimateMenu::handleSelect(uint8_t index) {
	switch (index) {
		case 2:
			eeprom_write_byte((uint8_t*)eeprom::PREHEAT_DURING_ESTIMATE,0);
			interface::popScreen();
			break;
		case 3:
			eeprom_write_byte((uint8_t*)eeprom::PREHEAT_DURING_ESTIMATE,1);
                	interface::popScreen();
			break;
	}
}

OverrideGCodeTempMenu::OverrideGCodeTempMenu() {
	itemCount = 4;
	reset();
}

void OverrideGCodeTempMenu::resetState() {
	if ( eeprom::getEeprom8(eeprom::OVERRIDE_GCODE_TEMP, 0) ) 	itemIndex = 3;
	else						 		itemIndex = 2;
	firstItemIndex = 2;
}

void OverrideGCodeTempMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg1[]   = "Override GCode";
	const static PROGMEM prog_uchar msg2[]   = "Temperature:";
	const static PROGMEM prog_uchar disable[] =  "Disable";
	const static PROGMEM prog_uchar enable[]  =  "Enable";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg1);
		break;
	case 1:
		lcd.writeFromPgmspace(msg2);
		break;
	case 2:
		lcd.writeFromPgmspace(disable);
		break;
	case 3:
		lcd.writeFromPgmspace(enable);
		break;
	}
}

void OverrideGCodeTempMenu::handleSelect(uint8_t index) {
	switch (index) {
		case 2:  
			eeprom_write_byte((uint8_t*)eeprom::OVERRIDE_GCODE_TEMP,0);
			interface::popScreen();
			break;
		case 3:
			eeprom_write_byte((uint8_t*)eeprom::OVERRIDE_GCODE_TEMP,1);
                	interface::popScreen();
			break;
	}
}

StepperDriverAcceleratedMenu::StepperDriverAcceleratedMenu() {
	itemCount = 5;
	reset();
}

void StepperDriverAcceleratedMenu::resetState() {
	uint8_t accel = eeprom::getEeprom8(eeprom::STEPPER_DRIVER, 0);
	if	( accel == 0x03 )	itemIndex = 4;
	else if ( accel == 0x01 )	itemIndex = 3;
	else				itemIndex = 2;
	firstItemIndex = 2;
}

void StepperDriverAcceleratedMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg1[]   = "Accelerated";
	const static PROGMEM prog_uchar msg2[]   = "Stepper Driver:";
	const static PROGMEM prog_uchar off[]    =  "Off";
	const static PROGMEM prog_uchar on[]     =  "On - No Planner";
	const static PROGMEM prog_uchar planner[]=  "On - Planner";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg1);
		break;
	case 1:
		lcd.writeFromPgmspace(msg2);
		break;
	case 2:
		lcd.writeFromPgmspace(off);
		break;
	case 3:
		lcd.writeFromPgmspace(on);
		break;
	case 4:
		lcd.writeFromPgmspace(planner);
		break;
	}
}

void StepperDriverAcceleratedMenu::handleSelect(uint8_t index) {
	uint8_t oldValue = eeprom::getEeprom8(eeprom::STEPPER_DRIVER, 0);
	uint8_t newValue = oldValue;
	
	switch (index) {
		case 2:  
			newValue = 0x00;
			interface::popScreen();
			break;
		case 3:
			newValue = 0x01;
                	interface::popScreen();
			break;
		case 4:
			newValue = 0x03;
                	interface::popScreen();
			break;
	}

	//If the value has changed, do a reset
	if ( newValue != oldValue ) {
		cli();
		eeprom_write_byte((uint8_t*)eeprom::STEPPER_DRIVER, newValue);
		sei();
		//Reset
		host::stopBuild();
	}
}

#define NUM_PROFILES 4
#define PROFILES_SAVED_AXIS 3

void writeProfileToEeprom(uint8_t pIndex, uint8_t *pName, int32_t homeX,
			  int32_t homeY, int32_t homeZ, uint8_t hbpTemp,
			  uint8_t tool0Temp, uint8_t tool1Temp, uint8_t extruderRpm) {
	uint16_t offset = eeprom::PROFILE_BASE + (uint16_t)pIndex * PROFILE_NEXT_OFFSET;

	cli();

	//Write profile name
	if ( pName )	eeprom_write_block(pName,(uint8_t*)offset, PROFILE_NAME_LENGTH);
	offset += PROFILE_NAME_LENGTH;

	//Write home axis
	eeprom_write_block(&homeX, (void*) offset, 4);		offset += 4;
	eeprom_write_block(&homeY, (void*) offset, 4);		offset += 4;
	eeprom_write_block(&homeZ, (void*) offset, 4);		offset += 4;

	//Write temps and extruder RPM
	eeprom_write_byte((uint8_t *)offset, hbpTemp);		offset += 1;
	eeprom_write_byte((uint8_t *)offset, tool0Temp);	offset += 1;
	eeprom_write_byte((uint8_t *)offset, tool1Temp);	offset += 1;
	eeprom_write_byte((uint8_t *)offset, extruderRpm);	offset += 1;
	
	sei();
}

void readProfileFromEeprom(uint8_t pIndex, uint8_t *pName, int32_t *homeX,
			   int32_t *homeY, int32_t *homeZ, uint8_t *hbpTemp,
			   uint8_t *tool0Temp, uint8_t *tool1Temp, uint8_t *extruderRpm) {
	uint16_t offset = eeprom::PROFILE_BASE + (uint16_t)pIndex * PROFILE_NEXT_OFFSET;

	cli();

	//Read profile name
	if ( pName )	eeprom_read_block(pName,(uint8_t*)offset, PROFILE_NAME_LENGTH);
	offset += PROFILE_NAME_LENGTH;

	//Write home axis
	eeprom_read_block(homeX, (void*) offset, 4);		offset += 4;
	eeprom_read_block(homeY, (void*) offset, 4);		offset += 4;
	eeprom_read_block(homeZ, (void*) offset, 4);		offset += 4;

	//Write temps and extruder RPM
	*hbpTemp	= eeprom_read_byte((uint8_t *)offset);	offset += 1;
	*tool0Temp	= eeprom_read_byte((uint8_t *)offset);	offset += 1;
	*tool1Temp	= eeprom_read_byte((uint8_t *)offset);	offset += 1;
	*extruderRpm	= eeprom_read_byte((uint8_t *)offset);	offset += 1;
	
	sei();
}

//buf should have length PROFILE_NAME_LENGTH + 1 

void getProfileName(uint8_t pIndex, uint8_t *buf) {
	uint16_t offset = eeprom::PROFILE_BASE + PROFILE_NEXT_OFFSET * (uint16_t)pIndex;

	cli();
	eeprom_read_block(buf,(void *)offset,PROFILE_NAME_LENGTH);
	sei();

	buf[PROFILE_NAME_LENGTH] = '\0';
}

#define NAME_CHAR_LOWER_LIMIT 32
#define NAME_CHAR_UPPER_LIMIT 126

bool isValidProfileName(uint8_t pIndex) {
	uint8_t buf[PROFILE_NAME_LENGTH + 1];

	getProfileName(pIndex, buf);
	for ( uint8_t i = 0; i < PROFILE_NAME_LENGTH; i ++ ) {
		if (( buf[i] < NAME_CHAR_LOWER_LIMIT ) || ( buf[i] > NAME_CHAR_UPPER_LIMIT ) || ( buf[i] == 0xff )) return false;
	}

	return true;
}

ProfilesMenu::ProfilesMenu() {
	itemCount = NUM_PROFILES;
	reset();

	//Setup defaults if required
	//If the value is 0xff, write the profile number
	uint8_t buf[PROFILE_NAME_LENGTH];

        const static PROGMEM prog_uchar defaultProfile[] =  "Profile?";

	//Get the home axis positions, we may need this to write the defaults
	homePosition = steppers::getPosition();

	for (uint8_t i = 0; i < PROFILES_SAVED_AXIS; i++) {
		uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*(uint16_t)i;
		cli();
		eeprom_read_block(&homePosition[i], (void*)offset, 4);
		sei();
	}

	for (int i = 0; i < NUM_PROFILES; i ++ ) {
		if ( ! isValidProfileName(i)) {
			//Create the default profile name
			for( uint8_t i = 0; i < PROFILE_NAME_LENGTH; i ++ )
				buf[i] = pgm_read_byte_near(defaultProfile+i);
			buf[PROFILE_NAME_LENGTH - 1] = '1' + i;

			//Write the defaults
			writeProfileToEeprom(i, buf, homePosition[0], homePosition[1], homePosition[2],
					    100, 210, 210, 19);
		}
	}
}

void ProfilesMenu::resetState() {
	firstItemIndex = 0;
	itemIndex = 0;
}

void ProfilesMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	uint8_t buf[PROFILE_NAME_LENGTH + 1];

	getProfileName(index, buf);

	lcd.writeString((char *)buf);
}

void ProfilesMenu::handleSelect(uint8_t index) {
	profileSubMenu.profileIndex = index;
	interface::pushScreen(&profileSubMenu);
}

ProfileSubMenu::ProfileSubMenu() {
	itemCount = 4;
	reset();
}

void ProfileSubMenu::resetState() {
	itemIndex = 0;
	firstItemIndex = 0;
}

void ProfileSubMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg1[]  = "Restore";
	const static PROGMEM prog_uchar msg2[]  = "Display Config";
	const static PROGMEM prog_uchar msg3[]  = "Change Name";
	const static PROGMEM prog_uchar msg4[]  = "Save To Profile";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg1);
		break;
	case 1:
		lcd.writeFromPgmspace(msg2);
		break;
	case 2:
		lcd.writeFromPgmspace(msg3);
		break;
	case 3:
		lcd.writeFromPgmspace(msg4);
		break;
	}
}

void ProfileSubMenu::handleSelect(uint8_t index) {
	uint8_t hbpTemp, tool0Temp, tool1Temp, extruderRpm;

	switch (index) {
		case 0:
			//Restore
			//Read settings from eeprom
			readProfileFromEeprom(profileIndex, NULL, &homePosition[0], &homePosition[1], &homePosition[2],
					      &hbpTemp, &tool0Temp, &tool1Temp, &extruderRpm);

			//Write out the home offsets
			for (uint8_t i = 0; i < PROFILES_SAVED_AXIS; i++) {
				uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*(uint16_t)i;
				cli();
				eeprom_write_block(&homePosition[i], (void*)offset, 4);
				sei();
			}

			cli();
			eeprom_write_byte((uint8_t *)eeprom::PLATFORM_TEMP, hbpTemp);
			eeprom_write_byte((uint8_t *)eeprom::TOOL0_TEMP,    tool0Temp);
			eeprom_write_byte((uint8_t *)eeprom::TOOL1_TEMP,    tool1Temp);
			eeprom_write_byte((uint8_t *)eeprom::EXTRUDE_RPM,   extruderRpm);
			sei();

                	interface::popScreen();
                	interface::popScreen();

			//Reset
			host::stopBuild();
			break;
		case 1:
			//Display settings
			profileDisplaySettingsMenu.profileIndex = profileIndex;
			interface::pushScreen(&profileDisplaySettingsMenu);
			break;
		case 2:
			//Change Profile Name
			profileChangeNameMode.profileIndex = profileIndex;
			interface::pushScreen(&profileChangeNameMode);
			break;
		case 3: //Save To Profile 
			//Get the home axis positions
			homePosition = steppers::getPosition();
			for (uint8_t i = 0; i < PROFILES_SAVED_AXIS; i++) {
				uint16_t offset = eeprom::AXIS_HOME_POSITIONS + 4*(uint16_t)i;
				cli();
				eeprom_read_block(&homePosition[i], (void*)offset, 4);
				sei();
			}

			hbpTemp		= eeprom::getEeprom8(eeprom::PLATFORM_TEMP, 110);
			tool0Temp	= eeprom::getEeprom8(eeprom::TOOL0_TEMP, 220);
			tool1Temp	= eeprom::getEeprom8(eeprom::TOOL1_TEMP, 220);
			extruderRpm	= eeprom::getEeprom8(eeprom::EXTRUDE_RPM, 19);

			writeProfileToEeprom(profileIndex, NULL, homePosition[0], homePosition[1], homePosition[2],
					     hbpTemp, tool0Temp, tool1Temp, extruderRpm);

                	interface::popScreen();
			break;
	}
}

void ProfileChangeNameMode::reset() {
	cursorLocation = 0;
	getProfileName(profileIndex, profileName);
}

void ProfileChangeNameMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Profile Name:";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]	   = " ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	lcd.setCursor(0,1);
	lcd.writeString((char *)profileName);

	//Draw the cursor
	lcd.setCursor(cursorLocation,2);
	lcd.write('^');

	//Write a blank before and after the cursor if we're not at the ends
	if ( cursorLocation >= 1 ) {
		lcd.setCursor(cursorLocation-1, 2);
		lcd.writeFromPgmspace(blank);
	}
	if ( cursorLocation < PROFILE_NAME_LENGTH ) {
		lcd.setCursor(cursorLocation+1, 2);
		lcd.writeFromPgmspace(blank);
	}
}

void ProfileChangeNameMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	uint16_t offset;

	switch (button) {
		case ButtonArray::CANCEL:
			interface::popScreen();
			break;
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			//Write the profile name
			offset = eeprom::PROFILE_BASE + (uint16_t)profileIndex * PROFILE_NEXT_OFFSET;

			cli();
			eeprom_write_block(profileName,(uint8_t*)offset, PROFILE_NAME_LENGTH);
			sei();

			interface::popScreen();
			break;
		case ButtonArray::YPLUS:
			profileName[cursorLocation] += 1;
			break;
		case ButtonArray::ZPLUS:
			profileName[cursorLocation] += 5;
			break;
		case ButtonArray::YMINUS:
			profileName[cursorLocation] -= 1;
			break;
		case ButtonArray::ZMINUS:
			profileName[cursorLocation] -= 5;
			break;
		case ButtonArray::XMINUS:
			if ( cursorLocation > 0 )			cursorLocation --;
			break;
		case ButtonArray::XPLUS:
			if ( cursorLocation < (PROFILE_NAME_LENGTH-1) )	cursorLocation ++;
			break;
	}

	//Hard limits
	if ( profileName[cursorLocation] < NAME_CHAR_LOWER_LIMIT )	profileName[cursorLocation] = NAME_CHAR_LOWER_LIMIT;
	if ( profileName[cursorLocation] > NAME_CHAR_UPPER_LIMIT )	profileName[cursorLocation] = NAME_CHAR_UPPER_LIMIT;
}

ProfileDisplaySettingsMenu::ProfileDisplaySettingsMenu() {
	itemCount = 8;
	reset();
}

void ProfileDisplaySettingsMenu::resetState() {
	readProfileFromEeprom(profileIndex, profileName, &homeX, &homeY, &homeZ,
			      &hbpTemp, &tool0Temp, &tool1Temp, &extruderRpm);
	itemIndex = 2;
	firstItemIndex = 2;
}

void ProfileDisplaySettingsMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar xOffset[]     = "XOff: ";
	const static PROGMEM prog_uchar yOffset[]     = "YOff: ";
	const static PROGMEM prog_uchar zOffset[]     = "ZOff: ";
	const static PROGMEM prog_uchar hbp[]         = "HBP Temp:   ";
	const static PROGMEM prog_uchar tool0[]       = "Tool0 Temp: ";
	const static PROGMEM prog_uchar extruder[]    = "ExtrdrRPM: ";

	switch (index) {
	case 0:
		lcd.writeString((char *)profileName);
		break;
	case 2:
		lcd.writeFromPgmspace(xOffset);
		lcd.writeFloat(stepsToMM(homeX, AXIS_X), 3);
		break;
	case 3:
		lcd.writeFromPgmspace(yOffset);
		lcd.writeFloat(stepsToMM(homeY, AXIS_Y), 3);
		break;
	case 4:
		lcd.writeFromPgmspace(zOffset);
		lcd.writeFloat(stepsToMM(homeZ, AXIS_Z), 3);
		break;
	case 5:
		lcd.writeFromPgmspace(hbp);
		lcd.writeFloat((float)hbpTemp, 0);
		break;
	case 6:
		lcd.writeFromPgmspace(tool0);
		lcd.writeFloat((float)tool0Temp, 0);
		break;
	case 7:
		lcd.writeFromPgmspace(extruder);
		lcd.writeFloat((float)extruderRpm / 10.0, 1);
		break;
	}
}

void ProfileDisplaySettingsMenu::handleSelect(uint8_t index) {
}

void CurrentPositionMode::reset() {
}

void CurrentPositionMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar msg1[] = "X:";
	const static PROGMEM prog_uchar msg2[] = "Y:";
	const static PROGMEM prog_uchar msg3[] = "Z:";
	const static PROGMEM prog_uchar msg4[] = "A:";
	const static PROGMEM prog_uchar mm[] = "mm";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(msg1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(msg2);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(msg3);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(msg4);
	}

	Point position = steppers::getPosition();

	lcd.setCursor(3, 0);
	lcd.writeFloat(stepsToMM(position[0], AXIS_X), 3);
	lcd.writeFromPgmspace(mm);

	lcd.setCursor(3, 1);
	lcd.writeFloat(stepsToMM(position[1], AXIS_Y), 3);
	lcd.writeFromPgmspace(mm);

	lcd.setCursor(3, 2);
	lcd.writeFloat(stepsToMM(position[2], AXIS_Z), 3);
	lcd.writeFromPgmspace(mm);

	lcd.setCursor(3, 3);
	lcd.writeFloat(stepsToMM(position[3], AXIS_A), 3);
	lcd.writeFromPgmspace(mm);
}

void CurrentPositionMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	interface::popScreen();
}

		//Unable to open file, filename too long?
UnableToOpenFileMenu::UnableToOpenFileMenu() {
	itemCount = 4;
	reset();
}

void UnableToOpenFileMenu::resetState() {
	itemIndex = 3;
	firstItemIndex = 3;
}

void UnableToOpenFileMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg1[]   = "Failed to open";
	const static PROGMEM prog_uchar msg2[]   = "file.  Name too";
	const static PROGMEM prog_uchar msg3[]   = "long?";
	const static PROGMEM prog_uchar cont[]   =  "Continue";

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg1);
		break;
	case 1:
		lcd.writeFromPgmspace(msg2);
		break;
	case 2:
		lcd.writeFromPgmspace(msg3);
		break;
	case 3:
		lcd.writeFromPgmspace(cont);
		break;
	}
}

void UnableToOpenFileMenu::handleSelect(uint8_t index) {
	interface::popScreen();
}

void AcceleratedSettingsMode::reset() {
	cli();
	values[0]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_X, 160);
	values[1]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Y, 160);
	values[2]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Z, 10);
	values[3]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_A, 100);
        values[4]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_X, 2000);
        values[5]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Y, 2000);
        values[6]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Z, 150);
        values[7]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_A, 60000);
        values[8]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_NORM, 5000);
        values[9]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_RETRACT, 3000);
	values[10]	= eeprom::getEepromUInt32(eeprom::ACCEL_MIN_FEED_RATE,0);
	values[11]	= eeprom::getEepromUInt32(eeprom::ACCEL_MIN_TRAVEL_FEED_RATE,0);
	values[12]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_XY_JERK,2);
	values[13]	= eeprom::getEepromUInt32(eeprom::ACCEL_MAX_Z_JERK,100);
	values[14]	= eeprom::getEepromUInt32(eeprom::ACCEL_ADVANCE_K,50);
	values[15]	= eeprom::getEepromUInt32(eeprom::ACCEL_FILAMENT_DIAMETER,175);
	sei();

	lastAccelerateSettingsState= AS_NONE;
	accelerateSettingsState= AS_MAX_FEEDRATE_X;
}

void AcceleratedSettingsMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1xMaxFeedRate[]  	= "X MaxFeedRate:";
	const static PROGMEM prog_uchar message1yMaxFeedRate[]  	= "Y MaxFeedRate:";
	const static PROGMEM prog_uchar message1zMaxFeedRate[]  	= "Z MaxFeedRate:";
	const static PROGMEM prog_uchar message1aMaxFeedRate[]  	= "A MaxFeedRate:";
	const static PROGMEM prog_uchar message1xMaxAccelRate[] 	= "X Max Accel:";
	const static PROGMEM prog_uchar message1yMaxAccelRate[] 	= "Y Max Accel:";
	const static PROGMEM prog_uchar message1zMaxAccelRate[] 	= "Z Max Accel:";
	const static PROGMEM prog_uchar message1aMaxAccelRate[] 	= "A Max Accel:";
	const static PROGMEM prog_uchar message1ExtruderNorm[]  	= "Acc Norm Move:";
	const static PROGMEM prog_uchar message1ExtruderRetract[]	= "Acc Extr Move:";
	const static PROGMEM prog_uchar message1MinFeedRate[]		= "Min Feed Rate:";
	const static PROGMEM prog_uchar message1MinTravelFeedRate[]	= "MinTrvlFeedRate:";
	const static PROGMEM prog_uchar message1MaxXYJerk[]		= "Max XY Jerk:";
	const static PROGMEM prog_uchar message1MaxZJerk[]		= "Max Z Jerk:";
	const static PROGMEM prog_uchar message1AdvanceK[]		= "Advance K:";
	const static PROGMEM prog_uchar message1FilamentDiameter[]	= "Filament Dia:";
	const static PROGMEM prog_uchar message4[]  = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]     = "    ";

	if ( accelerateSettingsState != lastAccelerateSettingsState )	forceRedraw = true;

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		switch(accelerateSettingsState) {
			case AS_MAX_FEEDRATE_X:
				lcd.writeFromPgmspace(message1xMaxFeedRate);
				break;
                	case AS_MAX_FEEDRATE_Y:
				lcd.writeFromPgmspace(message1yMaxFeedRate);
				break;
                	case AS_MAX_FEEDRATE_Z:
				lcd.writeFromPgmspace(message1zMaxFeedRate);
				break;
                	case AS_MAX_FEEDRATE_A:
				lcd.writeFromPgmspace(message1aMaxFeedRate);
				break;
                	case AS_MAX_ACCELERATION_X:
				lcd.writeFromPgmspace(message1xMaxAccelRate);
				break;
                	case AS_MAX_ACCELERATION_Y:
				lcd.writeFromPgmspace(message1yMaxAccelRate);
				break;
                	case AS_MAX_ACCELERATION_Z:
				lcd.writeFromPgmspace(message1zMaxAccelRate);
				break;
                	case AS_MAX_ACCELERATION_A:
				lcd.writeFromPgmspace(message1aMaxAccelRate);
				break;
                	case AS_MAX_EXTRUDER_NORM:
				lcd.writeFromPgmspace(message1ExtruderNorm);
				break;
                	case AS_MAX_EXTRUDER_RETRACT:
				lcd.writeFromPgmspace(message1ExtruderRetract);
				break;
                	case AS_MIN_FEED_RATE:
				lcd.writeFromPgmspace(message1MinFeedRate);
				break;
                	case AS_MIN_TRAVEL_FEED_RATE:
				lcd.writeFromPgmspace(message1MinTravelFeedRate);
				break;
                	case AS_MAX_XY_JERK:
				lcd.writeFromPgmspace(message1MaxXYJerk);
				break;
                	case AS_MAX_Z_JERK:
				lcd.writeFromPgmspace(message1MaxZJerk);
				break;
                	case AS_ADVANCE_K:
				lcd.writeFromPgmspace(message1AdvanceK);
				break;
                	case AS_FILAMENT_DIAMETER:
				lcd.writeFromPgmspace(message1FilamentDiameter);
				break;
		}

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	uint32_t value = 0;

	uint8_t currentIndex = accelerateSettingsState - AS_MAX_FEEDRATE_X;

	value = values[currentIndex];

	lcd.setCursor(0,1);

	switch(accelerateSettingsState) {
		case AS_MIN_FEED_RATE:
		case AS_MIN_TRAVEL_FEED_RATE:
		case AS_MAX_XY_JERK:
		case AS_MAX_Z_JERK:
					lcd.writeFloat((float)value / 10.0, 1);
					break;
		case AS_ADVANCE_K:
					lcd.writeFloat((float)value / 100000.0, 5);
					break;
		case AS_FILAMENT_DIAMETER:
					lcd.writeFloat((float)value / 100.0, 2);
					break;
		default:
					lcd.writeFloat((float)value, 0);
					break;
	}

	lcd.writeFromPgmspace(blank);

	lastAccelerateSettingsState = accelerateSettingsState;
}

void AcceleratedSettingsMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	if (( accelerateSettingsState == AS_FILAMENT_DIAMETER ) && (button == ButtonArray::OK )) {
		//Write the data
		cli();
		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_X,		values[0]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Y,		values[1]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Z,		values[2]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_A,		values[3]);
       		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_X,	values[4]);
        	eeprom::putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Y,	values[5]);
        	eeprom::putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Z,	values[6]);
        	eeprom::putEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_A,	values[7]);
        	eeprom::putEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_NORM,	values[8]);
        	eeprom::putEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_RETRACT,	values[9]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MIN_FEED_RATE,		values[10]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MIN_TRAVEL_FEED_RATE,	values[11]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_XY_JERK,		values[12]);
		eeprom::putEepromUInt32(eeprom::ACCEL_MAX_Z_JERK,		values[13]);
		eeprom::putEepromUInt32(eeprom::ACCEL_ADVANCE_K,		values[14]);
		eeprom::putEepromUInt32(eeprom::ACCEL_FILAMENT_DIAMETER,	values[15]);
		sei();

		host::stopBuild();
		return;
	}

	uint8_t currentIndex = accelerateSettingsState - AS_MAX_FEEDRATE_X;

	uint32_t lastValue = values[currentIndex];

	switch (button) {
		case ButtonArray::CANCEL:
			interface::popScreen();
			break;
		case ButtonArray::ZERO:
			break;
		case ButtonArray::OK:
			accelerateSettingsState = (enum accelerateSettingsState)((uint8_t)accelerateSettingsState + 1);
			return;
			break;
		case ButtonArray::ZPLUS:
			// increment more
			values[currentIndex] += 100;
			break;
		case ButtonArray::ZMINUS:
			// decrement more
			values[currentIndex] -= 100;
			break;
		case ButtonArray::YPLUS:
			// increment less
			values[currentIndex] += 1;
			break;
		case ButtonArray::YMINUS:
			// decrement less
			values[currentIndex] -= 1;
			break;
		case ButtonArray::XMINUS:
		case ButtonArray::XPLUS:
			break;
	}

	if (!(( accelerateSettingsState == AS_MIN_FEED_RATE ) || ( accelerateSettingsState == AS_MIN_TRAVEL_FEED_RATE ) || ( accelerateSettingsState == AS_ADVANCE_K ))) {
		if ( values[currentIndex] < 1 )	values[currentIndex] = 1;
	}

	if ( values[currentIndex] > 200000 ) values[currentIndex] = 1;
}

AccelerationMenu::AccelerationMenu() {
	itemCount = 3;

	reset();
}

void AccelerationMenu::resetState() {
	if ( eeprom::getEeprom8(eeprom::STEPPER_DRIVER, 0) )	acceleration = true;
	else							acceleration = false;

	if ( acceleration )	itemCount = 3;
	else			itemCount = 1;

	itemIndex = 0;
	firstItemIndex = 0;
}

void AccelerationMenu::drawItem(uint8_t index, LiquidCrystal& lcd) {
	const static PROGMEM prog_uchar msg1[]  = "Stepper Driver";
	const static PROGMEM prog_uchar msg2[]  = "Accel. Settings";
	const static PROGMEM prog_uchar msg3[]  = "Extdr. Steps/mm";

	if (( ! acceleration ) && ( index > 0 ))	return;

	switch (index) {
	case 0:
		lcd.writeFromPgmspace(msg1);
		break;
	case 1:
		lcd.writeFromPgmspace(msg2);
		break;
	case 2:
		lcd.writeFromPgmspace(msg3);
		break;
	}
}

void AccelerationMenu::handleSelect(uint8_t index) {
	if (( ! acceleration ) && ( index > 0 ))	return;

	switch (index) {
		case 0:
			interface::pushScreen(&stepperDriverAcceleratedMenu);
			break;
		case 1:
			interface::pushScreen(&acceleratedSettingsMode);
			break;
		case 2:
			interface::pushScreen(&eStepsPerMMMode);
			break;
	}
}

void EStepsPerMMMode::reset() {
	value = eeprom::getEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM, 44);
	if ( value < 1 ) {
		eeprom::putEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM, 44);
		value = eeprom::getEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM, 44); //Just in case
	}
}

void EStepsPerMMMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Extrdr Steps/mm:";
	const static PROGMEM prog_uchar message2[] = "(calib)";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]     = "  ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(message2);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(8,2);
	lcd.writeFloat((float)value / 10.0,1);
	lcd.writeFromPgmspace(blank);
}

void EStepsPerMMMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
		eeprom::putEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM,value);
		//Reset to read in the new value
		host::stopBuild();
		return;
		break;
        case ButtonArray::ZPLUS:
		// increment more
		value += 25;
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		value -= 25;
		break;
        case ButtonArray::YPLUS:
		// increment less
		value += 1;
		break;
        case ButtonArray::YMINUS:
		// decrement less
		value -= 1;
		break;

        case ButtonArray::XMINUS:
		interface::pushScreen(&eStepsPerMMStepsMode);
		break;
        case ButtonArray::XPLUS:
		break;
	}

	if (( value < 1 ) || ( value > 200000 )) value = 1;
}

void EStepsPerMMStepsMode::reset() {
	value = 200;
	steppers::switchToRegularDriver();
	overrideExtrudeSeconds = 0;
}

void EStepsPerMMStepsMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Extrude N steps:";
	const static PROGMEM prog_uchar message2[] = "(extrude)";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]     = " ";

	if (overrideExtrudeSeconds)	extrude(true);

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,2);
		lcd.writeFromPgmspace(message2);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(10,2);
	lcd.writeFloat((float)value,0);
	lcd.writeFromPgmspace(blank);
}

void EStepsPerMMStepsMode::extrude(bool overrideTempCheck) {
	//Check we're hot enough
	if ( ! overrideTempCheck )
	{
		OutPacket responsePacket;
		if (extruderControl(SLAVE_CMD_IS_TOOL_READY, EXTDR_CMD_GET, responsePacket, 0)) {
			uint8_t data = responsePacket.read8(1);
		
			if ( ! data )
			{
				overrideExtrudeSeconds = 1;
				interface::pushScreen(&extruderTooColdMenu);
				return;
			}
		}
	}

	Point position = steppers::getPosition();

	float rpm = (float)eeprom::getEeprom8(eeprom::EXTRUDE_RPM, 19) / 10.0;

	//60 * 1000000 = # uS in a minute
	//200 * 8 = 200 steps per revolution * 1/8 stepping
	int32_t interval = (int32_t)(60L * 1000000L) / (int32_t)((float)(200 * 8) * rpm);

	position[3] += -value;
	steppers::setTarget(position, interval);

	if (overrideTempCheck)	overrideExtrudeSeconds = 0;
}


void EStepsPerMMStepsMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	switch (button) {
        case ButtonArray::CANCEL:
    		steppers::switchToAcceleratedDriver();
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
    		steppers::switchToAcceleratedDriver();
		eStepsPerMMLengthMode.steps = value;
		interface::pushScreen(&eStepsPerMMLengthMode);
		break;
        case ButtonArray::ZPLUS:
		// increment more
		value += 25;
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		value -= 25;
		break;
        case ButtonArray::YPLUS:
		// increment less
		value += 1;
		break;
        case ButtonArray::YMINUS:
		// decrement less
		value -= 1;
		break;

        case ButtonArray::XMINUS:
		extrude(false);
		break;
        case ButtonArray::XPLUS:
		break;
	}
}

void EStepsPerMMLengthMode::reset() {
	value = 1;
}

void EStepsPerMMLengthMode::update(LiquidCrystal& lcd, bool forceRedraw) {
	const static PROGMEM prog_uchar message1[] = "Enter noodle";
	const static PROGMEM prog_uchar message2[] = "length in mm's";
	const static PROGMEM prog_uchar message4[] = "Up/Dn/Ent to Set";
	const static PROGMEM prog_uchar blank[]     = "  ";

	if (forceRedraw) {
		lcd.clear();

		lcd.setCursor(0,0);
		lcd.writeFromPgmspace(message1);

		lcd.setCursor(0,1);
		lcd.writeFromPgmspace(message2);

		lcd.setCursor(0,3);
		lcd.writeFromPgmspace(message4);
	}

	// Redraw tool info
	lcd.setCursor(0,2);
	lcd.writeFloat((float)value,0);
	lcd.writeFromPgmspace(blank);
}

void EStepsPerMMLengthMode::notifyButtonPressed(ButtonArray::ButtonName button) {
	uint32_t espm;

	switch (button) {
        case ButtonArray::CANCEL:
		interface::popScreen();
		break;
        case ButtonArray::ZERO:
		break;
        case ButtonArray::OK:
		if ( steps < 0 ) steps *= -1;
		espm = (uint32_t)lround(((float)steps / (float)value) * 10.0);
		eeprom::putEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM,espm);
	
		//Reset to read in the new value
		host::stopBuild();
		return;
		break;
        case ButtonArray::ZPLUS:
		// increment more
		value += 5;
		break;
        case ButtonArray::ZMINUS:
		// decrement more
		value -= 5;
		break;
        case ButtonArray::YPLUS:
		// increment less
		value += 1;
		break;
        case ButtonArray::YMINUS:
		// decrement less
		value -= 1;
		break;

        case ButtonArray::XMINUS:
        case ButtonArray::XPLUS:
		break;
	}

	if (( value < 1 ) || ( value > 200000 )) value = 1;
}

#endif
