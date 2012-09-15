#include "Eeprom.hh"
#include "EepromMap.hh"
#include "EepromDefaults.hh"

#include "Version.hh"
#include <avr/eeprom.h>

#ifdef EEPROM_MENU_ENABLE
	#include <avr/wdt.h>
	#include "SDCard.hh"
#endif

namespace eeprom {

void init() {
        uint8_t version[2];
        eeprom_read_block(version,(const uint8_t*)eeprom::VERSION_LOW,2);
        if ((version[1]*100+version[0]) == firmware_version) return;
        if (version[1] == 0xff || version[1] < 2) {
            setDefaults(false);
        }
        // Write version
        version[0] = firmware_version % 100;
        version[1] = firmware_version / 100;
        eeprom_write_block(version,(uint8_t*)eeprom::VERSION_LOW,2);
}

#if defined(ERASE_EEPROM_ON_EVERY_BOOT) || defined(EEPROM_MENU_ENABLE)

#if defined (__AVR_ATmega168__)
	#define EEPROM_SIZE 512
#elif defined (__AVR_ATmega328__)
	#define EEPROM_SIZE 1024
#elif defined (__AVR_ATmega644P__)
	#define EEPROM_SIZE 2048
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
	#define EEPROM_SIZE 4096
#else
	#define EEPROM_SIZE 0
#endif

//Complete erase of eeprom to 0xFF
void erase() {
	for (uint16_t i = 0; i < EEPROM_SIZE; i ++ ) {
		eeprom_write_byte((uint8_t*)i, 0xFF);
		wdt_reset();
	}
}

#endif

#ifdef EEPROM_MENU_ENABLE

//Saves the eeprom to filename on the sd card
bool saveToSDFile(const char *filename) {
	uint8_t v;

	//Open the file for writing
	if ( sdcard::startCapture((char *)filename) != sdcard::SD_SUCCESS )	return false;

	//Write the eeprom contents to the file
        for (uint16_t i = 0; i < EEPROM_SIZE; i ++ ) {
                v = eeprom_read_byte((uint8_t*)i);
		sdcard::writeByte(v);
		wdt_reset();
	}

	sdcard::finishCapture();

	return true;
}

//Restores eeprom from filename on the sdcard
bool restoreFromSDFile(const char *filename) {
	uint8_t v;

	if ( sdcard::startPlayback((char *)filename) != sdcard::SD_SUCCESS )	return false;

        for (uint16_t i = 0; i < EEPROM_SIZE; i ++ ) {
		if ( sdcard::playbackHasNext() ) {
			v = sdcard::playbackNext();
                	eeprom_write_byte((uint8_t*)i, v);
			wdt_reset();
		}
		else break;
	}

    	sdcard::finishPlayback();

	return true;
}

#endif

uint8_t getEeprom8Sub(const uint16_t location, const uint8_t default_value, const bool defaultCheck) {
        uint8_t data;
        eeprom_read_block(&data,(const uint8_t*)location,1);
        if (( defaultCheck ) && (data == 0xff)) data = default_value;
        return data;
}

uint8_t getEeprom8(const uint16_t location, const uint8_t default_value) {
	return getEeprom8Sub(location, default_value, true);
}

uint8_t getEeprom8(const uint16_t location) {
	return getEeprom8Sub(location, 0, false);
}

uint16_t getEeprom16(const uint16_t location, const uint16_t default_value) {
        uint16_t data;
        eeprom_read_block(&data,(const uint8_t*)location,2);
        if (data == 0xffff) data = default_value;
        return data;
}

float getEepromFixed16(const uint16_t location, const float default_value) {
        uint8_t data[2];
        eeprom_read_block(data,(const uint8_t*)location,2);
        if (data[0] == 0xff && data[1] == 0xff) return default_value;
        return ((float)data[0]) + ((float)data[1])/256.0;
}

int64_t getEepromInt64(const uint16_t location, const int64_t default_value) {
	int64_t *ret;
        uint8_t data[8];
        eeprom_read_block(data,(const uint8_t*)location,8);
        if (data[0] == 0xff && data[1] == 0xff && data[2] == 0xff && data[3] == 0xff &&
	    data[4] == 0xff && data[5] == 0xff && data[6] == 0xff && data[7] == 0xff)
		 return default_value;
	ret = (int64_t *)&data[0];
	return *ret;
}

uint32_t getEepromUInt32(const uint16_t location, const uint32_t default_value) {
	uint32_t *ret;
        uint8_t data[4];
        eeprom_read_block(data,(const uint8_t*)location,4);
        if (data[0] == 0xff && data[1] == 0xff && data[2] == 0xff && data[3] == 0xff)
		 return default_value;
	ret = (uint32_t *)&data[0];
	return *ret;
}

void putEepromInt64(const uint16_t location, const int64_t value) {
        void *data;
	data = (void *)&value;
        eeprom_write_block(data,(void*)location,8);
}

void putEepromUInt32(const uint16_t location, const uint32_t value) {
        void *data;
	data = (void *)&value;
        eeprom_write_block(data,(void*)location,4);
}

#ifdef STEPS_PER_MM_LOWER_LIMIT

int64_t getEepromStepsPerMMSub(const uint16_t location, const int64_t default_value, const bool defaultCheck) {
        int64_t value = eeprom::getEepromInt64(location, default_value);

	if ( ! defaultCheck ) 	return value;

        if (( value <= STEPS_PER_MM_LOWER_LIMIT ) || ( value >= STEPS_PER_MM_UPPER_LIMIT )) {
                eeprom::putEepromInt64(location, default_value);

                //Just to be on the safe side
                value = eeprom::getEepromInt64(location, default_value);
        }

        return value;
}

int64_t getEepromStepsPerMM(const uint16_t location, const int64_t default_value) {
	return getEepromStepsPerMMSub(location, default_value, true);
}

int64_t getEepromStepsPerMM(const uint16_t location) {
	return getEepromStepsPerMMSub(location, 0, false);
}

#endif

} // namespace eeprom
