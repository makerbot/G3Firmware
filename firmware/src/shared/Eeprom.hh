#ifndef EEPROM_HH
#define EEPROM_HH

#include <stdint.h>
#include "Configuration.hh"

namespace eeprom {

void init();

#if defined(ERASE_EEPROM_ON_EVERY_BOOT) || defined(EEPROM_MENU_ENABLE)
        void erase();
#endif

#ifdef EEPROM_MENU_ENABLE
	bool saveToSDFile(const char *filename);
	bool restoreFromSDFile(const char *filename);
#endif


uint8_t getEeprom8(const uint16_t location);
uint8_t getEeprom8(const uint16_t location, const uint8_t default_value);
uint16_t getEeprom16(const uint16_t location, const uint16_t default_value);
float getEepromFixed16(const uint16_t location, const float default_value);
int64_t getEepromInt64(const uint16_t location, const int64_t default_value);
uint32_t getEepromUInt32(const uint16_t location, const uint32_t default_value);
void putEepromInt64(const uint16_t location, const int64_t value);
void putEepromUInt32(const uint16_t location, const uint32_t value);
int64_t getEepromStepsPerMM(const uint16_t location);
int64_t getEepromStepsPerMM(const uint16_t location, const int64_t default_value);
}

#endif // EEPROM_HH
