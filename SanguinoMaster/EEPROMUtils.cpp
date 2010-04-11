#include "EEPROMOffsets.h"

bool hasEEPROMSettings() {
  return (EEPROM.read(0) == EEPROM_CHECK_LOW &&
	  EEPROM.read(1) == EEPROM_CHECK_HIGH);
}


uint16_t getEEPROMHeadTemp() {
  uint8_t tmplo = 0;
  uint8_t tmphi = 0;
  if (hasEEPROMSettings()) {
    tmplo = EEPROM.read(EEPROM_HEAD_TEMP_OFFSET);
    tmphi = EEPROM.read(EEPROM_HEAD_TEMP_OFFSET+1);
  }
  return ((((uint16_t)tmphi)<<8)+tmplo);
}



uint16_t getEEPROMPlatformTemp() {
  uint8_t tmplo = 0;
  uint8_t tmphi = 0;
  if (hasEEPROMSettings()) {
    tmplo = EEPROM.read(EEPROM_PLATFORM_TEMP_OFFSET);
    tmphi = EEPROM.read(EEPROM_PLATFORM_TEMP_OFFSET+1);
  }
  return ((((uint16_t)tmphi)<<8)+tmplo);
}



uint16_t getEEPROMLcdContrast() {
  uint8_t val = 254;
  if (hasEEPROMSettings()) {
    val = EEPROM.read(EEPROM_LCD_CONTRAST_OFFSET);
  }
  if (val == 0) {
    // Uninitialized EEPROM memory probably is 0.
    val = 254;
  }
  return val;
}



void setEEPROMHeadTemp(uint16_t val) {
  EEPROM.write(EEPROM_HEAD_TEMP_OFFSET, val & 0xff);
  EEPROM.write(EEPROM_HEAD_TEMP_OFFSET+1, val >> 8);
}



void setEEPROMPlatformTemp(uint16_t val) {
  EEPROM.write(EEPROM_PLATFORM_TEMP_OFFSET, val & 0xff);
  EEPROM.write(EEPROM_PLATFORM_TEMP_OFFSET+1, val >> 8);
}



