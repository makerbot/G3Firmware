#ifndef __EEPROM_OFFSETS_H__
#define __EEPROM_OFFSETS_H__

#include <EEPROM.h>

#define EEPROM_CHECK_LOW 0x5A
#define EEPROM_CHECK_HIGH 0x78

#define EEPROM_CHECK_OFFSET 0
#define EEPROM_AXIS_INVERSION_OFFSET 2
#define EEPROM_MACHINE_NAME_OFFSET 32
#define EEPROM_HEAD_TEMP_OFFSET 64
#define EEPROM_PLATFORM_TEMP_OFFSET 66
#define EEPROM_LCD_CONTRAST_OFFSET 68

bool hasEEPROMSettings();

uint16_t getEEPROMHeadTemp();
uint16_t getEEPROMPlatformTemp();
uint16_t getEEPROMLcdContrast();
void setEEPROMHeadTemp(uint16_t val);
void setEEPROMPlatformTemp(uint16_t val);
 

#endif // __EEPROM_OFFSETS_H__
