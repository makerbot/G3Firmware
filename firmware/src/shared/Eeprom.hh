#ifndef EEPROM_HH
#define EEPROM_HH

#include <stdint.h>

namespace eeprom {

void init();

uint8_t getEeprom8(const uint16_t location, const uint8_t default_value);
uint16_t getEeprom16(const uint16_t location, const uint16_t default_value);
float getEepromFixed16(const uint16_t location, const float default_value);
int64_t getEepromInt64(const uint16_t location, const int64_t default_value);
void putEepromInt64(const uint16_t location, const int64_t value);

}

#define STEPS_PER_MM_PADDING	 5
#define STEPS_PER_MM_PRECISION	 10
#define STEPS_PER_MM_LOWER_LIMIT 10000000
#define STEPS_PER_MM_UPPER_LIMIT 200000000000000
#define STEPS_PER_MM_X_DEFAULT   470698520000	//47.069852
#define STEPS_PER_MM_Y_DEFAULT   470698520000	//47.069852
#define STEPS_PER_MM_Z_DEFAULT	 2000000000000	//200.0
#define STEPS_PER_MM_A_DEFAULT   502354788069	//50.2354788069
#define STEPS_PER_MM_B_DEFAULT   502354788069	//50.2354788069

#endif // EEPROM_HH
