#ifndef _UTILS_H_
#define _UTILS_H_

#include <avr/pgmspace.h>


void delayMicrosecondsInterruptible(unsigned int us);

bool strendswith_P(const char* haystack, PGM_P needle);
void concat_decimal(char* outbuf, uint8_t buflen, int16_t val, int8_t wid, uint8_t decdigits, uint8_t prependzero = 0);
uint16_t isqrt(uint32_t x);

#endif _UTILS_H_
