#include <string.h>
#include <stdint.h>
#include "Utils.h"


void delayMicrosecondsInterruptible(unsigned int us)
{
  // for a one-microsecond delay, simply return.  the overhead
  // of the function call yields a delay of approximately 1 1/8 us.
  if (--us == 0)
    return;

  // the following loop takes a quarter of a microsecond (4 cycles)
  // per iteration, so execute it four times for each microsecond of
  // delay requested.
  us <<= 2;

  // account for the time taken in the preceeding commands.
  us -= 2;

  // busy wait
  __asm__ __volatile__ ("1: sbiw %0,1" "\n\t" // 2 cycles
"brne 1b" : 
  "=w" (us) : 
  "0" (us) // 2 cycles
    );
}



bool strendswith_P(const char* haystack, PGM_P needle)
{
    const char* ptr = haystack;
    const char* end = haystack + (strlen(haystack) - strlen_P(needle));
    do {
	ptr = strcasestr_P(ptr, needle);
    } while (ptr && ptr++ < end);
    return (ptr > end);
}



void concat_decimal(char* outbuf, uint8_t buflen, int16_t val, int8_t wid, uint8_t decdigits, uint8_t prependzero)
{
  char buf[10];
  char* ptr = buf;
  char* ptr2;
  bool force = false;
  if (val < 0) {
    val = -val;
    *ptr++ = '-';
  }
  if (decdigits == 5) {
      *ptr++ = '0';
      *ptr++ = '.';
      force = true;
  }
  if (force || val >= 10000) {
    *ptr++ = '0'+(val/10000);
    val = val % 10000;
    force = true;
  }
  if (decdigits == 4) {
      if (ptr == buf) {
	  *ptr++ = '0';
      }
      *ptr++ = '.';
      force = true;
  }
  if (force || val >= 1000) {
    *ptr++ = '0'+(val/1000);
    val = val % 1000;
    force = true;
  }
  if (decdigits == 3) {
      if (ptr == buf) {
	  *ptr++ = '0';
      }
      *ptr++ = '.';
      force = true;
  }
  if (force || val >= 100) {
    *ptr++ = '0'+(val/100);
    val = val % 100;
    force = true;
  }
  if (decdigits == 2) {
      if (ptr == buf) {
	  *ptr++ = '0';
      }
      *ptr++ = '.';
      force = true;
  }
  if (force || val >= 10) {
    *ptr++ = '0'+(val/10);
    val = val % 10;
  }
  if (decdigits == 1) {
      if (ptr == buf) {
	  *ptr++ = '0';
      }
      *ptr++ = '.';
  }
  *ptr++ = '0'+val;
  *ptr = '\0';
  if (ptr-buf < wid) {
    ptr2 = buf+wid;
    while (ptr >= buf) {
      *ptr2-- = *ptr--;
    }
    if (prependzero) {
      while (ptr2 >= buf) {
	*ptr2-- = '0';
      } 
    } else {
      while (ptr2 >= buf) {
	*ptr2-- = ' ';
      } 
    } 
  }
  ptr = buf;
  ptr2 = outbuf;
  while (*ptr2 && ptr2 - outbuf < buflen)
    *ptr2++;
  while (*ptr && ptr2 - outbuf < buflen)
    *ptr2++ = *ptr++;
  if (ptr2 - outbuf < buflen) {
    *ptr2 = '\0';
  }
}



uint16_t
isqrt(uint32_t x)
{
  uint32_t op, res, one;

  op = x;
  res = 0;

  /* "one" starts at the highest power of four <= than the argument. */
  one = 1L << 30;  /* second-to-top bit set */
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res + (one << 1);
    }
    res >>= 1;
    one >>= 2;
  }
  return(res);
}



