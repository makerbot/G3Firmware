#include <string.h>
#include "ModtronixLCD2S.hh"
#include "twi.h"


static int i2c_write(uint8_t address, uint8_t cmd, uint8_t* data, uint8_t len)
{
    twi_data bufs[] = { {0, 1, &cmd}, {0, len, data} };
  return twi_writeTo_buffers(address, bufs, 2, 0 );
}



static int i2c_write_P(uint8_t address, uint8_t cmd, PGM_P data, uint8_t len)
{
    twi_data bufs[] = { {0, 1, &cmd}, {1, len, (const uint8_t*) data} };
  return twi_writeTo_buffers(address, bufs, 2, 0 );
}



static int i2c_cmd_read(uint8_t address, uint8_t cmd, uint8_t* data, uint8_t expectedlen)
{
//  return asynctwi::write_and_read_async(address, &cmd, 1, data, expectedlen, NULL);
    return 0;
}


/*
static int i2c_cmd_read_async(uint8_t address, uint8_t cmd, uint8_t expectedlen, asynctwi::callback_t cb)
{
  return asynctwi::write_and_read_async(address, &cmd, 1, NULL, expectedlen, cb);
}

*/


ModtronixLCD2S::ModtronixLCD2S(uint8_t addr)
{
  address = addr;
  charset = 255;
}

void ModtronixLCD2S::init()
{
  twi_init();
//  clear();
//  home();
}

void ModtronixLCD2S::remember()
{
  i2c_write(address, 0x8D, NULL, 0);
}



int8_t ModtronixLCD2S::read_status()
{
  uint8_t val = 0;
  int8_t len =  i2c_cmd_read(address, 0xD0, &val, 1);
  if (len == -1) {
      return -1;
  }
  return val;
}


/*
void ModtronixLCD2S::read_status_async(asynctwi::callback_t cb)
{
  i2c_cmd_read_async(address, 0xD0, 1, cb);
}
*/



char ModtronixLCD2S::read_keypad_data()
{
  uint8_t data[2];
  i2c_cmd_read(address, 0xD1, data, 1);
  return (char)(data[0]);
}


/*
void ModtronixLCD2S::read_keypad_data_async(asynctwi::callback_t cb)
{
  i2c_cmd_read_async(address, 0xD1, 1, cb);
}
*/



uint8_t ModtronixLCD2S::read_gpio123()
{
  uint8_t val = 0;
  i2c_cmd_read(address, 0xD3, &val, 1);
  return val;
}


/*
void ModtronixLCD2S::read_gpio123_async(asynctwi::callback_t cb)
{
  i2c_cmd_read_async(address, 0xD3, 1, cb);
}
*/





void ModtronixLCD2S::config(uint8_t display, uint8_t contrast, uint8_t brightness, uint8_t keypadio, uint8_t keypadbuzz)
{
  uint8_t data[4];
  data[0] = display;
  data[1] = ((brightness & 0x3) << 6) | (contrast & 0x3f);
  data[2] = keypadio;
  data[3] = keypadbuzz;
  i2c_write(address, 0xD0, data, 4);
}



void ModtronixLCD2S::set_base_address(uint8_t addr) {
  remember();
  i2c_write(address, 0x91, &addr, 1);
}



void ModtronixLCD2S::write_decimal(int16_t val, int8_t wid, uint8_t decdigits) {
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
    while (ptr2 >= buf) {
      *ptr2-- = ' ';
    } 
  }
  i2c_write(address, 0x80, (uint8_t*)buf, strlen(buf));
}

void ModtronixLCD2S::writeInt(uint16_t value, uint8_t digits) {
  char buf[digits];

	uint16_t currentDigit;
	uint16_t nextDigit;

	switch (digits) {
	case 1:		currentDigit = 10;		break;
	case 2:		currentDigit = 100;		break;
	case 3:		currentDigit = 1000;	break;
	case 4:		currentDigit = 10000;	break;
	default: 	return;
	}

    uint8_t i;
	for (i = 0; i < digits; i++) {
		nextDigit = currentDigit/10;
        buf[i] = (value%currentDigit)/nextDigit+'0';
		currentDigit = nextDigit;
	}

  i2c_write(address, 0x80, (uint8_t*)buf, i);
}


void ModtronixLCD2S::write_string(const char* str) {
  i2c_write(address, 0x80, (uint8_t*)str, strlen(str));
}



void ModtronixLCD2S::write_string_P(PGM_P pstr) {
  i2c_write_P(address, 0x80, pstr, strlen_P(pstr));
}



void ModtronixLCD2S::set_startup_line(uint8_t row, const char* str) {
  uint8_t buf[22];
  buf[0] = row;
  strcpy((char*)buf+1,str);
  i2c_write(address, 0x90, buf, strlen(str)+1);
}



void ModtronixLCD2S::write_char(char ch) {
  uint8_t buf[2];
  buf[0] = ch;
  buf[1] = '\0';
  i2c_write(address, 0x80, buf, 1);
}



void ModtronixLCD2S::backlight_on() {
  i2c_write(address, 0x28, NULL, 0);
}



void ModtronixLCD2S::backlight_off() {
  i2c_write(address, 0x20, NULL, 0);
}



void ModtronixLCD2S::backlight_brightness(uint8_t val) {
  i2c_write(address, 0x81, &val, 1);
}



void ModtronixLCD2S::set_contrast(uint8_t val) {
  i2c_write(address, 0x82, &val, 1);
}



void ModtronixLCD2S::dir_forward() {
  i2c_write(address, 0x09, NULL, 0);
}



void ModtronixLCD2S::dir_backwards() {
  i2c_write(address, 0x01, NULL, 0);
}



void ModtronixLCD2S::display_on() {
  i2c_write(address, 0x1A, NULL, 0);
}



void ModtronixLCD2S::display_off() {
  i2c_write(address, 0x12, NULL, 0);
}



void ModtronixLCD2S::cursor_underline_on() {
  i2c_write(address, 0x19, NULL, 0);
}



void ModtronixLCD2S::cursor_underline_off() {
  i2c_write(address, 0x11, NULL, 0);
}



void ModtronixLCD2S::cursor_block_on() {
  i2c_write(address, 0x18, NULL, 0);
}



void ModtronixLCD2S::cursor_block_off() {
  i2c_write(address, 0x10, NULL, 0);
}



void ModtronixLCD2S::shift_right() {
  i2c_write(address, 0x85, NULL, 0);
}



void ModtronixLCD2S::shift_left() {
  i2c_write(address, 0x86, NULL, 0);
}



void ModtronixLCD2S::shift_up() {
  i2c_write(address, 0x87, NULL, 0);
}



void ModtronixLCD2S::shift_down() {
  i2c_write(address, 0x88, NULL, 0);
}



void ModtronixLCD2S::home() {
  i2c_write(address, 0x8B, NULL, 0);
}



void ModtronixLCD2S::clear() {
  i2c_write(address, 0x8C, NULL, 0);
}



void ModtronixLCD2S::load_charset(uint8_t val) {
  if (val != charset) {
    i2c_write(address, 0x8E, &val, 1);
    charset = val;
  }
}



void ModtronixLCD2S::set_position(uint8_t row, uint8_t col) {
  uint8_t data[2];
  data[0] = row;
  data[1] = col;
  i2c_write(address, 0x8A, data, 2);
}


void ModtronixLCD2S::interrupt_open_collection() {
  i2c_write(address, 0x2A, NULL, 0);
}


void ModtronixLCD2S::interrupt_push_pull() {
  i2c_write(address, 0x22, NULL, 0);
}


void ModtronixLCD2S::out1_on() {
  i2c_write(address, 0x38, NULL, 0);
}



void ModtronixLCD2S::out1_off() {
  i2c_write(address, 0x30, NULL, 0);
}



void ModtronixLCD2S::out2_on() {
  i2c_write(address, 0x39, NULL, 0);
}



void ModtronixLCD2S::out2_off() {
  i2c_write(address, 0x31, NULL, 0);
}



void ModtronixLCD2S::gpio1_on() {
  i2c_write(address, 0x48, NULL, 0);
}



void ModtronixLCD2S::gpio1_off()  {
  i2c_write(address, 0x40, NULL, 0);
}



void ModtronixLCD2S::gpio2_on() {
  i2c_write(address, 0x49, NULL, 0);
}



void ModtronixLCD2S::gpio2_off() {
  i2c_write(address, 0x41, NULL, 0);
}



void ModtronixLCD2S::config_keypad_and_io(uint8_t val) {
  remember();
  i2c_write(address, 0xE0, &val, 1);
}



void ModtronixLCD2S::config_gpio1(uint8_t val) {
  remember();
  i2c_write(address, 0xE3, &val, 1);
}



void ModtronixLCD2S::config_gpio2(uint8_t val) {
  remember();
  i2c_write(address, 0xE4, &val, 1);
}



void ModtronixLCD2S::config_gpio3(uint8_t val) {
  remember();
  i2c_write(address, 0xE5, &val, 1);
}



void ModtronixLCD2S::set_max_backlight_brightness(uint8_t val) {
  remember();
  i2c_write(address, 0xA3, &val, 1);
}



void ModtronixLCD2S::set_keypad_debounce_time(uint8_t val) {
  remember();
  i2c_write(address, 0xE1, &val, 1);
}



void ModtronixLCD2S::set_keypad_repeat_delay(uint8_t val) {
  remember();
  i2c_write(address, 0xA0, &val, 1);
}



void ModtronixLCD2S::set_keypad_repeat_rate(uint8_t val) {
  remember();
  i2c_write(address, 0xA1, &val, 1);
}



void ModtronixLCD2S::set_keypad_buzzer_period(uint8_t val) {
  remember();
  i2c_write(address, 0xA2, &val, 1);
}



void ModtronixLCD2S::write_large_number_string(const char* str) {
  load_charset(2);
  i2c_write(address, 0x8F, (uint8_t*)str, strlen(str));
}



void ModtronixLCD2S::set_cursor_addr(uint8_t val) {
  i2c_write(address, 0x89, &val, 1);
}



void ModtronixLCD2S::set_out12(uint8_t val1, uint8_t val2) {
  uint8_t val = (val1? 0x1 : 0) | (val2? 0x2 : 0);
  i2c_write(address, 0xE2, &val, 1);
}



void ModtronixLCD2S::draw_vertical_bar_graph(uint8_t row, uint8_t col, uint8_t val) {
  uint8_t buf[3];
  buf[0] = row;
  buf[1] = col;
  buf[2] = val;
  load_charset(0);
  i2c_write(address, 0x93, buf, 3);
}



void ModtronixLCD2S::draw_tall_bar_graph(uint8_t row, uint8_t col, uint8_t val) {
  uint8_t buf[3];
  buf[0] = row;
  buf[1] = col;
  buf[2] = val;
  load_charset(1);
  i2c_write(address, 0x94, buf, 3);
}



void ModtronixLCD2S::define_custom_char(uint8_t adr, const uint8_t *data) {
  uint8_t buf[9];
  buf[0] = adr;
  memcpy(buf+1, data, 8);
  i2c_write(address, 0x92, buf, 9);
}



// vim: set sw=2 autoindent nowrap expandtab: settings


