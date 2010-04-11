// Yep, this is actually -*- c++ -*-
#include <avr/delay.h>
#include "Configuration.h"
#include "Tools.h"
#include "ModtronixLCD2S.h"
#include "Screens.h"
#include "EEPROMOffsets.h"

#ifdef LCD_I2C_ADDRESS

ModtronixLCD2S Lcd(LCD_I2C_ADDRESS);
#define LCD_MAXBUF 80


const char keypadmap[] = {
  KEY_EMINUS,  KEY_ZPLUS,   KEY_YPLUS,   KEY_STOP,
  KEY_XMINUS,  KEY_ESTOP,   KEY_XPLUS,   KEY_PAUSE,
  KEY_YMINUS,  KEY_ZMINUS,  KEY_EPLUS,   KEY_MENU,
  KEY_ZERO,    KEY_TEMPS,   KEY_UNITS,   KEY_ENTER
};


char     lcd_last_polled_key;
uint8_t  lcd_last_polled_lcdbuf_free;
uint32_t lcd_keypad_last_millis;
uint32_t lcd_temps_last_millis;


extern "C" {
  uint32_t millis();
}



void lcd_wait_for_free_space(uint8_t num) {
    while ((Lcd.read_status() & 0x7f) < num) {
        _delay_ms(5);
        continue;
    }
}



void lcd_init_or_flash(bool doflash)
{
  char buf[25];
  PGM_P line1 = PSTR("                    ");
  PGM_P line2 = PSTR(" Belfry Fabber  Mk1 ");
  PGM_P line3 = PSTR("   Firmware  106    ");

  Lcd.clear();

  if (doflash) {
    lcd_wait_for_free_space(22);
    strcpy_P(buf, line2);
    Lcd.set_startup_line(2, buf);
    _delay_ms(20*6);

    lcd_wait_for_free_space(22);
    strcpy_P(buf, line3);
    Lcd.set_startup_line(3, buf);
    _delay_ms(20*6);

    lcd_wait_for_free_space(22);
    strcpy_P(buf, line1);
    Lcd.set_startup_line(1, buf);
    _delay_ms(20*6);

    lcd_wait_for_free_space(22);
    Lcd.set_startup_line(4, buf);
    _delay_ms(20*6);

    lcd_wait_for_free_space(10);
    Lcd.config_keypad_and_io(0);
    _delay_ms(6);
    Lcd.set_keypad_debounce_time(100/8);
    _delay_ms(6);
  }

  lcd_wait_for_free_space(50);
  Lcd.write_string_P(line1);
  Lcd.write_string_P(line2);

  lcd_wait_for_free_space(50);
  Lcd.write_string_P(line3);
  Lcd.write_string_P(line1);

  const uint8_t pausechar[8]  = { 0x00,0x1b,0x1b,0x1b,0x1b,0x1b,0x1b,0x00 };
  const uint8_t playchar[8]   = { 0x00,0x18,0x1e,0x1f,0x1e,0x18,0x00,0x00 };
  const uint8_t extfwdchar[8] = { 0x1f,0x1f,0x1f,0x04,0x00,0x04,0x04,0x03 };
  const uint8_t extrevchar[8] = { 0x00,0x04,0x0e,0x1f,0x00,0x1f,0x00,0x00 };

  lcd_wait_for_free_space(20);
  Lcd.define_custom_char(LCD_PAUSE_CHAR, pausechar);
  lcd_wait_for_free_space(20);
  Lcd.define_custom_char(LCD_PLAY_CHAR, playchar);
  lcd_wait_for_free_space(20);
  Lcd.define_custom_char(LCD_EXT_FWD_CHAR, extfwdchar);
  lcd_wait_for_free_space(20);
  Lcd.define_custom_char(LCD_EXT_REV_CHAR, extrevchar);

  lcd_wait_for_free_space(20);
  Lcd.display_on();
  Lcd.backlight_on();
  Lcd.backlight_brightness(254);
  Lcd.set_contrast(getEEPROMLcdContrast());
}




void lcd_init()
{
  lcd_init_or_flash(false);

  lcd_last_polled_key = '\0';
  lcd_last_polled_lcdbuf_free = LCD_MAXBUF;
  lcd_keypad_last_millis = 0;;
  lcd_temps_last_millis = 0;;
}




static void lcd_keypad_poll_cb(uint8_t cmd, uint8_t* data, uint8_t len)
{
  char key = (char)data[0];
  if (!key) {
    return;
  }
  lcd_last_polled_key = keypadmap[key-'a'];
}



static void lcd_status_poll_cb(uint8_t cmd, uint8_t* data, uint8_t len)
{
  uint8_t status = data[0];
  if ((status & 0x80)) {
    // Keypress data to read.
    Lcd.read_keypad_data_async(lcd_keypad_poll_cb);
  }
  lcd_last_polled_lcdbuf_free = (status & 0x7f);
}




void lcd_update()
{
  uint32_t now = millis();

  // Every tenth of a second or so, poll for keypad status.
  if (now - lcd_keypad_last_millis > 100) {
    Lcd.read_status_async(lcd_status_poll_cb);
    lcd_keypad_last_millis = now;
  }

  // Every second or so, get an update on the temperatures.
  if (now - lcd_temps_last_millis > 1000) {
    poll_current_tool_temps();
    lcd_temps_last_millis = now;
  }

  // If we get a character from the keypad, process it.
  if (lcd_last_polled_key) {
    Screen::processKey(lcd_last_polled_key);
    lcd_last_polled_key = '\0';
  }

  // Finish sending pending commands before we task the LCD with more.
  if (twi_pending() > 0) {
    return;
  }

  // Don't spam LCD with data until it has a clear buffer.
  if (lcd_last_polled_lcdbuf_free < LCD_MAXBUF-5) {
      return;
  }

  // Display the current Screen if necessary.
  Screen::update();
}




/////////////////////////////////////////////////////////////////////////
// LCD control abstractions for the LCD control Screens
// This is set up to use the ModtronixLCD2S controller object.
// We're expecting a 4x20 LCD screen with 4x4 keypad support.
/////////////////////////////////////////////////////////////////////////


void lcd_write_char(char c)
{
  char buf[2];
  buf[0] = c;
  buf[1] = '\0';
  Lcd.write_string(buf);
}


void lcd_write(const char *str)
{
  Lcd.write_string(str);
}


void lcd_write_P(PGM_P str)
{
  Lcd.write_string_P(str);
}


void lcd_set_position(uint8_t row, uint8_t col)
{
  Lcd.set_position(row, col);
}


void lcd_clear()
{
  Lcd.clear();
}


void lcd_cursor_on()
{
  Lcd.cursor_block_on();
}


void lcd_cursor_off()
{
  Lcd.cursor_block_off();
}


void lcd_set_contrast(uint8_t val)
{
  Lcd.set_contrast(val);
}


void lcd_set_brightness(uint8_t val)
{
  Lcd.backlight_brightness(val);
}


#endif

// vim: set sw=2 autoindent nowrap expandtab: settings


