// LCD Controls for a Modtronix LCD2S 4x20 LCD with I2C serial controller.
// More details at http://www.modtronix.com/products/lcd2s

#ifndef _MODTRONIXLCD2S_H
#define _MODTRONIXLCD2S_H
#include "display.hh"

class ModtronixLCD2S : public Display {
  uint8_t address;
  uint8_t charset;

public:
  ModtronixLCD2S(uint8_t addr = 0x50>>1);

  virtual void init();
  virtual void clear();
  virtual size_t width() const { return 16; }
  virtual size_t height() const { return 4; }

  virtual void setCursor(uint8_t col, uint8_t row) { return set_position(row+1,col+1); }
  virtual void write(uint8_t value) { return write_char(value); }

  virtual void writeInt(uint16_t value, uint8_t digits);
  virtual void writeString(const char message[]) { return write_string(message); }
  virtual void writeFromPgmspace(const prog_char message[]) { return write_string_P(message ); }

  void remember();
  void config(uint8_t display, uint8_t contrast, uint8_t brightness, uint8_t keypadio, uint8_t keypadbuzz);
  void set_base_address(uint8_t addr);

  int8_t read_status();
//  void read_status_async(asynctwi::callback_t cb);
  char read_keypad_data();
//  void read_keypad_data_async(asynctwi::callback_t cb);
  uint8_t read_gpio123();
//  void read_gpio123_async(asynctwi::callback_t cb);

  void backlight_on();
  void backlight_off();
  void backlight_brightness(uint8_t val);
  void set_contrast(uint8_t val);

  void display_on();
  void display_off();
  void cursor_underline_on();
  void cursor_underline_off();
  void cursor_block_on();
  void cursor_block_off();
  void dir_forward();
  void dir_backwards();
  void load_charset(uint8_t val);
  void define_custom_char(uint8_t adr, const uint8_t *data);

  void home();
//  void clear();
  void set_position(uint8_t row, uint8_t col);
  void set_cursor_addr(uint8_t val);

  void write_string(const char* str);
  void write_string_P(PGM_P str);
  void write_char(char ch);
  void write_decimal(int16_t val, int8_t wid = 0, uint8_t decdigits = 0);
  void set_startup_line(uint8_t row, const char* str);
  void write_large_number_string(const char* str);
  void draw_vertical_bar_graph(uint8_t row, uint8_t col, uint8_t val);
  void draw_tall_bar_graph(uint8_t row, uint8_t col, uint8_t val);

  void shift_right();
  void shift_left();
  void shift_up();
  void shift_down();

  void interrupt_open_collection();
  void interrupt_push_pull();
  void config_keypad_and_io(uint8_t val);
  void config_gpio1(uint8_t val);
  void config_gpio2(uint8_t val);
  void config_gpio3(uint8_t val);

  void set_out12(uint8_t val1, uint8_t val2); // 0xE2 VAL
  void out1_on();
  void out1_off();
  void out2_on();
  void out2_off();

  void gpio1_on();
  void gpio1_off();
  void gpio2_on();
  void gpio2_off();

  void set_max_backlight_brightness(uint8_t val);
  void set_keypad_debounce_time(uint8_t val);
  void set_keypad_repeat_delay(uint8_t val);
  void set_keypad_repeat_rate(uint8_t val);
  void set_keypad_buzzer_period(uint8_t val);
};

#endif // _MODTRONIXLCD2S_H

// vim: set sw=2 autoindent nowrap expandtab: settings

