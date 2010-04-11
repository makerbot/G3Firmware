#ifndef SCREENS_H
#define SCREENS_H

#include <avr/pgmspace.h>
#include "LCD.h"

// These are the defines to change to interface with the LCD code.
#define SCREEN_CLEAR()             lcd_clear()
#define SCREEN_SET_POS(row,col)    lcd_set_position(row,col)
#define SCREEN_WRITE(str)          lcd_write(str)
#define SCREEN_WRITE_P(str)        lcd_write_P(str)
#define SCREEN_WRITECHAR(x)        lcd_write_char(x)
#define SCREEN_CURSOR_ON()         lcd_cursor_on()
#define SCREEN_CURSOR_OFF()        lcd_cursor_off()
#define SCREEN_WAIT_FOR_LCD(num)   lcd_wait_for_free_space(num);
#define SCREEN_SET_CONTRAST(num)   lcd_set_contrast(num);
#define SCREEN_SET_BRIGHTNESS(num) lcd_set_brightness(num);

// LCD-specific characters.
#define SCREEN_UP_ARROW_CHAR    '\305'
#define SCREEN_DOWN_ARROW_CHAR  '\306'
#define SCREEN_RIGHT_ARROW_CHAR '\307'
#define SCREEN_LEFT_ARROW_CHAR  '\310'
#define SCREEN_PAUSE_CHAR       LCD_PAUSE_CHAR
#define SCREEN_PLAY_CHAR        LCD_PLAY_CHAR
#define SCREEN_EXTFWD_CHAR      LCD_EXT_FWD_CHAR
#define SCREEN_EXTREV_CHAR      LCD_EXT_REV_CHAR

typedef enum {
  NONE,DRAW,REDRAW} 
ScreenRedraw_t;

class Screen {
private:
  static Screen* current_screen;
  static ScreenRedraw_t needs_update;

protected:
  static uint32_t last_update_millis;

public:
  virtual void init();                 // Called before screen draw.
  virtual void draw();                 // To draw initial screen.
  virtual void redraw();               // To update an already drawn screen.
  virtual uint8_t handleKey(char c);   // To handle keypad entries.
  virtual uint8_t isUpdateNeeded();    // Lets subclass force early redraw.

  static void change(Screen* scrn);
  static void update();
  static void processKey(char c);
  static void setNeedsDraw();
  static void setNeedsRedraw();
};


class IntegerEntryScreen: 
public Screen {
private:
  int16_t value;

public:
  virtual void init();
  virtual uint16_t getValue();
  virtual void setValue(int16_t val);
  virtual void draw();
  virtual void redraw();
  virtual uint8_t handleKey(char c);
  virtual void commit();
  virtual void cancel();
};


#endif // SCREENS_H

// vim: set sw=2 autoindent nowrap expandtab: settings


