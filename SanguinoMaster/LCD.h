#ifndef LCD_H
#define LCD_H

#include "Configuration.h"
#include "ModtronixLCD2S.h"

#define LCD_PAUSE_CHAR 1
#define LCD_PLAY_CHAR 2
#define LCD_EXT_FWD_CHAR 3
#define LCD_EXT_REV_CHAR 4
#define LCD_EXT_HEAT_CHAR 5
#define LCD_PLATFORM_HEAT_CHAR 6

void lcd_init();
void lcd_init_or_flash(bool doflash);
void lcd_update();

void lcd_wait_for_free_space(uint8_t num);
void lcd_write(const char *str);
void lcd_write_P(PGM_P str);
void lcd_write_char(char c);
void lcd_set_position(uint8_t row, uint8_t col);
void lcd_clear();
void lcd_cursor_on();
void lcd_cursor_off();
void lcd_set_contrast(uint8_t val);
void lcd_set_brightness(uint8_t val);


#ifdef KEYPAD_ORIENT_ONEONTOP
  // We expect a telephone style one-on-top keypad layout
  //  1E-   2Z+   3Y+   Stop
  //  4X-   5E0   6X+   Pause
  //  7Y-   8Z-   9E+   Menu
  //  0Zero Temp  Units Enter

# define KEY_EMINUS  '1'
# define KEY_ZPLUS   '2'
# define KEY_YPLUS   '3'

# define KEY_XMINUS  '4'
# define KEY_ESTOP   '5'
# define KEY_XPLUS   '6'

# define KEY_YMINUS  '7'
# define KEY_ZMINUS  '8'
# define KEY_EPLUS   '9'

#else
  // We expect a computer style one-on-bottom keypad layout
  //  7E-   8Z+   9Y+   Stop
  //  4X-   5E0   6X+   Pause
  //  1Y-   2Z-   3E+   Menu
  //  0Zero Temp  Units Enter

# define KEY_EMINUS  '7'
# define KEY_ZPLUS   '8'
# define KEY_YPLUS   '9'

# define KEY_XMINUS  '4'
# define KEY_ESTOP   '5'
# define KEY_XPLUS   '6'

# define KEY_YMINUS  '1'
# define KEY_ZMINUS  '2'
# define KEY_EPLUS   '3'

#endif


#define KEY_STOP   'S'
#define KEY_PAUSE  'P'
#define KEY_MENU   'M'
#define KEY_ENTER  'E'
#define KEY_TEMPS  'T'
#define KEY_UNITS  'U'
#define KEY_ZERO   '0'
#define KEY_DELETE KEY_UNITS

#endif // LCD_H

// vim: set sw=2 autoindent nowrap expandtab: settings

