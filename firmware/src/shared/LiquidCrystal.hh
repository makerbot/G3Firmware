#ifndef LIQUID_CRYSTAL_HH
#define LIQUID_CRYSTAL_HH

// TODO: Proper attribution

#include <stdint.h>
#include <avr/pgmspace.h>
#include "PinTmplt.hh"
#include"Display.hh"

#define LCD_SCREEN_WIDTH        16
#define LCD_SCREEN_HEIGHT       4

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#if HAS_INTERFACE_BOARD > 0

class LiquidCrystal : public Display {
public:
  LiquidCrystal();

  void init(uint8_t fourbitmode);
    
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);

  virtual void init();

  virtual void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void createChar(uint8_t, uint8_t[]);
  virtual void setCursor(uint8_t, uint8_t); 
  virtual void write(uint8_t value) { send(value, true); }

  /** Added by MakerBot Industries to support storing strings in flash **/
  virtual void writeInt(uint16_t value, uint8_t digits);

  virtual void writeString(const char message[]);

  virtual void writeFromPgmspace(const prog_char message[]);

  inline void command(uint8_t value) { send(value, false); }



private:
  void send(uint8_t, bool);
  void write4bits(uint8_t);
  void pulseEnable();


  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _initialized;

  uint8_t _numlines,_currline;
};

#endif //HAS_INTERFACE_BOARD > 0

#endif // LIQUID_CRYSTAL_HH
