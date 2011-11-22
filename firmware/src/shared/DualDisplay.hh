#ifndef DUALDISPLAY_HH
#define DUALDISPLAY_HH

// TODO: Proper attribution

#include <stdint.h>
#include "Display.hh"
#include "LiquidCrystal.hh"
#include "ModtronixLCD2S.hh"

class DualDisplay : public Display {
public:
    DualDisplay() {}
  virtual void init();
  virtual void clear();
  virtual size_t width() const { return 16; }
  virtual size_t height() const { return 4; }
  virtual void setCursor(uint8_t col, uint8_t row); 
  virtual void write(uint8_t);

  virtual void writeInt(uint16_t value, uint8_t digits);
  virtual void writeString(const char message[]);
  virtual void writeFromPgmspace(const prog_char message[]);

private:
//    Display lcd; // provide a NULL display
    LiquidCrystal lcd;
    ModtronixLCD2S lcd2s;
};

#endif // DUALDISPLAY_HH
