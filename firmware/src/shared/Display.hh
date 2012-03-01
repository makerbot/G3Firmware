#ifndef DISPLAY_HH
#define DISPLAY_HH

// TODO: Proper attribution

#include <stdint.h>
#include <avr/pgmspace.h>

class Display {
public:
  Display() {};

    virtual void init() {};
    virtual void clear() {};
    virtual size_t width() const { return 1; }
    virtual size_t height() const { return 1; }
    virtual void setCursor(uint8_t, uint8_t) {}; 
    virtual void write(uint8_t) {};

    virtual void writeInt(uint16_t value, uint8_t digits) {}
    virtual void writeString(const char message[]) {};
    virtual void writeFromPgmspace(const prog_char message[]) {};

    // note: the base interface provides a NULL Display implementation which does nothing.
};

#endif // LIQUID_CRYSTAL_HH
