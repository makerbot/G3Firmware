#include "DualDisplay.hh"

#include <stdio.h>
#include <string.h>
#include <util/delay.h>

void DualDisplay::init()
{
    lcd.init();
    lcd2s.init();
    clear();
}

/********** high level commands, for the user! */
void DualDisplay::clear()
{
    lcd.clear();
    lcd2s.clear();
}


void DualDisplay::setCursor(uint8_t col, uint8_t row)
{
    lcd.setCursor(col,row);
    lcd2s.setCursor(col,row);
}

void DualDisplay::write(uint8_t c) {
    lcd.write(c);
    lcd2s.write(c);
}


void DualDisplay::writeInt(uint16_t value, uint8_t digits) {
    lcd.writeInt(value, digits);
    lcd2s.writeInt(value, digits);
}

void DualDisplay::writeString(const char message[]) {
    lcd.writeString(message);
    lcd2s.writeString(message);
}

void DualDisplay::writeFromPgmspace(const prog_char message[]) {
    lcd.writeFromPgmspace(message);
    lcd2s.writeFromPgmspace(message);
}

