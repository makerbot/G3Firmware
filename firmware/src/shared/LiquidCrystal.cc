
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "configuration.hh"

#if HAS_INTERFACE_BOARD > 0
#include "LiquidCrystal.hh"

#define _rs_pin     LCD_RS_PIN
#define _enable_pin LCD_ENABLE_PIN
#define _d0_pin     LCD_D0_PIN
#define _d1_pin     LCD_D1_PIN
#define _d2_pin     LCD_D2_PIN
#define _d3_pin     LCD_D3_PIN

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00


// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LiquidCrystal::LiquidCrystal()
{

  _rs_pin::setDirection(true);
  _enable_pin::setDirection(true);

  _rs_pin::setValue(false);
  _enable_pin::setValue(false);

  _d0_pin::setDirection(true);
  _d1_pin::setDirection(true);
  _d2_pin::setDirection(true);
  _d3_pin::setDirection(true);

  _d0_pin::setValue(false);
  _d1_pin::setValue(false);
  _d2_pin::setValue(false);
  _d3_pin::setValue(false);

    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  
}

void LiquidCrystal::init()
{
	begin(LCD_SCREEN_WIDTH, LCD_SCREEN_HEIGHT);
    clear();
    home();
}


/************ low level data pushing commands **********/
static inline void _delay_248ns(void)
{
    __asm__ __volatile__ ("nop \n\tnop \n\tnop \n\tnop \n\t");
}

inline void LiquidCrystal::pulseEnable(void) {
  _enable_pin::setValue(true);
    _delay_248ns();
  _enable_pin::setValue(false);
}

inline void LiquidCrystal::write4bits(uint8_t value) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    {
        _d0_pin::setValue(((value >> 0) & 0x01) != 0, false);
        _d1_pin::setValue(((value >> 1) & 0x01) != 0, false);
        _d2_pin::setValue(((value >> 2) & 0x01) != 0, false);
        _d3_pin::setValue(((value >> 3) & 0x01) != 0, false);
    }

  pulseEnable();
}

// write either command or data, with automatic 4/8-bit selection
void LiquidCrystal::send(uint8_t value, bool mode) {
  _rs_pin::setValue(mode);

    write4bits(value>>4);
    _delay_248ns(); // delay to keep enable_pin low before half a cycle time.
    write4bits(value);

    _delay_us(41);   // commands need > 37us to settle [citation needed]
}




void LiquidCrystal::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;
  _currline = 0;

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  _delay_us(50000);

  
  //put the LCD into 4 bit o
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    write4bits(0x03);
    _delay_us(4100); // wait min 4.1ms

    // second try
    write4bits(0x03);
    _delay_us(4100); // wait min 4.1ms
    
    // third go!
    write4bits(0x03); 
    _delay_us(150);

    // finally, set to 8=4-bit interface
    write4bits(0x02); 

  // finally, set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

}

/********** high level commands, for the user! */
void LiquidCrystal::clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}

void LiquidCrystal::home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}

void LiquidCrystal::setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x10, 0x50 };
  if ( row > _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }
  
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void LiquidCrystal::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LiquidCrystal::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal::cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LiquidCrystal::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal::blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LiquidCrystal::scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LiquidCrystal::scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LiquidCrystal::leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LiquidCrystal::rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LiquidCrystal::autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LiquidCrystal::noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LiquidCrystal::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}

/*********** mid level commands, for sending data/cmds */


void LiquidCrystal::writeInt(uint16_t value, uint8_t digits) {

	uint16_t currentDigit;
	uint16_t nextDigit;

	switch (digits) {
	case 1:		currentDigit = 10;		break;
	case 2:		currentDigit = 100;		break;
	case 3:		currentDigit = 1000;	break;
	case 4:		currentDigit = 10000;	break;
	default: 	return;
	}

	for (uint8_t i = 0; i < digits; i++) {
		nextDigit = currentDigit/10;
		write(uint8_t((value%currentDigit)/nextDigit)+'0');
		currentDigit = nextDigit;
	}
}


void LiquidCrystal::writeString(const char message[]) {
	const char* letter = message;
	while (*letter != 0) {
		write(*letter);
		letter++;
	}
}

void LiquidCrystal::writeFromPgmspace(const prog_char message[]) {
	char letter;
	while ((letter = pgm_read_byte(message++))) {
		write(letter);
	}
}

#endif // HAS_INTERFACE_BOARD > 0
