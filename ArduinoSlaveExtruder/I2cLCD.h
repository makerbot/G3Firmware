#ifndef I2CLCD_H
#define I2CLCD_H

#include <stdint.h>
#include <Wire.h>

// This code currently assumes a 4x20 LCD display
// with a I2C adapter which defaults to this configuration

// The slave address defaults to 0xfe (8-bit), i.e. 0x7f (7-bit)
#define kI2C_LCD_SLAVE_ADR  (uint8_t)0x7f

// I2C Commands for Coptonix I2C LC-Display Adapter
#define kI2C_ClrDisplay     (uint8_t)0x61
#define kI2C_ReturnHome     (uint8_t)0x62
#define kI2C_SetCursor      (uint8_t)0x63
#define kI2C_CharToLCD      (uint8_t)0x64
#define kI2C_CMDToLCD       (uint8_t)0x65
#define kI2C_Delete         (uint8_t)0x66
#define kI2C_Copy           (uint8_t)0x67
#define kI2C_Paste          (uint8_t)0x68
#define kI2C_GetCursorAdr   (uint8_t)0x69
#define kI2C_GetCharAtCur   (uint8_t)0x6A
#define kI2C_ReadRAM        (uint8_t)0x6B
#define kI2C_DisplayON_OFF  (uint8_t)0x6C
#define kI2C_Shift          (uint8_t)0x6D
#define kI2C_WriteString    (uint8_t)0x76

// ClrDisplay parameters
#define kI2C_DisplayON  ((uint8_t)1<<2)
#define kI2C_CursorON  ((uint8_t)1<<1)
#define kI2C_BlinkON  ((uint8_t)1<<0)

// Display Data Address Chart for 4x20 MC2004E Series Display
#define kLCD_DDA_Line0  (uint8_t)0x00
#define kLCD_DDA_Line1  (uint8_t)0x40
#define kLCD_DDA_Line2  (uint8_t)0x14
#define kLCD_DDA_Line3  (uint8_t)0x54

class I2cLCD
{
    private:
        unsigned long last_lcd_update_time; // ms
    
    protected:
        void setCursorRC(uint8_t row, uint8_t column);
        void updateTemp(int line, int current, int target);
        
    public:
        void initialize();
        
        void updateTempDisplay();
        void updateStatus(char* status);
};
#endif //I2CLCD_H