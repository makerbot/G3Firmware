#include "Configuration.h"
#include <WProgram.h>
#include "I2cLCD.h"
#include "Version.h"
#include "Heater.h"
#include "Variables.h"

void I2cLCD::initialize()
{
    last_lcd_update_time = 0;
    
    // Setup I2C hardware
    Wire.begin();
    
    // Switch on LCD
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_DisplayON_OFF);
    Wire.send(kI2C_DisplayON); // On, no Cursor, no Blink
    Wire.endTransmission();
        
    // Clear the Display
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_ClrDisplay);
    Wire.endTransmission();
   
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send("Welcome to MakerBot");
    Wire.endTransmission();
    
    setCursorRC(1,0);
    char fwVersion[32];
    sprintf(fwVersion,"Extruder FW v%d.%d%s",FIRMWARE_VERSION/100,FIRMWARE_VERSION%100, FIRMWARE_BUILD_NUMBER_STRING);
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send(fwVersion);
    Wire.endTransmission();
    
    delay(3000);
    
    // Setup the basic screen
    
    // Clear the Display
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_ClrDisplay);
    Wire.endTransmission();
    
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send("Extruder:");
    Wire.endTransmission();
    
#if HAS_HEATED_BUILD_PLATFORM
    setCursorRC(1,0);
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send("Platform:");
    Wire.endTransmission();
#endif

    updateStatus("Idle");

    sprintf(fwVersion,"v%d.%d%s",FIRMWARE_VERSION/100,FIRMWARE_VERSION%100, FIRMWARE_BUILD_NUMBER_STRING);
    setCursorRC(3,20-strlen(fwVersion));
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send(fwVersion);
    Wire.endTransmission();
}

void I2cLCD::setCursorRC(uint8_t row, uint8_t column)
{
    if(row<4 && column<20)
    {
        Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
        Wire.send(kI2C_SetCursor);
        switch(row)
        {
            case 0: Wire.send(kLCD_DDA_Line0+column); break;
            case 1: Wire.send(kLCD_DDA_Line1+column); break;
            case 2: Wire.send(kLCD_DDA_Line2+column); break;
            case 3: Wire.send(kLCD_DDA_Line3+column); break;
        }
        
        Wire.endTransmission();
    }
}

void I2cLCD::updateTemp(int line, int current, int target)
{
    setCursorRC(line, 9);
    char tempString[12];
    sprintf(tempString,"%3d%cC%c%3d%cC", current, 0xdf, 0x7e, target, 0xdf);
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send(tempString);
    Wire.endTransmission();
}

void I2cLCD::updateStatus(char* status)
{
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_Delete);
    Wire.send((uint8_t)3);
    Wire.send((uint8_t)1);
    Wire.send((uint8_t)20);
    Wire.endTransmission();
   
    setCursorRC(2, 0);
    Wire.beginTransmission(kI2C_LCD_SLAVE_ADR);
    Wire.send(kI2C_WriteString);
    Wire.send(status);
    Wire.endTransmission();
}

void I2cLCD::updateTempDisplay()
{
  int dt;
  unsigned long time;

  // ignoring millis rollover for now
  time = millis();
 
  dt = time - last_lcd_update_time;
  if (dt > TEMP_LCD_UPDATE_INTERVAL)
  { 
    last_lcd_update_time = time;
    
    updateTemp(0, extruder_heater.get_current_temperature(), extruder_heater.get_target_temperature());
#if HAS_HEATED_BUILD_PLATFORM
    updateTemp(1, platform_heater.get_current_temperature(), platform_heater.get_target_temperature());
#endif
  }
}
