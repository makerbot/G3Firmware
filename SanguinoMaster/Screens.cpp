#include <string.h>
#include "Screens.h"
#include "Tools.h"
#include "SDSupport.h"
#include "Steppers.h"
#include "Configuration.h"
#include "Commands.h"
#include "Variables.h"
#include "Utils.h"
#include <EEPROM.h>
#include "EEPROMOffsets.h"
#include "PacketProcessor.h"

extern class DefaultScreen default_screen;
extern class MainMenuScreen main_menu_screen;
extern class FileSelectScreen file_select_screen;
extern class TempMenuScreen temp_menu_screen;
extern class ZeroMenuScreen zero_menu_screen;
extern class LcdContrastScreen lcd_contrast_screen;
extern void abort_print();

extern "C" {
  uint32_t millis();
}


uint32_t Screen::last_update_millis = 0;
Screen* Screen::current_screen = 0;
ScreenRedraw_t Screen::needs_update = DRAW;

const uint8_t jog_length_xy[] = {
  (uint8_t)( 0.1*STEPS_PER_MM_XY+0.5),  // ~0.127 mm
  (uint8_t)( 1.0*STEPS_PER_MM_XY+0.5),  //  ~1 mm
  (uint8_t)( 5.0*STEPS_PER_MM_XY+0.5),  //  ~5 mm
  (uint8_t)(10.0*STEPS_PER_MM_XY+0.5),  // ~10 mm
  (uint8_t)(20.0*STEPS_PER_MM_XY+0.5)   // ~20 mm
  };
const uint16_t jog_length_z[] = {
  (uint16_t)( 0.1*STEPS_PER_MM_Z+0.5),   // ~0.100 mm
  (uint16_t)( 1.0*STEPS_PER_MM_Z+0.5),   //  ~1 mm
  (uint16_t)( 5.0*STEPS_PER_MM_Z+0.5),   //  ~5 mm
  (uint16_t)(10.0*STEPS_PER_MM_Z+0.5),   // ~10 mm
  (uint16_t)(20.0*STEPS_PER_MM_Z+0.5)    // ~20 mm
  };
const char jogtxt_step[] PROGMEM = "Step";
const char jogtxt_1mm[] PROGMEM = " 1mm";
const char jogtxt_5mm[] PROGMEM = " 5mm";
const char jogtxt_10mm[] PROGMEM = "10mm";
const char jogtxt_20mm[] PROGMEM = "20mm";
PGM_P jog_length_txt[] = {jogtxt_step, jogtxt_1mm, jogtxt_5mm, jogtxt_10mm, jogtxt_20mm };



static void toggle_pause_state()
{
  if (is_machine_paused) {
    //unpause our machine.
    is_machine_paused = false;

    //unpause our tools
    set_tool_pause_state(false);

    //resume stepping.
    resume_stepping();
  } 
  else {
    //pause our activity.
    is_machine_paused = true;

    //pause our tools
    set_tool_pause_state(true);

    //pause stepping
    pause_stepping();
  }
}



uint8_t Screen::isUpdateNeeded() { 
  return 0; 
}
void Screen::init() { 
}
void Screen::draw() { 
}
void Screen::redraw() { 
}



uint8_t Screen::handleKey(char c)
{
  switch (c) {
  case KEY_STOP:
    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("\n    Resetting...    "));
    abort_print();
    Screen::change((Screen*)&default_screen);
    return 1;
  case KEY_PAUSE:
    toggle_pause_state();
    Screen::change((Screen*)&default_screen);
    return 1;
  case KEY_ZERO:
    Screen::change((Screen*)&zero_menu_screen);
    return 1;
  case KEY_TEMPS:
    Screen::change((Screen*)&temp_menu_screen);
    return 1;
  }
  return 0;
}



void Screen::change(Screen* scrn)
{
  current_screen = scrn;
  Screen::setNeedsDraw();
}



void Screen::update()
{
  if (millis() - last_update_millis < 250) {
    // Don't try to update faster than the LCD can handle.
    return;
  }
  if (current_screen) {
    if (needs_update == NONE) {
      if (current_screen->isUpdateNeeded()) {
        needs_update = REDRAW;
      }
    }
    if (needs_update == REDRAW) {
      needs_update = NONE;
      current_screen->redraw();
      last_update_millis = millis();
    } 
    else if (needs_update == DRAW) {
      needs_update = NONE;
      current_screen->init();
      current_screen->draw();
      last_update_millis = millis();
    }
  }
}



void Screen::processKey(char c)
{
  if (current_screen) {
    if (!current_screen->handleKey(c)) {
      // Some keys may need to work in all, or most screens.
      current_screen->Screen::handleKey(c);
    }
  }
}



void Screen::setNeedsDraw()
{
  needs_update = DRAW;
}



void Screen::setNeedsRedraw()
{
  needs_update = REDRAW;
}




void IntegerEntryScreen::init()
{
  value = 0;
}



uint16_t IntegerEntryScreen::getValue()
{
  return value;
}



void IntegerEntryScreen::setValue(int16_t val)
{
  value = val;
}



void IntegerEntryScreen::draw()
{
  SCREEN_SET_POS(3,1);
  SCREEN_WRITE_P(PSTR("<Enter> when done.\n"));
  SCREEN_WRITE_P(PSTR("<Del> to backspace."));
  redraw();
}



void IntegerEntryScreen::redraw()
{
  char buf[10];
  buf[0] = '\0';
  concat_decimal(buf, sizeof(buf), value, 0, 0, 0);
  strcat_P(buf, PSTR("  \b\b"));
  SCREEN_SET_POS(2,1);
  SCREEN_WRITE(buf);
  SCREEN_CURSOR_ON();
}



uint8_t IntegerEntryScreen::handleKey(char c)
{
  if (c >= '0' && c <= '9') {
    value *= 10;
    value += c - '0';
    Screen::setNeedsRedraw();
    return 1;
  }
  if (c == KEY_DELETE) {
    value /= 10;
    Screen::setNeedsRedraw();
    return 1;
  }
  if (c == KEY_MENU || c == KEY_TEMPS) {
    SCREEN_CURSOR_OFF();
    this->cancel();
    return 1;
  }
  if (c == KEY_ENTER) {
    SCREEN_CURSOR_OFF();
    this->commit();
    return 1;
  }
  return 0;
}



void IntegerEntryScreen::commit()
{
}



void IntegerEntryScreen::cancel()
{
}





class ZeroMenuScreen: 
public Screen {
  virtual void draw() {
    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("1)Zero X   5)Home X "));
    SCREEN_WRITE_P(PSTR("2)Zero Y   6)Home Y "));
    SCREEN_WRITE_P(PSTR("3)Zero Z   7)Home Z "));
    SCREEN_WRITE_P(PSTR("4)Zero All 8)Home XY"));
  }

  virtual uint8_t handleKey(char c) {
    LongPoint currpos = get_position();
    if (c >= '1' && c <= '8') {
      if (is_playing()) { 
        return 0;
      }
    }
    switch (c) {
    case KEY_ZERO:
      Screen::change((Screen*)&default_screen);
      return 1;
    case '1':
      set_position(LongPoint(0,currpos.y,currpos.z));
      Screen::change((Screen*)&default_screen);
      return 1;
    case '2':
      set_position(LongPoint(currpos.x,0,currpos.z));
      Screen::change((Screen*)&default_screen);
      return 1;
    case '3':
      set_position(LongPoint(currpos.x,currpos.y,0));
      Screen::change((Screen*)&default_screen);
      return 1;
    case '4':
      set_position(LongPoint(0,0,0));
      Screen::change((Screen*)&default_screen);
      return 1;
    case '5':
      SCREEN_CLEAR();
      SCREEN_WRITE_P(PSTR("\nSeeking X Axis Home"));
      seek_minimums(1, 0, 0, 10000, 15);
      Screen::change((Screen*)&default_screen);
      return 1;
    case '6':
      SCREEN_CLEAR();
      SCREEN_WRITE_P(PSTR("\nSeeking Y Axis Home"));
      seek_minimums(0, 1, 0, 10000, 15);
      Screen::change((Screen*)&default_screen);
      return 1;
    case '7':
      SCREEN_CLEAR();
      SCREEN_WRITE_P(PSTR("\nSeeking Z Axis Home "));
      seek_minimums(0, 0, 1, 882, 60);
      Screen::change((Screen*)&default_screen);
      return 1;
    case '8':
      SCREEN_CLEAR();
      SCREEN_WRITE_P(PSTR("\nSeeking XY Home"));
      seek_minimums(1, 1, 0, 10000, 15);
      Screen::change((Screen*)&default_screen);
      return 1;
    case KEY_MENU:
      Screen::change((Screen*)&default_screen);
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
zero_menu_screen;




class HeadTempScreen: 
public IntegerEntryScreen {
public:
  virtual void init() {
    uint16_t val = getEEPROMHeadTemp();
    if (val == 0) {
        val = 220;
    }
    this->setValue(val);
  }

  virtual void draw() {
    SCREEN_CLEAR();
    SCREEN_WRITE_P(PSTR("Extruder Target Temp"));
    IntegerEntryScreen::draw();
  }

  virtual void cancel() {
    Screen::change((Screen*)&temp_menu_screen);
  }

  virtual void commit() {
    setEEPROMHeadTemp(this->getValue());
    Screen::change((Screen*)&default_screen);
  }
} 
head_temp_screen;




class PlatformTempScreen: 
public IntegerEntryScreen {
public:
  virtual void init() {
    uint16_t val = getEEPROMPlatformTemp();
    if (val == 0) {
        val = 60;
    }
    this->setValue(val);
  }

  virtual void draw() {
    SCREEN_CLEAR();
    SCREEN_WRITE_P(PSTR("Platform Target Temp"));
    IntegerEntryScreen::draw();
  }

  virtual void cancel() {
    Screen::change((Screen*)&temp_menu_screen);
  }

  virtual void commit() {
    setEEPROMPlatformTemp(this->getValue());
    Screen::change((Screen*)&default_screen);
  }
} 
platform_temp_screen;



class TempMenuScreen: 
public Screen {
  virtual void draw() {
    uint16_t head_targ_temp = get_target_head_temp();
    uint16_t plat_targ_temp = get_target_platform_temp();
    uint16_t htemp = getEEPROMHeadTemp();
    uint16_t ptemp = getEEPROMPlatformTemp();
    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("1) Extruder Heat "));
    if (head_targ_temp == htemp) {
      SCREEN_WRITE_P(PSTR("Off"));
    } else {
      SCREEN_WRITE_P(PSTR("On "));
    }
    SCREEN_WRITE_P(PSTR("2) Platform Heat "));
    if (plat_targ_temp == ptemp) {
      SCREEN_WRITE_P(PSTR("Off"));
    } else {
      SCREEN_WRITE_P(PSTR("On "));
    }
    SCREEN_WRITE_P(PSTR("3) Extruder Targ Tmp"));
    SCREEN_WRITE_P(PSTR("4) Platform Targ Tmp"));
  }

  virtual uint8_t handleKey(char c) {
    uint16_t head_targ_temp = get_target_head_temp();
    uint16_t plat_targ_temp = get_target_platform_temp();
    int16_t temp;
    switch (c) {
    case '1':
      temp = getEEPROMHeadTemp();
      if (head_targ_temp != temp) {
        head_targ_temp = temp;
      } else {
        head_targ_temp = 0;
      }
      send_tool_simple_command_with_word(0, SLAVE_CMD_SET_TEMP, head_targ_temp);
      Screen::change((Screen*)&default_screen);
      return 1;
    case '2':
      temp = getEEPROMPlatformTemp();
      if (plat_targ_temp != temp) {
        plat_targ_temp = temp;
      } else {
        plat_targ_temp = 0;
      }
      send_tool_simple_command_with_word(0, SLAVE_CMD_SET_PLATFORM_TEMP, plat_targ_temp);
      Screen::change((Screen*)&default_screen);
      return 1;
    case '3':
      Screen::change((Screen*)&head_temp_screen);
      return 1;
    case '4':
      Screen::change((Screen*)&platform_temp_screen);
      return 1;
    case KEY_MENU:
      Screen::change((Screen*)&default_screen);
      return 1;
    case KEY_TEMPS:
      Screen::change((Screen*)&default_screen);
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
temp_menu_screen;




class FileSelectScreen: 
public Screen {
private:
  int8_t selected;
  int8_t offset;

public:
  virtual void init() {
    selected = 0;
    offset = 0;
  }


  virtual void draw() {
    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("Select a File:   "));
    SCREEN_WRITECHAR(SCREEN_UP_ARROW_CHAR);
    SCREEN_WRITECHAR(SCREEN_DOWN_ARROW_CHAR);
    SCREEN_WRITECHAR('\n');
    SCREEN_WAIT_FOR_LCD(65);
    redraw();
  }


  uint8_t getNextFile(char* fnbuf, uint8_t len) {
    while(1) {
      uint8_t rspCode = sd_scan_next(fnbuf,len);
      if (rspCode != 0 || !fnbuf[0]) {
        fnbuf[0] = '\0';
        return 0;
      }
      fnbuf[len-1] = '\0';
      if (fnbuf[0] == '.') {
        continue;
      }
      if (strendswith_P(fnbuf, PSTR(".s3g"))) {
        break;
      }
      if (strendswith_P(fnbuf, PSTR(".gcode"))) {
        break;
      }
    }
    return 1;
  }


  virtual void redraw() {
    char fnbuf[MAX_FILENAME_SIZE];
    uint8_t i, index;
    uint8_t rspCode = sd_scan_reset();
    SCREEN_SET_POS(2,1);
    if (rspCode != 0) {
      SCREEN_WRITE_P(PSTR("Can't Read SD Card"));
      return;
    }
    if (selected < 0) {
      selected = 0;
    }
    if (selected > offset+2) {
      if (selected < 2) {
        offset = 0;
      } 
      else {
        offset = selected - 2;
      }
    }
    if (selected < offset) {
      offset = selected;
    }
    index = 0;
    for (i = 0; i < offset; i++) {
      if (!getNextFile(fnbuf, sizeof(fnbuf))) {
        if (selected > index || offset > index) {
          selected = index;
          offset = index;
          Screen::setNeedsRedraw();
        }
        return;
      }
      index++;
    }
    for (i = 1; i <= 3; i++) {
      if (!getNextFile(fnbuf, sizeof(fnbuf))) {
        if (index <= selected) {
          selected = index-1;
          SCREEN_SET_POS(i,1);
          SCREEN_WRITECHAR(SCREEN_RIGHT_ARROW_CHAR);
          SCREEN_SET_POS(i+1,1);
        }
        if (selected < offset) {
          offset = selected;
          Screen::setNeedsRedraw();
        }
        break;
      }
      if (index == selected) {
        SCREEN_WRITECHAR(SCREEN_RIGHT_ARROW_CHAR);
      } 
      else {
        SCREEN_WRITECHAR(' ');
      }

      fnbuf[19] = '\0';
      uint8_t len = strlen(fnbuf);
      char* ptr = fnbuf+len;
      for (; len < 19; len++) {
        *ptr++ = ' ';
      }
      *ptr = '\0';
      SCREEN_WRITE(fnbuf);
      index++;
    }
    for (; i <= 3; i++) {
      SCREEN_WRITE_P(PSTR("                    "));
    }
  }


  void selectFile() {
    char fnbuf[MAX_FILENAME_SIZE];
    uint8_t i, index;
    uint8_t rspCode = sd_scan_reset();
    if (rspCode != 0) {
      return;
    }
    index = 0;
    for (i = 0; i <= selected; i++) {
      if (!getNextFile(fnbuf, sizeof(fnbuf))) {
        selected = index;
        offset = index;
        Screen::setNeedsRedraw();
        return;
      }
      index++;
    }
    if (index > 0) {
      start_playback(fnbuf);
    }
  }


  virtual uint8_t handleKey(char c) {
    switch (c) {
    case KEY_ZMINUS:
      selected++;
      Screen::setNeedsRedraw();
      return 1;
    case KEY_ZPLUS:
      selected--;
      Screen::setNeedsRedraw();
      return 1;
    case KEY_MENU:
      Screen::change((Screen*)&main_menu_screen);
      return 1;
    case KEY_ENTER:
      if (is_playing()) { 
        return 0;
      }
      selectFile();
      Screen::change((Screen*)&default_screen);
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
file_select_screen;



class LcdContrastScreen: 
public Screen {
private:
  uint8_t contrastval;
  uint8_t oldcontrastval;

public:
  LcdContrastScreen() {
  }

  virtual void init() {
    oldcontrastval = contrastval = getEEPROMLcdContrast();
    SCREEN_SET_CONTRAST(contrastval);
  }


  virtual void draw() {
    char buf[5];
    buf[0] = SCREEN_UP_ARROW_CHAR;
    buf[1] = SCREEN_DOWN_ARROW_CHAR;
    buf[2] = '\n';
    buf[3] = '\0';

    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("Adjust Contrast: "));
    SCREEN_WRITE(buf);
    SCREEN_WRITE_P(PSTR("\n<Enter> to save"));
    SCREEN_WRITE_P(PSTR("\n<Menu> to cancel."));
  }


  virtual void redraw() {
  }


  virtual uint8_t handleKey(char c) {
    switch (c) {
    case KEY_ZMINUS:
      if (contrastval > 204) {
        contrastval -= 4;
      } else {
        contrastval = 200;
      }
      SCREEN_WAIT_FOR_LCD(10);
      SCREEN_SET_CONTRAST(contrastval);
      Screen::setNeedsRedraw();
      return 1;
    case KEY_ZPLUS:
      if (contrastval < 250) {
        contrastval += 4;
      } else {
        contrastval = 254;
      }
      SCREEN_WAIT_FOR_LCD(10);
      SCREEN_SET_CONTRAST(contrastval);
      Screen::setNeedsRedraw();
      return 1;
    case KEY_ENTER:
      EEPROM.write(EEPROM_LCD_CONTRAST_OFFSET, contrastval);
      Screen::change((Screen*)&default_screen);
      return 1;
    case KEY_MENU:
      contrastval = oldcontrastval;
      SCREEN_SET_CONTRAST(contrastval);
      Screen::change((Screen*)&main_menu_screen);
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
lcd_contrast_screen;



class MainMenuScreen: 
public Screen {
  virtual void draw() {
    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("1) Build File\n"));
    SCREEN_WRITE_P(PSTR("2) Change Contrast\n"));
    SCREEN_WRITE_P(PSTR("3) Init LCD Firmware"));
  }

  virtual uint8_t handleKey(char c) {
    switch (c) {
    case '1':
      if (is_playing()) {
        return 0;
      }
      Screen::change((Screen*)&file_select_screen);
      return 1;
    case '2':
      Screen::change((Screen*)&lcd_contrast_screen);
      return 1;
    case '3':
      lcd_init_or_flash(true);
      Screen::change((Screen*)&default_screen);
      return 1;
    case KEY_MENU:
      Screen::change((Screen*)&default_screen);
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
main_menu_screen;



class HeadWarmupScreen: public Screen {
private:
  int8_t extruder_dir;

public:
  void setExtruderDir(int8_t val) {
      extruder_dir = val;
  }

  virtual void init() {
  }


  virtual uint8_t isUpdateNeeded() {
    if (millis() - last_update_millis > 500) {
      return 1;
    }
    return 0;
  }


  virtual void draw() {
    SCREEN_CLEAR();
    SCREEN_CURSOR_OFF();
    SCREEN_WRITE_P(PSTR("Warming Up Extruder."));
    SCREEN_WRITE_P(PSTR("<Menu> to cancel.   \n"));
  }

  virtual void redraw() {
    char buf[80];
    char chbuf[3];
    uint16_t head_temp = get_last_head_temp();
    uint16_t head_targ_temp = get_target_head_temp();
    uint16_t eeprom_head_targ_temp = getEEPROMHeadTemp();
    if (head_targ_temp < eeprom_head_targ_temp) {
      head_targ_temp = eeprom_head_targ_temp;
      send_tool_simple_command_with_word(0, SLAVE_CMD_SET_TEMP, head_targ_temp);
    }
    if (head_temp >= head_targ_temp - 2) {
      if (extruder_dir > 0) {
        send_tool_simple_command_with_byte(0, SLAVE_CMD_SET_MOTOR_1_PWM, 0xff);
        send_tool_simple_command_with_byte(0, SLAVE_CMD_TOGGLE_MOTOR_1, 0x3);
      } else if (extruder_dir < 0) {
        send_tool_simple_command_with_byte(0, SLAVE_CMD_SET_MOTOR_1_PWM, 0xff);
        send_tool_simple_command_with_byte(0, SLAVE_CMD_TOGGLE_MOTOR_1, 0x1);
      }
      Screen::change((Screen*)&default_screen);
      return;
    }

    SCREEN_SET_POS(4,1);
    buf[0] = '\0';
    strcat_P(buf,PSTR(" Temp:"));
    concat_decimal(buf, sizeof(buf), head_temp, 3, 0);
    strcat_P(buf,PSTR("C "));
    chbuf[0] = SCREEN_RIGHT_ARROW_CHAR;
    chbuf[1] = ' ';
    chbuf[2] = '\0';
    strcat(buf,chbuf);
    concat_decimal(buf, sizeof(buf), head_targ_temp, 3, 0);
    strcat_P(buf,PSTR("C "));
    SCREEN_WRITE(buf);
  }

  virtual uint8_t handleKey(char c) {
    switch (c) {
    case KEY_TEMPS:
      return 1;
    case KEY_ZERO:
      return 1;
    case KEY_UNITS:
      return 1;
    case KEY_MENU:
      Screen::change((Screen*)&default_screen);
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
head_warmup_screen;



class DefaultScreen: 
public Screen {
private:
  static int8_t jog_units;
  static long pos_target_x;
  static long pos_target_y;
  static long pos_target_z;

public:
  DefaultScreen() {
    Screen::change((Screen*)this);
  }


  virtual void init() {
  }


  virtual uint8_t isUpdateNeeded() {
    if (millis() - last_update_millis > 250) {
      return 1;
    }
    return 0;
  }


  virtual void draw() {
    SCREEN_CURSOR_OFF();
  }


  virtual void redraw() {
    char buf[82];
    char chbuf[2];
    uint32_t now = millis();
    LongPoint currpos = get_position();

    int32_t xmm, ymm, zmm;
    xmm = ((((uint32_t)currpos.x)*((uint32_t)((1<<9)*10.0/STEPS_PER_MM_XY))>>8) + 1) >> 1;
    ymm = ((((uint32_t)currpos.y)*((uint32_t)((1<<9)*10.0/STEPS_PER_MM_XY))>>8) + 1) >> 1;
    zmm = ((((uint32_t)currpos.z)*((uint32_t)((1<<12)*10.0/STEPS_PER_MM_Z))>>11) + 1) >> 1;
    uint16_t head_temp = get_last_head_temp();
    uint16_t plat_temp = get_last_platform_temp();
    uint16_t head_targ_temp = get_target_head_temp();
    uint16_t plat_targ_temp = get_target_platform_temp();
    int8_t extdir = get_extruder_dir();

    if (is_playing()) {
      if (!playback_has_next()) {
        finish_playback();
      }

      int8_t pcnt = playback_percent_done();
      int16_t secs = playback_seconds_elapsed();
      int16_t mins = secs / 60;
      int16_t hrs = mins / 60;
      secs %= 60;
      mins %= 60;

      // calculate visible space for filename.
      int8_t trimback = 20;
      trimback -= 1; // play/pause char.
      if (hrs > 0) {
        trimback -= 3; // Hours
      }
      trimback -= 3; // Minutes
      trimback -= 3; // Seconds
      trimback -= 4; // Percent

      const char* filename = get_playback_filename();
      int8_t fnlen = strlen(filename);

      // If filename is larger than the visible space, scroll it.
      if (fnlen > trimback) {

        // Advance about one char/sec
        int8_t scrolloff = ((now >> 10) % ((fnlen+1-trimback)+2)) - 2;

        if (scrolloff < 0) {
          // Pause for a couple beats at the start of the name.
          scrolloff = 0;
        }
        while (scrolloff-->0)
          filename++;
      }

      // Playback Filename.
      strncpy(buf,filename,trimback);
      strcat_P(buf,PSTR("          "));
      buf[trimback] = '\0';

      // Playback % complete.
      concat_decimal(buf, sizeof(buf), pcnt, 3, 0);
      strcat_P(buf,PSTR("% "));

      // Playback time elapsed.
      if (hrs > 0) {
        concat_decimal(buf, sizeof(buf), hrs,  2, 0);
        strcat_P(buf,PSTR(":"));
      }
      concat_decimal(buf, sizeof(buf), mins, 2, 0, (hrs>0));
      strcat_P(buf,PSTR(":"));
      concat_decimal(buf, sizeof(buf), secs, 2, 0, 1);
      strcat_P(buf,PSTR("  "));

      buf[19] = SCREEN_PLAY_CHAR;
    } 
    else {
      strcpy_P(buf,PSTR("Ready...            "));
    }
    if (is_machine_paused) {
      // Blink pause icon.
      if (((now >> 9) & 0x1) == 0) {
        buf[19] = SCREEN_PAUSE_CHAR;
      } else {
        buf[19] = ' ';
      }
    }
    if (extdir < 0) {
      buf[19] = SCREEN_EXTREV_CHAR; 
    }
    buf[20] = '\0';
    SCREEN_SET_POS(1,1);
    SCREEN_WRITE(buf);

    strcpy_P(buf,PSTR("X:"));
    concat_decimal(buf, sizeof(buf), xmm, 5, 1);
    strcat_P(buf,PSTR("   "));
    buf[9] = '\0';
    strcat_P(buf,PSTR("T:"));
    concat_decimal(buf, sizeof(buf), head_temp, 3, 0);
    chbuf[0] = SCREEN_RIGHT_ARROW_CHAR;
    chbuf[1] = '\0';
    strcat(buf,chbuf);
    concat_decimal(buf, sizeof(buf), head_targ_temp, 3, 0);
    strcat_P(buf,PSTR("C "));
    if (extdir > 0) {
      buf[19] = SCREEN_EXTFWD_CHAR; 
    } else {
      buf[19] = ' '; 
    }
    buf[20] = '\0';
    SCREEN_WRITE(buf);
    SCREEN_WAIT_FOR_LCD(50);

    strcpy_P(buf,PSTR("Y:"));
    concat_decimal(buf, sizeof(buf), ymm, 5, 1);
    strcat_P(buf,PSTR("   "));
    buf[9] = '\0';
    strcat_P(buf,PSTR("P:"));
    concat_decimal(buf, sizeof(buf), plat_temp, 3, 0);
    chbuf[0] = SCREEN_RIGHT_ARROW_CHAR;
    chbuf[1] = '\0';
    strcat(buf,chbuf);
    concat_decimal(buf, sizeof(buf), plat_targ_temp, 3, 0);
    strcat_P(buf,PSTR("C "));
    buf[20] = '\0';
    SCREEN_WRITE(buf);

    strcpy_P(buf,PSTR("Z:"));
    concat_decimal(buf, sizeof(buf), zmm, 5, 1);
    strcat_P(buf,PSTR("   "));
    buf[9] = '\0';
    strcat_P(buf,PSTR("Jog:"));
    strcat_P(buf,jog_length_txt[jog_units]);
    strcat_P(buf,PSTR("   "));
    buf[20] = '\0';
    SCREEN_WRITE(buf);
  }


  virtual uint8_t handleKey(char c) {
    if (c >= '1' && c <= '9') {
      if (is_playing()) { 
        return 0;
      }
      if (c != '3' && c !='5' && c != '7') {
        if (is_point_buffer_empty()) {
          LongPoint currpos = get_position();
          pos_target_x = currpos.x;
          pos_target_y = currpos.y;
          pos_target_z = currpos.z;
        }
      }
    }
    LongPoint currpos = get_position();
    switch (c) {
    case KEY_YMINUS:
      // Y-
      pos_target_y -= jog_length_xy[jog_units];
      point_buffer_clear();
      queue_absolute_point(pos_target_x, pos_target_y, pos_target_z, JOG_STEP_DELAY_XY);
      return 1;
    case KEY_ZMINUS:
      // Z-
      pos_target_z -= jog_length_z[jog_units];
      point_buffer_clear();
      queue_absolute_point(pos_target_x, pos_target_y, pos_target_z, JOG_STEP_DELAY_Z);
      return 1;
    case KEY_EPLUS:
      // Extruder Forwards
      head_warmup_screen.setExtruderDir(1);
      Screen::change((Screen*)&head_warmup_screen);
      return 1;
    case KEY_XMINUS:
      // X-
      pos_target_x -= jog_length_xy[jog_units];
      point_buffer_clear();
      queue_absolute_point(pos_target_x, pos_target_y, pos_target_z, JOG_STEP_DELAY_XY);
      return 1;
    case KEY_ESTOP:
      // Extruder Stop
      send_tool_simple_command_with_byte(0, SLAVE_CMD_SET_MOTOR_1_PWM, 0x0);
      send_tool_simple_command_with_byte(0, SLAVE_CMD_TOGGLE_MOTOR_1, 0x0);
      Screen::setNeedsRedraw();
      return 1;
    case KEY_XPLUS:
      // X+
      pos_target_x += jog_length_xy[jog_units];
      point_buffer_clear();
      queue_absolute_point(pos_target_x, pos_target_y, pos_target_z, JOG_STEP_DELAY_XY);
      return 1;
    case KEY_EMINUS:
      // Extruder reverse
      head_warmup_screen.setExtruderDir(-1);
      Screen::change((Screen*)&head_warmup_screen);
      return 1;
    case KEY_ZPLUS:
      // Z+
      pos_target_z += jog_length_z[jog_units];
      point_buffer_clear();
      queue_absolute_point(pos_target_x, pos_target_y, pos_target_z, JOG_STEP_DELAY_Z);
      return 1;
    case KEY_YPLUS:
      // Y+
      pos_target_y += jog_length_xy[jog_units];
      point_buffer_clear();
      queue_absolute_point(pos_target_x, pos_target_y, pos_target_z, JOG_STEP_DELAY_XY);
      return 1;
    case KEY_ZERO:
      // Zero
      Screen::change((Screen*)&zero_menu_screen);
      return 1;
    case KEY_UNITS:
      // Units.  Rotate through jog sizes
      if (jog_units == 0) {
        jog_units = sizeof(jog_length_xy)-1;
      } 
      else {
        jog_units--;
      }
      Screen::setNeedsRedraw();
      return 1;
    case KEY_TEMPS:
      // Temp
      Screen::change((Screen*)&temp_menu_screen);
      return 1;
    case KEY_MENU:
      // Menu
      Screen::change((Screen*)&main_menu_screen);
      return 1;
    case KEY_ENTER:
      // Enter.  Do nothing.
      return 1;
    default:
      break;
    }
    return 0;
  }
} 
default_screen;

int8_t DefaultScreen::jog_units = 3;
int32_t DefaultScreen::pos_target_x = 0;
int32_t DefaultScreen::pos_target_y = 0;
int32_t DefaultScreen::pos_target_z = 0;



// vim: set sw=2 autoindent nowrap expandtab: settings


