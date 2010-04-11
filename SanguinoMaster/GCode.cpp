#include <ctype.h>
#include "Configuration.h"
#include <HardwareSerial.h>
#include <EEPROM.h>
#include "SDSupport.h"
#include "Tools.h"
#include <avr/wdt.h>
#include "RS485.h"
#include "WProgram.h"
#include "Commands.h"
#include "Variables.h"
#include "Version.h"
#include "Steppers.h"
#include "Utils.h"

typedef enum {RAPID, LINEAR, ARC_CW, ARC_CCW} motion_t;
typedef enum {ABSOLUTE, RELATIVE} positioning_t;
typedef enum {INCHES,MILLIMETERS} units_t;

extern CircularBuffer commandBuffer;
char gcode_buffer[GCODE_BUFFER_SIZE];
size_t gcode_chars_read = 0;
bool gcode_line_complete = false;
bool gcode_line_incomment = false;

uint8_t gcode_line_saw = 0;
const uint8_t GCODE_SAW_MOTION       = 0x01;
const uint8_t GCODE_SAW_PAUSE        = 0x02;
const uint8_t GCODE_SAW_MCODE        = 0x04;

motion_t      gcode_curr_motion = RAPID;
positioning_t gcode_curr_positioning = ABSOLUTE;
units_t       gcode_curr_units = MILLIMETERS;

int32_t gcode_prev_x = 0;
int32_t gcode_prev_y = 0;
int32_t gcode_prev_z = 0;
int32_t gcode_curr_x = 0;
int32_t gcode_curr_y = 0;
int32_t gcode_curr_z = 0;
long    gcode_curr_f = 2000;
uint8_t gcode_curr_m = 0;
long    gcode_curr_s = 0;


// parse an integer value from the given string, discarding any trailing decimal value.
static int32_t strtointeger(const char * nptr, char ** endptr)
{
    int32_t val = 0;
    bool isneg = false;
    while (*nptr == '-') {
      isneg = !isneg;
      nptr++;
    }
    while (isdigit(*nptr)) {
        val *= 10;
	val += (*nptr++ - '0');
    }
    if (*nptr == '.') {
	nptr++;
    }
    while (isdigit(*nptr)) {
	nptr++;
    }
    if (isneg) {
        val = -val;
    }
    *endptr = (char*)nptr;
    return val;
}



// parse a signed fixed-point 23.8 value from the given string.
static int32_t strtofixed(const char * nptr, char ** endptr)
{
    int32_t val = 0;
    uint16_t mod = 1;
    bool isneg = false;
    while (*nptr == '-') {
      isneg = !isneg;
      nptr++;
    }
    while (isdigit(*nptr)) {
        val *= 10;
	val += (*nptr++ - '0');
    }
    if (*nptr == '.') {
	nptr++;
    }
    while (isdigit(*nptr) && mod < 10000) {
	mod *= 10;
        val *= 10;
	val += (*nptr++ - '0');
    }
    while (isdigit(*nptr)) {
	nptr++;
    }
    if (isneg) {
        val = -val;
    }
    *endptr = (char*)nptr;
    return ((val<<8)/mod);
}



//translate a line from a .gcode file into internal .s3g commands.
void translate_gcode()
{
  if ((gcode_line_saw & GCODE_SAW_MCODE)) {
    if (commandBuffer.remainingCapacity() >= 6) {
      switch (gcode_curr_m) {
	case 101: // Extruder on, Forwards
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_TOGGLE_MOTOR_1);
	  commandBuffer.append(1);  // data length
	  commandBuffer.append(0x3); // Motor on, forwards.
	  break;

	case 102: // Extruder on, Reverse
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_TOGGLE_MOTOR_1);
	  commandBuffer.append(1);  // data length
	  commandBuffer.append(0x1); // Motor on, forwards.
	  break;

	case 103: // Extruder off.
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_TOGGLE_MOTOR_1);
	  commandBuffer.append(1);  // data length
	  commandBuffer.append(0x0); // Motor off
	  break;

	case 104: // Set temperature in celcius
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_SET_TEMP);
	  commandBuffer.append(2);  // data length
	  commandBuffer.append_16(gcode_curr_s); // temperature
	  break;

	case 105: // Print temperature
	  // Do nothing.
	  break;

	case 106: // Turn fan on
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_TOGGLE_FAN);
	  commandBuffer.append(2);  // data length
	  commandBuffer.append(0x1); // fan on
	  break;

	case 107: // Turn fan off
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_TOGGLE_FAN);
	  commandBuffer.append(2);  // data length
	  commandBuffer.append(0x0); // fan off
	  break;

	case 108: // Set extruder speed, PWM
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_SET_MOTOR_1_PWM);
	  commandBuffer.append(1);  // data length
	  commandBuffer.append(gcode_curr_s&0xff);
	  break;

	case 110: // Set platform temperature in celcius
	  commandBuffer.append(HOST_CMD_TOOL_COMMAND);
	  commandBuffer.append(0);  // Toolnum
	  commandBuffer.append(SLAVE_CMD_SET_PLATFORM_TEMP);
	  commandBuffer.append(2);  // data length
	  commandBuffer.append_16(gcode_curr_s); // temperature
	  break;

	case 111: // Set chamber temperature in celcius
	  // Do nothing yet.
	  break;
      }
      gcode_line_saw &= ~GCODE_SAW_MCODE;
    }
  }
  if ((gcode_line_saw & GCODE_SAW_PAUSE)) {
    if (commandBuffer.remainingCapacity() > 3) {
      commandBuffer.append(HOST_CMD_DELAY);
      commandBuffer.append_32(1000*gcode_curr_s);
      gcode_line_saw &= ~GCODE_SAW_PAUSE;
    }
  }
  if ((gcode_line_saw & GCODE_SAW_MOTION)) {
    uint32_t feedrate = gcode_curr_f;
    switch (gcode_curr_motion) {
      case RAPID:
	feedrate = (3*max(MAX_MM_PER_MIN_XY,MAX_MM_PER_MIN_Z))>>2;
	// Fall through vvv
      case LINEAR:
	if (commandBuffer.remainingCapacity() > 17) {
	  // Benchmark results in cycles:
	  // size      add   mult   div
	  // --------  ----  -----  -------
	  // float:    156    141   510
	  // int32_t:   24     67   658(!?)
	  // int16_t:   14     22   247

	  // Worst case speed calculation performs 1 square root,
	  // 4 divs, 12 mults and 5 add/subs.  Most times, we should
	  // only perform one or two divs.
	  // This speed calculation can take over 4000 cycles!
	  // Luckily, that's still just a quarter of a millisecond.
	  // With the command queue and point queue it's unlikely
	  // we'll stall out due to calculation overhead.

	  // If we used floats instead of fixedpoint math, this calculation
	  // would get up to around 5500 cycles.  Mostly, though, we want to
	  // avoid pulling in the overhead of the floating point library.

	  if (feedrate == 0) {
	    feedrate = 1;
	  }

	  uint32_t dx = labs(gcode_curr_x - gcode_prev_x);
	  uint32_t dy = labs(gcode_curr_y - gcode_prev_y);
	  uint32_t dz = labs(gcode_curr_z - gcode_prev_z);
	  // dx,dy,dz are in unsigned 24.8 fixed point format

	  // We only care about general proportions, so we can scale down large values for later calculation convenience.
	  while (dx >= (1l<<16) || dy >= (1l<<16) || dz >= (1l<<16)) {
	      dx >>= 1;
	      dy >>= 1;
	      dz >>= 1;
	  }

	  uint32_t steptime;

	  if (dz == 0 && (dx == 0 || dy == 0 || dx+dy < (1<<8))) {
	    // Motion is either single XY axis, or is less than a millimeter and thus too short to worry about speed precision.
	    if (feedrate > MAX_MM_PER_MIN_XY) {
	      feedrate = MAX_MM_PER_MIN_XY;
	    }
	    steptime = 60l * 1000000l / (( feedrate * (uint32_t)(STEPS_PER_MM_XY*(1l<<8)) )>>8);

	  } else if (dx == 0 && dy == 0) {
	    // Motion is Z axis only.
	    if (feedrate > MAX_MM_PER_MIN_Z) {
	      feedrate = MAX_MM_PER_MIN_Z;
	    }
	    steptime = 60l * 1000000l / (( feedrate * (uint32_t)(STEPS_PER_MM_Z *(1l<<8)) )>>8);

	  } else {
	    // Can't avoid this.  Square Root is just slow.
	    uint32_t dist = isqrt( (dx*dx) + (dy*dy) + (dz*dz) );
	    // dist is in unsigned 24.8 fixed point format

	    if (dist == 0) {
	      dist = 1;
	    }
	    uint32_t invdist = (1l<<24)/dist;
	    if (invdist == 0) {
		invdist = 1;
	    }
	    // invdist is in unsigned 16.16 fixed point format

	    // Get proportions of each axis to the move distance.
	    uint32_t part_x = (dx*invdist)>>8;
	    uint32_t part_y = (dy*invdist)>>8;
	    uint32_t part_z = (dz*invdist)>>8;
	    // scales are in unsigned 16.16 fixed point format

	    if (part_x == 0 && part_y == 0 && part_z == 0) {
	      // Cheat slightly on rounding errors and zero lengths.
	      part_x = 1l<<16;
	    }

	    // Scale back feedrate to fit in max speeds.  Check Z first since it's most likely to exceed.
	    if ((feedrate * part_z)>>16 > MAX_MM_PER_MIN_Z ) {
	      feedrate = (MAX_MM_PER_MIN_Z *(1l<<16)) / part_z;
	    }
	    if ((feedrate * part_y)>>16 > MAX_MM_PER_MIN_XY) {
	      feedrate = (MAX_MM_PER_MIN_XY*(1l<<16)) / part_y;
	    }
	    if ((feedrate * part_x)>>16 > MAX_MM_PER_MIN_XY) {
	      feedrate = (MAX_MM_PER_MIN_XY*(1l<<16)) / part_x;
	    }

	    // Calculate steps/min for each axis.
	    uint32_t stepratex = ( ((feedrate * part_x)>>16) * (uint32_t)(STEPS_PER_MM_XY*(1l<<12)) )>>12;
	    uint32_t stepratey = ( ((feedrate * part_y)>>16) * (uint32_t)(STEPS_PER_MM_XY*(1l<<12)) )>>12;
	    uint32_t stepratez = ( ((feedrate * part_z)>>16) * (uint32_t)(STEPS_PER_MM_Z *(1l<< 8)) )>>8;

	    // Use the fastest axis step-rate to calculate usecs/step.
	    steptime = 60l * 1000000l / max(1,max(stepratex,max(stepratey,stepratez)));
	  }

	  if (steptime == 0) {
	    steptime = 1;
	  }

	  commandBuffer.append(HOST_CMD_QUEUE_POINT_ABS);
	  commandBuffer.append_32(((gcode_curr_x>>4)*(int32_t)(STEPS_PER_MM_XY*(1l<<10)))>>14);
	  commandBuffer.append_32(((gcode_curr_y>>4)*(int32_t)(STEPS_PER_MM_XY*(1l<<10)))>>14);
	  commandBuffer.append_32(((gcode_curr_z>>4)*(int32_t)(STEPS_PER_MM_Z *(1l<<8)))>>12);
	  commandBuffer.append_32(steptime);

	  gcode_prev_x = gcode_curr_x;
	  gcode_prev_y = gcode_curr_y;
	  gcode_prev_z = gcode_curr_z;
	  gcode_line_saw &= ~GCODE_SAW_MOTION;
	}
	break;

      case ARC_CW:
	// We don't support this yet, but STL files don't generally slice into arcs, anyways.
	break;

      case ARC_CCW:
	// We don't support this yet, but STL files don't generally slice into arcs, anyways.
	break;
    }
  }
  if (gcode_line_saw == 0) {
    gcode_line_complete = false;
  }
}


//decode a line from a .gcode file.
void decode_gcode()
{
  char *ptr = gcode_buffer;
  long  line_f = gcode_curr_f;
  long  line_m = gcode_curr_m;
  long  line_s = gcode_curr_s;
  int32_t line_x = 0;
  int32_t line_y = 0;
  int32_t line_z = 0;

  //Serial.println(ptr);
  gcode_line_complete = true;
  gcode_line_saw = 0;
  while (ptr && *ptr) {
    switch(*ptr) {
      case 'F':  // Feed Rate
	line_f = strtointeger(++ptr, &ptr);
        break;
      case 'G':  // G-Code
	switch (strtointeger(++ptr, &ptr)) {
	  case 0:  // Rapid motion
	    gcode_curr_motion = RAPID;
	    break;
	  case 1:  // Linear Motion
	    gcode_curr_motion = LINEAR;
	    break;
	  case 2:  // Clockwise Arc
	    gcode_curr_motion = ARC_CW;
	    break;
	  case 3:  // Counter-Clockwise Arc
	    gcode_curr_motion = ARC_CCW;
	    break;
	  case 20: // Units Inches
	    gcode_curr_units = INCHES;
	    break;
	  case 21: // Units Millimeters
	    gcode_curr_units = MILLIMETERS;
	    break;
	  case 90: // Absolute coordinates.
	    gcode_curr_positioning = ABSOLUTE;
	    break;
	  case 91: // Relative coordinates.
	    gcode_curr_positioning = RELATIVE;
	    break;
	}
        break;
      case 'M':  // M-Code
	line_m = strtointeger(++ptr, &ptr);
	gcode_line_saw |= GCODE_SAW_MCODE;
        break;
      case 'N':  // Line Number
	strtointeger(++ptr, &ptr);
        break;
      case 'P':
      case 'S':  // P or S value
	line_s = strtointeger(++ptr, &ptr);
        break;
      case 'X':  // X position
	line_x = strtofixed(++ptr, &ptr);
	gcode_line_saw |= GCODE_SAW_MOTION;
        break;
      case 'Y':  // Y position
	line_y = strtofixed(++ptr, &ptr);
	gcode_line_saw |= GCODE_SAW_MOTION;
        break;
      case 'Z':  // Z position
	line_z = strtofixed(++ptr, &ptr);
	gcode_line_saw |= GCODE_SAW_MOTION;
        break;
      case '\r':
      case '\n':
      default:
	ptr++;
        break;
    }
  }
  gcode_curr_f = line_f;
  gcode_curr_m = line_m;
  gcode_curr_s = line_s;
  if ((gcode_line_saw & GCODE_SAW_MOTION)) {
    if (gcode_curr_units == INCHES) {
      // Convert to millimeters in 23.8 signed fixed-point format
      line_x = (line_x * ((254l<<8)/10) ) >> 8;
      line_y = (line_y * ((254l<<8)/10) ) >> 8;
      line_z = (line_z * ((254l<<8)/10) ) >> 8;
    }
    if (gcode_curr_positioning == RELATIVE) {
      line_x += gcode_curr_x;
      line_y += gcode_curr_y;
      line_z += gcode_curr_z;
    }
    gcode_curr_x = line_x;
    gcode_curr_y = line_y;
    gcode_curr_z = line_z;
  }

  translate_gcode();
}


//process bytes from a .gcode file.
void process_gcode_byte()
{
  if (gcode_line_complete) {
    translate_gcode();
  } else {
    while (playback_has_next()) {
      char ch = playback_next();
      if (ch == '\r') {
        continue;
      } else if (ch == '\n') {
	gcode_line_incomment = false;
	if (gcode_chars_read > 0) {
	  gcode_buffer[gcode_chars_read++] = '\0';
	  decode_gcode();
	  gcode_chars_read = 0;
	  break;
	}
      } else if (gcode_line_incomment) {
	if (ch == ')') {
	  gcode_line_incomment = false;
	}
      } else {
	if (ch == '(') {
	  gcode_line_incomment = true;
	} else if (ch != ' ') {
	  if (gcode_chars_read < sizeof(gcode_buffer)-1) {
	    gcode_buffer[gcode_chars_read++] = toupper(ch);
	  }
	}
      }
    }
  }
}


