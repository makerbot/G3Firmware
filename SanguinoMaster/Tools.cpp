#include "Tools.h"
#include "Datatypes.h"
#include "RS485.h"
#include "Commands.h"
#include "Version.h"
#include "WProgram.h"
#include "Utils.h"
#include "Configuration.h"

SimplePacket slavePacket(rs485_tx);

uint8_t currentToolIndex = 0;
unsigned long toolNextPing = 0;
unsigned long toolTimeoutEnd = 0;

uint16_t last_head_temp;
uint16_t last_platform_temp;
uint16_t targ_head_temp;
uint16_t targ_platform_temp;
int8_t targ_extruder_dir;

extern uint8_t commandMode;

//initialize our tools
void init_tools()
{
  //do a scan of tools from address 0-255?
  //with a 1 millisecond timeout, this takes ~0.256 seconds.
  //we may also want to store which tools are available in eeprom?
/*
#ifdef SCAN_TOOLS_ON_STARTUP
  for (int i=0; i<256; i++)
  {
    //are you out there?
    if (ping_tool(i))
    {
      init_tool(i);
    }
  }
#endif
*/
  last_head_temp = 0;
  last_platform_temp = 0;
  targ_head_temp = 0;
  targ_platform_temp = 0;
  targ_extruder_dir = 0;

  // Initialize at startup to using tool 0 as a sane start.
  init_tool(0);
  select_tool(0);
}

//ask a tool if its there.
bool ping_tool(uint8_t i)
{
  slavePacket.init();

  slavePacket.add_8(i);
  slavePacket.add_8(SLAVE_CMD_VERSION);
  slavePacket.add_16(FIRMWARE_VERSION);
  return send_packet();
}

//initialize a tool to its default state.
void init_tool(uint8_t i)
{
  slavePacket.init();

  slavePacket.add_8(i);
  slavePacket.add_8(SLAVE_CMD_INIT);
  send_packet();
}

//select a tool as our current tool, and let it know.
void select_tool(uint8_t tool)
{
  currentToolIndex = tool;

  slavePacket.init();

  slavePacket.add_8(tool);
  slavePacket.add_8(SLAVE_CMD_SELECT_TOOL);
  send_packet();

  wait_for_tool_ready_state(tool, 0, 0);
}

void check_tool_ready_state()
{
  //did we timeout?
  if (millis() >= toolTimeoutEnd)
  {
    commandMode = COMMAND_MODE_IDLE;
    return;
  }

  //is it time for the next ping?
  if (millis() >= toolNextPing)
  {
    //is the tool ready?
    if (is_tool_ready(currentToolIndex))
    {
      commandMode = COMMAND_MODE_IDLE;
      return;
    }
    
    //ping every x millis
    toolNextPing = millis() + 100;
  }
}

//ping the tool until it tells us its ready
void wait_for_tool_ready_state(uint8_t tool, int delay_millis, int timeout_seconds)
{
  //setup some defaults
  if (delay_millis == 0)
    delay_millis = 100;
  if (timeout_seconds == 0)
    timeout_seconds = 60;

  //check for our end time.
  unsigned long end = millis() + (timeout_seconds * 1000);

  //do it until we hear something, or time out.
  while (1)
  {
    //did we time out yet?
    if (millis() >= end)
      return;

    //did we hear back from the tool?
    if (is_tool_ready(tool))
      return;

    //try again...
    delay(delay_millis);
  }
}

//is our tool ready for action?
bool is_tool_ready(uint8_t tool)
{
  slavePacket.init();

  slavePacket.add_8(tool);
  slavePacket.add_8(SLAVE_CMD_IS_TOOL_READY);

  //did we get a response?
  if (send_packet())
  {
    //is it true?
    if (slavePacket.get_8(1) == 1)
      return true;
  }

  //bail.
  return false;
}


void poll_current_tool_temps()
{
  slavePacket.init();
  slavePacket.add_8(currentToolIndex);
  slavePacket.add_8(SLAVE_CMD_GET_TEMP);
  if (send_packet()) {
    last_head_temp = slavePacket.get_16(1);
  }

  slavePacket.init();
  slavePacket.add_8(0);  // Always assume platform is heated by Tool 0?
  slavePacket.add_8(SLAVE_CMD_GET_PLATFORM_TEMP);
  if (send_packet()) {
    last_platform_temp = slavePacket.get_16(1);
  }
}


uint16_t get_last_head_temp()
{
  return last_head_temp;
}


uint16_t get_last_platform_temp()
{
  return last_platform_temp;
}


uint16_t get_target_head_temp()
{
  return targ_head_temp;
}


uint16_t get_target_platform_temp()
{
  return targ_platform_temp;
}


int8_t get_extruder_dir()
{
  return targ_extruder_dir;
}


static void tool_command_sniff(uint8_t tool, uint8_t command, uint8_t len, uint8_t *data)
{
  switch(command) {
    case SLAVE_CMD_SET_TEMP:
	targ_head_temp = (((uint16_t)data[1])<<8) + data[0];
        break;
    case SLAVE_CMD_SET_PLATFORM_TEMP:
	targ_platform_temp = (((uint16_t)data[1])<<8) + data[0];
        break;
    case SLAVE_CMD_TOGGLE_MOTOR_1:
	if (data[0] & 0x1) {
	  targ_extruder_dir = (data[0] & 0x2)? 1 : -1;
	} else {
	  targ_extruder_dir = 0;
	}
        break;
  }
}


void send_tool_query(SimplePacket& hostPacket)
{
  //zero out our packet
  slavePacket.init();

  //load up our packet.
  for (uint8_t i=1; i<hostPacket.getLength(); i++)
    slavePacket.add_8(hostPacket.get_8(i));

  //send it and then get our response
  send_packet();

  //now load it up into the host. (skip the response code)
  //TODO: check the response code
  for (uint8_t i=1; i<slavePacket.getLength(); i++)
    hostPacket.add_8(slavePacket.get_8(i));
}

void send_tool_command(CircularBuffer::Cursor& cursor)
{
  //zero out our packet
  slavePacket.init();

  uint8_t tool = cursor.read_8();
  uint8_t tcmd = cursor.read_8();
  uint8_t len  = cursor.read_8();

  //add in our tool id and command.
  slavePacket.add_8(tool);
  slavePacket.add_8(tcmd);

  //load up our packet.
  uint8_t buf[len];
  for (uint8_t i=0; i<len; i++)
    slavePacket.add_8(buf[i] = cursor.read_8());

  //sniff it for any interesting commands.
  tool_command_sniff(tool, tcmd, len, buf);

  //send it and then get our response
  send_packet();
}

void send_tool_simple_command(uint8_t tool, uint8_t command)
{
  slavePacket.init();
  slavePacket.add_8(tool);
  slavePacket.add_8(command);
  send_packet();
}


void send_tool_simple_command_with_byte(uint8_t tool, uint8_t command, uint8_t byt)
{
  tool_command_sniff(tool, command, 1, &byt);
  slavePacket.init();
  slavePacket.add_8(tool);
  slavePacket.add_8(command);
  slavePacket.add_8(byt);
  send_packet();
}


void send_tool_simple_command_with_word(uint8_t tool, uint8_t command, uint16_t wrd)
{
  uint8_t buf[2];
  buf[0] = wrd&0xff;
  buf[1] = wrd>>8;
  tool_command_sniff(tool, command, 2, buf);
  slavePacket.init();
  slavePacket.add_8(tool);
  slavePacket.add_8(command);
  slavePacket.add_16(wrd);
  send_packet();
}


void abort_current_tool()
{
  send_tool_simple_command(currentToolIndex, SLAVE_CMD_ABORT);
}

bool send_packet()
{
  //take it easy.  no stomping on each other.
  delayMicrosecondsInterruptible(50);

  digitalWrite(TX_ENABLE_PIN, HIGH); //enable tx

  //take it easy.  no stomping on each other.
  delayMicrosecondsInterruptible(10);

  slavePacket.sendPacket();

  digitalWrite(TX_ENABLE_PIN, LOW); //disable tx

  rs485_packet_count++;

  return read_tool_response(PACKET_TIMEOUT);
}

bool read_tool_response(int timeout)
{
  //figure out our timeout stuff.
  long start = millis();
  long end = start + timeout;

  //keep reading until we got it.
  while (!slavePacket.isFinished())
  {
    //read through our available data
    if (Serial1.available() > 0)
    {
      //grab a byte and process it.
      uint8_t d = Serial1.read();
      slavePacket.process_byte(d);

      rs485_rx_count++;

#ifdef ENABLE_COMMS_DEBUG
      /*
      Serial.print("IN:");
       Serial.print(d, HEX);
       Serial.print("/");
       Serial.println(d, BIN);
       */
#endif
      //keep processing while there's data. 
      start = millis();
      end = start + timeout;

      if (slavePacket.getResponseCode() == RC_CRC_MISMATCH)
      {
        slave_crc_errors++;

#ifdef ENABLE_COMMS_DEBUG
        Serial.println("Slave CRC Mismatch");
#endif
        //retransmit?
      }
    }

    //not sure if we need this yet.
    //our timeout guy.
    if (millis() > end)
    {
      slave_timeouts++;

#ifdef ENABLE_COMMS_DEBUG
      Serial.println("Slave Timeout");
#endif
      return false;
    }
  }

  return true;
}

void set_tool_pause_state(bool paused)
{
  //TODO: pause/unpause tool.
}
