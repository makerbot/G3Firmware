#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <stdint.h>
#include "CircularBuffer.h"
#include <SimplePacket.h>

//initialize our tools
void init_tools();
//ask a tool if its there.
bool ping_tool(uint8_t i);
//initialize a tool to its default state.
void init_tool(uint8_t i);
//select a tool as our current tool, and let it know.
void select_tool(uint8_t tool);
void check_tool_ready_state();

void poll_current_tool_temps();
uint16_t get_last_head_temp();
uint16_t get_last_platform_temp();
uint16_t get_target_head_temp();
uint16_t get_target_platform_temp();
int8_t get_extruder_dir();

//ping the tool until it tells us its ready
void wait_for_tool_ready_state(uint8_t tool, int delay_millis, int timeout_seconds);
//is our tool ready for action?
bool is_tool_ready(uint8_t tool);
void send_tool_query(SimplePacket& hostPacket);
void send_tool_command(CircularBuffer::Cursor& cursor);
void send_tool_simple_command(uint8_t tool, uint8_t command);
void send_tool_simple_command_with_byte(uint8_t tool, uint8_t command, uint8_t byt);
void send_tool_simple_command_with_word(uint8_t tool, uint8_t command, uint16_t wrd);

void abort_current_tool();
bool send_packet();
bool read_tool_response(int timeout);
void set_tool_pause_state(bool paused);

extern unsigned long toolNextPing;
extern unsigned long toolTimeoutEnd;


#endif // _TOOLS_H_
