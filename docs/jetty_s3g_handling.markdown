# Jetty and G3Firmware Firmware Handling of s3g Commands

## Overview

The Jetty Firmware is an enhancement to the G3Firmware used by Thing-o-Matics and Cupcakes with the 5D3G shield.  As a result, the Jetty Firmware and the G3Firmware implements the same s3g commands.

In an effort to be concise, this document only lists the s3g commands that have behaviors different from those described in the general s3g specification @ https://github.com/makerbot/s3g/blob/master/doc/s3g_protocol.markdown

Note that when using the accelerated planner in the Jetty Firmware, only a single extruder is presently supported.  Dual extrusion on a Cupcake or Thing-o-Matic is only supported when using the non-accelerated planner.

## Ignored Commands

These commands are not implemented by the Host.  However, as they are all implemented as "buffered" Host Action commands, the Host acknowledges them with a "Success" response but then ignores the command.

With the exception of the build notification commands, these s3g commands all post-date the G3Firmware and as such are not known to that firmware.  In the case of the build notification commands, they were experimental and not supported by the G3Firmware.  Many of the remainder of these commands are not applicable to the G3 or G4 hardware families (e.g., queue song, set digital potentiometer value, etc.).

### Host Query Commands

*01 - Initialize firmware to boot state: this command is really a Host Action command, but the s3g protocol specification lists it as a Host Query command?)

### Host Action Commands

*145 - Set digital potentiometer value
*146 - Set RGB LED value
*147 - Set beep
*148 - Wait for button
*149 - Display message to LCD
*150 - Set build percentage
*151 - Queue song
*153 - Build start notification
*154 - Build end notification

### Tool Query Commands

### Tool Action Commands

*24 - Toolhead abort immediately: since this command is delivered to the host as the payload in a buffered Host Action command, it is immediately acknowledged with a "Success" response and then ignored.

## Unsupported Commands (returns "cmd unsupported")

These s3g commands all post-date the G3Firmware and as such are not known to the G3Firmware.  Their issuance to the G3Firmware (or Jetty Firmware) will be met with a "command unsupported" response.

### Host Query Commands

*23 - Get motherboard status
*24 - Get build statistics
*27 - Get advanced version

### Host Action Commands

### Tool Query Commands

### Tool Action Commands

## Commands With Limited or Specialized Behavior

### Host Query Commands

#### 00 - Get version: Query firmware for version information
    HostVersion <= 26 : returns firmware version 0x0000 (i.e., invalid version)
    HostVersion > 26 : returns motherboard firmware version
 
#### 03 - Clear buffer: Empty the command buffer
    Like the Replicator, the buffer is cleared by performing a soft reset.

#### 07 - Abort immediately: Stop machine, shut down job permanently
    Like the Replicator, this action is performed by executing a soft reset.

#### 10 - Tool query: Query a tool for information
    In the G3Firmware family, the "tool" is a separate device -- a "slave" device -- distinct from the Host.  Tool Query commands are relayed to the slave device, and the slave device's response is then relayed back by the host as the response to the query command.  Consequently, responses to Tool Query commands have more latency than with the MightyBoard.

#### 12 / 13 - Read from / Write to EEPROM
    Like the MightyBoard firmware, the AVR EEPROM library for EEPROM access is used.  That library does not handle invalid values for offset or lengths.  As a result, the firmware is most obliging in its attempts to process any supplied value.

#### 18 - Get next filename
    Owing to differences in SD card libraries, the Jetty and G3Firmware do not report the volume name as the first retrieved file name.

#### 22 - Extended stop: Stop a subset of systems
    A zero response is always sent.  There is no processed failure case.

#### 25 - Get communications stats
    Always returns a value of zero for the Packets Received field.

### Host Buffered Commands

#### 131 - Find axes minimums: Move specified axes in the negative direction until their limit switch is triggered.
    The Cupcake does not have endstops. The Thing-o-Matic has X and Y minimum endstops.  Thus get axes minimums for the Z axis will result in the stepper trying to move past the minimum point.  For the A and B axes, the command will timeout.

#### 132 - Find axes maximums: Move specified axes in the positive direction until their limit switch is triggered.
    The Cupcake does not have endstops.  The Thing-o-Matic has only a Z axis maximum endstop.  Any attempt to find the axes maximums for the X or Y axes will result in those axes moving past their maximum positions.  For the A and B axes, the command will timeout.
    
#### 135 - Wait for tool ready : Wait until a tool is ready before proceeding
    Either Tool ID 0 or 1 may be selected.
    Nominal timeout values are not implemented, all values accepted for timeout. 
    The delay parameter is ignored
    A tool timeout of zero will not wait for the tool to heat.  

#### 136 - Tool action command: Send an action command to a tool for execution
     In the G3Firmware family, the "tool" is a separate device -- a "slave" device -- distinct from the Host.  Tool Action commands are relayed to the slave device and an immediate "Success" response is returned as Tool Action commands are the payload of a buffered Host Action command.

#### 141 - Wait for platform ready: Wait until a build platform is ready before proceeding
    The bot has no knowledge of nominal values.
    Tool ID is ignored.
    Delay is ignored
    Timeout accepts all values.  A timeout of 0 or small will not wait for the tool to heat

#### 152 - Reset to Factory
    With the exception of the home offsets and filament counters, all EEPROM parameters are reset to their default values.  Note that they are not written to 0xff but explicitly set to the default value of the parameter stored at that location.

### Tool Query Commands

#### 36 - Get tool status
    As mentioned in the s3g specification, there is a deprecated form of the response to this command.  The G3Firmware and Jetty Firmware generate that legacy form.  As such, statuses temperature dropping, not heating, software cutoff, and not plugged are not generated. Instead the AVR WDRF, BORF, EXTRF, and PORF bit flag statuses are returned.
    Also, the G3Firmware and Jetty Firmware do detect reliably platform error caused by an open connection.  The v3.6 Extruder Controller with the v3.1 Extruder Controller firmware just sees a fluctuating temperature which may or may not trigger the firmware's error detection.  A connection shorted closed, however, does tend to lead to detection of a problem.

# Tool Action Commands

#### 03 - Set toolhead target temperature
    Unlike the Replicator, a value less than zero is accepted.
    There is no max temp enforced.

#### 13 - Enable/disable extra output
    This command, also known as the open/close valve command, is intercepted by the Jetty Firmware and used to enable or disable acceleration in the planner.  By default, when the accelerated stepper driver is used, acceleration is enabled.  When a valve close command is seen, acceleration is disabled for the subsequent line segments to be printed.  The planner plans them but with no acceleration steps.  When a valve open command is seen, acceleration is enabled for the subsequent line segments: the planner plans them and uses acceleration in their planning.  Using this mechanism, acceleration can be temporarily disabled and then later re-enabled.  It is how the Skeinforge "Altshell" plugin causes perimeter shells to be printed without acceleration (and hence no artifacts caused by acceleration).

#### 31 - Set build platform target temperature
    Unlike the Replicator, a value less than zero is accepted.
    There is no max temp enforced.


