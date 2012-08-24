import os, sys 
lib_path = os.path.abspath('./s3g')
sys.path.append(lib_path)
import s3g

"""
A dictionary of s3g function names and suitable call arguments for testing.

Entries in the dictionary are keyed by the actual name of a method
function in s3g.py,

     a = s3g.s3g()
     for f in dir( a ):
       if inspect.ismethod( getattr( a, f ) ):
         print "I'm the %s method function of s3g.s3g()" % f

Values in the dictionary take one of three forms:

  None
    Do not attempt to call the function

  tuple
    (call-args)
    Call arguments to pass to the function

  list
    [ (call-args), 'post-call-statements-string' ]
    The first list element is the tuple of call argument.  The second list element
    is a string containing an instruction to execute after the function call.  A
    single %s will be replaced with an object of type s3g capable of calling s3g
    functions.

NOTE BENE: if there's just one call argument AND it is a list, then for the "tuple"
  case above, use "([a,b,c,...],)".  The extra comma after the list preserves the
  tuple-ness.  Without the comma, the value is reduced to "[a,b,c,...]" and becomes
  indistinguishable from the "list" case above.

Note: Yes, it's possible to design this table with a more uniform value; e.g., use
  [ (args), 'post-call-string' ] for all entries.  However, that's more cumbersome for
  simple entries which are the majority case.  It's hoped that keeping most entries
  simple will help with maintainability.  (OTOH, consistency could be a better choice.)
"""

s3g_function_calls = {

# Host Query Commands

    'get_version' : (),                                                             # 000
    'init' : (),                                                                    # 001
    'get_available_buffer_size' : (),                                               # 002
    'clear_buffer' : [ (), 'time.sleep(5)' ],                                       # 003
    'get_position' : (),                                                            # 004
    'abort_immediately' : [ (), 'time.sleep(5)' ],                                  # 007
    'pause' : (),                                                                   # 008
    'tool_query' : (0, s3g.slave_query_command_dict['GET_VERSION']),                # 010
    'is_finished' : (),                                                             # 011
    'read_from_EEPROM' : (0x00, 1),                                                 # 012
    'write_to_EEPROM' : (0xFFC, '\xff'),                                            # 013
    'capture_to_file' : None,                                                       # 014
    'end_capture_to_file' : None,                                                   # 015
    'playback_capture' : None,                                                      # 016
    'reset' : [ (), 'time.sleep(5)' ],                                              # 017
    'get_next_filename' : (False),                                                  # 018
    'get_build_name' : (),                                                          # 020
    'get_extended_position' : (),                                                   # 021
    'extended_stop' : (True, True),                                                 # 022
    'get_motherboard_status' : (),                                                  # 023
    'get_build_stats' : (),                                                         # 024
    'get_communication_stats' : (),                                                 # 025
    'get_advanced_version' : (),                                                    # 027

# Host Action Commands (buffered)

    'queue_point_absolute' : ([0, 0, 0,], 500),                                     # 129
    'set_position' : ([0, 0, 0],),                                                  # 130
    'find_axes_maximums' : (['x', 'y'], 1, 0),                                      # 131
    'find_axes_minimums' : (['z'], 1, 0),                                           # 132
    'delay' : (10),                                                                 # 133
    'change_tool' : (0),                                                            # 134
    'wait_for_tool_ready' : (0, 100, 50),                                           # 135
    'tool_action_command' : (0, s3g.slave_action_command_dict['PAUSE']),            # 136
    'toggle_axes' : (['X', 'Y', 'Z', 'A', 'B'], True),                              # 137
    'queue_extended_point' : ([0, 0, 0, 0, 0], 500),                                # 139
    'set_extended_position' : ([0, 0, 0, 0, 0],),                                   # 140
    'wait_for_platform_ready' : (0, 100, 50),                                       # 141
    'queue_extended_point_new' : ([0, 0, 0, 0, 0], 1, ['X', 'Y', 'Z', 'A', 'B']),   # 142
    'store_home_positions' : (['X', 'Y', 'Z', 'A', 'B'],),                          # 143
    'recall_home_positions' : (['X', 'Y', 'Z', 'A', 'B'],),                         # 144
    'set_potentiometer_value' : ('x',  118),                                        # 145
    'set_RGB_LED' : (255, 0, 0, 0),                                                 # 146
    'set_beep' : (1000, 3),                                                         # 147
    'wait_for_button' : ('up', 0, True, False, False),                              # 148
    'display_message' : (0, 0, "TESTING", 1, False, False, False),                  # 149
    'set_build_percent' : (100),                                                    # 150
    'queue_song' : (1),                                                             # 151
    'reset_to_factory' : [ (), 'time.sleep(5)' ],                                   # 152
    'build_start_notification' : [ ('aTest'), '%s.build_end_notification()' ],      # 153
    'build_end_notification' : (),                                                  # 154

# Tool Query Commands

    'get_toolhead_version' : (0),                                                   # 000
    'get_toolhead_temperature' : (0),                                               # 002
    'get_motor1_speed' : (0),                                                       # 017
    'get_motor1_speed_PWM' : (0),                                                   # 019
    'is_tool_ready' : (0),                                                          # 022
    'read_from_toolhead_EEPROM' : (0, 0x00, 0),                                     # 025
    'write_to_toolhead_EEPROM' : (0, 0x1FC, '\xff'),                                # 026
    'get_platform_temperature' : (0),                                               # 030
    'get_toolhead_target_temperature' : (0),                                        # 032
    'get_platform_target_temperature' : (0),                                        # 033
    'is_platform_ready' : (0),                                                      # 035
    'get_tool_status' : (0),                                                        # 036
    'get_PID_state' : (0),                                                          # 037

# Tool Action Commands

    'toolhead_init' : (0),                                                          # 001
    'set_toolhead_temperature' : [ (0, 100), '%s.set_toolhead_temperature(0, 0)' ], # 003
    'set_motor1_speed_PWM' : (0, 0),                                                # 004
    'set_motor1_speed_RPM' : (0, 0),                                                # 006
    'set_motor1_direction' : (0, False),                                            # 008
    'toggle_motor1' : (0, True, True),                                              # 010
    'toggle_fan' : (0, False),                                                      # 012
    'toggle_extra_output' : (0, False),                                             # 013
    'set_servo1_position' : (0, 10),                                                # 014
    'set_servo2_position' : (0, 10),                                                # 015
    'toolhead_pause' : (0),                                                         # 023
    'toolhead_abort' : (0),                                                         # 024
    'toggle_abp' : (0, False),                                                      # 027
    'set_platform_temperature' : [ (0, 100), '%s.set_platform_temperature(0, 0)' ]  # 031
}

"""
A list of s3g class methods to ignore.
"""
s3g_functions_to_ignore = [ '__init__', 'from_filename' ]

"""
A list of the s3g functions which are buffered Host Action commands and thus always
return a success.
"""
s3g_buffered_functions = [
    'queue_point_absolute',                                                         # 129
    'set_position',                                                                 # 130
    'find_axes_maximums',                                                           # 131
    'find_axes_minimums',                                                           # 132
    'delay',                                                                        # 133
    'change_tool',                                                                  # 134
    'wait_for_tool_ready',                                                          # 135
    'tool_action_command',                                                          # 136
    'toggle_axes',                                                                  # 137
    'queue_extended_point',                                                         # 139
    'set_extended_position',                                                        # 140
    'wait_for_platform_ready',                                                      # 141
    'queue_extended_point_new',                                                     # 142
    'store_home_positions',                                                         # 143
    'recall_home_positions',                                                        # 144
    'set_potentiometer_value',                                                      # 145
    'set_RGB_LED',                                                                  # 146
    'set_beep',                                                                     # 147
    'wait_for_button',                                                              # 148
    'display_message',                                                              # 149
    'set_build_percent',                                                            # 150
    'queue_song',                                                                   # 151
    'reset_to_factory',                                                             # 152
    'build_start_notification',                                                     # 153
    'build_end_notification'                                                        # 154
]
