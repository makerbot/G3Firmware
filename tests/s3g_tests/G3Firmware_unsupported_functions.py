"""
List of the unsupported s3g functions.  It's easier to list what isn't
supported than what is.

These functions can be attempted and will return an "unknown command"
response if they are not buffered.
"""

unsupported_functions = [

# Host Query Commands

    'get_motherboard_status',                                              # 023
    'get_build_stats',                                                     # 024
    'get_advanced_version',                                                # 027

# Host Action Commands (buffered)

    'set_potentiometer_value',                                             # 145
    'set_RGB_LED',                                                         # 146
    'set_beep',                                                            # 147
    'wait_for_button',                                                     # 148
    'display_message',                                                     # 149
    'set_build_percent',                                                   # 150
    'queue_song',                                                          # 151
    'reset_to_factory',                                                    # 152
    'build_start_notification',                                            # 153
    'build_end_notification',                                              # 154

# Tool Query Commands

# Tool Action Commands

    'toolhead_abort'                                                       # 024
]


"""
List of functions to not attempt: they will return a result which is incorrect.
(Unsupported functions can be tried and they will return an "unknown command" response.)

"""
buggy_functions = [

    # Bug in v3.6 Extruder Controller code: has a missing return/break statement
    # in the switch() clause that handles this.  It falls through to another query
    # and thus generates a result with too much data.

    'get_PID_state'
]
