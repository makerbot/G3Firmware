"""
List of the unsupported s3g functions.  It's easier to list what isn't
supported than what is.

These functions can be attempted and will return an "unknown command"
response if they are not buffered.
"""

unsupported_functions = [

# Host Query Commands

    'get_communication_stats',                                          # 025

# Host Action Commands (buffered)

    'queue_point_absolute',                                             # 129
    'set_position',                                                     # 130

# Tool Query Commands

    'get_motor1_speed_PWM',                                             # 019
    'get_motor1_speed',                                                 # 017
    'read_from_toolhead_EEPROM',                                        # 026

# Tool Action Commands

    'toolhead_abort',                                                   # 024
    'write_to_toolhead_EEPROM',                                         # 026

]


"""
List of functions to not attempt: they will return a result which is incorrect.
(Unsupported functions can be tried and they will return an "unknown command" response.)
"""
buggy_functions = [
]
