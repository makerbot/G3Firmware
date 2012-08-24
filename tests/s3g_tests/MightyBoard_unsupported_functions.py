"""
List of the unsupported s3g functions.  It's easier to list what isn't
supported than what is.

These functions can be attempted and will return an "unknown command"
response if they are not buffered.
"""

unsupported_functions = [

# Host Query Commands

    'get_position',                                                     # 004
    'get_communication_stats',                                          # 025

# Host Action Commands (buffered)

    'queue_point_absolute',                                             # 129
    'set_position',                                                     # 130

# Tool Query Commands

    'get_motor1_speed_PWM',                                             # 019

# Tool Action Commands

    'set_motor1_speed_PWM',                                             # 004
    'set_motor1_direction',                                             # 008
    'set_servo2_position',                                              # 015
    'toggle_abp',                                                       # 027

]


"""
List of functions to not attempt: they will return a result which is incorrect.
(Unsupported functions can be tried and they will return an "unknown command" response.)
"""
buggy_functions = [
]
