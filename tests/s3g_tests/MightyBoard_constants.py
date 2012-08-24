"""
Constants unique to the MightyBoard firmware
"""

# EEPROM field offsets and lengths

EEPROM_offsets = {

    # key : ( offset, length, default-value )

    'read_test' :      ( 0x0044, 4, [0x23C1, 0xB404], '<HH' ),
    'machine_name' :   ( 0x0022, 16, '\x00' ),
    'home_positions' : ( 0x000E, 20, None ),
    'x_home' :         ( 0x000E,  4, '\x00\x00\x00\x00' ),
    'y_home' :         ( 0x0012,  4, '\x00\x00\x00\x00' ),
    'z_home' :         ( 0x0016,  4, '\x00\x00\x00\x00' ),
    'a_home' :         ( 0x001A,  4, '\x00\x00\x00\x00' ),
    'b_home' :         ( 0x001E,  4, '\x00\x00\x00\x00' )
}

# Other firmware constants

constants = {}

# Primary tool head index
constants['toolhead']  = 0

# For commands which want to control all toolheads (such as ensuring the heaters are off)
constants['toolheads'] = [ 0, 1 ]  # [0, 1] for two tool heads

# Temperature to heat things up to
constants['target_temp'] = 100

# Size of the command buffer for reading s3g packets
constants['buffer_size'] = 512
