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


"""
Endstop max/min info
"""
constants['endstops_max']  = ['x', 'y']
constants['max_test_bits'] = 0x0A  # bits lit when at max end stops (ymax | xmax)
constants['max_test_mask'] = 0x0F  # bits to check when at max end stops (ymax|ymin|xmax|xmin)

constants['endstops_min']  = ['z']
constants['min_test_bits'] = 0x10  # bits lit when at min end stops (zmin)
constants['min_test_mask'] = 0x30  # bits to check when at min end stops (zmax | zmin)

"""
New point to move to
"""
constants['new_ext_point'] = [-50, -50, 100, 0, 0]

# 1 for dual extruder, 0 for single extruder
constants['dual_extruder'] = 1
