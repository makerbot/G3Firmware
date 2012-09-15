"""
Constants unique to the G3Firmware and used by the G3Firmware tests
"""

platform = 'mb24-2560'
# platform = 'mb24'
# platform = 'rrmbv12'

# EEPROM field offsets and lengths

EEPROM_offsets = {

    # key : ( offset, length, default-value )

    'read_test' :      ( 0x0173,  1, [0x00], '<B' ),
 # 0x0044, 4, [0x23C1, 0xB404], '<HH'
    'machine_name' :   ( 0x0020, 32, '\x00' ),
    'home_positions' : ( 0x0060, 20, None ),
    'x_home' :         ( 0x0060,  4, '\x00\x00\x00\x00' ),
    'y_home' :         ( 0x0064,  4, '\x00\x00\x00\x00' ),
    'z_home' :         ( 0x0068,  4, '\x00\x00\x00\x00' ),
    'a_home' :         ( 0x006c,  4, '\x00\x00\x00\x00' ),
    'b_home' :         ( 0x0070,  4, '\x00\x00\x00\x00' )
}

# Other firmware constants

constants = {}

# Primary tool head index
constants['toolhead']  = 0

# For commands which want to control all toolheads (such as ensuring the heaters are off)
constants['toolheads'] = [ constants['toolhead'] ]  # [0, 1] for two tool heads

# Temperature to heat things up to
constants['target_temp'] = 100

if platform != 'rrmbv12':
    constants['buffer_size'] = 512
else:
    constants['buffer_size'] = 256


"""
Endstop max/min info
"""
constants['endstops_max']  = ['z']
constants['max_test_bits'] = 0x20  # bits lit when at max end stops (zmax)
constants['max_test_mask'] = 0x30  # bits to check when at max end stops (zmax | zmin)

constants['endstops_min']  = ['x', 'y']
constants['min_test_bits'] = 0x05  # bits lit when at min end stops (ymin | zmin)
constants['min_test_mask'] = 0x0F  # bits to check when at min end stops (ymax|ymin|xmax|xmin)

"""
New point to move to.  Need a decent distance, otherwise the accelerated planner's
minimum speeds will cut in
"""
constants['new_ext_point'] = [500, 500, -10, 0, 0]

# 1 for dual extruder, 0 for single extruder
constants['dual_extruder'] = 0
