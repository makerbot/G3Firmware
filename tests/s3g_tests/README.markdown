## Usage

Before running the G3FirmwareTests on a Thing-o-Matic with the Jetty Firmware, you will need to take the following steps:

1. If the firmware was built with the EEPROM menu enabled, then save a copy of your EEPROM settings to an SD card for later restoration.

2. Disable the Temperature Override function if it was previously enabled. It's disabled by default.  From a Gen 4 LCD interface, it can be disabled from the "Build Settings" menu.  If it is enabled, it will override set target temps thereby causing the tests which set a non-zero temperature to later read back the wrong set point.

3. For the SD Card Tests, you will want an SD card on which the first file is named `box_1.s3g`.  It's easiest to start with a card for which that is the only file is actually a valid, buildable (printable) file.

To run the test script,

1. **HACK ALERT:** Edit the file G3FirmwareTests.py and towards the bottom of the file, place in the `suites` list the tests suites to run.

2. Issue the command `G3FirmwareTests.py -f jetty -p <port>`


## Common Tests (commonFunctionTests)

    % G3FirmwareTests.py -f Jetty -p /dev/tty.usbmodemfa131
    test_ConvertFromNUL (__main__.commonFunctionTests) ... ok
    ----------------------------------------------------------------------
    Ran 1 test in 0.000s
    OK

## Packet Tests (s3gPacketTests)
#### expected timeout errors not shown for brevity

    % G3FirmwareTests.py -f Jetty -p /dev/tty.usbmodemfa131
    test_BadCRC (__main__.s3gPacketTests) ... ok
    test_EmptyPacket (__main__.s3gPacketTests) ... ok
    test_GetVersionPacket (__main__.s3gPacketTests) ... ok
    test_GetVersionPayload (__main__.s3gPacketTests) ... ok
    test_LongLength (__main__.s3gPacketTests) ... ok
    test_LongPayload (__main__.s3gPacketTests) ... ok
    test_MaxLength (__main__.s3gPacketTests) ... ok
    test_NoHeader (__main__.s3gPacketTests) ... ok
    test_OversizedLength (__main__.s3gPacketTests) ... ok
    test_PreceedingPacket (__main__.s3gPacketTests) ... ok
    test_ShortLength (__main__.s3gPacketTests) ... ok
    test_ShortPayload (__main__.s3gPacketTests) ... ok
    test_TrailingPacket (__main__.s3gPacketTests) ... ok
    ----------------------------------------------------------------------
    Ran 13 tests in 181.614s
    OK


## Send Receive Tests (s3gSendReceiveTests)

    % G3FirmwareTests.py -f Jetty -p /dev/tty.usbmodemfa131
    test_s3g_functions (__main__.s3gSendReceiveTests) ... Invoking abort_immediately()
    Invoking build_end_notification()
    Invoking build_start_notification(aTest)
    Skipping capture_to_file()
    Invoking change_tool(0)
    Invoking clear_buffer()
    Invoking delay(10)
    Invoking display_message(0, 0, 'TESTING', 1, False, False, False)
    Skipping end_capture_to_file()
    Invoking extended_stop(True, True)
    Invoking find_axes_maximums([], 1, 0)
    Invoking find_axes_minimums([], 1, 0)
    Invoking get_advanced_version()
    Invoking get_available_buffer_size()
    Invoking get_build_name()
    Invoking get_build_stats()
    Invoking get_communication_stats()
    Invoking get_extended_position()
    Invoking get_motherboard_status()
    Invoking get_motor1_speed(0)
    Invoking get_motor1_speed_PWM(0)
    Invoking get_next_filename(False)
    Invoking get_platform_target_temperature(0)
    Invoking get_platform_temperature(0)
    Invoking get_position()
    Invoking get_tool_status(0)
    Invoking get_toolhead_target_temperature(0)
    Invoking get_toolhead_temperature(0)
    Invoking get_toolhead_version(0)
    Invoking get_version()
    Invoking init()
    Invoking is_finished()
    Invoking is_platform_ready(0)
    Invoking is_tool_ready(0)
    Invoking pause()
    Skipping playback_capture()
    Invoking queue_extended_point([0, 0, 0, 0, 0], 500)
    Invoking queue_extended_point_new([0, 0, 0, 0, 0], 1, ['X', 'Y', 'Z', 'A', 'B'])
    Invoking queue_point_absolute([0, 0, 0], 500)
    Invoking queue_song(1)
    Invoking read_from_EEPROM(0, 1)
    Invoking read_from_toolhead_EEPROM(0, 0, 0)
    Invoking recall_home_positions(['X', 'Y', 'Z', 'A', 'B'],)
    Invoking reset()
    Invoking reset_to_factory()
    Invoking set_RGB_LED(255, 0, 0, 0)
    Invoking set_beep(1000, 3)
    Invoking set_build_percent(100)
    Invoking set_extended_position([0, 0, 0, 0, 0],)
    Invoking set_motor1_direction(0, False)
    Invoking set_motor1_speed_PWM(0, 0)
    Invoking set_motor1_speed_RPM(0, 0)
    Invoking set_platform_temperature(0, 100)
    Invoking set_position([0, 0, 0],)
    Invoking set_potentiometer_value('x', 118)
    Invoking set_servo1_position(0, 10)
    Invoking set_servo2_position(0, 10)
    Invoking set_toolhead_temperature(0, 100)
    Invoking store_home_positions(['X', 'Y', 'Z', 'A', 'B'],)
    Invoking toggle_abp(0, False)
    Invoking toggle_axes(['X', 'Y', 'Z', 'A', 'B'], True)
    Invoking toggle_extra_output(0, False)
    Invoking toggle_fan(0, False)
    Invoking toggle_motor1(0, True, True)
    Invoking tool_action_command(0, 23)
    Invoking tool_query(0, 0)
    Invoking toolhead_abort(0)
    Invoking toolhead_init(0)
    Invoking toolhead_pause(0)
    Invoking wait_for_button('up', 0, True, False, False)
    Invoking wait_for_platform_ready(0, 100, 50)
    Invoking wait_for_tool_ready(0, 100, 50)
    Invoking write_to_EEPROM(4092, '\xff')
    Invoking write_to_toolhead_EEPROM(0, 508, '\xff')
    ok
    ----------------------------------------------------------------------
    Ran 1 test in 20.499s
    OK

## Function Tests (s3gFunctionTests)

    % G3FirmwareTests.py -f Jetty -p /dev/tty.usbmodemfa131
    test_AbortImmediately (__main__.s3gFunctionTests) ... ok
    test_ClearBuffer (__main__.s3gFunctionTests) ... ok
    test_CommStats (__main__.s3gFunctionTests) ... ok
    test_Delay (__main__.s3gFunctionTests) ... 0
    1
    2
    3
    4
    5
    6
    7
    8
    9
    ok
    test_ExtendedStop (__main__.s3gFunctionTests) ... ok
    test_FindAxesMaximums (__main__.s3gFunctionTests) ... 
    Please move the platform and extruder away from any endstops and then press enter to continue.
    ok
    test_FindAxesMinimums (__main__.s3gFunctionTests) ... 
    Please move the platform and extruder away from any endstops and then press enter to continue.
    ok
    test_GetAvailableBufferSize (__main__.s3gFunctionTests) ... ok
    test_Init (__main__.s3gFunctionTests) ... ok
    test_IsFinished (__main__.s3gFunctionTests) ... ok
    test_IsPlatformReady (__main__.s3gFunctionTests) ... ok
    test_IsToolReady (__main__.s3gFunctionTests) ... ok
    test_QueueExtendedPoint (__main__.s3gFunctionTests) ... ok
    test_QueueExtendedPointNew (__main__.s3gFunctionTests) ... ok
    test_ReadWordsFromEeprom (__main__.s3gFunctionTests) ... ok
    test_RecallHomePositions (__main__.s3gFunctionTests) ... ok
    test_Reset (__main__.s3gFunctionTests) ... ok
    test_SetGetExtendedPosition (__main__.s3gFunctionTests) ... ok
    test_SetGetPlatformTargetTemperature (__main__.s3gFunctionTests) ... ok
    test_SetGetToolheadTargetTemperature (__main__.s3gFunctionTests) ... ok
    test_StoreHomePositions (__main__.s3gFunctionTests) ... ok
    test_WriteToEeprom (__main__.s3gFunctionTests) ... ok
    ----------------------------------------------------------------------
    Ran 22 tests in 191.835s
    OK

## User Function Tests (s3gUserFunctionTests)

    % G3FirmwareTests.py -f Jetty -p /dev/tty.usbmodemfa131
    test_GetToolStatus (__main__.s3gUserFunctionTests) ... 
    Please turn the bot off and unplug extruder 0's thermocouple!! Press enter to continue.

    Please turn the bot off and plug in the platform and extruder 0's thermocouple!! Press enter to continue.
    ok
    test_GetToolheadVersion (__main__.s3gUserFunctionTests) ... 
    What is the version number of toolhead 0 on your bot? 3.1
    ok
    test_GetVersion (__main__.s3gUserFunctionTests) ... 
    What is the version number of your bot? 3.1
    ok
    test_ToggleAxes (__main__.s3gUserFunctionTests) ... 
    Please try to move all (x,y,z) the axes!  Can you move them without using too much force? (y/n) n

    Please try to move all (x,y,z) the axes!  Can you move them without using too much force? (y/n) y
    ok
    test_ToggleFan (__main__.s3gUserFunctionTests) ... 
    Is toolhead 0's fan on? (y/n) y
    ok
    ----------------------------------------------------------------------
    Ran 5 tests in 102.575s
    OK

## Sd Card Tests (s3gSDCardTests)

    % G3FirmwareTests.py -f Jetty -p /dev/tty.usbmodemfa131
    test_CaptureToFile (__main__.s3gSDCardTests) ... ok
    test_CaptureToFileReply (__main__.s3gSDCardTests) ... ok
    test_EndCaptureToFile (__main__.s3gSDCardTests) ... ok
    test_EndCaptureToFileReply (__main__.s3gSDCardTests) ... ok
    test_GetBuildName (__main__.s3gSDCardTests) ... 
    Please load the test SD card into the machine, select one of the files and begin to print it.  Then type the name _exactly_ as it appears on the bot's screen. box_1.s3g
    ok
    test_GetNextFilename (__main__.s3gSDCardTests) ... 
    Please make sure the only files on the SD card plugged into the bot are the files inside the testFiles directory!! Press enter to continue
    ok
    test_PlaybackCapture (__main__.s3gSDCardTests) ... ok
    test_PlaybackCaptureReply (__main__.s3gSDCardTests) ... ok
    ----------------------------------------------------------------------
    Ran 8 tests in 43.424s
    OK

