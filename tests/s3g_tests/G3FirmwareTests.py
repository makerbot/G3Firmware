#!/usr/bin/python

import unittest
import optparse
import serial
import io
import struct
import array
import time
import os, sys 
import logging
import tempfile
import inspect

lib_path = os.path.abspath('./s3g')
sys.path.append(lib_path)

# The s3g API
import s3g

# The s3g functions to test and the call arguments to use
from test_s3g_functions import *

# Testing utility routines
from test_utilities import *

# Defaults
extensive    = True
port         = '/dev/ttyACM0'
hasInterface = True

firmware   = 'G3Firmware'
#firmware  = 'Jetty'
#firmware  = 'MightyBoard'

# Process our command line arguments now
# we need to know which firmware we will be testing in order to know
# which profiles to load

if __name__ == '__main__':

  parser = optparse.OptionParser()
  parser.add_option("-e", "--extensive", dest="extensive", default=str(extensive))
  parser.add_option("-m", "--mightyboard", dest="isMightyBoard", default="True")
  parser.add_option("-f", "--firmware", dest="firmware", default=firmware)
  parser.add_option("-i", "--interface", dest="hasInterface", default="True")
  parser.add_option("-p", "--port", dest="port", default=port)
  (options, args) = parser.parse_args()

  if options.extensive.lower() == "false":
    print "Foregoing Heater Tests"
    extensive = False
  else:
    extensive = True

  if options.hasInterface.lower() in ['0', 'false', 'no']:
    print "Foregoing tests requiring Interface Boards"
    hasInterface = False

  port     = options.port
  firmware = options.firmware

# Now load the firmware 'profiles'

if firmware.lower() in ['g3firmware', 'jetty']:

  from G3Firmware_constants import *
  from G3Firmware_unsupported_functions import *
  hasInterface = False

else:

  from MightyBoard_constants import *
  from MightyBoard_unsupported_functions import *

class commonFunctionTests(unittest.TestCase):

  def test_ConvertFromNUL(self):
    b = bytearray("asdf\x00")
    expectedReturn = "asdf"
    self.assertEqual(expectedReturn, ConvertFromNUL(b))

"""
Test core packet handling routines used by the s3g module
  -- test encode/decode functionality
  -- test error handling (malformed packets, etc.)
"""

class s3gPacketTests(unittest.TestCase):

  def setUp(self):
    self.r = s3g.s3g()
    self.r.writer = initWriterComms(port, 115200, timeout=1)

  def tearDown(self):
    self.r.writer.file.close()
    self.r = None

  def GetVersionPayload(self):
    payload = struct.pack('<BH',
                          s3g.host_query_command_dict['GET_VERSION'],
                          s3g.s3g_version)
    return payload

  def GetVersionPacket(self):
    """
    Helper method to generate a Get Version packet to be modified and sent
    """
    return s3g.Encoder.encode_payload(self.GetVersionPayload())

  def test_GetVersionPayload(self):
    payload = self.GetVersionPayload()
    self.assertEqual(payload[0], chr(s3g.host_query_command_dict['GET_VERSION']))
    self.assertEqual(payload[1:], s3g.Encoder.encode_uint16(s3g.s3g_version))

  def test_GetVersionPacket(self):
    testPayload = self.GetVersionPayload()
    packet = self.GetVersionPacket()
    self.assertEqual(packet[0], s3g.header)
    self.assertEqual(packet[1], len(packet[2:-1]))
    self.assertEqual(packet[2:-1], testPayload)
    self.assertEqual(packet[-1], s3g.Encoder.CalculateCRC(testPayload))

  def test_NoHeader(self):
    packet = self.GetVersionPacket()
    packet[0] = '\x00'
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)
      
  def test_EmptyPacket(self):
    packet = bytearray()
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)

  def test_TrailingPacket(self):
    packet = self.GetVersionPacket()
    addition = bytearray('\xff\xff')
    packet.extend(addition)
    self.r.writer.send_packet(packet)
    self.assertTrue(True)

  def test_PreceedingPacket(self):
    packet = self.GetVersionPacket()
    addition = bytearray('\xa4\x5f')
    addition.extend(packet)
    self.r.writer.send_packet(addition)
    self.assertTrue(True)

  def test_BadCRC(self):
    packet = self.GetVersionPacket()
    payload = packet[2:-1]
    crc = s3g.Encoder.CalculateCRC(payload)
    packet[-1] = crc+1 
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)

  def test_LongLength(self):
    packet = self.GetVersionPacket()
    packet[1] = '\x0f'
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)

  def test_ShortLength(self):
    packet = self.GetVersionPacket()
    packet[1] = '\x01'
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)

  def test_LongPayload(self):
    packet = self.GetVersionPacket()
    packet.insert(2, '\x00')
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)

  def test_ShortPayload(self):
    packet = self.GetVersionPacket()
    packet = packet[0:2] + packet[3:]
    self.assertRaises(s3g.TransmissionError, self.r.writer.send_packet, packet)

  def test_MaxLength(self):
    """
    Packet.cc code in the G3Firmware and MightyBoard firmware only allows a
    maximum payload length of 31 (s3g.maximum_payload_length - 1).  Refer to
    InPacket::processByte() where, when processing the length byte, it only
    allows "(b < MAX_PACKET_PAYLOAD)" and not "(b <= MAX_PACKET_PAYLOAD)"
    """
    payload = self.GetVersionPayload()
    for i in range(s3g.maximum_payload_length - (1 + len(payload))):
      payload += chr(0x00)
    self.r.writer.send_command(payload)

  def test_OversizedLength(self):
    payload = bytearray(s3g.maximum_payload_length+1)
    self.assertRaises(s3g.PacketLengthError, s3g.Encoder.encode_payload, payload)

class s3gSendReceiveTests(unittest.TestCase):

  def setUp(self):
    self.r = s3g.s3g()
    self.r.writer = initWriterComms(port, 115200, timeout=1)

  def tearDown(self):
    self.r.writer.file.close()
    self.r = None

  """
  Use introspection to discover every member function of the s3g class.  Then if
  the discovered member function is NOT in s3g_functions_to_ignore[] and is in
  s3g_functions{} with a call argument tuple (!= None), then call the method
  function with the call argument tuple.

  If the method function is in the firmware-specific unsupported_functions[] list,
  then an error will be anticipated.  If it is not in that list, then a successful
  return will be expected.

  Additionally,

  1. The keys of the s3g_functions{} are enumerated, looking for keys no longer
     specifying member functions of the s3g class.  This is done to help keep this
     test suite and the s3g class in sync.

  2. When a member function of the s3g class is encountered which is not in the
     s3g_functions_to_ignore[] list and also not in the s3g_functions{} dictionary,
     an error is signalled.  This too is done to keep the s3g class and this test
     suite in sync.
  """

  @skipUnless(all([x in globals() for x in ['s3g_functions_to_ignore', 's3g_function_calls', 'unsupported_functions', 'buggy_functions']]))
  def test_s3g_functions(self):
    # Loop over every member of the s3g.s3g class
    for name in dir( self.r ):

      # Proceed only if this member is a class method
      method = getattr( self.r, name)
      if inspect.ismethod( method ):

        # Proceed only if this method function is NOT in the ignore list, s3g_functions_to_ignore[]
        if ( not ( name in s3g_functions_to_ignore ) ) and ( not ( name in buggy_functions ) ):

          # See if this method function is a key in the s3g_function_calls{} dictionary
          if name in s3g_function_calls:

            args = s3g_function_calls[ name ]

            # Is this a supported function which should be skipped?
            if args is None:

              print 'Skipping ' + name + '()'

            else:

              # See if there are instructions to execute after calling the function
              # E.g., turning a heater off or waiting a few seconds for the bot's
              # firmware to initialize itself after a reset.
              post_call = ''
              if isinstance( args, list ):
                post_call = args[1]
                args      = args[0]

              # Note whether the firmware supports this s3g function
              # We want to call it regardless of whether it is supported or not
              # The isSupported flag merely tells us if we should expect a "Success"
              # or "Unsupported" response from the bot.
              #
              # There's a slight twist on all of this: an unsupported BUT buffered
              # command actually gets a "Success" response back from the bot.  That's
              # because the bot merely acknowledges receipt of a well-formed packet
              # when it sees that it is a buffered command (Host Action Command).

              isSupported = not( name in unsupported_functions )
              if name in s3g_buffered_functions:
                isSupported = True

              # Now call the method (r.self.method).  The isSupported flag indicates
              # whether we should expect an exception to be raised or not

              print 'Invoking ' + name + ( '%s' % (args,) if isinstance( args, tuple)  else '(%s)' % args )
              callFunc( method, self, isSupported, args )

              # And execute any additional instructions
              if post_call != '':
                evalStr( self.r, post_call )
                        
          else:

            # We've encountered a member function of the class s3g.s3g which
            # isn't in the s3g_function_calls{} dictionary nor in the list
            # s3g_functions_to_ignore[].
            #
            # This suggests that s3g.py was updated but not the unit tests.

            print 'WARNING: the member function %s() is in the class s3g ' % name + \
                'but not in either s3g_functions_to_ignore[] or ' + \
                's3g_function_calls{}.  Please correct the unit test ' + \
                'test_Blah.'

    # Now another test to ensure that the test scripts are in sync with s3g.py:
    # see if s3g_function_calls{} contains names not in dir(s3g.s3g()).

    names = dir( self.r )
    for key in s3g_function_calls:
      if not ( key in names ) or ( not inspect.ismethod( getattr( self.r, key) ) ):
        print 'WARNING: the s3g_function_calls{} dictionary contains an entry, ' + \
            '%s, which is not a member function of the s3g class of s3g.py' % key

    # Made it this far: do an explicit assert if we managed to get by
    # without doing any asserts...

    self.assertTrue(True)

class s3gFunctionTests(unittest.TestCase):

  def setUp(self):
    self.r = s3g.s3g()
    self.r.writer = initWriterComms(port, 115200, timeout=1)

  def tearDown(self): 
    self.r.writer.file.close()
    self.r = None

  def test_SetGetPlatformTargetTemperature(self):
    target = constants['target_temp']
    self.assertEqual(self.r.get_platform_target_temperature(0), 0)
    self.r.set_platform_temperature(0, target)
    time.sleep(0.1)
    temp = self.r.get_platform_target_temperature(0)
    if (temp != target) and ( firmware.lower() in ['g3firmware', 'jetty'] ):
      print "Note that test_SetGetPlatformTargetTemperature() will fail if temperature override is enabled!"
    self.r.set_platform_temperature(0, 0)
    self.assertEqual(temp, target)

  def test_SetGetToolheadTargetTemperature(self):
    target = constants['target_temp']
    toolhead = constants['toolhead']
    self.assertEqual(self.r.get_toolhead_target_temperature(toolhead), 0)
    self.r.set_toolhead_temperature(toolhead, target)
    time.sleep(0.1)
    temp = self.r.get_toolhead_target_temperature(toolhead)
    if (temp != target) and ( firmware.lower() in ['g3firmware', 'jetty'] ):
      print "Note that test_SetGetToolheadTargetTemperature() will fail if temperature override is enabled!"
    self.r.set_toolhead_temperature(toolhead, 0)
    self.assertEqual(temp, target)

  def test_ReadWordsFromEeprom(self):
    """
    Read the LCD type and compare to our expected value
    """
    offset, length, default, format = EEPROM_offsets['read_test']
    test = self.r.read_from_EEPROM(offset, length)
    test = array.array('B', test)
    test = struct.unpack(format, test)
    self.assertTrue(all([test[i] == default[i] for i in range(0,len(test))]))

  def test_WriteToEeprom(self):
    """
    Write a new bot name to the EEPROM and verify that it reads back correctly
    """
    nameOffset, length, default = EEPROM_offsets['machine_name']
    name = 'ILOVETESTINGALOT'
    self.r.write_to_EEPROM(nameOffset, name)
    readName = self.r.read_from_EEPROM(nameOffset, len(name))
    self.assertEqual(name, readName)

  def test_IsPlatformReady(self):
    """
    Determine if the platform is ready by setting the temperature to its current reading and asking if its ready (should return true, then setting the temperature to +50 what it is now then querying it agian, expecting a false answer

    WARNING: THIS TEST WILL FAIL WITH THE JETTY FIRMWARE IF TEMP OVERRIDE IS ENABLED
    """
    curTemp = self.r.get_platform_temperature(0)
    self.r.set_platform_temperature(0, curTemp - 1)  # Hysteresis allows for 2 degree swing
    time.sleep(0.1)
    self.assertTrue(self.r.is_platform_ready(0))
    self.r.set_platform_temperature(0, curTemp+50)
    time.sleep(0.1)
    self.assertEqual(self.r.is_platform_ready(0), False)
    self.r.set_platform_temperature(0, 0)

  def test_IsToolReady(self):
    """
    WARNING: THIS TEST WILL FAIL WITH THE JETTY FIRMWARE IF TEMP OVERRIDE IS ENABLED
    """
    toolhead = constants['toolhead']
    curTemp = self.r.get_toolhead_temperature(toolhead)
    self.r.set_toolhead_temperature(toolhead, curTemp - 1)  # Hysteresis allows for 2 degree swing
    time.sleep(0.1)
    self.assertTrue(self.r.is_tool_ready(toolhead))
    self.r.set_toolhead_temperature(toolhead, curTemp + 50)
    time.sleep(0.1)
    self.assertEqual(self.r.is_tool_ready(toolhead), False)
    self.r.set_toolhead_temperature(toolhead, 0)

  def test_SetGetExtendedPosition(self):
    position = [50, 51, 52, 53, 54 * constants['dual_extruder']]
    self.r.set_extended_position(position)
    self.assertEqual(position, self.r.get_extended_position()[0])

  def test_QueueExtendedPoint(self):
    self.r.find_axes_maximums(constants['endstops_max'], 500, 100)
    self.r.find_axes_minimums(constants['endstops_min'], 500, 100)
    self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
    self.r.set_extended_position([0, 0, 0, 0, 0])
    newPosition = constants['new_ext_point']
    rate = 500
    self.r.queue_extended_point(newPosition, rate)
    time.sleep(5)
    self.assertEqual(newPosition, self.r.get_extended_position()[0])

  def test_FindAxesMaximums(self):
    self.r.toggle_axes(['X', 'Y', 'Z', 'A', 'B'], False)
    raw_input("\nPlease move the platform and extruder away from any endstops and then press enter to continue.")
    axes = constants['endstops_max']
    test_bits = constants['max_test_bits']
    test_mask = constants['max_test_mask']
    rate = 500
    timeout = 10
    self.r.find_axes_maximums(axes, rate, timeout)
    time.sleep(timeout)
    self.assertEqual(self.r.get_extended_position()[1]&test_mask, test_bits)

  def test_FindAxesMinimums(self):
    self.r.toggle_axes(['X', 'Y', 'Z', 'A', 'B'], False)
    raw_input("\nPlease move the platform and extruder away from any endstops and then press enter to continue.")
    axes = constants['endstops_min']
    test_bits = constants['min_test_bits']
    test_mask = constants['min_test_mask']
    rate = 500
    timeout = 50
    self.r.find_axes_minimums(axes, rate, timeout)
    time.sleep(timeout)
    self.assertEqual(self.r.get_extended_position()[1]&test_mask, test_bits)

  def test_Init(self):
    position = [10, 9, 8, 7, 6 * constants['dual_extruder']]
    self.r.set_extended_position(position)
    #this function doesn't do anything, so we are testing that position is NOT cleared after command recieved
    self.r.init()
    time.sleep(5)
    self.assertEqual(position, self.r.get_extended_position()[0])

  def test_GetAvailableBufferSize(self):
    bufferSize = constants['buffer_size']
    self.assertEqual(bufferSize, self.r.get_available_buffer_size())

  def test_AbortImmediately(self):
    bufferSize = constants['buffer_size']
    toolheads = constants['toolheads']
    target = constants['target_temp']
    for toolhead in toolheads:
      self.r.set_toolhead_temperature(toolhead, target)
    self.r.set_platform_temperature(0, target)
    self.r.find_axes_minimums(constants['endstops_min'], 500, 5)
    self.r.find_axes_maximums(constants['endstops_max'], 500, 5)
    self.r.abort_immediately()
    time.sleep(5)
    self.assertEqual(bufferSize, self.r.get_available_buffer_size())
    for toolhead in toolheads:
      self.assertEqual(0, self.r.get_toolhead_target_temperature(toolhead))
    self.assertEqual(0, self.r.get_platform_target_temperature(0))
    self.assertTrue(self.r.is_finished())

  @skipIf(any(x in unsupported_functions for x in ['build_start_notification', 'build_end_notification', 'get_build_name', 'get_build_stats']))
  def test_BuildStartNotification(self):
    buildName = "test"
    self.r.build_start_notification(buildName)
    readBuildName = self.r.get_build_name()
    readBuildName = ConvertFromNUL(readBuildName)
    self.assertEqual(buildName, readBuildName)
    stats = self.r.get_build_stats()
    build_running_state = 1
    self.assertEqual(stats['BuildState'], build_running_state)
    self.r.build_end_notification()

  @skipIf(any(x in unsupported_functions for x in ['build_start_notification', \
                                                            'build_end_notification', \
                                                            'get_build_stats']))
  def test_BuildEndNotification(self):
    self.r.build_start_notification("test")
    time.sleep(2)
    stats = self.r.get_build_stats()
    build_running_state = 1
    self.assertEqual(stats['BuildState'], build_running_state)
    self.r.build_end_notification()
    stats = self.r.get_build_stats()
    build_finished_state = 2
    self.assertEqual(stats['BuildState'], build_finished_state)

  @skipIf(any(x in unsupported_functions for x in ['build_start_notification', \
                                                            'get_build_stats']))
  def test_BuildStats(self):
    """
    we've tested build start, stop and pause in other tests
    test build cancel 
    test line number
    test print time
    """
    # start build
    self.r.build_start_notification("test")
    start_time = time.time()
    stats = self.r.get_build_stats()
    build_running_state = 1
    self.assertEqual(stats['BuildState'], build_running_state)
    # send 5 commands
    for cmd in range(0,5):
      self.r.toggle_axes(['x', 'y', 'z', 'a', 'b'], True)
    stats = self.r.get_build_stats()
    self.assertEqual(stats['LineNumber'], 5)
    # send 100 commands
    for cmd in range(0,100):
      self.r.toggle_axes(['x', 'y', 'z', 'a', 'b'], False)
    stats = self.r.get_build_stats()
    self.assertEqual(stats['LineNumber'], 105)
    time.sleep(60)
    #check how much time has passed
    time_minutes = int((time.time()-start_time) / 60)
    stats = self.r.get_build_stats()
    self.assertEqual(stats['BuildMinutes'], time_minutes)
    #cancel build
    self.r.abort_immediately()
    self.assertRaises(s3g.BuildCancelledError, self.r.get_build_stats)
    stats = self.r.get_build_stats()
    build_cancelled_state = 4
    self.assertEqual(stats['BuildState'], build_cancelled_state)

  @skipIf('get_communication_stats' in unsupported_functions)
  def test_CommStats(self):
    info1 = self.r.get_communication_stats()
    # Now generate some traffic over the tool network
    self.r.get_platform_target_temperature(0)
    info2 = self.r.get_communication_stats()
    self.assertNotEqual(info1['PacketsSent'], info2['PacketsSent'])

  def test_ClearBuffer(self):
    bufferSize = constants['buffer_size']
    target = constants['target_temp']
    self.r.set_platform_temperature(0, target)
    self.r.wait_for_platform_ready(0, 0, 0xFFFF)
    axes = constants['endstops_min']
    rate = 500
    timeout = 5
    for i in range(10):
      self.r.find_axes_minimums(axes, rate, timeout)
    self.assertNotEqual(bufferSize, self.r.get_available_buffer_size())
    self.r.clear_buffer()
    time.sleep(5) # we need to sleep after sending any reset functions
    self.assertEqual(bufferSize, self.r.get_available_buffer_size())
    self.r.set_platform_temperature(0, 0) # Just to be safe

  @skipIf('get_build_stats' in unsupported_functions)
  def test_PauseandBuildStats(self):
    stats = self.r.get_build_stats()
    build_paused_state = 3
    self.assertNotEqual(stats['BuildState'], build_paused_state)
    self.r.pause()
    stats = self.r.get_build_stats()
    self.assertEqual(stats['BuildState'], build_paused_state)

  @skipIf('get_build_stats' in unsupported_functions)
  def test_ToolheadPauseandBuildStats(self):
    stats = self.r.get_build_stats()
    build_paused_state = 3
    self.assertNotEqual(stats['BuildState'], build_paused_state)
    self.r.toolhead_pause(0)
    stats = self.r.get_build_stats()
    self.assertEqual(stats['BuildState'], build_paused_state)

  def test_IsFinished(self):
    timeout = 30
    # home axes
    self.r.find_axes_maximums(constants['endstops_max'], 500, timeout)
    self.r.find_axes_minimums(constants['endstops_min'], 500, timeout)
    self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
    self.r.set_extended_position([0, 0, 0, 0, 0])
    time.sleep(timeout)
    # go to a new point
    newPoint = constants['new_ext_point']
    duration = 5
    micros = 1000000
    self.r.queue_extended_point_new(newPoint, duration*micros, [])
    # assert that bot is not finished
    self.assertFalse(self.r.is_finished())
    time.sleep(duration)
    # assert that bot is finished
    self.assertTrue(self.r.is_finished())

  def test_Reset(self):
    bufferSize = constants['buffer_size']
    toolhead = constants['toolhead']
    target = constants['target_temp']
    self.r.set_toolhead_temperature(toolhead, target)
    self.r.set_platform_temperature(0, target)
    self.r.wait_for_platform_ready(0, 0, 0xFFFF)
    for i in range(30):
      self.r.toggle_axes(['x', 'y', 'z'], False)
    self.assertNotEqual(self.r.get_available_buffer_size(), bufferSize)
    self.r.reset()
    time.sleep(5)
    self.assertEqual(self.r.get_available_buffer_size(), bufferSize)
    self.assertTrue(self.r.is_finished())
    self.assertEqual(self.r.get_toolhead_target_temperature(toolhead), 0)
    self.assertEqual(self.r.get_platform_target_temperature(0), 0)
 
  def test_Delay(self):
    self.r.find_axes_maximums(constants['endstops_max'], 500, 50)
    self.r.find_axes_minimums(constants['endstops_min'], 500, 50)
    self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
    self.r.set_extended_position([0, 0, 0, 0, 0])
    delay = 5
    duration = 5
    millis = 1000
    micros = 1000 * millis
    newPoint = constants['new_ext_point']
    self.r.delay(delay*millis)
    self.r.queue_extended_point_new(newPoint, duration*micros, [])
    self.assertFalse(self.r.is_finished())
    for i in range(0, duration+delay):
      print i
      self.assertFalse(self.r.is_finished())
      time.sleep(1)
    time.sleep(1.5)
    self.assertTrue(self.r.is_finished())

  def test_ExtendedStop(self):
    bufferSize = constants['buffer_size']
    self.r.find_axes_maximums(constants['endstops_max'], 200, 50)
    self.r.find_axes_minimums(constants['endstops_min'], 500, 50)
    self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
    self.r.set_extended_position([0, 0, 0, 0, 0])
    delay = 5
    duration = 5
    millis = 1000
    micros = millis * 1000
    newPoint = constants['new_ext_point']
    self.r.delay(delay*millis)
    self.r.queue_extended_point_new(newPoint, duration*micros, [])
    for i in range(5):
      self.r.toggle_axes(['x', 'y', 'z'], False)
    self.r.extended_stop(True, True)
    time.sleep(2) #Give the machine time to respond
    self.assertTrue(self.r.is_finished())
    self.assertEqual(bufferSize, self.r.get_available_buffer_size())

  def test_QueueExtendedPointNew(self):
    self.r.find_axes_maximums(constants['endstops_max'], 200, 50)
    self.r.find_axes_minimums(constants['endstops_min'], 500, 50)
    self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
    self.r.set_extended_position([0, 0, 0, 0, 0])
    while(self.r.is_finished is False):
      time.sleep(1)
    newPoint = constants['new_ext_point']
    micros = 1000 * 1000
    duration = 5
    self.r.queue_extended_point_new(newPoint, duration*micros, [])
    time.sleep(duration)
    self.assertEqual(newPoint, self.r.get_extended_position()[0])
    anotherPoint = [ 2*p for p in newPoint]
    self.r.queue_extended_point_new(anotherPoint, duration*micros, [])
    time.sleep(duration)
    self.assertEqual(anotherPoint, self.r.get_extended_position()[0])
 
  def test_StoreHomePositions(self):
    old_home = self.r.read_from_EEPROM(EEPROM_offsets['home_positions'][0], EEPROM_offsets['home_positions'][1])
    position = [20,20,30,40,50*constants['dual_extruder']]
    self.r.set_extended_position(position)
    self.r.store_home_positions(['X', 'Y', 'Z', 'A', 'B'])
    x = self.r.read_from_EEPROM(EEPROM_offsets['x_home'][0], EEPROM_offsets['x_home'][1])
    y = self.r.read_from_EEPROM(EEPROM_offsets['y_home'][0], EEPROM_offsets['y_home'][1])
    z = self.r.read_from_EEPROM(EEPROM_offsets['z_home'][0], EEPROM_offsets['z_home'][1])
    a = self.r.read_from_EEPROM(EEPROM_offsets['a_home'][0], EEPROM_offsets['a_home'][1])
    b = self.r.read_from_EEPROM(EEPROM_offsets['b_home'][0], EEPROM_offsets['b_home'][1])
    readHome = []
    for cor in [x, y, z, a, b]:
      readHome.append(s3g.Encoder.decode_int32(cor))
    self.r.write_to_EEPROM(EEPROM_offsets['home_positions'][0], old_home)
    self.assertEqual(readHome, position)

  def test_RecallHomePositions(self):
    old_home = self.r.read_from_EEPROM(EEPROM_offsets['home_positions'][0], EEPROM_offsets['home_positions'][1])
    pointToSet = [1, 2, 3, 4, 5 * constants['dual_extruder']]
    self.r.set_extended_position(pointToSet)
    self.r.store_home_positions(['X', 'Y', 'Z', 'A', 'B'])
    newPoint = [50, 51, 52, 53, 54 * constants['dual_extruder']]
    self.r.set_extended_position(newPoint)
    self.r.recall_home_positions([])
    time.sleep(5)
    self.assertEqual(newPoint, self.r.get_extended_position()[0])
    self.r.recall_home_positions(['X', 'Y', 'Z', 'A', 'B'])
    time.sleep(5)
    self.r.write_to_EEPROM(EEPROM_offsets['home_positions'][0], old_home)
    self.assertEqual(pointToSet, self.r.get_extended_position()[0])
 
  @skipIf('get_PID_state' in buggy_functions)
  def test_GetPIDState(self):
    toolhead = constants['toolhead']
    pidDict = self.r.get_PID_state(toolhead)
    for key in pidDict:
      self.assertNotEqual(pidDict[key], None)

class s3gUserFunctionTests(unittest.TestCase):

  def setUp(self):
    self.r = s3g.s3g()
    self.r.writer = initWriterComms(port, 115200, timeout=1)
    time.sleep(1)

  def tearDown(self):
    self.r.writer.file.close()
    self.r = None

  """
  def test_WaitForPlatformReady(self):

  def test_WaitForToolReady(self):
  """

  def test_ToggleAxes(self):
    self.r.toggle_axes(['X', 'Y', 'Z', 'A', 'B'], True)
    obs = raw_input("\nPlease try to move all (x,y,z) the axes!  Can you move them without using too much force? (y/n) ")
    self.assertEqual('n', obs)
    self.r.toggle_axes(['X', 'Y', 'Z', 'A', 'B'], False)
    obs = raw_input("\nPlease try to move all (x,y,z) the axes!  Can you move them without using too much force? (y/n) ")
    self.assertEqual('y', obs)

  @skipUnless(hasInterface and not('display_message' in unsupported_functions))
  def test_DisplayMessage(self):
    message = str(time.clock())
    self.r.display_message(0, 0, message, 10, False, False, False)
    readMessage = raw_input("\nWhat is the message on the replicator's display? ")
    self.assertEqual(message, readMessage)

  def test_ToggleFan(self):
    toolhead = constants['toolhead']
    self.r.toggle_fan(toolhead, True)
    obs = raw_input("\nIs toolhead %i's fan on? (y/n) " % constants['toolhead'])
    self.r.toggle_fan(toolhead, False)
    self.assertEqual(obs, 'y')

  def test_GetVersion(self):
    expectedVersion = raw_input("\nWhat is the version number of your bot? ")
    expectedVersion = int(expectedVersion.replace('.', '0'))
    self.assertEqual(expectedVersion, self.r.get_version())

  def test_GetToolheadVersion(self):
    toolhead = constants['toolhead']
    expectedVersion = raw_input("\nWhat is the version number of toolhead %d on your bot? " % toolhead)
    expectedVersion = int(expectedVersion.replace('.', '0'))
    self.assertEqual(expectedVersion, self.r.get_toolhead_version(toolhead))

  def test_GetToolStatus(self):
    toolhead = constants['toolhead']
    target = constants['target_temp']
    self.r.set_toolhead_temperature(toolhead, 0)
    time.sleep(0.1)
    returnDic = self.r.get_tool_status(toolhead)
    self.assertFalse(returnDic['ExtruderReady'])  # Extruder is not at temp
    self.assertFalse(returnDic['PlatformError'])
    self.assertFalse(returnDic['ExtruderError'])
    self.r.set_toolhead_temperature(toolhead, target)
    # G3Firmware has a little latency owing to the Extruder Controller being 
    # a separate microprocessor.  Sleep for 0.1 s before querying
    time.sleep(0.1)
    returnDic = self.r.get_tool_status(toolhead)
    self.assertEqual(returnDic['ExtruderReady'], self.r.is_tool_ready(toolhead))
    # G3Firmware for the v3.6 EC will detected a shorted thermistor on the HBP,
    # but not an open, floating connection to the thermistor
    if not ( firmware.lower() in ['g3firmware', 'jetty'] ):
      raw_input("\nPlease unplug the platform!!  Press enter to continue.")
      self.r.writer.file.close()
      self.r.writer.file = initSerialComms(port, 115200, timeout=1)                              
      self.r.set_platform_temperature(toolhead, target)
      time.sleep(5)
      returnDic = self.r.get_tool_status(toolhead)
      self.assertTrue(returnDic['PlatformError'])
      raw_input("\nPlease turn the bot off, plug in the platform and unplug extruder %d's thermocouple!! Press enter to continue." % toolhead)
    else:
      raw_input("\nPlease turn the bot off and unplug extruder %d's thermocouple!! Press enter to continue." % toolhead)
    self.r.writer.file.close()
    self.r.writer.file = initSerialComms(port, 115200, timeout=1)
    self.r.set_toolhead_temperature(toolhead, target)
    time.sleep(5)
    returnDic = self.r.get_tool_status(constants['toolhead'])
    self.assertTrue(returnDic['ExtruderError'])
    self.r.writer.file.close()
    raw_input("\nPlease turn the bot off and plug in the platform and extruder %d's thermocouple!! Press enter to continue." % toolhead)
    self.r.writer.file = initSerialComms(port, 115200, timeout=1)

  @skipIf('wait_for_button' in unsupported_functions)
  def test_WaitForButton(self):
    self.r.wait_for_button('up', 0, False, False, False)
    obs = raw_input("\nIs the center button flashing? (y/n) ")
    self.assertEqual(obs, 'y')
    obs = raw_input("\nPress all the buttons EXCEPT the 'up' button.  Is the center button still flashing? (y/n) ")
    self.assertEqual(obs, 'y')
    obs = raw_input("\nPress the 'up' button.  Did the center button stop flashing? (y/n) ")
    self.assertEqual(obs, 'y')
    raw_input("\nTesting wait_for_button timeout.  Please watch the interface board and note the time! Press enter to continue")
    self.r.wait_for_button('up', 5, True, False, False)
    obs = raw_input("\nDid the center button flash for about 5 seconds and stop? (y/n) ")
    self.assertEqual(obs, 'y')
    raw_input("\nTesting bot reset after tiemout.  Please watch/listen to verify if the replicator is resetting. Press enter to continue.")
    self.r.wait_for_button('up', 1, False, True, False)
    time.sleep(1)
    obs = raw_input("\nDid the bot just reset? (y/n) ")
    self.assertEqual(obs, 'y')
    self.r.wait_for_button('up', 0, False, False, True)
    obs = raw_input("\nPlease press the up button and note if the LCD screen resest or not.  Did the screen reset? (y/n) ")
    self.assertEqual(obs, 'y')

  @skipIf('set_RGB_LED' in unsupported_functions)
  def test_SetRGBLED(self):
    self.r.set_RGB_LED(0, 255, 0, 0)
    obs = raw_input("\nAre the LEDs in the bot green? (y/n) ")
    self.assertEqual(obs, 'y')
    self.r.set_RGB_LED(0, 255, 0, 128)
    obs = raw_input("\nAre the LEDs blinking? (y/n) ")
    self.assertEqual('y', obs)
 
  @skipIf('set_beep' in unsupported_functions)
  def test_SetBeep(self):
    raw_input("\nAbout to start playing some music.  Start listening! Press enter to continue")
    self.r.set_beep(261.626, 5)
    obs = raw_input("\nDid you hear a C note? (y/n) ")
    self.assertEqual('y', obs)

  @skipIf(any(x in unsupported_functions for x in ['set_build_percent', 'build_start_notification']))
  def test_SetBuildPercent(self):
    percent = 42
    self.r.build_start_notification("percentTest")
    self.r.set_build_percent(percent)
    obs = raw_input("\nLook at the interface board for your bot.  Does the build percent say that it is %1 percent of the way done? (y/n) " %(percent))
    self.assertEqual('y', obs)

  @skipIf('queue_song' in unsupported_functions)
  def test_QueueSong(self):
    raw_input("\nGetting ready to play a song.  Make sure you are listening!  Press enter to continue.")
    self.r.queue_song(1)
    obs = raw_input("\nDid you hear the song play? (y/n) ")
    self.assertEqual(obs, 'y')
 
class s3gSDCardTests(unittest.TestCase):

  def setUp(self):
    self.r = s3g.s3g()
    self.r.writer = initWriterComms(port, 115200, timeout=1)

  def tearDown(self):
    self.r.writer.file.close()
    self.r = None    

  @skipIf('playback_capture' in unsupported_functions)
  def test_PlaybackCaptureReply(self):
    self.assertRaises(s3g.SDCardError, self.r.playback_capture, 'aTest')

  @skipIf('capture_to_file' in unsupported_functions)
  def test_CaptureToFileReply(self):
    self.r.capture_to_file('test')

  @skipIf('end_capture_to_file' in unsupported_functions)
  def test_EndCaptureToFileReply(self):
    self.r.end_capture_to_file()

  @skipIf(any(x in unsupported_functions for x in ['capture_to_file', 'end_capture_to_file']))
  def test_EndCaptureToFile(self):
    filename = str(time.clock())+".s3g"
    self.r.capture_to_file(filename)
    findAxesMaximums = 8+32+16
    numCmd = 5
    totalBytes = findAxesMaximums*numCmd/8 + numCmd
    #Add some commands to the file
    for i in range(numCmd):
      self.r.find_axes_maximums(constants['endstops_max'], 500, 10)
    time.sleep(0.1)
    self.assertEqual(totalBytes, self.r.end_capture_to_file())

  @skipIf(any(x in unsupported_functions for x in ['capture_to_file', 'get_next_filename']))
  def test_CaptureToFile(self):
    filename = str(time.clock())+".s3g" #We want to keep changing the filename so this test stays nice and fresh
    self.r.capture_to_file(filename)
    #Get the filenames off the SD card
    files = []
    curFile = ConvertFromNUL(self.r.get_next_filename(True))
    while curFile != '':
      curFile = ConvertFromNUL(self.r.get_next_filename(False))
      files.append(curFile)
    self.assertTrue(filename in files)

  @skipIf('get_build_name' in unsupported_functions)
  def test_GetBuildName(self):
    """
    Copy the contents of the testFiles directory onto an sd card to do this test
    """
    buildName = raw_input("\nPlease load the test SD card into the machine, select one of the files and begin to print it.  Then type the name _exactly_ as it appears on the bot's screen. ")
    name = self.r.get_build_name()
    self.assertEqual(buildName, ConvertFromNUL(name))

  @skipIf('get_next_filename' in unsupported_functions)
  def test_GetNextFilename(self):
    """
    Copy the contents of the testFiles directory onto an sd card to do this test
    """
    raw_input("\nPlease make sure the only files on the SD card plugged into the bot are the files inside the testFiles directory!! Press enter to continue")
    if not firmware.lower() in ['g3firmware', 'jetty']:
      volumeName = raw_input("\nPlease type the VOLUME NAME of the replicator's SD card exactly! Press enter to continue")
      readVolumeName = self.r.get_next_filename(True)
      self.assertEqual(volumeName, ConvertFromNUL(readVolumeName))
    filename = 'box_1.s3g'
    readFilename = self.r.get_next_filename(False)
    self.assertEqual(filename, ConvertFromNUL(readFilename))
  
  @skipIf('playback_capture' in unsupported_functions)
  def test_PlaybackCapture(self):
    filename = 'box_1.s3g'
    self.r.playback_capture(filename)
    readName = self.r.get_build_name()
    self.assertEqual(filename, ConvertFromNUL(readName))

if __name__ == '__main__':

  logging.basicConfig()
  del sys.argv[1:]

  commonTests = unittest.TestLoader().loadTestsFromTestCase(commonFunctionTests)
  packetTests = unittest.TestLoader().loadTestsFromTestCase(s3gPacketTests)
  sendReceiveTests = unittest.TestLoader().loadTestsFromTestCase(s3gSendReceiveTests)
  functionTests = unittest.TestLoader().loadTestsFromTestCase(s3gFunctionTests)
  userFunctionTests = unittest.TestLoader().loadTestsFromTestCase(s3gUserFunctionTests)
  sdTests = unittest.TestLoader().loadTestsFromTestCase(s3gSDCardTests)

  suites = [commonTests, packetTests, sendReceiveTests, functionTests] # userFunctionTests]
  suites = [userFunctionTests]

  for suite in suites:
    unittest.TextTestRunner(verbosity=2).run(suite)
