import inspect
import os, sys
lib_path = os.path.abspath('./s3g')
sys.path.append(lib_path)
import s3g
import serial
import time

import G3Firmware_constants

"""
Evaluate the supplied string.  The string may contain a '%s' which will be replaced
with the name of a s3g class object.  This allows evaluation of a s3g class member
function,

  <s3g class object>.<s3g class member function>(args)

For example, to turn a heater off, supply the string

   %s.set_toolhead_temperature(0, 0)

which will get changed to

   obj.set_toolhead_temperature(0, 0)

and then evaluated.
"""
def evalStr(obj, string):
    str = string
    if str.find('%s') >= 0:
        str = string % 'obj'
    try:
        eval(str)
    except:
        print 'Evaluating %s raised an exception, %s' % ( str, sys.exc_info()[0] )

"""
Call the supplied function with the specified arguments.  If isSupported is False,
then a s3g.CommandNotSupportedError exception will be expected.
"""
def callFunc(func, obj, isSupported, args):
    if isSupported:
        if isinstance(args, tuple):
            func(*args)
        else:
            func(args)
    else:
        if isinstance(args, tuple):
            obj.assertRaises(s3g.CommandNotSupportedError, func, *args)
        else:
            obj.assertRaises(s3g.CommandNotSupportedError, func, args)

"""
Open a serial connection to the specified port and then clear the serial port's
I/O buffers.
"""
def initSerialComms(port, speed=115200, timeout=1):
    sp = serial.Serial(port, speed, timeout=timeout)
    #sp.setRTS()   # Helps on Windows
    #sp.setDTR()   # Helps on Windows
    sp.flushInput()
    sp.flushOutput()
    time.sleep(0.1)
    return sp

"""
Open a serial connection to the specified port and then instantiate
a s3g.Writer.StreamWriter() object which uses that serial connection.
"""
def initWriterComms(port, speed=115200, timeout=1):
    return s3g.Writer.StreamWriter(initSerialComms(port, speed, timeout))

"""
Strip the trailing NUL from the end of a NUL terminated string
"""
def ConvertFromNUL(b):
    if b[-1] != 0:
        raise TypeError("Cannot convert from non-NUL terminated string")
    return str(b[:-1])

"""
Disable the stepper motors
Set the heater target temperatures to 0
"""
def powerDown(obj):
    if obj is None:
        return
    try:
        # Turn heaters off
        obj.set_platform_temperature(0, 0)
        for toolhead in constants['toolheads']:
            obj.set_toolhead_temperature(toolhead, 0)
        # Disable stepper motors
        obj.toggle_axes(['x', 'y', 'z', 'a', 'b'], false)
    except:
        pass

"""
The following functions are used to implement @skipIf and @skipUnless decorators
"""
def _id(obj):
    return obj

def skip(*args, **kwargs):
    if inspect.isfunction( args[0] ) or inspect.ismethod( args[0] ):
        print "Skipping " + getattr(args[0], '__name__')

def skipIf(condition):
    if condition:
        return skip
    return _id

def skipUnless(condition):
    if not condition:
        return skip
    return _id
