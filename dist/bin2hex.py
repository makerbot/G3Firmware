#!/usr/bin/python

'''
Quick and dirty Python script to convert a eeprom_dump.bin file
to Intel hex format.  It assumes that the starting offset in EEPROM
is 0 but you can alter that with a command line switch.

Dan Newman ( dan . newman @ mtbaldy . us )
27 July 2012
'''

import struct
import sys
import getopt

DEF_INFILE  = 'eeprom_dump.bin'
DEF_OUTFILE = 'eeprom_dump.hex'
DEF_ADDRESS = int( 0 )

def usage( prog, exit_stat=0 ):
    str = \
'Usage: %s [-h] [-s starting-address] [-o outfile] [infile]\n' % prog
    str += \
' The input file, infile, defaults to %s\n' % DEF_INFILE
    str += \
' -h, --help\n' + \
'   This usage information\n' + \
' -o outfile, --output=outfile\n' + \
'   Specify the output file name.  The name defaults to %s\n' % DEF_OUTFILE
    str += \
' -s address, --start=address\n' + \
'   The starting address (offset) for the data.  The default is %d\n' % DEF_ADDRESS
    if exit_stat:
        sys.stderr.write( str )
    else:
        sys.stdout.write( str )
    sys.exit( exit_stat )

# Process the command line switches

infile  = DEF_INFILE
outfile = DEF_OUTFILE
address = DEF_ADDRESS

try:
    opts, args = getopt.getopt( sys.argv[1:], 'ho:s:',
                                [ 'help', 'output=', 'start=' ] )

except:
    usage( sys.argv[0], 1 )

for opt, val in opts:
    if opt in ( '-h', '--help' ):
        usage( sys.argv[0], 0 )
    elif opt in ( '-o', '--output' ):
        outfile = val
    elif opt in ( '-s', '--start' ):
        address = int( val, 0 )

# Process the command line arguments: there can only be zero or one

if len( args ) == 1:
    infile = args[0]
elif len( args ) > 1:
    usage( sys.argv[0], 1 )

fin = open( infile, 'rb' )
if fin < 0:
    sys.stderr.write( 'Unable to open the file "%s"\n' % infile )
    sys.exit( 1 )

# Read the entire input file

bytes = fin.read()
fin.close()

# Create the output file

fout = open( outfile, 'w' )
if fout < 0:
    sys.stderr.write( 'Unable to create the file "%s"\n' % outfile )
    sys.exit( 1 )

# Now loop over the input data, generating an Intel hex file
# We'll process the input data in blocks of 32 bytes

# Index into the list of bytes read from the input file
index  = 0

# While loop counter
length = len( bytes )

while length > 0:

    # Convert the starting address to big endian
    l = struct.pack( '>H', address )

    # Number of bytes for this line of output
    nbytes = 32
    if nbytes > length:
        nbytes = length

    # Generate the initial part of the output file's line
    #        ':' -- start code
    #      %0.2x -- data length
    # %0.2x%0.2x -- starting address as a big-endian, unsigned short
    #       '00' -- record type

    line = ':%02x%02x%02x00' % ( nbytes, ord( l[0] ), ord( l[1] ) )

    sum = nbytes + ord( l[0] ) + ord( l[1] )

    for i in range( 0, nbytes ):
        sum  += ord( bytes[index + i] )
        line += '%02x' % ord( bytes[index + i] )

    '''
    Checksum is the 2's complement of the least significant byte of the
    sum of the fields.  Need to do a final & with 0xff to suppress overflow
    '''
    cksum = ( 0x100 - ( 0xff & sum ) ) & 0xff
    line += '%02x\n' % cksum
    fout.write( line.upper() )

    index   += nbytes
    address += nbytes
    length  -= nbytes

fout.write( ':00000001FF\n' )
fout.close()
