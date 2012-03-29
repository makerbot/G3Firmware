@ECHO OFF
:startBurn
echo Ready to flash firmware onto ECv3.6.
pause

listports.py > serialPort
set /p port= < serialPort

set firmware=ECv3.6\EC-ecv34-v3.1_328.hex
set programmer=stk500v1
set baud=57600
set part=atmega328p


echo Attempting to connect to port %port% 

tools-win\avrdude -c%programmer% -b%baud% -D -v -V -F -p%part% -P%port% -Uflash:w:%firmware%:i 

if errorlevel 1 (
echo *** FAILURE *** Failed to verify program.  Try again.
)

goto startBurn
