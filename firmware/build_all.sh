#!/bin/bash

SCONS=SConstruct

PLATFORMS=( rrmbv12 'rrmbv12 fived=true' mb24 mb24-2560 ecv22 ecv34 ecv34_328p )

LOG_FILE=build_all_output


function build_firmware {
    for platform in "${PLATFORMS[@]}"
    do
	echo -n "Building firmware for ${platform}... "

	echo -e "\n\n\n\n" >> ${LOG_FILE}
	echo Building firmware for ${platform} >> ${LOG_FILE}

	scons -f "${SCONS}" platform=${platform} >> ${LOG_FILE} 2>&1

	if [ "$?" -ne "0" ]; then
	    echo Failure
	else
	    echo Success
	fi
    done
}


function build_documentation {
    echo -n "Building documentation..."

    echo Building documentation >> ${LOG_FILE}
    echo -e "\n\n\n\n" >> ${LOG_FILE}
    doxygen G3Firmware.doxyfile >> ${LOG_FILE} 2>&1
    
    if [ "$?" -ne "0" ]; then
        echo Failure
    else
	echo Success
    fi
}



echo Building all firmware
echo "Building all firmware" > ${LOG_FILE}

build_firmware

build_documentation