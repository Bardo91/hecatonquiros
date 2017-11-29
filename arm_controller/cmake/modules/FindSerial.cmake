################################################################################
##
## FindSerial.cmake
##		Set variables
##			- SERIAL_INDLUDE_DIR
##			- SERIAL_LIBRARIES
##
################################################################################

# DO IT RIGHT

if(WIN32)
	set(SERIAL_INDLUDE_DIR "C:/Program Files (x86)/serial/include")
	set(SERIAL_LIBRARIES optimized "C:/Program Files (x86)/serial/lib/serial.lib" debug "C:/Program Files (x86)/serial/lib/seriald.lib")
elseif(UNIX)
	set( SERIAL_INDLUDE_DIR "/usr/local/serial")
	set(SERIAL_LIBRARIES "/usr/local/lib/libserial.so")
endif()

mark_as_advanced(SERIAL_INDLUDE_DIR SERIAL_LIB)
