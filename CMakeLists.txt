###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 3.13 FATAL_ERROR)
project(hecatonquiros VERSION 0.4.1 LANGUAGES CXX)

# CONFIG
set(BUILD_APPS OFF)

# Load macros
include(cmake/hecatonquiros_macros.cmake)
include(cmake/install_macro.cmake)

# Compile
add_subdirectory(modules/hecatonquiros)

add_subdirectory(modules/ros_manip_description)

### DOC generation
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
	${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)


# install 

install_hecatonquiros()
if(BUILD_APPS)
	add_subdirectory(modules/apps)
	install_hecatonquiros_apps()
endif()
