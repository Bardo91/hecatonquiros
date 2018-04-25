###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(hecatonquiros)
set(PROJECT_VERSION 0.4.0)

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(src)

#########################################
###### Loading 3rd party libraries.######
#########################################

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

## Load serial
find_package(Serial REQUIRED)

## Load tinyxml2
find_package(tinyxml2 REQUIRED)

## Load opencv
find_package(OpenCV 3 REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

## Check if OpenRAVE is installed and use it
find_package(OpenRAVE)
if(${OpenRAVE_FOUND})
	add_definitions(-DHAS_OPENRAVE)
else(${OpenRAVE_FOUND})
	MESSAGE(WARNING "OpenRAVE NOT FOUND")
endif(${OpenRAVE_FOUND})

find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			sensor_msgs
			std_msgs
			roscpp
			message_generation
			)
if(${catkin_FOUND})
	add_definitions(-DHAS_ROS)
endif(${catkin_FOUND})

add_service_files(
  FILES
  SetJoints.srv
  SetPose.srv
)

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

catkin_package(
	INCLUDE_DIRS ./include
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()


## Load PCL
find_package(PCL QUIET)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
endif()

## Loading boost
if(WIN32)
	set(Boost_USE_STATIC_LIBS ON)
endif()
find_package(Boost REQUIRED COMPONENTS chrono)

#########################################
######			Executable		   ######
#########################################
IF(UNIX)
	add_definitions(-std=c++11)
ENDIF()

file(GLOB_RECURSE SRC_FILES "src/*.cpp" "include/*.h" "include/*.inl" )

GroupSources(src)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${SERIAL_INDLUDE_DIR})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${SERIAL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES})

#target_include_directories(${PROJECT_NAME} PUBLIC tinyxml2:tinyxml2)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC tinyxml2)

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


if(${OpenRAVE_FOUND})
	MESSAGE(STATUS "OpenRAVE FOUND")
	target_include_directories(${PROJECT_NAME} PUBLIC ${OpenRAVE_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenRAVE_CORE_LIBRARIES})
	set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
else(${OpenRAVE_FOUND})
	MESSAGE(WARNING "OpenRAVE NOT FOUND")
endif(${OpenRAVE_FOUND})

### DOC generation
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
	${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

### INSTALL TARGET
