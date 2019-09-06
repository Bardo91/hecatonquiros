###############################################################
######                                                   ######
######		  Author: Manuel P. J. (aka. manuoso)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

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
find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			std_msgs
			roscpp
			hecatonquiros
            )

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs hecatonquiros geometry_msgs
)

## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()

#########################################
######			Executable		   ######
#########################################
IF(UNIX)
	add_definitions(-std=c++11)
ENDIF()

file(GLOB_RECURSE SRC_FILES "src/*.cpp")
file(GLOB_RECURSE HDR_FILES "src/*.h" "src/*.inl" "include/*.h")

set(TOOL_NAME serialPort_multi_arms_controller)

add_executable(${TOOL_NAME} ${SRC_FILES} ${HDR_FILES}) 
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${TOOL_NAME} pthread)

target_include_directories(${TOOL_NAME} PUBLIC "/usr/include/eigen3")

target_include_directories(${TOOL_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${TOOL_NAME} ${catkin_LIBRARIES} hecatonquiros)

add_dependencies(${TOOL_NAME} ${catkin_EXPORTED_TARGETS} ${TOOL_NAME}_generate_messages_cpp)
