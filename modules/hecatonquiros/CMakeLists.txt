###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 3.2 FATAL_ERROR)

#########################################
###### Loading 3rd party libraries.######
#########################################

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# Add ROS library
find_package(catkin COMPONENTS
			geometry_msgs
			sensor_msgs
			std_msgs
			roscpp
			message_generation
			tf
			tf_conversions
			)
if(${catkin_FOUND})
	add_definitions(-DHAS_ROS)

	add_service_files(
	  FILES
	  SetJoints.srv
	  SetPose.srv
	  ReqData.srv
	  ConfigData.srv
	)

	generate_messages(
	    DEPENDENCIES std_msgs geometry_msgs sensor_msgs tf
	)

	catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs tf tf_conversions
	)

endif(${catkin_FOUND})

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


## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()


## Load PCL
find_package(PCL QUIET)

## Loading boost
if(WIN32)
	set(Boost_USE_STATIC_LIBS ON)
endif()
find_package(Boost REQUIRED COMPONENTS chrono)

#########################################
######			Executable		   ######
#########################################

file(GLOB_RECURSE SRC_FILES "src/*.cpp" "include/*.h" "include/*.inl" )

add_library(hecatonquiros ${SRC_FILES})
target_compile_features(hecatonquiros PUBLIC cxx_std_11)
target_include_directories(hecatonquiros PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
												$<INSTALL_INTERFACE:include)

target_include_directories(hecatonquiros PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hecatonquiros LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(hecatonquiros PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(hecatonquiros LINK_PUBLIC ${PCL_LIBRARIES})

target_include_directories(hecatonquiros PUBLIC ${SERIAL_INDLUDE_DIR})
target_link_libraries(hecatonquiros LINK_PUBLIC ${SERIAL_LIBRARIES})

target_link_libraries(hecatonquiros LINK_PUBLIC tinyxml2)

target_include_directories(hecatonquiros PUBLIC "/usr/include/eigen3")


if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	target_compile_definitions(hecatonquiros PUBLIC ENABLE_PCL)
	target_compile_definitions(hecatonquiros PUBLIC ${PCL_DEFINITIONS})
endif()

if(${catkin_FOUND})
	target_include_directories(hecatonquiros PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(hecatonquiros LINK_PUBLIC ${catkin_LIBRARIES})
	add_dependencies(hecatonquiros ${catkin_EXPORTED_TARGETS} hecatonquiros_generate_messages_cpp)
endif(${catkin_FOUND})

if(${OpenRAVE_FOUND})
	MESSAGE(STATUS "OpenRAVE FOUND")
	target_include_directories(hecatonquiros PUBLIC ${OpenRAVE_INCLUDE_DIRS})
	target_link_libraries(hecatonquiros LINK_PUBLIC ${OpenRAVE_CORE_LIBRARIES})
	set_target_properties(hecatonquiros PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
	set_target_properties(hecatonquiros PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
else(${OpenRAVE_FOUND})
	MESSAGE(WARNING "OpenRAVE NOT FOUND")
endif(${OpenRAVE_FOUND})

