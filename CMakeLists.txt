###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(hecatonquiros)
set(PROJECT_VERSION 0.4.0)

# Load macros
include(cmake/hecatonquiros_macros.cmake)

# Common configuration
IF(UNIX)
	add_definitions(-std=c++11)
ENDIF()

# Add core library
find_package(catkin REQUIRED COMPONENTS
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
endif(${catkin_FOUND})

add_service_files(
  FILES
  SetJoints.srv
  SetPose.srv
  ReqData.srv
  ConfigData.srv
)

find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
endif()

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs tf
)

catkin_package(
	INCLUDE_DIRS ./arm_controller/include
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs tf tf_conversions
)

add_subdirectory(arm_controller)

# Add set of tools
file(GLOB_RECURSE TOOL_SRC_velocity_test "tools/velocity_test/src/*.h"  "tools/velocity_test/src/*.inl" "tools/velocity_test/src/*.cpp")
compile_tool(velocity_test)

file(GLOB_RECURSE TOOL_SRC_arm_joystick "tools/arm_joystick/src/*.h"  "tools/arm_joystick/src/*.inl" "tools/arm_joystick/src/*.cpp")
compile_tool(arm_joystick)

file(GLOB_RECURSE TOOL_SRC_arm_tester "tools/arm_tester/src/*.h" "tools/arm_tester/src/*.inl" "tools/arm_tester/src/*.cpp")
compile_tool(arm_tester)

file(GLOB_RECURSE TOOL_SRC_calibration_feetech "tools/calibration_feetech/src/*.h" "tools/calibration_feetech/src/*.inl" "tools/calibration_feetech/src/*.cpp")
compile_tool(calibration_feetech)

file(GLOB_RECURSE TOOL_SRC_manipulator_controller "tools/manipulator_controller/src/*.h" "tools/manipulator_controller/src/*.inl" "tools/manipulator_controller/src/*.cpp")
compile_tool(manipulator_controller)

file(GLOB_RECURSE TOOL_SRC_individual_arm_controller "tools/individual_arm_controller/src/*.h" "tools/individual_arm_controller/src/*.inl" "tools/individual_arm_controller/src/*.cpp")
compile_tool(individual_arm_controller)

file(GLOB_RECURSE TOOL_SRC_serialPort_multi_arms_controller "tools/serialPort_multi_arms_controller/src/*.h" "tools/serialPort_multi_arms_controller/src/*.inl" "tools/serialPort_multi_arms_controller/src/*.cpp")
compile_tool(serialPort_multi_arms_controller)

file(GLOB_RECURSE TOOL_SRC_manipulator_controller_tester "tools/manipulator_controller_tester/src/*.h" "tools/manipulator_controller_tester/src/*.inl" "tools/manipulator_controller_tester/src/*.cpp")
compile_tool(manipulator_controller_tester)

file(GLOB_RECURSE TOOL_SRC_plotter_poses "tools/plotter_poses/src/*.h" "tools/plotter_poses/src/*.inl" "tools/plotter_poses/src/*.cpp")
compile_tool_qt(plotter_poses)

file(GLOB_RECURSE TOOL_SRC_positioner_test "tools/positioner_tester/src/*.h" "tools/positioner_tester/src/*.inl" "tools/positioner_tester/src/*.cpp")
compile_tool(positioner_test)


file(GLOB_RECURSE TOOL_SRC_plotter_joints "tools/plotter_joints/src/*.h" "tools/plotter_joints/src/*.inl" "tools/plotter_joints/src/*.cpp")
compile_tool_qt(plotter_joints)

file(GLOB_RECURSE TOOL_SRC_positioner_ros_interface "tools/positioner_ros_interface/src/*.h" "tools/positioner_ros_interface/src/*.inl" "tools/positioner_ros_interface/src/*.cpp")
compile_tool(positioner_ros_interface)

file(GLOB_RECURSE TOOL_SRC_positioner_ros_visualization "tools/positioner_ros_visualization/src/*.h" "tools/positioner_ros_visualization/src/*.inl" "tools/positioner_ros_visualization/src/*.cpp")
compile_tool(positioner_ros_visualization)

file(GLOB_RECURSE TOOL_SRC_arm_position_holder_publisher_vicon "tools/arm_position_holder_publisher_vicon/src/*.h" "tools/arm_position_holder_publisher_vicon/src/*.inl" "tools/arm_position_holder_publisher_vicon/src/*.cpp")
compile_tool(arm_position_holder_publisher_vicon)

file(GLOB_RECURSE TOOL_SRC_positioner_workspace_plotter "tools/positioner_workspace_plotter/src/*.h" "tools/positioner_workspace_plotter/src/*.inl" "tools/positioner_workspace_plotter/src/*.cpp")
compile_tool(positioner_workspace_plotter)

