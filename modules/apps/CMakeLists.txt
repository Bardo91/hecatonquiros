
# Add set of tools
file(GLOB_RECURSE TOOL_SRC_velocity_test "velocity_test/src/*.h"  "velocity_test/src/*.inl" "velocity_test/src/*.cpp")
compile_tool(velocity_test)

file(GLOB_RECURSE TOOL_SRC_arm_joystick "arm_joystick/src/*.h"  "arm_joystick/src/*.inl" "arm_joystick/src/*.cpp")
compile_tool(arm_joystick)

file(GLOB_RECURSE TOOL_SRC_arm_tester "arm_tester/src/*.h" "arm_tester/src/*.inl" "arm_tester/src/*.cpp")
compile_tool(arm_tester)

file(GLOB_RECURSE TOOL_SRC_calibration_feetech "calibration_feetech/src/*.h" "calibration_feetech/src/*.inl" "calibration_feetech/src/*.cpp")
compile_tool(calibration_feetech)

file(GLOB_RECURSE TOOL_SRC_manipulator_controller "manipulator_controller/src/*.h" "manipulator_controller/src/*.inl" "manipulator_controller/src/*.cpp")
compile_tool(manipulator_controller)

file(GLOB_RECURSE TOOL_SRC_individual_arm_controller "individual_arm_controller/src/*.h" "individual_arm_controller/src/*.inl" "individual_arm_controller/src/*.cpp")
compile_tool(individual_arm_controller)

file(GLOB_RECURSE TOOL_SRC_serialPort_multi_arms_controller "serialPort_multi_arms_controller/src/*.h" "serialPort_multi_arms_controller/src/*.inl" "serialPort_multi_arms_controller/src/*.cpp")
compile_tool(serialPort_multi_arms_controller)

file(GLOB_RECURSE TOOL_SRC_manipulator_controller_tester "manipulator_controller_tester/src/*.h" "manipulator_controller_tester/src/*.inl" "manipulator_controller_tester/src/*.cpp")
compile_tool(manipulator_controller_tester)

file(GLOB_RECURSE TOOL_SRC_plotter_poses "plotter_poses/src/*.h" "plotter_poses/src/*.inl" "plotter_poses/src/*.cpp")
compile_tool_qt(plotter_poses)

file(GLOB_RECURSE TOOL_SRC_positioner_test "positioner_tester/src/*.h" "positioner_tester/src/*.inl" "positioner_tester/src/*.cpp")
compile_tool(positioner_test)

file(GLOB_RECURSE TOOL_SRC_plotter_joints "plotter_joints/src/*.h" "plotter_joints/src/*.inl" "plotter_joints/src/*.cpp")
compile_tool_qt(plotter_joints)

file(GLOB_RECURSE TOOL_SRC_positioner_ros_interface "positioner_ros_interface/src/*.h" "positioner_ros_interface/src/*.inl" "positioner_ros_interface/src/*.cpp")
compile_tool(positioner_ros_interface)

file(GLOB_RECURSE TOOL_SRC_positioner_ros_visualization "positioner_ros_visualization/src/*.h" "positioner_ros_visualization/src/*.inl" "positioner_ros_visualization/src/*.cpp")
compile_tool(positioner_ros_visualization)

file(GLOB_RECURSE TOOL_SRC_arm_position_holder_publisher_vicon "arm_position_holder_publisher_vicon/src/*.h" "arm_position_holder_publisher_vicon/src/*.inl" "arm_position_holder_publisher_vicon/src/*.cpp")
compile_tool(arm_position_holder_publisher_vicon)

file(GLOB_RECURSE TOOL_SRC_feetech_arm_tester "feetech_arm_tester/src/*.h" "feetech_arm_tester/src/*.inl" "feetech_arm_tester/src/*.cpp")
compile_tool(feetech_arm_tester)
