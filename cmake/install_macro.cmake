macro(install_hecatonquiros)
	include(CMakePackageConfigHelpers)

	if(NOT DEFINED CMAKE_INSTALL_PREFIX)
		set(CMAKE_INSTALL_PREFIX "/usr/local")
	endif() 

	install(TARGETS hecatonquiros
			EXPORT hecatonquirosTargets
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib
			INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
			)

	write_basic_package_version_file(
		hecatonquirosConfigVersion.cmake
		VERSION ${PACKAGE_VERSION}
		COMPATIBILITY AnyNewerVersion
		)


	# This installs the include folder
	install(DIRECTORY modules/hecatonquiros/include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
	install(DIRECTORY modules/hecatonquiros/include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

	export(TARGETS hecatonquiros NAMESPACE hecatonquiros:: FILE hecatonquirosTargets.cmake)

	# This generates hecatonquirosTargets.cmake
	install(EXPORT hecatonquirosTargets
		FILE hecatonquirosTargets.cmake
		NAMESPACE hecatonquiros::
		DESTINATION lib/cmake/hecatonquiros
		)

	configure_file(cmake/hecatonquirosConfig.cmake.in hecatonquirosConfig.cmake @ONLY)
	install(FILES "${CMAKE_CURRENT_BINARY_DIR}/hecatonquirosConfig.cmake"
			DESTINATION lib/cmake/hecatonquiros
			)



	# uninstall target 
	if(NOT TARGET uninstall) 
	configure_file( 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in" 
		"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
		IMMEDIATE @ONLY) 
	
	add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
	endif() 

endmacro()

macro(install_hecatonquiros_apps)
install(TARGETS 	velocity_test
					arm_joystick
					arm_tester
					calibration_feetech
					manipulator_controller
					individual_arm_controller
					serialPort_multi_arms_controller
					manipulator_controller_tester
					plotter_poses
					plotter_joints
					positioner_test
					positioner_ros_interface
					positioner_ros_visualization
					arm_position_holder_publisher_vicon
					feetech_arm_tester
				
				ARCHIVE DESTINATION bin
	)

endmacro()
