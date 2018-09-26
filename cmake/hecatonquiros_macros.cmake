macro(compile_tool tool_name)

	add_executable(${tool_name} ${TOOL_SRC_${tool_name}})

	target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${tool_name} ${catkin_LIBRARIES} hecatonquiros)

	add_dependencies(${tool_name} ${catkin_EXPORTED_TARGETS} hecatonquiros_generate_messages_cpp)

endmacro()

macro(compile_tool_qt tool_name)
	find_package(Qt5Widgets REQUIRED)
	find_package(Qt5PrintSupport REQUIRED)
	set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
	set(CMAKE_AUTOMOC ON)


	add_executable(${tool_name} ${TOOL_SRC_${tool_name}})

	target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${tool_name} ${catkin_LIBRARIES} hecatonquiros)

	target_link_libraries		(${tool_name} Qt5::Widgets)
	target_link_libraries		(${tool_name} Qt5::PrintSupport)

	add_dependencies(${tool_name} ${catkin_EXPORTED_TARGETS} hecatonquiros_generate_messages_cpp)

endmacro()

macro(compile_library_tool tool_name)

	add_library(${tool_name} ${TOOL_SRC_${tool_name}})

	target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${tool_name} ${catkin_LIBRARIES} hecatonquiros)

	add_dependencies(${tool_name} ${catkin_EXPORTED_TARGETS} hecatonquiros_generate_messages_cpp)

endmacro()
