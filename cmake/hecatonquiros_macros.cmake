macro(compile_tool tool_name)

	add_executable(${tool_name} ${TOOL_SRC_${tool_name}})

	target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${tool_name} ${catkin_LIBRARIES} hecatonquiros)

	add_dependencies(${tool_name} ${catkin_EXPORTED_TARGETS} hecatonquiros_generate_messages_cpp)

endmacro()

macro(compile_library_tool tool_name)

	add_library(${tool_name} ${TOOL_SRC_${tool_name}})

	target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${tool_name} ${catkin_LIBRARIES} hecatonquiros)

	add_dependencies(${tool_name} ${catkin_EXPORTED_TARGETS} hecatonquiros_generate_messages_cpp)

endmacro()
