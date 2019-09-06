macro(HECATONQUIROS_EXMAPLE app_name main_file)
	add_executable(${app_name} ${main_file})
	target_include_directories(${app_name} PUBLIC ${PROJECT_SOURCE_DIR}/include)
	target_link_libraries(${app_name} LINK_PUBLIC ${PROJECT_NAME})
	
	target_include_directories(${app_name} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${app_name} LINK_PUBLIC ${catkin_LIBRARIES})
endmacro()


HECATONQUIROS_EXMAPLE(example_openrave examples/example_openrave.cpp)