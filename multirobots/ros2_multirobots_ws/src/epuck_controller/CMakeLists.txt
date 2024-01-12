cmake_minimum_required(VERSION 3.8)
project(epuck_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mr_aggregator REQUIRED)
find_package(mr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)


set(dependencies
	rclcpp
	sensor_msgs
	geometry_msgs
	nav_msgs
	tf2
	tf2_geometry_msgs
	mr_aggregator
	mr_interfaces
	ament_index_cpp
	)

include_directories(include)

# executable

add_executable(controller src/main.cpp src/control.cpp src/odometry.cpp
	src/laser_scan.cpp src/control/shared.cpp src/control/dfa.cpp
	src/control/sensors_specific.cpp src/ccomputations.cpp src/utils.cpp)
ament_target_dependencies(controller PUBLIC ${dependencies})

# install

install(TARGETS
	controller
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()

	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()

	find_package(ament_cmake_gtest REQUIRED)

	#add_subdirectory(tests)
endif()


#ament_export_include_directories(include)
#ament_export_dependencies(${dependencies})
ament_package()
