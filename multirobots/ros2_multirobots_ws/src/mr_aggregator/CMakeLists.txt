cmake_minimum_required(VERSION 3.8)
project(mr_aggregator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
	rclcpp
	std_msgs
	mr_interfaces
	ament_index_cpp
	)

include_directories(include)

# executable

#add_executable(mr_aggregator src/g_aggregator.cpp )
#ament_target_dependencies(xg_aggregator PUBLIC ${dependencies})


#add_executable(mr_worker  src/g_worker.cpp )
#ament_target_dependencies(xg_worker PUBLIC ${dependencies})

# install

ament_export_include_directories("include")

#install(TARGETS
#	xg_aggregator
#	xg_worker
#	ARCHIVE DESTINATION lib
#	LIBRARY DESTINATION lib
#	RUNTIME DESTINATION lib/${PROJECT_NAME}
#	)

install(DIRECTORY include/
        DESTINATION include/
        )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
