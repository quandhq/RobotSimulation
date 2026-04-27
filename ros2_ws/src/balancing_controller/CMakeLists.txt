cmake_minimum_required(VERSION 3.8)
project(balancing_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# tell CMake where .h files are
include_directories(include)

# define the executable
add_executable(balancing_controller src/balancing_controller.cpp)

#link the ros2 libraries to the executable

ament_target_dependencies(balancing_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

#install instructions (So 'ros2 run' can find the file)
install(TARGETS
  balancing_controller
  DESTINATION lib/${PROJECT_NAME}
)


#install the include folder so that other packages can see it
install(
  DIRECTORY include/
  DESTINATION include
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
