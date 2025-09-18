cmake_minimum_required(VERSION 3.8)
project(engineer_gazebo_sim)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


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

install(
  DIRECTORY gazebo_classic/launch 
            gazebo_classic/worlds 
            gazebo_classic/config
            gazebo_classic/models
  DESTINATION share/${PROJECT_NAME}/gazebo_classic
)

add_executable(keyboard_control gazebo_classic/src/keyboard_control.cpp)
ament_target_dependencies(keyboard_control "rclcpp" "geometry_msgs")

add_executable(exchanger_control gazebo_classic/src/exchanger_control.cpp)
ament_target_dependencies(exchanger_control "rclcpp" "geometry_msgs" "moveit_ros_planning_interface")

install(TARGETS 
  keyboard_control exchanger_control
  DESTINATION lib/${PROJECT_NAME}/gazebo_classic
)

ament_package()