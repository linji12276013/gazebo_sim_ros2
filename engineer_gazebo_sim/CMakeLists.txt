cmake_minimum_required(VERSION 3.8)
project(engineer_gazebo_sim)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
  boeing_gazebo_model_attachment_plugin_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(boeing_gazebo_model_attachment_plugin_msgs REQUIRED)


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

add_executable(engineer_simple_control gazebo_classic/src/engineer_simple_control.cpp)
ament_target_dependencies(engineer_simple_control "rclcpp" "geometry_msgs" "moveit_ros_planning_interface")

add_executable(engineer_servo_control gazebo_classic/src/engineer_servo_control.cpp)
ament_target_dependencies(engineer_servo_control "rclcpp" "geometry_msgs" "moveit_ros_planning_interface" "moveit_servo")

add_library(servo_controller_input SHARED gazebo_classic/src/joystick_servo_example.cpp)
ament_target_dependencies(servo_controller_input ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(servo_controller_input "engineer::JoyToServoPub")

add_executable(servo_keyboard_input gazebo_classic/src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input "rclcpp" "geometry_msgs" "control_msgs")

add_executable(image_raw_process gazebo_classic/src/image_raw_process.cpp)
ament_target_dependencies(image_raw_process "rclcpp" "sensor_msgs" "OpenCV" "cv_bridge")

add_executable(chassis_pid_control gazebo_classic/src/chassis_pid_control.cpp)
ament_target_dependencies(chassis_pid_control "rclcpp" "geometry_msgs" "nav_msgs" "tf2_ros")

install(TARGETS 
  keyboard_control exchanger_control engineer_simple_control engineer_servo_control servo_keyboard_input image_raw_process chassis_pid_control 
  DESTINATION lib/${PROJECT_NAME}
)

install (TARGETS
  servo_controller_input
  DESTINATION lib
)

ament_package()
