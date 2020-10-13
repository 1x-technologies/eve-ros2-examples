cmake_minimum_required(VERSION 3.5)
project(eve-ros2-examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(halodi_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(drive_command_publisher src/drive_command_publisher.cpp)
ament_target_dependencies(drive_command_publisher rclcpp std_msgs halodi_msgs action_msgs)

add_executable(whole_body_trajectory_publisher src/whole_body_trajectory_publisher.cpp)
ament_target_dependencies(whole_body_trajectory_publisher rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(wave_right_hand_publisher src/wave_right_hand_publisher.cpp)
ament_target_dependencies(wave_right_hand_publisher rclcpp halodi_msgs action_msgs)

install(TARGETS drive_command_publisher whole_body_trajectory_publisher wave_right_hand_publisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
