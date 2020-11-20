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

add_executable(drive_in_circle src/drive_command_publisher.cpp)
ament_target_dependencies(drive_in_circle rclcpp std_msgs halodi_msgs action_msgs)

add_executable(move_left_hand src/left_hand_task_space_publisher.cpp)
ament_target_dependencies(move_left_hand rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(wave_right_hand src/wave_right_hand_publisher.cpp)
ament_target_dependencies(wave_right_hand rclcpp halodi_msgs action_msgs)

add_executable(go_to_default src/default_pose_publisher.cpp)
ament_target_dependencies(go_to_default rclcpp halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(go_wild src/go_wild_publisher.cpp)
ament_target_dependencies(go_wild rclcpp halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(camera_calib_left src/camera_calibration_trajectory_left_hand.cpp)
ament_target_dependencies(camera_calib_left rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(neck_up_down src/neck_up_down_trajectory.cpp)
ament_target_dependencies(neck_up_down rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

install(TARGETS drive_in_circle move_left_hand wave_right_hand go_to_default go_wild neck_up_down camera_calib_left DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY launch rviz
#        DESTINATION share/${PROJECT_NAME}
#        )

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
