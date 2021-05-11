cmake_minimum_required(VERSION 3.5)
project(eve_ros2_examples)

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

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(drive_in_circle src/trajectory_publishers/drive_command_publisher.cpp)
ament_target_dependencies(drive_in_circle rclcpp std_msgs halodi_msgs action_msgs)

add_executable(move_left_hand src/trajectory_publishers/left_hand_task_space_publisher.cpp)
ament_target_dependencies(move_left_hand rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(wave_right_hand src/trajectory_publishers/wave_right_hand_publisher.cpp)
ament_target_dependencies(wave_right_hand rclcpp halodi_msgs action_msgs)

add_executable(go_to_default src/trajectory_publishers/default_pose_publisher.cpp)
ament_target_dependencies(go_to_default rclcpp halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(neck_up_down src/trajectory_publishers/neck_up_down_trajectory.cpp)
ament_target_dependencies(neck_up_down rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(go_wild src/trajectory_publishers/go_wild.cpp)
ament_target_dependencies(go_wild rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(random_walk src/trajectory_publishers/random_walk.cpp)
ament_target_dependencies(random_walk rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

install(TARGETS random_walk go_wild drive_in_circle move_left_hand wave_right_hand go_to_default neck_up_down DESTINATION lib/${PROJECT_NAME})

# Install Python executables
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/whole_body_trajectory_example.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  
  find_package(ament_cmake_clang_tidy REQUIRED)
  ament_clang_tidy(${CMAKE_BINARY_DIR} CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()  
endif()

ament_package()
