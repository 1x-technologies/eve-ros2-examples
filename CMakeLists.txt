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

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(halodi_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(default_pose src/trajectory_publishers/default_pose.cpp)
ament_target_dependencies(default_pose rclcpp halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(drive_and_turn src/trajectory_publishers/drive_and_turn.cpp)
ament_target_dependencies(drive_and_turn rclcpp std_msgs halodi_msgs action_msgs)

add_executable(full_body_extreme src/trajectory_publishers/full_body_extreme.cpp)
ament_target_dependencies(full_body_extreme rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(left_hand_task_space_box src/trajectory_publishers/left_hand_task_space_box.cpp)
ament_target_dependencies(left_hand_task_space_box rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(neck_up_down src/trajectory_publishers/neck_up_down.cpp)
ament_target_dependencies(neck_up_down rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(random_arm_motion_low_range src/trajectory_publishers/random_arm_motion_low_range.cpp)
ament_target_dependencies(random_arm_motion_low_range rclcpp std_msgs halodi_msgs action_msgs tf2 tf2_geometry_msgs)

add_executable(wave_right_hand src/trajectory_publishers/wave_right_hand.cpp)
ament_target_dependencies(wave_right_hand rclcpp halodi_msgs action_msgs)

#############
## Testing ##
#############
if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  include_directories(test)
  set(TEST_SOURCES test/gtest_main.cpp test/utils_test.cpp)
  set(TEST_PKG_EXE test_eve_ros2_examples)
  ament_add_gtest(${TEST_PKG_EXE} ${TEST_SOURCES})
  ament_target_dependencies(${TEST_PKG_EXE}  unique_identifier_msgs halodi_msgs)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

  find_package(ament_cmake_clang_tidy REQUIRED)
  # ament_clang_tidy(${CMAKE_BINARY_DIR} CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(LANGUAGE C++ EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/test")

  find_package(ament_cmake_cpplint REQUIRED)
  # ament_cpplint()

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  find_package(ament_cmake_pep257 REQUIRED)
  # ament_pep257()
  
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

#############
## Install ##
#############
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/whole_body_trajectory_example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS default_pose drive_and_turn full_body_extreme left_hand_task_space_box neck_up_down random_arm_motion_low_range wave_right_hand
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
