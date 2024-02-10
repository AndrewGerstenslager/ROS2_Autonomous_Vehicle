cmake_minimum_required(VERSION 3.5)
project(dokalman_library)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(caster_joint_publisher src/caster_joint_publisher.cpp)
ament_target_dependencies(caster_joint_publisher rclcpp sensor_msgs)

add_executable(heading_publisher src/gazebo_publish_robot_heading.cpp)
ament_target_dependencies(heading_publisher rclcpp nav_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch worlds meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install your executable
install(TARGETS
  caster_joint_publisher
  heading_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()