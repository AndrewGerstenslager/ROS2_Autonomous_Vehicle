cmake_minimum_required(VERSION 3.5)
project(cpp_image_processing)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# build packages
add_executable(raw_image_static src/static_image_publisher.cpp)
ament_target_dependencies(raw_image_static rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(line_threshold src/line_thresholding.cpp)
ament_target_dependencies(line_threshold rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(live_feed src/live_image_publisher.cpp)
ament_target_dependencies(live_feed rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(image_skew src/image_skewing.cpp)
ament_target_dependencies(image_skew rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)


# install packages
install(TARGETS
  raw_image_static
  line_threshold
  live_feed
  image_skew
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY calibration_data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()