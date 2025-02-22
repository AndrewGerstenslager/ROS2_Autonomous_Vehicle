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
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  
find_package(tf2_msgs REQUIRED) 


# build packages
add_executable(static_feed src/static_image_publisher.cpp)
ament_target_dependencies(static_feed rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)


add_executable(live_feed src/live_image_publisher.cpp)
ament_target_dependencies(live_feed rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(image_threshold src/image_thresholding.cpp)
ament_target_dependencies(image_threshold rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(image_skew src/image_skewing.cpp)
ament_target_dependencies(image_skew rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(image_to_pointcloud src/image_to_pointcloud.cpp)
ament_target_dependencies(image_to_pointcloud rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV pcl_conversions geometry_msgs)

add_executable(calibrate_camera_feed src/calibrate_camera_feed.cpp)
ament_target_dependencies(calibrate_camera_feed rclcpp sensor_msgs image_transport cv_bridge OpenCV)

# install packages
install(TARGETS
  static_feed
  live_feed
  image_threshold
  image_skew
  image_to_pointcloud
  calibrate_camera_feed
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY calibration_data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()