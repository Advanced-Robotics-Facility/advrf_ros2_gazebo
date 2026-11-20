cmake_minimum_required(VERSION 3.8)
project(advrf_ros2_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(advrf_ros2_interfaces REQUIRED)

install(
  DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
