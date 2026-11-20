cmake_minimum_required(VERSION 3.8)
project(advrf_ros2_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(advrf_ros2_interfaces REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

install(
  DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

add_library(joint_feedback SHARED src/joint_feedback.cpp)
target_compile_features(joint_feedback PUBLIC cxx_std_17)
target_link_libraries(joint_feedback
  gz-plugin2::register
  gz-sim8::gz-sim8
)

install(TARGETS joint_feedback
  LIBRARY DESTINATION lib
)

ament_package()
