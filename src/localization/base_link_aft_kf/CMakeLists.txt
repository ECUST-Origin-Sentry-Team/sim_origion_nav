cmake_minimum_required(VERSION 3.8)
project(base_link_aft_kf)

add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(base_link_aft_kf
  src/kf_data.cpp
  src/kf_node.cpp
)

ament_target_dependencies(base_link_aft_kf
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS base_link_aft_kf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
