cmake_minimum_required(VERSION 3.8)
project(sc_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(sc_relocalization_node
  src/sc_relocalization_node.cpp
  src/ScanContext.cpp
  src/Relocalization.cpp
  src/Mapping.cpp
)

target_link_libraries(sc_relocalization_node
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
  OpenMP::OpenMP_CXX
  Qt5::Core
)

ament_target_dependencies(sc_relocalization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  small_gicp
)

# Install
install(TARGETS sc_relocalization_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
