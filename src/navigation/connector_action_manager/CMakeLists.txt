cmake_minimum_required(VERSION 3.8)
project(connector_action_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(referee_msg REQUIRED)


find_package(topology_global_planner REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/connector_action_manager_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  std_msgs
  referee_msg
  topology_global_planner
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  std_msgs
  topology_global_planner
)

ament_package()
