cmake_minimum_required(VERSION 3.8)
project(topology_nav2_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(referee_msg REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(topology_global_planner REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/topology_nav2_bt_nodes.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  referee_msg
  tf2
  tf2_ros
  topology_global_planner
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES nav2_tree_nodes.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  referee_msg
  tf2
  tf2_ros
  topology_global_planner
)

ament_package()
