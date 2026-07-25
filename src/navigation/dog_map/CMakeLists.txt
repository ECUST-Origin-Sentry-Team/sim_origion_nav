cmake_minimum_required(VERSION 3.8)
project(dog_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3)
add_compile_definitions(TBB_DO)
# add_compile_definitions(DEBUG)
add_compile_definitions(FIX_MAP)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
include_directories(
  include
)
set(SOURCE_FILES
  src/occ_map.cpp
  src/node.cpp
  src/static_fix_map.cpp
)
add_executable(dog_map_node ${SOURCE_FILES})



ament_target_dependencies(
  dog_map_node
  rclcpp
  message_filters
  PCL
  tf2_ros
  nav_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  OpenCV
)
target_link_libraries(dog_map_node
TBB::tbb
  yaml-cpp
)
install(TARGETS
  dog_map_node
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
