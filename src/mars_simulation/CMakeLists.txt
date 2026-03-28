cmake_minimum_required(VERSION 3.8)
project(mars_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Drone velocity control plugin
add_library(drone_plugin SHARED src/drone_plugin.cpp)
target_include_directories(drone_plugin PUBLIC
  ${gazebo_dev_INCLUDE_DIRS}
)
ament_target_dependencies(drone_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install plugin
install(TARGETS drone_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install models, worlds, launch, config
install(DIRECTORY
  models
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
