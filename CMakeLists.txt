cmake_minimum_required(VERSION 3.5)
project(vacuum_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)

include_directories(
  include 
  /usr/local/include
)

# Executables
add_executable(step_controller_node
    src/step_controller_node.cpp
    src/easywsclient.cpp
)
add_executable(web_bridge_node
    src/web_bridge_node.cpp
    src/easywsclient.cpp
)
add_executable(coverage_nav2_node src/coverage_nav2_node.cpp)

# Libraries
# add_library(coverage_planner SHARED src/coverage_planner.cpp)

# # Link libraries
# target_link_libraries(step_controller_node easywsclient)
# target_link_libraries(web_bridge_node easywsclient)

# Dependencies
ament_target_dependencies(step_controller_node
  rclcpp
  geometry_msgs
  nav2_core
  pluginlib
)

ament_target_dependencies(web_bridge_node
  rclcpp
  geometry_msgs
)


ament_target_dependencies(coverage_nav2_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_core
  pluginlib
  Fields2Cover
  lifecycle_msgs
  opennav_coverage_msgs
)

# ament_target_dependencies(coverage_planner
#   rclcpp
#   nav2_core
#   pluginlib
#   Fields2Cover
# )

# Plugin export
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

# Install targets
install(TARGETS
  step_controller_node
  web_bridge_node
  coverage_nav2_node
  # coverage_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
