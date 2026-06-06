cmake_minimum_required(VERSION 3.8)
project(topology_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(PLUGIN_LIB_NAME ${PROJECT_NAME}_plugin)

add_library(${PLUGIN_LIB_NAME} SHARED
  src/planner_lifecycle.cpp
  src/topology_io.cpp
  src/geometry_utils.cpp
  src/topology_search.cpp
  src/portal_optimizer.cpp
  src/path_planning.cpp
)

target_include_directories(${PLUGIN_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PLUGIN_LIB_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PLUGIN_LIB_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${PLUGIN_LIB_NAME}
  yaml-cpp
)

pluginlib_export_plugin_description_file(nav2_core topology_global_planner_plugin.xml)

install(TARGETS ${PLUGIN_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES topology_global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PLUGIN_LIB_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SwitchRouteMode.srv"
)
rosidl_target_interfaces(${PLUGIN_LIB_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
