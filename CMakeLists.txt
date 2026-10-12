
cmake_minimum_required(VERSION 3.8)
project(topology_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(map_server REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(TOPO_LIB_NAME ${PROJECT_NAME}_core)

add_library(${TOPO_LIB_NAME} SHARED
  # Core
  src/core/planner_lifecycle.cpp
  src/core/path_planning.cpp
  # Topology
  src/algo/topology/topology_io.cpp
  src/algo/topology/topology_search.cpp
  src/algo/topology/portal_optimizer.cpp
  src/algo/topology/geometry_utils.cpp
  # Grid
  src/algo/grid/navfn.cpp
  src/algo/grid/grid_planner.cpp
)

target_include_directories(${TOPO_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${TOPO_LIB_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${TOPO_LIB_NAME}
  rclcpp
  map_server
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

target_link_libraries(${TOPO_LIB_NAME}
  yaml-cpp
)

install(TARGETS
  ${TOPO_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${TOPO_LIB_NAME})

ament_export_dependencies(
  rclcpp
  map_server
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

ament_package()
