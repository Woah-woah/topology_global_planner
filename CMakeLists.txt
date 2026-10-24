
cmake_minimum_required(VERSION 3.8)
project(topology_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# EDT 与浮点搜索用于实时导航，未指定构建类型时启用优化。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
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
  # 规划入口
  src/planner_lifecycle.cpp
  src/path_planning.cpp
  # 拓扑搜索与几何
  src/topology/topology_io.cpp
  src/topology/topology_search.cpp
  src/topology/portal_optimizer.cpp
  src/topology/geometry_utils.cpp
  # 栅格规划
  src/grid/eikonal_field.cpp
  src/grid/grid_planner.cpp
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
