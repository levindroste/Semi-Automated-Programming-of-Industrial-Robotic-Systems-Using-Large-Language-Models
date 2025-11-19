cmake_minimum_required(VERSION 3.8)
project(ur10e_hl_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# Include directories
include_directories(
  include
  ${TinyXML2_INCLUDE_DIRS}
)

# ========================================
# Library: robot_hl_interface
# ========================================
add_library(${PROJECT_NAME}
  src/robot_hl_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  control_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  tinyxml2_vendor
)

target_link_libraries(${PROJECT_NAME}
  ${TinyXML2_LIBRARIES}
)

# ========================================
# Executable: add_objects_node
# ========================================
add_executable(add_objects_node 
  src/add_objects_node.cpp
)

ament_target_dependencies(add_objects_node 
  rclcpp 
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  shape_msgs
  geometry_msgs
  tinyxml2_vendor
)

target_link_libraries(add_objects_node
  ${TinyXML2_LIBRARIES}
)


# ========================================
# Executable: clipfix_bewegung
# ========================================
add_executable(clipfix_bewegung 
  src/clipfix_bewegung.cpp
)

target_link_libraries(clipfix_bewegung 
  ${PROJECT_NAME}
)

ament_target_dependencies(clipfix_bewegung
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
)

# ========================================
# Install section
# ========================================
# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS 
    clipfix_bewegung 
    add_objects_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install meshes if directory exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )
endif()

# ========================================
# Testing
# ========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================================
# Export dependencies
# ========================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  control_msgs
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
  tf2_eigen
  geometric_shapes
  tinyxml2_vendor
)

ament_package()

