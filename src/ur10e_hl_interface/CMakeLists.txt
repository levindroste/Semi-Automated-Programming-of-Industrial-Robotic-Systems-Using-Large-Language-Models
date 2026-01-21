cmake_minimum_required(VERSION 3.8)
project(ur10e_hl_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# Include directories
include_directories(
  include
  ${TinyXML2_INCLUDE_DIRS}
)

# ========================================
# Library: robot_hl_interface_simple
# ========================================
add_library(robot_hl_interface_simple
  src/robot_hl_interface_simple.cpp
)

target_include_directories(robot_hl_interface_simple PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_hl_interface_simple
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
  tinyxml2_vendor
  geometric_shapes
)

target_link_libraries(robot_hl_interface_simple
  ${TinyXML2_LIBRARIES}
)

# ========================================
# Executable: pick_and_place_test
# ========================================
add_executable(pick_and_place_test
  src/pick_and_place_test.cpp
)

target_link_libraries(pick_and_place_test
  robot_hl_interface_simple
)

ament_target_dependencies(pick_and_place_test
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
)

# ========================================
# Executable: script (LLM-generated code)
# ========================================
add_executable(script
  src/script.cpp
)

target_link_libraries(script
  robot_hl_interface_simple
)

ament_target_dependencies(script
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
)

# ========================================
# Install section
# ========================================
# Install libraries
install(
  TARGETS robot_hl_interface_simple
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS
    pick_and_place_test
    script
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

# Install Python scripts
install(
  PROGRAMS
    scripts/execute_on_real_robot.py
    scripts/irb120_control.py
    scripts/add_collision_objects.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
  geometric_shapes
  tinyxml2_vendor
)

ament_package()

