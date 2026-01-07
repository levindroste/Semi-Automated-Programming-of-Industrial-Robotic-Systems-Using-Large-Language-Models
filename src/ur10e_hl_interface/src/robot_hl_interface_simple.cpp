#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

#include <chrono>
#include <sstream>
#include <fstream>
#include <thread>
#include <iomanip>
#include <set>
#include <algorithm>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// For mesh loading (AddCube)
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

using namespace std::chrono_literals;

// Color map for AddCube (German color names -> RGBA)
namespace {
  std::map<std::string, std::array<float, 4>> COLOR_MAP = {
    {"Schwarz", {0.2f, 0.2f, 0.2f, 1.0f}},
    {"Rot", {1.0f, 0.0f, 0.0f, 1.0f}},
    {"Gruen", {0.0f, 1.0f, 0.0f, 1.0f}},
    {"Gelb", {1.0f, 1.0f, 0.0f, 1.0f}},
    {"Blau", {0.0f, 0.0f, 1.0f, 1.0f}},
    {"Grau", {0.5f, 0.5f, 0.5f, 1.0f}},
    {"Orange", {1.0f, 0.5f, 0.0f, 1.0f}},
  };

  // Minimum Z height for real robot movements (real robot table is higher than simulation)
  // Calibrated: z=0.18m was 4cm too high, adjusted to 0.15m (1cm higher than 0.14m)
  constexpr double REAL_ROBOT_MIN_Z = 0.15;
}

shape_msgs::msg::Mesh RobotHLInterfaceSimple::loadSTLMesh(const std::string& filepath, double scale)
{
  shape_msgs::msg::Mesh mesh;
  std::ifstream file(filepath);

  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open STL file: %s", filepath.c_str());
    return mesh;
  }

  std::string line;
  std::vector<uint32_t> current_triangle;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    iss >> token;

    if (token == "vertex") {
      double x, y, z;
      iss >> x >> y >> z;
      geometry_msgs::msg::Point pt;
      pt.x = x * scale;
      pt.y = y * scale;
      pt.z = z * scale;
      current_triangle.push_back(static_cast<uint32_t>(mesh.vertices.size()));
      mesh.vertices.push_back(pt);
    } else if (token == "endfacet" && current_triangle.size() == 3) {
      shape_msgs::msg::MeshTriangle tri;
      tri.vertex_indices[0] = current_triangle[0];
      tri.vertex_indices[1] = current_triangle[1];
      tri.vertex_indices[2] = current_triangle[2];
      mesh.triangles.push_back(tri);
      current_triangle.clear();
    }
  }

  RCLCPP_INFO(this->get_logger(), "Loaded STL mesh: %zu vertices, %zu triangles",
              mesh.vertices.size(), mesh.triangles.size());
  return mesh;
}

RobotHLInterfaceSimple::RobotHLInterfaceSimple()
  : Node("robot_hl_interface_simple",
         rclcpp::NodeOptions()
           .automatically_declare_parameters_from_overrides(true)
           .allow_undeclared_parameters(true))
{
  // Declare and set use_sim_time for Gazebo simulation
  // This must be done early, before any subscriptions that depend on time
  if (!this->has_parameter("use_sim_time")) {
    this->declare_parameter("use_sim_time", true);
  }
  this->set_parameter(rclcpp::Parameter("use_sim_time", true));
  RCLCPP_INFO(this->get_logger(), "use_sim_time set to true");

  // Declare parameters (only if not already declared from launch file)
  if (!this->has_parameter("aml_file")) {
    this->declare_parameter("aml_file", std::string(""));
  }

  // Real robot mode parameters
  if (!this->has_parameter("real_robot")) {
    this->declare_parameter("real_robot", false);
  }
  if (!this->has_parameter("robot_ip")) {
    this->declare_parameter("robot_ip", std::string("192.168.125.1"));
  }
  if (!this->has_parameter("robot_port")) {
    this->declare_parameter("robot_port", 5000);
  }

  // Initialize default orientation (180° rotation around x-axis)
  config_.standard_orientation.w = 0.0;
  config_.standard_orientation.x = 1.0;
  config_.standard_orientation.y = 0.0;
  config_.standard_orientation.z = 0.0;

  // Gripper touch links for attachment
  gripper_touch_links_ = {
    "irb120_tool0",
    "gripper_base",
    "left_finger",
    "right_finger"
  };

  RCLCPP_INFO(this->get_logger(), "RobotHLInterfaceSimple node created");
}

bool RobotHLInterfaceSimple::initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing RobotHLInterfaceSimple...");

  // Get AML file path
  aml_file_path_ = this->get_parameter("aml_file").as_string();
  if (aml_file_path_.empty()) {
    // Default path
    aml_file_path_ = std::string(std::getenv("HOME")) +
      "/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models"
      "/src/ur10e_hl_interface/config/irb120_simple_config.aml";
  }
  RCLCPP_INFO(this->get_logger(), "Using AML file: %s", aml_file_path_.c_str());

  // Get mesh directory for AddCube function
  mesh_directory_ = ament_index_cpp::get_package_share_directory("ur10e_hl_interface") + "/meshes";
  RCLCPP_INFO(this->get_logger(), "Mesh directory: %s", mesh_directory_.c_str());

  // Create MoveIt interfaces
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "irb120_arm");
  planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Create service client for ACM updates (PlanningSceneInterface doesn't apply ACM correctly)
  apply_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

  // Create publisher for planning scene updates (same approach as add_objects_node.cpp)
  scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  rclcpp::sleep_for(std::chrono::seconds(1));  // Wait for subscriber connection

  // Get frame info
  eef_link_ = move_group_->getEndEffectorLink();
  planning_frame_ = move_group_->getPlanningFrame();
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s, EEF link: %s",
              planning_frame_.c_str(), eef_link_.c_str());

  // Configure MoveIt
  move_group_->setMaxVelocityScalingFactor(config_.velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  move_group_->setPlanningTime(config_.planning_time);
  move_group_->setPlannerId(config_.planner_id);
  move_group_->setGoalPositionTolerance(0.001);
  move_group_->setGoalOrientationTolerance(0.01);

  // Start state monitor
  move_group_->startStateMonitor();

  // Parse AML file
  if (!parseAMLFile(aml_file_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse AML file");
    return false;
  }

  // Skip adding collision objects - the simulation already adds them via add_collision_objects.py
  // if (!addCollisionObjects()) {
  //   RCLCPP_WARN(this->get_logger(), "Failed to add some collision objects");
  // }

  // Don't set ACM explicitly - let MoveIt handle collision checking normally
  // The touch_links in attachObject() handles gripper-object collisions during pick
  // (This matches the old working code from ur10_ws(alt) which never set ACM)

  RCLCPP_INFO(this->get_logger(), "Initialization complete. Loaded %zu grid positions and %zu objects",
              grid_positions_.size(), objects_.size());

  // Initialize real robot mode if enabled
  real_robot_mode_ = this->get_parameter("real_robot").as_bool();
  robot_ip_ = this->get_parameter("robot_ip").as_string();
  robot_port_ = this->get_parameter("robot_port").as_int();

  if (real_robot_mode_) {
    RCLCPP_WARN(this->get_logger(), "=== REAL ROBOT MODE ENABLED ===");
    RCLCPP_INFO(this->get_logger(), "Connecting to robot at %s:%d", robot_ip_.c_str(), robot_port_);

    if (!connectToRealRobot()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to real robot!");
      RCLCPP_ERROR(this->get_logger(), "Make sure:");
      RCLCPP_ERROR(this->get_logger(), "  1. Robot is powered on");
      RCLCPP_ERROR(this->get_logger(), "  2. SocketMain is running on FlexPendant");
      RCLCPP_ERROR(this->get_logger(), "  3. Robot is in AUTO mode with Motors On");
      return false;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Simulation mode - no real robot connection");
  }

  return true;
}

// === MAIN INTERFACE ===

bool RobotHLInterfaceSimple::PickAndPlace(const std::string& object_name, const std::string& goal_name, int level)
{
  RCLCPP_INFO(this->get_logger(), "=== PickAndPlace: %s -> %s (level %d) ===",
              object_name.c_str(), goal_name.c_str(), level);

  // Constants for stacking calculations
  const double CUBE_HEIGHT = 0.03;  // 3cm cube height
  const double CUBE_GAP = 0.0;      // No gap - cubes stack directly (30mm per level)
  const double BASE_GRIP_Z = grip_offset_.z;  // Grip height from config (default 0.1m)

  // Validate object exists
  auto obj_it = objects_.find(object_name);
  if (obj_it == objects_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Object '%s' not found", object_name.c_str());
    return false;
  }
  ObjectInfo& object = obj_it->second;

  // Validate goal exists (check both standard grid and special positions)
  if (!isValidGridName(goal_name)) {
    RCLCPP_ERROR(this->get_logger(), "Goal position '%s' not found", goal_name.c_str());
    return false;
  }

  // Check gripper clearance for place position (position in front must be free)
  // Note: "X" (eject position) is always accessible
  if (goal_name != "X" && !isGripperClearanceOk(goal_name)) {
    // Auto-move blocking cube out of the way
    std::string blocking_cube = findBlockingCube(goal_name);
    if (!blocking_cube.empty()) {
      std::string temp_pos = findFreeRow1Position();
      if (!temp_pos.empty()) {
        // Track this position as claimed during recursive auto-move
        pending_auto_move_destinations_.insert(temp_pos);

        RCLCPP_WARN(this->get_logger(),
          "Place position %s blocked by %s - moving to %s temporarily",
          goal_name.c_str(), blocking_cube.c_str(), temp_pos.c_str());

        // Recursively call PickAndPlace to move blocking cube
        bool success = PickAndPlace(blocking_cube, temp_pos, 0);

        // Release the pending claim (move completed or failed)
        pending_auto_move_destinations_.erase(temp_pos);

        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Failed to move blocking cube %s", blocking_cube.c_str());
          return false;
        }
        // Now clearance should be OK - continue with original operation
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "Cannot place at %s: no free position to move blocking cube",
          goal_name.c_str());
        return false;
      }
    }
  }

  // Check if goal at this level is already occupied (by another object)
  // Build the full location string to compare (e.g., "A1" for level 0, "A1:1" for level 1)
  std::string target_location = (level > 0) ? (goal_name + ":" + std::to_string(level)) : goal_name;
  for (const auto& [name, info] : objects_) {
    if (name != object_name && info.location == target_location) {
      RCLCPP_ERROR(this->get_logger(), "Goal '%s' at level %d is already occupied by '%s'",
                   goal_name.c_str(), level, name.c_str());
      return false;
    }
  }

  // Get current object position (extract base position if stacked, e.g., "A1:1" -> "A1")
  std::string pick_base_position = object.location;
  int pick_level = 0;
  size_t colon_pos = object.location.find(':');
  if (colon_pos != std::string::npos) {
    pick_base_position = object.location.substr(0, colon_pos);
    pick_level = std::stoi(object.location.substr(colon_pos + 1));
  }

  // Validate pick position exists
  if (!isValidGridName(pick_base_position)) {
    RCLCPP_ERROR(this->get_logger(), "Object location '%s' not found in grid", pick_base_position.c_str());
    return false;
  }

  // Check gripper clearance for pick position (position in front must be free)
  if (!isGripperClearanceOk(pick_base_position)) {
    // Auto-move blocking cube out of the way
    std::string blocking_cube = findBlockingCube(pick_base_position);
    if (!blocking_cube.empty()) {
      std::string temp_pos = findFreeRow1Position();
      if (!temp_pos.empty()) {
        // Track this position as claimed during recursive auto-move
        pending_auto_move_destinations_.insert(temp_pos);

        RCLCPP_WARN(this->get_logger(),
          "Pick position %s blocked by %s - moving to %s temporarily",
          pick_base_position.c_str(), blocking_cube.c_str(), temp_pos.c_str());

        // Recursively call PickAndPlace to move blocking cube
        bool success = PickAndPlace(blocking_cube, temp_pos, 0);

        // Release the pending claim (move completed or failed)
        pending_auto_move_destinations_.erase(temp_pos);

        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Failed to move blocking cube %s", blocking_cube.c_str());
          return false;
        }
        // Now clearance should be OK - continue with original operation
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "Cannot pick from %s: no free position to move blocking cube",
          pick_base_position.c_str());
        return false;
      }
    }
  }

  // Calculate pick pose using GRIP position (spawn + offset)
  geometry_msgs::msg::Pose pick_pose;
  pick_pose.position = calculateGripPosition(pick_base_position);
  pick_pose.orientation = config_.standard_orientation;

  // Adjust Z for stacked objects (include gap between cubes)
  if (pick_level > 0) {
    pick_pose.position.z = BASE_GRIP_Z + (pick_level * (CUBE_HEIGHT + CUBE_GAP));
    RCLCPP_INFO(this->get_logger(), "Picking from stack level %d: pick_z = %.3f", pick_level, pick_pose.position.z);
  }

  // Calculate goal pose using GRIP position
  geometry_msgs::msg::Pose goal_pose;
  if (goal_name == "X") {
    // Special eject position - use stored position directly
    auto special_it = grid_positions_.find("X");
    if (special_it != grid_positions_.end()) {
      goal_pose = special_it->second;
    }
  } else {
    goal_pose.position = calculateGripPosition(goal_name);
    goal_pose.orientation = config_.standard_orientation;
  }

  // For normal grid positions, place at height adjusted by stack level
  // Each level adds (cube height + gap) to the Z position
  // Base Z = grip_offset_.z (default 0.1m), then +(0.03 + 0.002) per level
  // Keep eject position X at its original height (Z=0.35)
  double place_z = goal_pose.position.z;  // Original grid Z for approach calculation
  if (goal_name != "X") {
    // Place at grip height + level offset (including gap between cubes)
    place_z = BASE_GRIP_Z + (level * (CUBE_HEIGHT + CUBE_GAP));
    RCLCPP_INFO(this->get_logger(), "Stack level %d: place_z = %.3f (base %.3f + %d * %.3f)",
                level, place_z, BASE_GRIP_Z, level, CUBE_HEIGHT + CUBE_GAP);
  }

  RCLCPP_INFO(this->get_logger(), "Pick from %s grip position (%.3f, %.3f, %.3f)",
              object.location.c_str(), pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Place at %s grip position (%.3f, %.3f, %.3f)",
              goal_name.c_str(), goal_pose.position.x, goal_pose.position.y, place_z);

  // === PICK SEQUENCE ===
  RCLCPP_INFO(this->get_logger(), "--- PICK SEQUENCE ---");
  std::vector<double> trajectory_joints;  // Store joints from planned trajectory

  // Open gripper before picking (real robot mode)
  if (real_robot_mode_) {
    sendRelease();
  }

  // Calculate approach height in mm for MOVEZ commands (use configured value, not simulation coordinates)
  double approach_height_mm = config_.approach_height * 1000.0;  // e.g., 0.05m = 50mm

  // 1. Move to approach position above object (single approach - no intermediates)
  geometry_msgs::msg::Pose pick_approach = calculateApproachPose(pick_pose);

  // Calculate real robot approach based on pick level (for stacking support)
  double real_robot_grip_z = REAL_ROBOT_MIN_Z + (pick_level * (CUBE_HEIGHT + CUBE_GAP));
  double real_robot_approach_z = real_robot_grip_z + config_.approach_height;
  if (real_robot_mode_ && pick_approach.position.z < real_robot_approach_z) {
    RCLCPP_WARN(this->get_logger(), "Pick approach z=%.3f too low for real robot (level %d), clamping to %.3f",
                pick_approach.position.z, pick_level, real_robot_approach_z);
    pick_approach.position.z = real_robot_approach_z;
  }

  if (!moveToPosition(pick_approach, "Move to pick approach", trajectory_joints)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to pick approach position");
    return false;
  }
  // Send to real robot if connected - use joints from trajectory, not simulation feedback
  if (real_robot_mode_ && !trajectory_joints.empty()) {
    sendMoveJ(trajectory_joints);
  }

  // 2. Move down linearly to object
  // For real robot: skip simulation planning if target is below min Z, but still send MOVEZ
  if (real_robot_mode_ && pick_pose.position.z < REAL_ROBOT_MIN_Z) {
    // Use configured approach height for MOVEZ (not simulation coordinates!)
    RCLCPP_INFO(this->get_logger(), "Linear down to pick: MOVEZ -%.1f mm", approach_height_mm);
    sendMoveZ(-approach_height_mm);
    sendGrip();
  } else {
    if (!moveCartesian(pick_approach, pick_pose, "Linear down to object", trajectory_joints)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move down to object");
      return false;
    }
    if (real_robot_mode_) {
      // Linear move straight down using MOVEZ with configured approach height
      RCLCPP_INFO(this->get_logger(), "Linear down to pick: MOVEZ -%.1f mm", approach_height_mm);
      sendMoveZ(-approach_height_mm);
      sendGrip();
    }
  }

  // 3. Attach object to gripper (simulation only - causes planning issues in real robot mode)
  if (!real_robot_mode_) {
    if (!attachObject(object.id)) {
      RCLCPP_WARN(this->get_logger(), "Failed to attach object (continuing anyway)");
    }
    std::this_thread::sleep_for(100ms);
  }

  // 4. Move up linearly to approach position
  // For real robot: skip simulation planning if below min Z, but still send MOVEZ
  if (real_robot_mode_ && pick_pose.position.z < REAL_ROBOT_MIN_Z) {
    // Use configured approach height for MOVEZ (not simulation coordinates!)
    RCLCPP_INFO(this->get_logger(), "Linear up from pick: MOVEZ +%.1f mm", approach_height_mm);
    sendMoveZ(approach_height_mm);
  } else {
    if (!moveCartesian(pick_pose, pick_approach, "Linear up from object", trajectory_joints)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move up from object");
      return false;
    }
    if (real_robot_mode_) {
      // Linear move straight up using MOVEZ with configured approach height
      RCLCPP_INFO(this->get_logger(), "Linear up from pick: MOVEZ +%.1f mm", approach_height_mm);
      sendMoveZ(approach_height_mm);
    }
  }

  // Object is now attached and safely lifted - no need to restore collision
  // since touch_links handle the gripper-object collision while attached

  // === PLACE SEQUENCE ===
  RCLCPP_INFO(this->get_logger(), "--- PLACE SEQUENCE ---");

  // 5. Create final placement pose with correct Z for stacking level
  geometry_msgs::msg::Pose final_place_pose = goal_pose;
  final_place_pose.position.z = place_z;

  // 6. Move to approach position above goal (single approach - no intermediates)
  // Important: Calculate approach from final_place_pose, not goal_pose, to handle stacking
  geometry_msgs::msg::Pose goal_approach = calculateApproachPose(final_place_pose);

  // Calculate real robot approach based on place level (for stacking support)
  double real_robot_place_grip_z = REAL_ROBOT_MIN_Z + (level * (CUBE_HEIGHT + CUBE_GAP));
  double real_robot_place_approach_z = real_robot_place_grip_z + config_.approach_height;
  if (real_robot_mode_ && goal_approach.position.z < real_robot_place_approach_z) {
    RCLCPP_WARN(this->get_logger(), "Place approach z=%.3f too low for real robot (level %d), clamping to %.3f",
                goal_approach.position.z, level, real_robot_place_approach_z);
    goal_approach.position.z = real_robot_place_approach_z;
  }

  if (!moveToPosition(goal_approach, "Move to place approach", trajectory_joints)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to place approach position");
    return false;
  }
  // Send to real robot if connected - use joints from trajectory
  if (real_robot_mode_ && !trajectory_joints.empty()) {
    sendMoveJ(trajectory_joints);
  }

  // 7. Move down linearly to final placement position
  // For real robot: skip simulation planning if target is below min Z, but still send MOVEZ
  if (real_robot_mode_ && final_place_pose.position.z < REAL_ROBOT_MIN_Z) {
    // Use configured approach height for MOVEZ (not simulation coordinates!)
    RCLCPP_INFO(this->get_logger(), "Linear down to place: MOVEZ -%.1f mm", approach_height_mm);
    sendMoveZ(-approach_height_mm);
    sendRelease();
  } else {
    if (!moveCartesian(goal_approach, final_place_pose, "Linear down to goal", trajectory_joints)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move down to goal");
      return false;
    }
    if (real_robot_mode_) {
      // Linear move straight down using MOVEZ with configured approach height
      RCLCPP_INFO(this->get_logger(), "Linear down to place: MOVEZ -%.1f mm", approach_height_mm);
      sendMoveZ(-approach_height_mm);
      sendRelease();
    }
  }

  // 8. Detach object from gripper (simulation only - causes planning issues in real robot mode)
  if (!real_robot_mode_) {
    if (!detachObject(object.id)) {
      RCLCPP_WARN(this->get_logger(), "Failed to detach object (continuing anyway)");
    }
    std::this_thread::sleep_for(100ms);
  }

  // 9. Update object location and stack level in memory
  object.stack_level = level;
  if (level > 0) {
    // Store location with level suffix (e.g., "A1:1")
    object.location = goal_name + ":" + std::to_string(level);
  } else {
    object.location = goal_name;
  }

  // 10. Move up linearly to approach position
  // For real robot: skip simulation planning if below min Z, but still send MOVEZ
  if (real_robot_mode_ && final_place_pose.position.z < REAL_ROBOT_MIN_Z) {
    // Use configured approach height for MOVEZ (not simulation coordinates!)
    RCLCPP_INFO(this->get_logger(), "Linear up from place: MOVEZ +%.1f mm", approach_height_mm);
    sendMoveZ(approach_height_mm);
  } else {
    if (!moveCartesian(final_place_pose, goal_approach, "Linear up from goal", trajectory_joints)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move up from goal");
      return false;
    }
    if (real_robot_mode_) {
      // Linear move straight up using MOVEZ with configured approach height
      RCLCPP_INFO(this->get_logger(), "Linear up from place: MOVEZ +%.1f mm", approach_height_mm);
      sendMoveZ(approach_height_mm);
    }
  }

  // 11. Check if goal is "X" (eject position) - remove object after 3 seconds
  if (goal_name == "X") {
    RCLCPP_INFO(this->get_logger(), "Object placed at eject position. Waiting 3 seconds before removal...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Reduced from 3000ms

    if (!removeCollisionObject(object.id, object_name)) {
      RCLCPP_WARN(this->get_logger(), "Failed to remove object from planning scene");
    }
    RCLCPP_INFO(this->get_logger(), "Object ejected successfully");
  } else {
    // 12. For normal positions, update AML file with location (includes level if > 0)
    if (!updateObjectLocationInAML(object_name, object.location)) {
      RCLCPP_WARN(this->get_logger(), "Failed to update AML file");
    }
  }

  RCLCPP_INFO(this->get_logger(), "=== PickAndPlace COMPLETE ===");
  return true;
}

std::string RobotHLInterfaceSimple::AddCube(const std::string& grid_position,
                                            const std::string& color,
                                            int level)
{
  RCLCPP_INFO(this->get_logger(), "=== AddCube: %s level=%d (%s) ===",
              grid_position.c_str(), level, color.c_str());

  // 1. Validate grid position exists
  auto grid_it = grid_positions_.find(grid_position);
  if (grid_it == grid_positions_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Grid position '%s' not found", grid_position.c_str());
    return "";
  }

  // 2. Build location string with level
  std::string location_with_level = grid_position;
  if (level > 0) {
    location_with_level = grid_position + ":" + std::to_string(level);
  }

  // 3. Check if this specific level is occupied
  for (const auto& [name, obj] : objects_) {
    if (obj.location == location_with_level) {
      RCLCPP_ERROR(this->get_logger(), "Position '%s' is already occupied by %s",
                   location_with_level.c_str(), name.c_str());
      return "";
    }
  }

  // 4. Generate unique cube name
  std::string cube_name = "cube_" + std::to_string(next_cube_id_++);

  // 5. Create ObjectInfo with level
  const double CUBE_HEIGHT = 0.03;  // 3cm cube height
  ObjectInfo info;
  info.name = cube_name;
  info.id = cube_name;
  info.location = location_with_level;
  info.stack_level = level;
  info.width = 0.03;   // 3cm cube
  info.depth = 0.03;
  info.height = 0.03;
  info.color = color.empty() ? "Schwarz" : color;

  // 6. Add to objects map
  objects_[cube_name] = info;

  // 7. Create collision object with correct position
  // Use grid X/Y and calculate Z based on stack level
  const double CUBE_GAP = 0.0;  // No gap - cubes stack directly (30mm per level)
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = grid_it->second.position.x;
  cube_pose.position.y = grid_it->second.position.y;
  cube_pose.position.z = level * (CUBE_HEIGHT + CUBE_GAP);  // 0, 0.030, 0.060, ...
  cube_pose.orientation.x = 0.0;
  cube_pose.orientation.y = 0.0;
  cube_pose.orientation.z = 0.0;
  cube_pose.orientation.w = 1.0;

  // Create collision object with mesh from Wuerfel.stl
  moveit_msgs::msg::CollisionObject obj;
  obj.id = info.id;
  obj.header.frame_id = "world";

  // Load mesh manually (matches Python add_collision_objects.py parsing)
  // This ensures consistent appearance - no vertex deduplication
  std::string stl_path = mesh_directory_ + "/Wuerfel.stl";
  double scale = 0.001;  // STL is in mm, convert to m

  shape_msgs::msg::Mesh mesh_msg = loadSTLMesh(stl_path, scale);
  if (mesh_msg.vertices.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load mesh: %s", stl_path.c_str());
    objects_.erase(cube_name);
    return "";
  }

  obj.meshes.push_back(mesh_msg);
  obj.mesh_poses.push_back(cube_pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // 7. Create ObjectColor for visualization
  moveit_msgs::msg::ObjectColor obj_color;
  obj_color.id = info.id;
  auto color_it = COLOR_MAP.find(info.color);
  if (color_it != COLOR_MAP.end()) {
    obj_color.color.r = color_it->second[0];
    obj_color.color.g = color_it->second[1];
    obj_color.color.b = color_it->second[2];
    obj_color.color.a = color_it->second[3];
  } else {
    obj_color.color.r = 0.5f;
    obj_color.color.g = 0.5f;
    obj_color.color.b = 0.5f;
    obj_color.color.a = 1.0f;
  }

  // 8. Build scene message with collision object and color (NO ACM)
  // Don't set ACM - let MoveIt handle collision checking normally
  // The touch_links in attachObject() handles gripper-object collisions
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.world.collision_objects.push_back(obj);
  scene_msg.object_colors.push_back(obj_color);

  // Use ApplyPlanningScene service (same as add_collision_objects.py) for reliable delivery
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = scene_msg;

  if (!apply_scene_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "ApplyPlanningScene service not available");
    objects_.erase(cube_name);
    return "";
  }

  auto future = apply_scene_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                          std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (!result->success) {
      RCLCPP_WARN(this->get_logger(), "ApplyPlanningScene service returned false");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "ApplyPlanningScene service call timed out");
  }

  RCLCPP_INFO(this->get_logger(), "Added cube '%s' (%s) at position %s (%.3f, %.3f, %.3f)",
              cube_name.c_str(), info.color.c_str(), grid_position.c_str(),
              cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);

  // 9. Persist to AML file (use info.location which includes level suffix)
  if (!addObjectToAML(cube_name, info.location, info.color,
                      info.width, info.depth, info.height)) {
    RCLCPP_WARN(this->get_logger(), "Failed to add object to AML file (object exists in memory only)");
  }

  return cube_name;
}

void RobotHLInterfaceSimple::ClearAllCubes()
{
  RCLCPP_INFO(this->get_logger(), "=== ClearAllCubes: Removing all cubes from scene ===");

  // If no cubes exist, just log and return
  if (objects_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No cubes to remove (scene is already empty)");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Removing %zu cube(s)...", objects_.size());

  // Collect all object IDs and names to remove
  std::vector<std::string> objects_to_remove;
  for (const auto& [name, info] : objects_) {
    objects_to_remove.push_back(name);
  }

  // Remove each cube from planning scene and AML
  for (const auto& object_name : objects_to_remove) {
    auto obj_it = objects_.find(object_name);
    if (obj_it != objects_.end()) {
      const auto& obj_info = obj_it->second;
      RCLCPP_INFO(this->get_logger(), "  Removing '%s' from %s", object_name.c_str(), obj_info.location.c_str());

      // Remove from planning scene
      moveit_msgs::msg::CollisionObject remove_obj;
      remove_obj.id = obj_info.id;
      remove_obj.header.frame_id = planning_frame_;
      remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

      std::vector<moveit_msgs::msg::CollisionObject> remove_objs = {remove_obj};
      planning_scene_->applyCollisionObjects(remove_objs);

      // Remove from AML file
      removeObjectFromAML(object_name);

      // Remove from internal map
      objects_.erase(obj_it);
    }
  }

  // Wait for scene updates to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(150));  // Reduced from 300ms

  // Reset cube ID counter for clean numbering
  next_cube_id_ = 0;

  RCLCPP_INFO(this->get_logger(), "=== ClearAllCubes complete: All cubes removed ===");
}

bool RobotHLInterfaceSimple::moveToHome()
{
  RCLCPP_INFO(this->get_logger(), "Moving to home position...");

  // First move to a safe high position above workspace
  // This ensures we don't collide with floor during transition to home
  geometry_msgs::msg::Pose high_safe;
  high_safe.position.x = 0.4;  // In front of robot
  high_safe.position.y = 0.0;  // Centered
  high_safe.position.z = 0.35; // High above workspace
  high_safe.orientation = config_.standard_orientation;

  std::vector<double> trajectory_joints;
  if (!moveToPosition(high_safe, "Move to safe high position", trajectory_joints)) {
    RCLCPP_WARN(this->get_logger(), "Failed to move to safe high position, trying direct home");
  } else if (real_robot_mode_ && !trajectory_joints.empty()) {
    sendMoveJ(trajectory_joints);
  }

  move_group_->setNamedTarget("home");

  bool success = planAndExecute("Move to home", trajectory_joints);

  // Send to real robot if connected - use joints from trajectory
  if (success && real_robot_mode_ && !trajectory_joints.empty()) {
    sendMoveJ(trajectory_joints);
  }

  return success;
}

// === QUERY FUNCTIONS ===

std::vector<std::string> RobotHLInterfaceSimple::getAvailableObjects() const
{
  std::vector<std::string> names;
  for (const auto& [name, info] : objects_) {
    names.push_back(name);
  }
  return names;
}

std::vector<std::string> RobotHLInterfaceSimple::getAvailableGoals() const
{
  std::vector<std::string> names;
  for (const auto& [name, pose] : grid_positions_) {
    names.push_back(name);
  }
  return names;
}

std::string RobotHLInterfaceSimple::getObjectLocation(const std::string& object_name) const
{
  auto it = objects_.find(object_name);
  if (it != objects_.end()) {
    return it->second.location;
  }
  return "";
}

geometry_msgs::msg::Pose RobotHLInterfaceSimple::getGridPose(const std::string& grid_name) const
{
  auto it = grid_positions_.find(grid_name);
  if (it != grid_positions_.end()) {
    return it->second;
  }
  return geometry_msgs::msg::Pose();
}

bool RobotHLInterfaceSimple::isGridPositionOccupied(const std::string& grid_name) const
{
  for (const auto& [name, info] : objects_) {
    if (info.location == grid_name) {
      return true;
    }
  }
  return false;
}

// === AML PARSING ===

bool RobotHLInterfaceSimple::parseAMLFile(const std::string& aml_file_path)
{
  RCLCPP_INFO(this->get_logger(), "Parsing AML file: %s", aml_file_path.c_str());

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(aml_file_path.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load AML file: %s", doc.ErrorStr());
    return false;
  }

  tinyxml2::XMLElement* root = doc.RootElement();
  if (!root) {
    RCLCPP_ERROR(this->get_logger(), "No root element in AML file");
    return false;
  }

  // Parse each section
  if (!parseRobotParameters(root)) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse robot parameters (using defaults)");
  }

  if (!parseGridConfig(root)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse grid configuration");
    return false;
  }

  if (!parseObjects(root)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse objects");
    return false;
  }

  return true;
}

bool RobotHLInterfaceSimple::parseRobotParameters(tinyxml2::XMLElement* root)
{
  // Look for Robot-Config hierarchy (matches AML structure)
  tinyxml2::XMLElement* hierarchy = findInstanceHierarchy(root, "Robot-Config");
  if (!hierarchy) {
    return false;
  }

  // Find IRB120 element (matches AML structure)
  for (auto* elem = hierarchy->FirstChildElement("InternalElement");
       elem != nullptr;
       elem = elem->NextSiblingElement("InternalElement"))
  {
    const char* name = elem->Attribute("Name");
    if (name && std::string(name) == "IRB120") {
      std::string approach_str = getAttributeValue(elem, "ApproachHeight");
      if (!approach_str.empty()) {
        config_.approach_height = std::stod(approach_str);
      }

      std::string velocity_str = getAttributeValue(elem, "VelocityScaling");
      if (!velocity_str.empty()) {
        config_.velocity_scaling = std::stod(velocity_str);
      }

      std::string accel_str = getAttributeValue(elem, "AccelerationScaling");
      if (!accel_str.empty()) {
        config_.acceleration_scaling = std::stod(accel_str);
      }

      std::string planner_str = getAttributeValue(elem, "PlannerID");
      if (!planner_str.empty()) {
        config_.planner_id = planner_str;
      }

      std::string orient_str = getAttributeValue(elem, "GripperOrientation");
      if (!orient_str.empty()) {
        config_.standard_orientation = parseQuaternion(orient_str);
      }

      RCLCPP_INFO(this->get_logger(), "Loaded config: approach=%.3fm, vel=%.2f, planner=%s, orientation=(%.3f,%.3f,%.3f,%.3f)",
                  config_.approach_height, config_.velocity_scaling, config_.planner_id.c_str(),
                  config_.standard_orientation.x, config_.standard_orientation.y,
                  config_.standard_orientation.z, config_.standard_orientation.w);
      return true;
    }
  }

  return false;
}

bool RobotHLInterfaceSimple::parseGridConfig(tinyxml2::XMLElement* root)
{
  tinyxml2::XMLElement* hierarchy = findInstanceHierarchy(root, "Grid-Config");
  if (!hierarchy) {
    RCLCPP_ERROR(this->get_logger(), "Grid-Config hierarchy not found");
    return false;
  }

  // Parse GridParameters
  for (auto* elem = hierarchy->FirstChildElement("InternalElement");
       elem != nullptr;
       elem = elem->NextSiblingElement("InternalElement"))
  {
    const char* name = elem->Attribute("Name");
    if (!name) continue;

    if (std::string(name) == "GridParameters") {
      // Parse A1 spawn position
      std::string a1_str = getAttributeValue(elem, "A1SpawnPosition");
      if (!a1_str.empty()) {
        a1_spawn_position_ = parsePosition(a1_str);
        RCLCPP_INFO(this->get_logger(), "A1 spawn position: (%.3f, %.3f, %.3f)",
                    a1_spawn_position_.x, a1_spawn_position_.y, a1_spawn_position_.z);
      }

      // Parse grid spacing
      std::string spacing_str = getAttributeValue(elem, "GridSpacing");
      if (!spacing_str.empty()) {
        grid_spacing_ = std::stod(spacing_str);
        RCLCPP_INFO(this->get_logger(), "Grid spacing: %.4f m", grid_spacing_);
      }

      // Parse grip offset
      std::string offset_str = getAttributeValue(elem, "GripOffset");
      if (!offset_str.empty()) {
        grip_offset_ = parsePosition(offset_str);
        RCLCPP_INFO(this->get_logger(), "Grip offset: (%.3f, %.3f, %.3f)",
                    grip_offset_.x, grip_offset_.y, grip_offset_.z);
      }
    }
    else if (std::string(name) == "SpecialPositions") {
      // Parse special positions like "X"
      for (auto* special = elem->FirstChildElement("InternalElement");
           special != nullptr;
           special = special->NextSiblingElement("InternalElement"))
      {
        const char* special_name = special->Attribute("Name");
        if (!special_name) continue;

        std::string pos_str = getAttributeValue(special, "Position");
        if (!pos_str.empty()) {
          geometry_msgs::msg::Pose pose;
          pose.position = parsePosition(pos_str);
          pose.orientation = config_.standard_orientation;
          grid_positions_[special_name] = pose;
          RCLCPP_INFO(this->get_logger(), "Special position %s: (%.3f, %.3f, %.3f)",
                      special_name, pose.position.x, pose.position.y, pose.position.z);
        }
      }
    }
  }

  // Generate all grid positions A1-E5 from parameters
  for (char col = 'A'; col <= 'E'; ++col) {
    for (int row = 1; row <= 5; ++row) {
      std::string grid_name = std::string(1, col) + std::to_string(row);
      geometry_msgs::msg::Pose pose;
      pose.position = calculateSpawnPosition(grid_name);
      pose.orientation = config_.standard_orientation;
      grid_positions_[grid_name] = pose;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Generated %zu grid positions from config", grid_positions_.size());
  return true;
}

geometry_msgs::msg::Point RobotHLInterfaceSimple::calculateSpawnPosition(const std::string& grid_name) const
{
  geometry_msgs::msg::Point pos;

  // Handle special positions (like "X")
  auto it = grid_positions_.find(grid_name);
  if (it != grid_positions_.end() && grid_name.length() == 1) {
    return it->second.position;
  }

  // Parse grid name (e.g., "A1" -> col='A', row=1)
  if (grid_name.length() >= 2 && std::isalpha(grid_name[0]) && std::isdigit(grid_name[1])) {
    int col_index = std::toupper(grid_name[0]) - 'A';  // A=0, B=1, C=2, D=3, E=4
    int row_index = grid_name[1] - '1';                 // 1=0, 2=1, 3=2, 4=3, 5=4

    pos.x = a1_spawn_position_.x - (row_index * grid_spacing_);
    pos.y = a1_spawn_position_.y + (col_index * grid_spacing_);
    pos.z = a1_spawn_position_.z;
  } else {
    // Default to A1 position if invalid
    pos = a1_spawn_position_;
  }

  return pos;
}

geometry_msgs::msg::Point RobotHLInterfaceSimple::calculateGripPosition(const std::string& grid_name) const
{
  geometry_msgs::msg::Point spawn = calculateSpawnPosition(grid_name);
  geometry_msgs::msg::Point grip;

  grip.x = spawn.x + grip_offset_.x;
  grip.y = spawn.y + grip_offset_.y;
  grip.z = spawn.z + grip_offset_.z;

  return grip;
}

bool RobotHLInterfaceSimple::isValidGridName(const std::string& grid_name) const
{
  // Check special positions
  if (grid_positions_.find(grid_name) != grid_positions_.end()) {
    return true;
  }

  // Check standard grid (A1-E5)
  if (grid_name.length() == 2 &&
      grid_name[0] >= 'A' && grid_name[0] <= 'E' &&
      grid_name[1] >= '1' && grid_name[1] <= '5') {
    return true;
  }

  return false;
}

bool RobotHLInterfaceSimple::isGripperClearanceOk(const std::string& grid_position) const
{
  // X1 row positions (A1, B1, C1, D1, E1) are always accessible - nothing in front
  if (grid_position.length() == 2 && grid_position[1] == '1') {
    return true;
  }

  // Special position "X" is always accessible
  if (grid_position == "X") {
    return true;
  }

  // For XN where N > 1, check if X(N-1) is free
  // Due to large gripper, the position in front must be clear for access
  if (grid_position.length() == 2) {
    char col = grid_position[0];
    int row = grid_position[1] - '0';

    if (row > 1 && row <= 5) {
      std::string front_position = std::string(1, col) + std::to_string(row - 1);

      // Check if any object is at front_position (any level)
      for (const auto& [name, info] : objects_) {
        std::string obj_base = info.location;
        size_t colon = obj_base.find(':');
        if (colon != std::string::npos) {
          obj_base = obj_base.substr(0, colon);
        }
        if (obj_base == front_position) {
          return false;  // Position in front is occupied
        }
      }
    }
  }
  return true;
}

std::string RobotHLInterfaceSimple::findBlockingCube(const std::string& grid_position) const
{
  // Row 1 positions have no blocking cube (nothing in front)
  if (grid_position.length() != 2 || grid_position[1] == '1') {
    return "";
  }

  // Special position "X" has no blocking
  if (grid_position == "X") {
    return "";
  }

  // Find which cube is at position X(N-1)
  char col = grid_position[0];
  int row = grid_position[1] - '0';
  std::string front_position = std::string(1, col) + std::to_string(row - 1);

  for (const auto& [name, info] : objects_) {
    std::string obj_base = info.location;
    size_t colon = obj_base.find(':');
    if (colon != std::string::npos) {
      obj_base = obj_base.substr(0, colon);
    }
    if (obj_base == front_position) {
      return name;  // Return blocking cube's name
    }
  }
  return "";
}

std::string RobotHLInterfaceSimple::findFreeRow1Position() const
{
  // Row 1 positions are always accessible (nothing in front)
  const std::vector<std::string> row1_positions = {"A1", "B1", "C1", "D1", "E1"};

  for (const auto& pos : row1_positions) {
    // Check if position is pending (claimed by ongoing recursive auto-move)
    if (pending_auto_move_destinations_.count(pos) > 0) {
      continue;  // Skip positions already claimed in current auto-move chain
    }

    bool occupied = false;
    for (const auto& [name, info] : objects_) {
      std::string obj_base = info.location;
      size_t colon = obj_base.find(':');
      if (colon != std::string::npos) {
        obj_base = obj_base.substr(0, colon);
      }
      if (obj_base == pos) {
        occupied = true;
        break;
      }
    }
    if (!occupied) {
      return pos;
    }
  }
  return "";  // No free row 1 position
}

bool RobotHLInterfaceSimple::parseObjects(tinyxml2::XMLElement* root)
{
  tinyxml2::XMLElement* hierarchy = findInstanceHierarchy(root, "Objects");
  if (!hierarchy) {
    RCLCPP_ERROR(this->get_logger(), "Objects hierarchy not found");
    return false;
  }

  for (auto* elem = hierarchy->FirstChildElement("InternalElement");
       elem != nullptr;
       elem = elem->NextSiblingElement("InternalElement"))
  {
    const char* name = elem->Attribute("Name");
    const char* id = elem->Attribute("ID");
    if (!name) continue;

    ObjectInfo info;
    info.name = name;
    info.id = id ? id : name;

    info.location = getAttributeValue(elem, "Location");
    if (info.location.empty()) {
      RCLCPP_WARN(this->get_logger(), "Object '%s' has no Location", name);
      continue;
    }

    // Parse location to get base position (handle stacked positions like "A1:1")
    std::string base_location = info.location;
    size_t colon_pos = info.location.find(':');
    if (colon_pos != std::string::npos) {
      base_location = info.location.substr(0, colon_pos);
      info.stack_level = std::stoi(info.location.substr(colon_pos + 1));
    }

    // Verify base location is valid
    if (!isValidGridName(base_location)) {
      RCLCPP_WARN(this->get_logger(), "Object '%s' location '%s' not in grid",
                  name, info.location.c_str());
      continue;
    }

    std::string dim_str = getAttributeValue(elem, "Dimensions");
    if (!dim_str.empty()) {
      parseDimensions(dim_str, info.width, info.depth, info.height);
    } else {
      info.width = 0.03;
      info.depth = 0.03;
      info.height = 0.03;
    }

    info.color = getAttributeValue(elem, "Color");

    objects_[name] = info;
    RCLCPP_INFO(this->get_logger(), "Object '%s' at %s (%.3fx%.3fx%.3f)",
                name, info.location.c_str(), info.width, info.depth, info.height);
  }

  // Return true even if no objects - it's valid to have an empty Objects section
  // and add cubes dynamically via AddCube()
  RCLCPP_INFO(this->get_logger(), "Loaded %zu objects from AML", objects_.size());
  return true;
}

bool RobotHLInterfaceSimple::updateObjectLocationInAML(const std::string& object_name,
                                                        const std::string& new_location)
{
  RCLCPP_INFO(this->get_logger(), "Updating AML: %s -> %s", object_name.c_str(), new_location.c_str());

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(aml_file_path_.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load AML for update");
    return false;
  }

  tinyxml2::XMLElement* root = doc.RootElement();
  tinyxml2::XMLElement* hierarchy = findInstanceHierarchy(root, "Objects");
  if (!hierarchy) {
    return false;
  }

  // Find the object
  for (auto* elem = hierarchy->FirstChildElement("InternalElement");
       elem != nullptr;
       elem = elem->NextSiblingElement("InternalElement"))
  {
    const char* name = elem->Attribute("Name");
    if (name && std::string(name) == object_name) {
      // Find Location attribute
      for (auto* attr = elem->FirstChildElement("Attribute");
           attr != nullptr;
           attr = attr->NextSiblingElement("Attribute"))
      {
        const char* attr_name = attr->Attribute("Name");
        if (attr_name && std::string(attr_name) == "Location") {
          auto* value = attr->FirstChildElement("Value");
          if (value) {
            value->SetText(new_location.c_str());
            if (doc.SaveFile(aml_file_path_.c_str()) == tinyxml2::XML_SUCCESS) {
              RCLCPP_INFO(this->get_logger(), "AML updated successfully");
              return true;
            }
          }
        }
      }
    }
  }

  RCLCPP_ERROR(this->get_logger(), "Failed to find object in AML");
  return false;
}

bool RobotHLInterfaceSimple::addObjectToAML(const std::string& object_name,
                                             const std::string& location,
                                             const std::string& color,
                                             double width, double depth, double height)
{
  RCLCPP_INFO(this->get_logger(), "Adding to AML: %s at %s (%s)",
              object_name.c_str(), location.c_str(), color.c_str());

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(aml_file_path_.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load AML for adding object");
    return false;
  }

  tinyxml2::XMLElement* root = doc.RootElement();
  tinyxml2::XMLElement* hierarchy = findInstanceHierarchy(root, "Objects");
  if (!hierarchy) {
    RCLCPP_ERROR(this->get_logger(), "Objects hierarchy not found in AML");
    return false;
  }

  // Create new InternalElement for the object
  tinyxml2::XMLElement* new_elem = doc.NewElement("InternalElement");
  new_elem->SetAttribute("Name", object_name.c_str());
  new_elem->SetAttribute("ID", object_name.c_str());

  // Add Location attribute
  tinyxml2::XMLElement* loc_attr = doc.NewElement("Attribute");
  loc_attr->SetAttribute("Name", "Location");
  loc_attr->SetAttribute("AttributeDataType", "xs:string");
  tinyxml2::XMLElement* loc_value = doc.NewElement("Value");
  loc_value->SetText(location.c_str());
  loc_attr->InsertEndChild(loc_value);
  new_elem->InsertEndChild(loc_attr);

  // Add Dimensions attribute
  tinyxml2::XMLElement* dim_attr = doc.NewElement("Attribute");
  dim_attr->SetAttribute("Name", "Dimensions");
  dim_attr->SetAttribute("AttributeDataType", "xs:string");
  dim_attr->SetAttribute("Unit", "m");
  tinyxml2::XMLElement* dim_value = doc.NewElement("Value");
  std::stringstream dim_ss;
  dim_ss << std::fixed << std::setprecision(3) << width << "," << depth << "," << height;
  dim_value->SetText(dim_ss.str().c_str());
  dim_attr->InsertEndChild(dim_value);
  new_elem->InsertEndChild(dim_attr);

  // Add Color attribute
  tinyxml2::XMLElement* color_attr = doc.NewElement("Attribute");
  color_attr->SetAttribute("Name", "Color");
  color_attr->SetAttribute("AttributeDataType", "xs:string");
  tinyxml2::XMLElement* color_value = doc.NewElement("Value");
  color_value->SetText(color.c_str());
  color_attr->InsertEndChild(color_value);
  new_elem->InsertEndChild(color_attr);

  // Add to hierarchy
  hierarchy->InsertEndChild(new_elem);

  // Save file
  if (doc.SaveFile(aml_file_path_.c_str()) == tinyxml2::XML_SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Object added to AML successfully");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "Failed to save AML file");
  return false;
}

bool RobotHLInterfaceSimple::removeObjectFromAML(const std::string& object_name)
{
  RCLCPP_INFO(this->get_logger(), "Removing from AML: %s", object_name.c_str());

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(aml_file_path_.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load AML for removing object");
    return false;
  }

  tinyxml2::XMLElement* root = doc.RootElement();
  tinyxml2::XMLElement* hierarchy = findInstanceHierarchy(root, "Objects");
  if (!hierarchy) {
    RCLCPP_ERROR(this->get_logger(), "Objects hierarchy not found in AML");
    return false;
  }

  // Find and remove the object element
  for (auto* elem = hierarchy->FirstChildElement("InternalElement");
       elem != nullptr;
       elem = elem->NextSiblingElement("InternalElement"))
  {
    const char* name = elem->Attribute("Name");
    if (name && std::string(name) == object_name) {
      hierarchy->DeleteChild(elem);

      if (doc.SaveFile(aml_file_path_.c_str()) == tinyxml2::XML_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Object removed from AML successfully");
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save AML file after removal");
        return false;
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "Object %s not found in AML (may already be removed)", object_name.c_str());
  return true;  // Return true since object is not there anyway
}

// === AML HELPERS ===

geometry_msgs::msg::Point RobotHLInterfaceSimple::parsePosition(const std::string& str)
{
  geometry_msgs::msg::Point point;
  std::stringstream ss(str);
  char comma;
  ss >> point.x >> comma >> point.y >> comma >> point.z;
  return point;
}

geometry_msgs::msg::Quaternion RobotHLInterfaceSimple::parseQuaternion(const std::string& str)
{
  geometry_msgs::msg::Quaternion quat;
  std::stringstream ss(str);
  char comma;
  // Format: x,y,z,w (standard AML format)
  ss >> quat.x >> comma >> quat.y >> comma >> quat.z >> comma >> quat.w;
  return quat;
}

void RobotHLInterfaceSimple::parseDimensions(const std::string& str,
                                              double& width, double& depth, double& height)
{
  std::stringstream ss(str);
  char comma;
  ss >> width >> comma >> depth >> comma >> height;
}

std::string RobotHLInterfaceSimple::getAttributeValue(tinyxml2::XMLElement* element,
                                                       const std::string& attr_name)
{
  for (auto* attr = element->FirstChildElement("Attribute");
       attr != nullptr;
       attr = attr->NextSiblingElement("Attribute"))
  {
    const char* name = attr->Attribute("Name");
    if (name && std::string(name) == attr_name) {
      auto* value = attr->FirstChildElement("Value");
      if (value && value->GetText()) {
        return value->GetText();
      }
    }
  }
  return "";
}

tinyxml2::XMLElement* RobotHLInterfaceSimple::findInstanceHierarchy(tinyxml2::XMLElement* root,
                                                                      const std::string& name)
{
  for (auto* elem = root->FirstChildElement("InstanceHierarchy");
       elem != nullptr;
       elem = elem->NextSiblingElement("InstanceHierarchy"))
  {
    const char* elem_name = elem->Attribute("Name");
    if (elem_name && std::string(elem_name) == name) {
      return elem;
    }
  }
  return nullptr;
}

// === MOVEMENT PRIMITIVES ===

bool RobotHLInterfaceSimple::moveToPosition(const geometry_msgs::msg::Pose& target,
                                             const std::string& description)
{
  std::vector<double> dummy;
  return moveToPosition(target, description, dummy);
}

bool RobotHLInterfaceSimple::moveToPosition(const geometry_msgs::msg::Pose& target,
                                             const std::string& description,
                                             std::vector<double>& final_joints)
{
  RCLCPP_INFO(this->get_logger(), "%s: (%.3f, %.3f, %.3f)",
              description.c_str(), target.position.x, target.position.y, target.position.z);

  move_group_->setPoseTarget(target);
  return planAndExecute(description, final_joints);
}

bool RobotHLInterfaceSimple::moveCartesian(const geometry_msgs::msg::Pose& start,
                                            const geometry_msgs::msg::Pose& end,
                                            const std::string& description)
{
  std::vector<double> dummy;
  return moveCartesian(start, end, description, dummy);
}

bool RobotHLInterfaceSimple::moveCartesian(const geometry_msgs::msg::Pose& start,
                                            const geometry_msgs::msg::Pose& end,
                                            const std::string& description,
                                            std::vector<double>& final_joints)
{
  RCLCPP_INFO(this->get_logger(), "%s: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
              description.c_str(),
              start.position.x, start.position.y, start.position.z,
              end.position.x, end.position.y, end.position.z);

  final_joints.clear();

  // Get current EEF pose and use its orientation for the target
  // This ensures we use an achievable orientation
  geometry_msgs::msg::Pose target_pose;
  target_pose.position = end.position;

  // Get current state and extract EEF orientation
  auto current_state = move_group_->getCurrentState(5.0);
  if (current_state) {
    const Eigen::Isometry3d& eef_transform = current_state->getGlobalLinkTransform(eef_link_);
    Eigen::Quaterniond q(eef_transform.rotation());
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    RCLCPP_INFO(this->get_logger(), "Using current EEF orientation: (%.3f, %.3f, %.3f, %.3f)",
                q.x(), q.y(), q.z(), q.w());
  } else {
    // Fallback to the end pose orientation if we can't get current state
    target_pose.orientation = end.orientation;
    RCLCPP_WARN(this->get_logger(), "Could not get current state, using target orientation");
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005;  // 5mm resolution
  const double jump_threshold = 5.0;

  double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(this->get_logger(), "Cartesian path: %.1f%% achieved", fraction * 100.0);

  if (fraction < 0.95) {
    RCLCPP_WARN(this->get_logger(), "Cartesian path incomplete (%.1f%%), trying PTP fallback",
                fraction * 100.0);
    move_group_->setPoseTarget(target_pose);
    return planAndExecute(description + " (PTP fallback)", final_joints);
  }

  // Extract final joint positions from the trajectory BEFORE execution
  auto& points = trajectory.joint_trajectory.points;
  if (!points.empty()) {
    final_joints = points.back().positions;
    RCLCPP_INFO(this->get_logger(), "Extracted %zu joint values from Cartesian trajectory", final_joints.size());
  }

  // Execute the cartesian path
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  auto result = move_group_->execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Cartesian execution failed");
    final_joints.clear();
    return false;
  }

  return true;
}

bool RobotHLInterfaceSimple::planAndExecute(const std::string& description)
{
  std::vector<double> dummy;
  return planAndExecute(description, dummy);
}

bool RobotHLInterfaceSimple::planAndExecute(const std::string& description,
                                             std::vector<double>& final_joints)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  final_joints.clear();

  // Try planning up to 3 times
  for (int attempt = 1; attempt <= 3; ++attempt) {
    auto result = move_group_->plan(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Plan succeeded (attempt %d)", attempt);

      // Extract final joint positions from the planned trajectory BEFORE execution
      auto& points = plan.trajectory_.joint_trajectory.points;
      if (!points.empty()) {
        final_joints = points.back().positions;
        RCLCPP_INFO(this->get_logger(), "Extracted %zu joint values from trajectory", final_joints.size());
      }

      result = move_group_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Execution failed on attempt %d", attempt);
        final_joints.clear();
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed on attempt %d", attempt);
    }

    std::this_thread::sleep_for(200ms);
  }

  RCLCPP_ERROR(this->get_logger(), "All planning attempts failed for: %s", description.c_str());
  return false;
}

geometry_msgs::msg::Pose RobotHLInterfaceSimple::calculateApproachPose(
    const geometry_msgs::msg::Pose& target)
{
  geometry_msgs::msg::Pose approach = target;
  approach.position.z += config_.approach_height;
  return approach;
}

// === ATTACH/DETACH ===

bool RobotHLInterfaceSimple::attachObject(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Attaching object: %s to %s", object_id.c_str(), eef_link_.c_str());

  // Use MoveGroupInterface's simple attach method
  // This handles removing from world and adding as attached correctly
  move_group_->attachObject(object_id, eef_link_, gripper_touch_links_);

  // Wait for attachment to propagate (increased for planning scene sync)
  std::this_thread::sleep_for(500ms);

  // Force planning scene state refresh to ensure consistency
  move_group_->getCurrentState(5.0);

  RCLCPP_INFO(this->get_logger(), "Successfully attached %s", object_id.c_str());
  return true;
}

bool RobotHLInterfaceSimple::detachObject(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Detaching object: %s", object_id.c_str());

  // Use MoveGroupInterface's simple detach method
  // This returns the object to the world at its current position
  move_group_->detachObject(object_id);

  // Wait for detachment to propagate (increased for planning scene sync)
  std::this_thread::sleep_for(500ms);

  // Force planning scene state refresh to ensure consistency
  move_group_->getCurrentState(5.0);

  RCLCPP_INFO(this->get_logger(), "Successfully detached %s", object_id.c_str());
  return true;
}

// === COLLISION OBJECTS ===

bool RobotHLInterfaceSimple::addCollisionObjects()
{
  RCLCPP_INFO(this->get_logger(), "Adding collision objects to planning scene...");

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  for (const auto& [name, info] : objects_) {
    // Get pose from grid position
    auto grid_it = grid_positions_.find(info.location);
    if (grid_it == grid_positions_.end()) {
      RCLCPP_WARN(this->get_logger(), "Cannot add object '%s': location '%s' not found",
                  name.c_str(), info.location.c_str());
      continue;
    }

    moveit_msgs::msg::CollisionObject obj;
    obj.id = info.id;
    obj.header.frame_id = planning_frame_;

    // Create box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = info.width;
    primitive.dimensions[1] = info.depth;
    primitive.dimensions[2] = info.height;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(grid_it->second);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    collision_objects.push_back(obj);
    RCLCPP_INFO(this->get_logger(), "Added collision object: %s at %s",
                info.id.c_str(), info.location.c_str());
  }

  if (!collision_objects.empty()) {
    planning_scene_->addCollisionObjects(collision_objects);
    std::this_thread::sleep_for(500ms);  // Wait for planning scene to update
  }

  return true;
}

bool RobotHLInterfaceSimple::updateCollisionObjectPose(const std::string& object_name,
                                                        const geometry_msgs::msg::Pose& new_pose)
{
  auto obj_it = objects_.find(object_name);
  if (obj_it == objects_.end()) {
    return false;
  }

  const auto& obj_info = obj_it->second;

  // First, remove the old collision object
  moveit_msgs::msg::CollisionObject remove_obj;
  remove_obj.id = obj_info.id;
  remove_obj.header.frame_id = planning_frame_;
  remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  std::vector<moveit_msgs::msg::CollisionObject> remove_objs = {remove_obj};
  planning_scene_->applyCollisionObjects(remove_objs);

  // Wait for removal to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Reduced from 100ms

  // Then, add the collision object at the new position
  moveit_msgs::msg::CollisionObject add_obj;
  add_obj.id = obj_info.id;
  add_obj.header.frame_id = planning_frame_;

  // Create a box primitive with the object's dimensions
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = obj_info.width;   // X
  primitive.dimensions[1] = obj_info.depth;   // Y
  primitive.dimensions[2] = obj_info.height;  // Z

  add_obj.primitives.push_back(primitive);
  add_obj.primitive_poses.push_back(new_pose);
  add_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  std::vector<moveit_msgs::msg::CollisionObject> add_objs = {add_obj};
  planning_scene_->applyCollisionObjects(add_objs);

  RCLCPP_INFO(this->get_logger(), "Updated collision object %s to position (%.3f, %.3f, %.3f)",
              object_name.c_str(), new_pose.position.x, new_pose.position.y, new_pose.position.z);

  return true;
}

bool RobotHLInterfaceSimple::removeCollisionObject(const std::string& object_id,
                                                   const std::string& object_name)
{
  RCLCPP_INFO(this->get_logger(), "Removing collision object: %s", object_id.c_str());

  moveit_msgs::msg::CollisionObject remove_obj;
  remove_obj.id = object_id;
  remove_obj.header.frame_id = planning_frame_;
  remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  std::vector<moveit_msgs::msg::CollisionObject> remove_objs = {remove_obj};
  planning_scene_->applyCollisionObjects(remove_objs);

  std::this_thread::sleep_for(std::chrono::milliseconds(150));  // Reduced from 300ms

  // Remove from AML file
  if (!removeObjectFromAML(object_name)) {
    RCLCPP_WARN(this->get_logger(), "Failed to remove object from AML file");
  }

  objects_.erase(object_name);

  RCLCPP_INFO(this->get_logger(), "Successfully removed collision object: %s", object_id.c_str());
  return true;
}

void RobotHLInterfaceSimple::allowCubeCollisions()
{
  // Wait for service to be available
  if (!apply_scene_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Service /apply_planning_scene not available");
    return;
  }

  // Collect all object names (cubes + mesh_object)
  std::vector<std::string> object_names;
  object_names.push_back("mesh_object");
  for (const auto& [name, info] : objects_) {
    if (name.find("cube_") == 0 || name.find("wuerfel_") == 0) {
      object_names.push_back(info.id);
    }
  }

  // Robot links that need collision allowance with objects
  std::vector<std::string> robot_links = {
    "irb120_base_link", "irb120_link_1", "irb120_link_2", "irb120_link_3",
    "irb120_link_4", "irb120_link_5", "irb120_link_6", "irb120_flange", "irb120_tool0",
    "gripper_base", "left_finger", "right_finger", "mounting_block"
  };

  // Build service request
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;

  // Build ACM using name-based lookup to handle MoveIt's alphabetical reordering
  auto& acm = request->scene.allowed_collision_matrix;

  // Combine all names
  std::vector<std::string> all_names;
  all_names.insert(all_names.end(), object_names.begin(), object_names.end());
  all_names.insert(all_names.end(), robot_links.begin(), robot_links.end());

  // Sort names to match MoveIt's alphabetical ordering
  std::sort(all_names.begin(), all_names.end());

  // Create sets for quick lookup
  std::set<std::string> object_set(object_names.begin(), object_names.end());
  std::set<std::string> robot_set(robot_links.begin(), robot_links.end());

  acm.entry_names = all_names;
  size_t n = all_names.size();

  // Build the collision matrix - for each entry, check by NAME not index
  for (size_t i = 0; i < n; ++i) {
    const std::string& name_i = all_names[i];
    bool is_object_i = (object_set.count(name_i) > 0);

    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled.resize(n, false);

    for (size_t j = 0; j < n; ++j) {
      if (i == j) continue;  // No self-collision allowance

      const std::string& name_j = all_names[j];
      bool is_object_j = (object_set.count(name_j) > 0);
      bool is_robot_j = (robot_set.count(name_j) > 0);

      // Allow collision if:
      // 1. Both are objects (for stacking)
      // 2. One is object, one is robot link
      if ((is_object_i && is_object_j) ||
          (is_object_i && is_robot_j) ||
          (!is_object_i && is_object_j)) {
        entry.enabled[j] = true;
      }
    }

    acm.entry_values.push_back(entry);
  }

  RCLCPP_INFO(this->get_logger(), "Applying ACM: %zu objects + %zu robot links = %zu entries (sorted)",
              object_names.size(), robot_links.size(), n);

  // Call service synchronously
  auto future = apply_scene_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                          std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "ACM applied successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to apply ACM (service returned false)");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "ACM service call timed out");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Reduced from 100ms
}

void RobotHLInterfaceSimple::allowGripperCubeCollision(const std::string& cube_id, bool allow)
{
  // Instead of modifying ACM (which can corrupt the planning scene),
  // we temporarily remove the cube from collision checking by
  // removing it from the planning scene, then add it back after the pick.
  //
  // allow=true  -> Remove cube from planning scene (disable collision)
  // allow=false -> Cube will be re-added during attach/detach operations

  if (allow) {
    RCLCPP_INFO(this->get_logger(), "Temporarily removing %s from collision checking for approach",
                cube_id.c_str());

    // Remove the cube from planning scene temporarily
    std::vector<std::string> object_ids = {cube_id};
    planning_scene_->removeCollisionObjects(object_ids);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Reduced from 200ms
    RCLCPP_INFO(this->get_logger(), "Cube %s removed from planning scene for approach", cube_id.c_str());
  } else {
    // When allow=false, the cube should be re-added.
    // However, in our PickAndPlace flow, after the pick:
    // - If successful: cube is attached to gripper (handled by attachObject)
    // - If failed: we need to re-add the cube
    //
    // For now, we'll re-add the cube at its original position
    auto obj_it = objects_.find(cube_id);
    if (obj_it == objects_.end()) {
      RCLCPP_WARN(this->get_logger(), "Cannot restore cube %s - not found in objects map", cube_id.c_str());
      return;
    }

    const auto& info = obj_it->second;

    // Parse the base grid position (without stack level)
    std::string base_position = info.location;
    size_t colon_pos = base_position.find(':');
    if (colon_pos != std::string::npos) {
      base_position = base_position.substr(0, colon_pos);
    }

    auto grid_it = grid_positions_.find(base_position);
    if (grid_it == grid_positions_.end()) {
      RCLCPP_WARN(this->get_logger(), "Cannot restore cube %s - grid position %s not found",
                  cube_id.c_str(), base_position.c_str());
      return;
    }

    // Create collision object at original position
    moveit_msgs::msg::CollisionObject obj;
    obj.id = cube_id;
    obj.header.frame_id = "world";

    // Load mesh
    std::string mesh_path = "file://" + mesh_directory_ + "/Wuerfel.stl";
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
    if (!mesh) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh for restoring cube");
      return;
    }

    // Scale and convert to message
    mesh->scale(0.001);
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
    delete mesh;

    obj.meshes.push_back(mesh_msg);

    // Set pose (including 2mm gap between stacked cubes)
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.position.x = grid_it->second.position.x;
    mesh_pose.position.y = grid_it->second.position.y;
    mesh_pose.position.z = info.stack_level * 0.032;  // 3cm cube + 2mm gap per level
    mesh_pose.orientation.w = 1.0;
    obj.mesh_poses.push_back(mesh_pose);

    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {obj};
    planning_scene_->applyCollisionObjects(collision_objects);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Reduced from 200ms
    RCLCPP_INFO(this->get_logger(), "Cube %s restored to planning scene at %s level %d",
                cube_id.c_str(), base_position.c_str(), info.stack_level);
  }
}

void RobotHLInterfaceSimple::buildACM(moveit_msgs::msg::AllowedCollisionMatrix& acm)
{
  // Collect object names (mesh_object + all cubes)
  std::vector<std::string> object_names;
  object_names.push_back("mesh_object");  // Hutschienen-Halter
  for (const auto& [name, info] : objects_) {
    object_names.push_back(info.id);
  }

  // Robot links that need collision allowance with objects
  std::vector<std::string> robot_links = {
    "gripper_base", "left_finger", "right_finger", "mounting_block",
    "irb120_base_link", "irb120_link_1", "irb120_link_2", "irb120_link_3",
    "irb120_link_4", "irb120_link_5", "irb120_link_6", "irb120_flange", "irb120_tool0"
  };

  // Combine all names and sort (MoveIt sorts ACM entries alphabetically)
  std::vector<std::string> all_names;
  all_names.insert(all_names.end(), object_names.begin(), object_names.end());
  all_names.insert(all_names.end(), robot_links.begin(), robot_links.end());
  std::sort(all_names.begin(), all_names.end());

  // Create set for quick lookup
  std::set<std::string> object_set(object_names.begin(), object_names.end());

  acm.entry_names = all_names;

  // Build the collision matrix
  for (size_t i = 0; i < all_names.size(); ++i) {
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled.resize(all_names.size(), false);

    bool is_object_i = (object_set.count(all_names[i]) > 0);

    for (size_t j = 0; j < all_names.size(); ++j) {
      if (i == j) continue;  // No self-collision allowance

      bool is_object_j = (object_set.count(all_names[j]) > 0);

      // Allow collision if either is an object (handles cube-cube, cube-robot, cube-mesh)
      if (is_object_i || is_object_j) {
        entry.enabled[j] = true;
      }
    }

    acm.entry_values.push_back(entry);
  }

  RCLCPP_INFO(this->get_logger(), "Built ACM with %zu entries (%zu objects, %zu robot links)",
              all_names.size(), object_names.size(), robot_links.size());
}

// === REAL ROBOT COMMUNICATION ===

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>

bool RobotHLInterfaceSimple::connectToRealRobot()
{
  RCLCPP_INFO(this->get_logger(), "Connecting to real robot at %s:%d...",
              robot_ip_.c_str(), robot_port_);

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
    return false;
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(robot_port_);

  if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address: %s", robot_ip_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot - is SocketMain running?");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Test connection with PING
  std::string response;
  if (sendSocketCommand("PING", response) && response.find("PONG") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "Connected to real robot! (PONG received)");

    // Query current robot position to initialize J6 tracking
    if (sendSocketCommand("GETPOS", response)) {
      // Parse response: "POS:j1 j2 j3 j4 j5 j6"
      if (response.find("POS:") == 0) {
        std::istringstream iss(response.substr(4));
        double j1, j2, j3, j4, j5, j6;
        if (iss >> j1 >> j2 >> j3 >> j4 >> j5 >> j6) {
          last_j6_deg_ = j6;
          RCLCPP_INFO(this->get_logger(), "Robot current J6: %.2f deg (will use for normalization)", j6);
        }
      }
    }

    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "Robot connection test failed: %s", response.c_str());
  close(socket_fd_);
  socket_fd_ = -1;
  return false;
}

void RobotHLInterfaceSimple::disconnectFromRealRobot()
{
  if (socket_fd_ >= 0) {
    std::string response;
    sendSocketCommand("QUIT", response, 5.0);
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_INFO(this->get_logger(), "Disconnected from real robot");
  }
}

bool RobotHLInterfaceSimple::sendSocketCommand(const std::string& cmd, std::string& response, double timeout_sec)
{
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Not connected to real robot");
    return false;
  }

  // Send command
  std::string cmd_with_newline = cmd + "\n";
  ssize_t sent = send(socket_fd_, cmd_with_newline.c_str(), cmd_with_newline.size(), 0);
  if (sent < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", cmd.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Sent to robot: %s", cmd.c_str());

  // Wait for response with timeout
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  int timeout_ms = static_cast<int>(timeout_sec * 1000);
  int ret = poll(&pfd, 1, timeout_ms);

  if (ret < 0) {
    RCLCPP_ERROR(this->get_logger(), "Poll error");
    return false;
  } else if (ret == 0) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for robot response");
    return false;
  }

  // Receive response
  char buffer[1024];
  ssize_t received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  if (received <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive response");
    return false;
  }

  buffer[received] = '\0';
  response = std::string(buffer);

  // Trim trailing newlines
  while (!response.empty() && (response.back() == '\n' || response.back() == '\r')) {
    response.pop_back();
  }

  RCLCPP_INFO(this->get_logger(), "Robot response: %s", response.c_str());
  return true;
}

bool RobotHLInterfaceSimple::sendMoveJ(const std::vector<double>& joints_rad)
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  if (joints_rad.size() != 6) {
    RCLCPP_ERROR(this->get_logger(), "sendMoveJ requires 6 joint values, got %zu", joints_rad.size());
    return false;
  }

  // Convert radians to degrees
  std::vector<double> joints_deg(6);
  for (int i = 0; i < 6; i++) {
    joints_deg[i] = joints_rad[i] * 180.0 / M_PI;
  }

  // J6 offset for real robot gripper orientation
  // Pass through J6 directly from simulation (no 180° offset)
  // Just normalize to [-180, 180] range for safety
  double j6 = joints_deg[5];
  double j6_original = j6;

  // Normalize to [-180, 180] range
  while (j6 > 180.0) j6 -= 360.0;
  while (j6 < -180.0) j6 += 360.0;

  RCLCPP_INFO(this->get_logger(), "J6: %.2f -> %.2f (normalized)",
              j6_original, j6);
  joints_deg[5] = j6;

  // Build command
  std::ostringstream cmd;
  cmd << "MOVEJ";
  for (double j : joints_deg) {
    cmd << " " << std::fixed << std::setprecision(2) << j;
  }

  std::string response;
  bool success = sendSocketCommand(cmd.str(), response, 60.0);  // 60 second timeout for moves

  if (success && response.find("OK") != std::string::npos) {
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "MOVEJ failed: %s", response.c_str());
  return false;
}

bool RobotHLInterfaceSimple::sendMoveL(const geometry_msgs::msg::Pose& pose)
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  // Convert position from meters to mm (ABB uses mm)
  double x_mm = pose.position.x * 1000.0;
  double y_mm = pose.position.y * 1000.0;
  double z_mm = pose.position.z * 1000.0;

  // Quaternion: ROS uses (w, x, y, z), ABB MOVEL expects q0=w, q1=x, q2=y, q3=z
  double q0 = pose.orientation.w;
  double q1 = pose.orientation.x;
  double q2 = pose.orientation.y;
  double q3 = pose.orientation.z;

  RCLCPP_INFO(this->get_logger(), "MoveL to: (%.1f, %.1f, %.1f) mm, quat (%.3f, %.3f, %.3f, %.3f)",
              x_mm, y_mm, z_mm, q0, q1, q2, q3);

  // Build command: MOVEL x y z q0 q1 q2 q3
  std::ostringstream cmd;
  cmd << "MOVEL"
      << " " << std::fixed << std::setprecision(2) << x_mm
      << " " << std::fixed << std::setprecision(2) << y_mm
      << " " << std::fixed << std::setprecision(2) << z_mm
      << " " << std::fixed << std::setprecision(4) << q0
      << " " << std::fixed << std::setprecision(4) << q1
      << " " << std::fixed << std::setprecision(4) << q2
      << " " << std::fixed << std::setprecision(4) << q3;

  std::string response;
  bool success = sendSocketCommand(cmd.str(), response, 60.0);  // 60 second timeout for moves

  if (success && response.find("OK") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "MoveL complete");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "MOVEL failed: %s", response.c_str());
  return false;
}

bool RobotHLInterfaceSimple::getCartesianPosition(double& x_mm, double& y_mm, double& z_mm,
                                                   double& q1, double& q2, double& q3, double& q4)
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return false;
  }

  std::string response;
  bool success = sendSocketCommand("GETCART", response, 5.0);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "GETCART command failed");
    return false;
  }

  // Parse response: "CART: x y z q1 q2 q3 q4"
  if (response.find("CART:") != 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid GETCART response: %s", response.c_str());
    return false;
  }

  // Extract values after "CART:"
  std::string values = response.substr(5);  // Skip "CART:"
  std::istringstream iss(values);

  if (!(iss >> x_mm >> y_mm >> z_mm >> q1 >> q2 >> q3 >> q4)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse GETCART response: %s", response.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Current position: (%.1f, %.1f, %.1f) mm, quat (%.4f, %.4f, %.4f, %.4f)",
              x_mm, y_mm, z_mm, q1, q2, q3, q4);
  return true;
}

bool RobotHLInterfaceSimple::sendMoveLZ(double target_z_meters)
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  // Get current position from robot (to get X, Y, and orientation)
  double x_mm, y_mm, z_mm, q1, q2, q3, q4;
  if (!getCartesianPosition(x_mm, y_mm, z_mm, q1, q2, q3, q4)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current position for MoveLZ");
    return false;
  }

  // Use saved X, Y, orientation with new Z
  double target_z_mm = target_z_meters * 1000.0;

  RCLCPP_INFO(this->get_logger(), "MoveLZ: Z %.1f -> %.1f mm (keeping X=%.1f, Y=%.1f, orientation unchanged)",
              z_mm, target_z_mm, x_mm, y_mm);

  // Build MOVEL command
  std::ostringstream cmd;
  cmd << "MOVEL"
      << " " << std::fixed << std::setprecision(2) << x_mm
      << " " << std::fixed << std::setprecision(2) << y_mm
      << " " << std::fixed << std::setprecision(2) << target_z_mm
      << " " << std::fixed << std::setprecision(4) << q1
      << " " << std::fixed << std::setprecision(4) << q2
      << " " << std::fixed << std::setprecision(4) << q3
      << " " << std::fixed << std::setprecision(4) << q4;

  std::string response;
  bool success = sendSocketCommand(cmd.str(), response, 60.0);

  if (success && response.find("OK") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "MoveLZ complete");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "MoveLZ failed: %s", response.c_str());
  return false;
}

bool RobotHLInterfaceSimple::sendMoveLWithOrientation(double x_mm, double y_mm, double z_mm,
                                                       double q1, double q2, double q3, double q4)
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  RCLCPP_INFO(this->get_logger(), "MoveL to: (%.1f, %.1f, %.1f) mm with saved orientation",
              x_mm, y_mm, z_mm);

  // Build MOVEL command with provided position and orientation
  std::ostringstream cmd;
  cmd << "MOVEL"
      << " " << std::fixed << std::setprecision(2) << x_mm
      << " " << std::fixed << std::setprecision(2) << y_mm
      << " " << std::fixed << std::setprecision(2) << z_mm
      << " " << std::fixed << std::setprecision(4) << q1
      << " " << std::fixed << std::setprecision(4) << q2
      << " " << std::fixed << std::setprecision(4) << q3
      << " " << std::fixed << std::setprecision(4) << q4;

  std::string response;
  bool success = sendSocketCommand(cmd.str(), response, 60.0);

  if (success && response.find("OK") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "MoveL complete");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "MoveL failed: %s", response.c_str());
  return false;
}

bool RobotHLInterfaceSimple::sendMoveZ(double delta_z_mm)
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  RCLCPP_INFO(this->get_logger(), "MoveZ: delta=%.1f mm", delta_z_mm);

  // Build MOVEZ command - robot will get its current position and add delta to Z
  std::ostringstream cmd;
  cmd << "MOVEZ " << std::fixed << std::setprecision(2) << delta_z_mm;

  std::string response;
  bool success = sendSocketCommand(cmd.str(), response, 60.0);

  if (success && response.find("OK") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "MoveZ complete (delta=%.1f mm)", delta_z_mm);
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "MOVEZ failed: %s", response.c_str());
  return false;
}

bool RobotHLInterfaceSimple::sendGrip()
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  std::string response;
  bool success = sendSocketCommand("GRIP", response, 5.0);

  if (success && response.find("OK") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "Real robot gripper closed");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "GRIP failed: %s", response.c_str());
  return false;
}

bool RobotHLInterfaceSimple::sendRelease()
{
  if (!real_robot_mode_ || socket_fd_ < 0) {
    return true;  // Not in real robot mode, success by default
  }

  std::string response;
  bool success = sendSocketCommand("RELEASE", response, 5.0);

  if (success && response.find("OK") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "Real robot gripper opened");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "RELEASE failed: %s", response.c_str());
  return false;
}
