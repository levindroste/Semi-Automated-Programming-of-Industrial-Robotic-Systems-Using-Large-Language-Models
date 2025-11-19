#include "ur10e_hl_interface/robot_hl_interface.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <sstream>
#include <algorithm>
#include <cmath>

RobotHLInterface::RobotHLInterface() : Node("robot_hl_interface")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Robot High-Level Interface");

  // Declare AML file parameter with default path
  std::string default_aml_path = std::string(std::getenv("HOME")) +
      "/ur10_ws/src/ur10e_hl_interface/config/AML-Datei-V04.aml";
  this->declare_parameter("aml_file", default_aml_path);
  aml_file_path_ = this->get_parameter("aml_file").as_string();

  // State file path
  state_file_path_ = std::string(std::getenv("HOME")) +
      "/ur10_ws/src/ur10e_hl_interface/config/SchaltschrankZustand.aml";

  RCLCPP_INFO(this->get_logger(), "Using AML file: %s", aml_file_path_.c_str());
}

void RobotHLInterface::initialize()
{
  // Initialize MoveIt
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur10e_arm");

  total_placed_count_ = 0;

  // Store default parameters
  default_params_.velocity_scaling = 1.0;
  default_params_.acceleration_scaling = 1.0;
  default_params_.goal_position_tolerance = 0.001;
  default_params_.goal_orientation_tolerance = 0.001;
  default_params_.goal_joint_tolerance = 0.001;
  default_params_.planning_time = 10.0;
  default_params_.planner_id = "PRM";

  // Initialize current parameters
  current_params_ = default_params_;

  // Initialisiere adaptive Level
  initializeAdaptiveLevels();

  // Apply default parameters
  applyPlanningParams(default_params_);
  move_group_->startStateMonitor();

  planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  eef_link_ = move_group_->getEndEffectorLink();

  // Wait for initialization
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Parse AML file
  if (!parseAMLFile(aml_file_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse AML file");
    throw std::runtime_error("Failed to parse AML configuration");
  }

  if (!validateAMLConfig()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid AML configuration");
    throw std::runtime_error("Invalid AML configuration");
  }

  // State file path
  state_file_path_ = std::string(std::getenv("HOME")) +
      "/ur10_ws/src/ur10e_hl_interface/config/SchaltschrankZustand.aml";

  // Initialize component states
  for (const auto& [type, info] : objects_) {
    for (int i = 0; i < info.count; i++) {
      ComponentState state;
      state.type = type;
      state.object_id = info.base_id + std::to_string(i + 1);
      state.instance_number = i;
      state.location = StorageLocation::COMPONENT_STORAGE;
      state.rail_index = -1;
      state.position_on_rail = 0.0;
      state.current_pose = calculateComponentStoragePose(type, i);

      component_states_[{type, i}] = state;
    }
  }

  // Initialize rail states
  for (int i = 0; i < max_rails_; i++) {
    RailState state;
    state.rail_id = i + 1;
    state.location = StorageLocation::RAIL_STORAGE;
    state.cabinet_position = -1;
    state.fill_level = 0.0;
    state.current_pose = calculateRailStoragePose(i);

    rail_states_[i + 1] = state;
  }

  // First rail is at workspace initially
  rail_states_[1].location = StorageLocation::WORKSPACE;
  workspace_rail_id_ = 1;

  // Initialize cabinet positions based on max rails
  for (int i = 1; i <= max_rails_; i++) {
    CabinetPosition pos;
    pos.name = "Schaltschrank-Position" + std::to_string(i);
    pos.pose = calculateCabinetPose(i);
    pos.occupied = false;
    pos.rail_id = -1;

    cabinet_positions_[i] = pos;
  }

  RCLCPP_INFO(this->get_logger(), "Initialization complete with %zu object types and %d rails",
              objects_.size(), max_rails_);
}

bool RobotHLInterface::move_to_home()
{
  RCLCPP_INFO(this->get_logger(), "Moving to home position");
  move_group_->setNamedTarget("up");
  return planAndExecute("Move to home position");
}

bool RobotHLInterface::pick(const std::string& component_name)
{
  RCLCPP_INFO(this->get_logger(), "=== Picking component: %s ===", component_name.c_str());

  // Find object type by name
  ObjectType obj_type;
  bool found = false;
  for (const auto& [type, info] : objects_) {
    if (info.name == component_name) {
      obj_type = type;
      found = true;
      break;
    }
  }

  if (!found) {
    RCLCPP_ERROR(this->get_logger(), "Unknown component: %s", component_name.c_str());
    return false;
  }

  // Get next available instance
  int instance = getNextAvailableComponentInstance(obj_type);
  if (instance < 0) {
    RCLCPP_ERROR(this->get_logger(), "No more %s components available", component_name.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Selected instance %d of %s", instance, component_name.c_str());

  // Check if we have a rail on workspace
  if (workspace_rail_id_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "No rail on workspace");
    return false;
  }

  // Get component and rail states
  auto comp_state = getComponentState(obj_type, instance);
  auto rail_state = getRailState(workspace_rail_id_);

  if (!comp_state || !rail_state) {
    RCLCPP_ERROR(this->get_logger(), "Invalid state");
    return false;
  }

  const ObjectInfo& obj_info = objects_[obj_type];

  // Check if component fits on current rail - NO AUTOMATIC SWITCHING
  if (isRailFull(workspace_rail_id_, obj_info.width)) {
    RCLCPP_ERROR(this->get_logger(), "Current rail is full - cannot place %s (width: %.3fm)",
                 component_name.c_str(), obj_info.width);
    RCLCPP_ERROR(this->get_logger(), "Remaining space: %.3fm", getRemainingSpaceOnCurrentRail());
    return false;
  }

  // Calculate positions
  geometry_msgs::msg::Pose pick_pose = comp_state->current_pose;
  double position_on_rail = calculateNextPositionOnRail(workspace_rail_id_, obj_info.width);
  geometry_msgs::msg::Pose place_pose = calculatePoseOnRail(workspace_rail_id_, position_on_rail);

  // Log calculated positions
  RCLCPP_INFO(this->get_logger(), "Pick position: (%.3f, %.3f, %.3f)",
              pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Place position on rail: (%.3f, %.3f, %.3f) at rail position %.3f",
              place_pose.position.x, place_pose.position.y, place_pose.position.z, position_on_rail);

  // Approach pick position
  geometry_msgs::msg::Pose pick_approach = pick_pose;
  pick_approach.position.z += approach_height_;

  if (!moveToPosition(pick_approach, "Move to pick approach")) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Move down to pick - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(pick_approach, pick_pose, "Move down to pick", 1.0)) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Attach object (simulation only)
  if (!attachObject(comp_state->object_id)) {
    return false;
  }

  // Lift up - MIT PTP-FALLBACK
  geometry_msgs::msg::Pose lift_pose = pick_pose;
  lift_pose.position.z += approach_height_;
  if (!moveCartesianWithPTPFallback(pick_pose, lift_pose, "Lift object", 1.0)) {
    return false;
  }

  // Update component state to gripper
  updateComponentState(obj_type, instance, StorageLocation::GRIPPER);

  // Move to place position
  geometry_msgs::msg::Pose place_approach = place_pose;
  place_approach.position.z += approach_height_;

  if (!moveToPosition(place_approach, "Move to place approach")) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Place on rail - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(place_approach, place_pose, "Place on rail", 1.0)) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Detach object (simulation only)
  if (!detachObject(comp_state->object_id)) {
    return false;
  }

  // Retreat - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(place_pose, place_approach, "Retreat from place", 1.0)) {
    return false;
  }

  // Update states
  updateComponentState(obj_type, instance, StorageLocation::WORKSPACE,
                      workspace_rail_id_, position_on_rail);
  comp_state->current_pose = place_pose;

  // Add to rail state
  rail_state->components.push_back(*comp_state);
  rail_state->fill_level = position_on_rail + obj_info.width / 2.0;

  total_placed_count_++;

  // Check if we need to go home
  if (!checkAndMoveToHomeIfNeeded()) {
    RCLCPP_WARN(this->get_logger(), "Could not move to home position");
  }

  RCLCPP_INFO(this->get_logger(), "Successfully placed %s (instance %d) at position %.3fm",
              component_name.c_str(), instance, position_on_rail);

  return true;
}

bool RobotHLInterface::place_rail(int position)
{
  RCLCPP_INFO(this->get_logger(), "=== Placing rail in cabinet (position: %d) ===", position);

  // Check if we have a rail on workspace
  if (workspace_rail_id_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "No rail on workspace");
    return false;
  }

  auto rail_state = getRailState(workspace_rail_id_);
  if (!rail_state) {
    RCLCPP_ERROR(this->get_logger(), "Invalid rail state");
    return false;
  }

  // Determine cabinet position
  int target_position = position;
  if (target_position < 0) {
    target_position = getNextAvailableCabinetPosition();
    if (target_position < 0) {
      RCLCPP_ERROR(this->get_logger(), "No available cabinet positions");
      return false;
    }
  }

  // Validate position
  if (cabinet_positions_.find(target_position) == cabinet_positions_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid cabinet position: %d", target_position);
    return false;
  }

  if (cabinet_positions_[target_position].occupied) {
    RCLCPP_ERROR(this->get_logger(), "Cabinet position %d already occupied", target_position);
    return false;
  }

  // Calculate rail center position
  geometry_msgs::msg::Pose rail_pose;
  rail_pose.position.x = rail_center_.x();
  rail_pose.position.y = rail_center_.y();
  rail_pose.position.z = rail_center_.z();
  rail_pose.orientation = rail_orientation_;

  geometry_msgs::msg::Pose rail_approach = rail_pose;
  rail_approach.position.z += approach_height_;

  // Move to rail
  if (!moveToPosition(rail_approach, "Move to rail for pickup")) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Move down to rail - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(rail_approach, rail_pose, "Move down to rail", 1.0)) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Attach rail with all components (simulation only)
  if (!attachRailWithComponents(workspace_rail_id_)) {
    return false;
  }

  // Calculate cabinet position and approach
  geometry_msgs::msg::Pose cabinet_pose = calculateCabinetPose(target_position);
  geometry_msgs::msg::Pose cabinet_approach = cabinet_pose;
  cabinet_approach.position.z += 0.6;

  // Use cabinet approach height as transport height
  double transport_height = cabinet_approach.position.z;

  RCLCPP_INFO(this->get_logger(), "Using transport height: %.3fm (cabinet approach height)",
              transport_height);

  // 1. Lift rail to transport height - MIT PTP-FALLBACK
  geometry_msgs::msg::Pose lift_pose = rail_pose;
  lift_pose.position.z = transport_height;
  if (!moveCartesianWithPTPFallback(rail_pose, lift_pose, "Lift rail to transport height", 0.3)) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 2. Move to cabinet approach position (PTP)
  RCLCPP_INFO(this->get_logger(), "Moving directly to cabinet approach position");

  bool approach_success = moveToPosition(cabinet_approach, "Move to cabinet approach");

  if (!approach_success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reach cabinet approach position");
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // 3. Place in cabinet (vertical movement down) - MIT PTP-FALLBACK + CLEANUP
  if (!moveCartesianWithPTPFallback(cabinet_approach, cabinet_pose, "Place rail in cabinet", 1.0)) {
    // CLEANUP bei Fehler
    RCLCPP_ERROR(this->get_logger(), "Failed to place rail - performing cleanup");
    detachRailWithComponents(workspace_rail_id_);
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Detach rail and components (simulation only)
  if (!detachRailWithComponents(workspace_rail_id_)) {
    return false;
  }

  // 4. Retreat (vertical movement up) - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(cabinet_pose, cabinet_approach, "Retreat from cabinet", 1.0)) {
    return false;
  }

  // Update states
  updateRailState(workspace_rail_id_, StorageLocation::CABINET, target_position);
  cabinet_positions_[target_position].occupied = true;
  cabinet_positions_[target_position].rail_id = workspace_rail_id_;

  // Update rail state with new pose
  rail_state->current_pose = cabinet_pose;
  rail_state->cabinet_position = target_position;
  rail_state->location = StorageLocation::CABINET;

  // Update component locations
  for (auto& comp : rail_state->components) {
    updateComponentState(comp.type, comp.instance_number, StorageLocation::CABINET,
                        workspace_rail_id_, comp.position_on_rail);
  }

  // Clear workspace
  workspace_rail_id_ = -1;

  RCLCPP_INFO(this->get_logger(), "Successfully placed rail %d at cabinet position %d",
              rail_state->rail_id, target_position);

  return true;
}

bool RobotHLInterface::moveCartesianWithPTPFallback(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& end_pose,
    const std::string& description,
    double velocity_factor)
{
  // Phase 1: Versuche kartesische Bewegung (10 Versuche)
  if (moveCartesian(start_pose, end_pose, description, velocity_factor, 10)) {
    return true;
  }

  // Phase 2: Fallback auf PTP-Bewegung (nutzt bereits planAndExecute mit 10 Versuchen)
  RCLCPP_WARN(this->get_logger(),
              "Cartesian '%s' failed after 10 attempts, switching to PTP fallback",
              description.c_str());

  // moveToPosition nutzt bereits planAndExecute mit 10 adaptiven Versuchen
  if (moveToPosition(end_pose, description + " (PTP fallback)")) {
    RCLCPP_INFO(this->get_logger(), "PTP fallback succeeded for '%s'", description.c_str());
    return true;
  }

  RCLCPP_ERROR(this->get_logger(),
               "Both cartesian and PTP failed for '%s'", description.c_str());
  return false;
}

bool RobotHLInterface::pick_rail(int position)
{
  RCLCPP_INFO(this->get_logger(), "=== Picking rail (position: %d) ===", position);

  // Check if workspace is clear
  if (workspace_rail_id_ > 0) {
    RCLCPP_ERROR(this->get_logger(), "Workspace already occupied by rail %d", workspace_rail_id_);
    return false;
  }

  int rail_id = -1;
  geometry_msgs::msg::Pose pick_pose;
  StorageLocation from_location;

  // Determine rail source (storage or cabinet)
  if (position < 0) {
    // Pick from storage
    rail_id = getNextAvailableRailFromStorage();
    if (rail_id < 0) {
      RCLCPP_ERROR(this->get_logger(), "No rails available in storage");
      return false;
    }

    auto rail_state = getRailState(rail_id);
    if (!rail_state) {
      RCLCPP_ERROR(this->get_logger(), "Invalid rail state");
      return false;
    }

    pick_pose = calculateRailGripPose(rail_state->current_pose, StorageLocation::RAIL_STORAGE);
    from_location = StorageLocation::RAIL_STORAGE;
    RCLCPP_INFO(this->get_logger(), "Picking rail %d from storage", rail_id);
  } else {
    // Pick from cabinet
    if (cabinet_positions_.find(position) == cabinet_positions_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid cabinet position: %d", position);
      return false;
    }

    if (!cabinet_positions_[position].occupied) {
      RCLCPP_ERROR(this->get_logger(), "Cabinet position %d is empty", position);
      return false;
    }

    rail_id = cabinet_positions_[position].rail_id;
    pick_pose = calculateCabinetPose(position);
    from_location = StorageLocation::CABINET;
    RCLCPP_INFO(this->get_logger(), "Picking rail %d from cabinet position %d", rail_id, position);
  }

  auto rail_state = getRailState(rail_id);
  if (!rail_state) {
    RCLCPP_ERROR(this->get_logger(), "Rail %d not found", rail_id);
    return false;
  }

  // Calculate workspace center position
  geometry_msgs::msg::Pose workspace_pose;
  workspace_pose.position.x = rail_center_.x();
  workspace_pose.position.y = rail_center_.y();
  workspace_pose.position.z = rail_center_.z();
  workspace_pose.orientation = rail_orientation_;

  // Define transport height
  double transport_offset = (from_location == StorageLocation::CABINET) ? 0.6 : 0.15;
  double transport_height = std::max(pick_pose.position.z, workspace_pose.position.z) + transport_offset;

  RCLCPP_INFO(this->get_logger(), "Using transport offset: %.2fm for location %d",
              transport_offset, static_cast<int>(from_location));

  // Approach position above pick location
  geometry_msgs::msg::Pose pick_approach = pick_pose;
  pick_approach.position.z = transport_height;

  // 1. Move to approach position (PTP only)
  RCLCPP_INFO(this->get_logger(), "Moving to %s approach position",
              (from_location == StorageLocation::RAIL_STORAGE) ? "storage" : "cabinet");

  if (!moveToPosition(pick_approach, "Move to rail approach")) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 2. Move down to rail - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(pick_approach, pick_pose, "Move down to rail", 1.0)) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // 3. Attach rail with components
  if (!attachRailWithComponents(rail_id)) {
    return false;
  }

  // 4. Lift rail to transport height - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(pick_pose, pick_approach, "Lift rail to transport height", 1.0)) {
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Rail lifted to transport height: %.3fm", transport_height);

  // 5. Move to workspace
  geometry_msgs::msg::Pose workspace_approach = workspace_pose;
  workspace_approach.position.z = transport_height;

  if (from_location == StorageLocation::CABINET) {
    // For cabinet: Use motion planning (PTP)
    RCLCPP_INFO(this->get_logger(), "Moving rail from cabinet to workspace (motion planning)");
    if (!moveToPosition(workspace_approach, "Move rail to workspace")) {
      return false;
    }
  } else {
    // For storage: Cartesian with PTP fallback
    RCLCPP_INFO(this->get_logger(), "Moving rail back to workspace (cartesian Y-movement)");
    if (!moveCartesianWithPTPFallback(pick_approach, workspace_approach, "Move rail to workspace Y-axis", 1.0)) {
      return false;
    }
  }

  // 6. Place on workspace - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(workspace_approach, workspace_pose, "Place rail on workspace", 1.0)) {
    return false;
  }

  // 7. Detach rail and components
  if (!detachRailWithComponents(rail_id)) {
    RCLCPP_WARN(this->get_logger(), "Could not detach rail with components");
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // 8. Retreat - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(workspace_pose, workspace_approach, "Retreat from workspace", 1.0)) {
    return false;
  }

  // Update states
  updateRailState(rail_id, StorageLocation::WORKSPACE);
  workspace_rail_id_ = rail_id;

  rail_state->current_pose.position.x = rail_start_.x();
  rail_state->current_pose.position.y = rail_start_.y();
  rail_state->current_pose.position.z = rail_start_.z();
  rail_state->current_pose.orientation = rail_orientation_;

  // Update cabinet position if from cabinet
  if (from_location == StorageLocation::CABINET && position > 0) {
    cabinet_positions_[position].occupied = false;
    cabinet_positions_[position].rail_id = -1;
    rail_state->cabinet_position = -1;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully picked rail %d to workspace", rail_id);

  return true;
}

// Helper function implementations

double RobotHLInterface::getRemainingSpaceOnCurrentRail() const
{
  if (workspace_rail_id_ < 0) {
    return 0.0;
  }

  auto it = rail_states_.find(workspace_rail_id_);
  if (it == rail_states_.end()) {
    return 0.0;
  }

  const RailState& rail_state = it->second;
  return rail_length_ - rail_state.fill_level;
}

RobotHLInterface::ComponentState* RobotHLInterface::getFarthestComponentOnRail(int rail_id,
                                                            const std::string& component_name) {
  auto rail_state = getRailState(rail_id);
  if (!rail_state) return nullptr;

  ComponentState* farthest = nullptr;
  double max_position = -1.0;

  for (auto& comp : rail_state->components) {
    if (objects_[comp.type].name == component_name &&
        comp.position_on_rail > max_position) {
      max_position = comp.position_on_rail;
      farthest = &comp;
    }
  }

  return farthest;
}

RobotHLInterface::ObjectType RobotHLInterface::getObjectTypeFromName(const std::string& name) {
  if (name == "CLIPFIX_35") return ObjectType::CLIPFIX_35;
  else if (name == "PT_6_TWIN_BU") return ObjectType::PT_6_TWIN_BU;
  else if (name == "UT_16_PE") return ObjectType::UT_16_PE;
  else if (name == "VAL_MS_230") return ObjectType::VAL_MS_230;
  else if (name == "VAL_MS_T1_T2") return ObjectType::VAL_MS_T1_T2;
  else if (name == "PTU_2_5_TWIN_BU") return ObjectType::PTU_2_5_TWIN_BU;
  else if (name == "QTCU_2_5") return ObjectType::QTCU_2_5;
  else if (name == "STU_35_4X10_YE") return ObjectType::STU_35_4X10_YE;
  else if (name == "UT_1_5_VT") return ObjectType::UT_1_5_VT;
  else if (name == "UT_1_5_OG") return ObjectType::UT_1_5_OG;
  throw std::runtime_error("Unknown object type: " + name);
}

geometry_msgs::msg::Pose RobotHLInterface::calculateRailGripPose(const geometry_msgs::msg::Pose& rail_start_pose,
                                                                 StorageLocation location) {
  geometry_msgs::msg::Pose grip_pose = rail_start_pose;

  if (location == StorageLocation::CABINET) {
    // For Cabinet: Use directly the passed position (already center)
    return rail_start_pose;

  } else if (location == StorageLocation::RAIL_STORAGE) {
    // For Storage: Rails have the same X position as workspace rail center
    grip_pose.position.x = rail_center_.x();

    // Y: Base Y (from rail_center_) plus index offset
    int rail_index = static_cast<int>((rail_start_pose.position.y - rail_storage_start_.y()) / rail_storage_spacing_ + 0.5);
    grip_pose.position.y = rail_center_.y() + (rail_index * rail_storage_spacing_);

    // Z: Use work height (same as workspace rail)
    grip_pose.position.z = rail_center_.z();

    RCLCPP_INFO(this->get_logger(), "Rail grip pose for storage rail %d: (%.3f, %.3f, %.3f)",
                rail_index + 1,
                grip_pose.position.x, grip_pose.position.y, grip_pose.position.z);

  } else if (location == StorageLocation::WORKSPACE) {
    // At workspace: Use rail_center_
    grip_pose.position.x = rail_center_.x();
    grip_pose.position.y = rail_center_.y();
    grip_pose.position.z = rail_center_.z();
  }

  grip_pose.orientation = rail_orientation_;
  return grip_pose;
}

geometry_msgs::msg::Pose RobotHLInterface::calculateCabinetPose(int position) {
  geometry_msgs::msg::Pose pose;

  // Calculate position
  pose.position.x = cabinet_base_.x();
  pose.position.y = cabinet_base_.y() + ((position - 1) * cabinet_rail_spacing_);
  pose.position.z = cabinet_base_.z();

  // Original orientation
  Eigen::Quaterniond original_quat(rail_orientation_.w, rail_orientation_.x,
                                    rail_orientation_.y, rail_orientation_.z);

  // 180° rotation around Z-axis
  Eigen::AngleAxisd rotation(M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond z_rotation(rotation);

  // Apply rotation
  Eigen::Quaterniond result_quat = original_quat * z_rotation;
  result_quat.normalize();

  // Set the rotated orientation
  pose.orientation.x = result_quat.x();
  pose.orientation.y = result_quat.y();
  pose.orientation.z = result_quat.z();
  pose.orientation.w = result_quat.w();

  RCLCPP_INFO(this->get_logger(), "Cabinet pose for position %d: (%.3f, %.3f, %.3f) orient(%.3f, %.3f, %.3f, %.3f)",
              position, pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  return pose;
}

bool RobotHLInterface::detach_from_rail(const std::string& component_name)
{
  RCLCPP_INFO(this->get_logger(), "=== Detaching component %s from rail ===", component_name.c_str());

  // Check if we have a rail on workspace
  if (workspace_rail_id_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "No rail on workspace");
    return false;
  }

  auto rail_state = getRailState(workspace_rail_id_);
  if (!rail_state) {
    RCLCPP_ERROR(this->get_logger(), "Invalid rail state");
    return false;
  }

  // Find the farthest component with the given name on the rail
  auto comp_state = getFarthestComponentOnRail(workspace_rail_id_, component_name);
  if (!comp_state) {
    RCLCPP_ERROR(this->get_logger(), "No %s component found on rail", component_name.c_str());
    return false;
  }

  // Calculate pick position on rail
  geometry_msgs::msg::Pose pick_pose = calculatePoseOnRail(workspace_rail_id_,
                                                          comp_state->position_on_rail);

  // Approach
  geometry_msgs::msg::Pose approach_pose = pick_pose;
  approach_pose.position.z += approach_height_;

  if (!moveToPosition(approach_pose, "Move to component for removal")) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Move down - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(approach_pose, pick_pose, "Move down to pick component", 0.5)) {
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // Attach component
  if (!attachObject(comp_state->object_id)) {
    RCLCPP_WARN(this->get_logger(), "Could not attach component");
  }

  // Lift - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(pick_pose, approach_pose, "Lift component", 0.5)) {
    return false;
  }

  // Update component state to gripper
  updateComponentState(comp_state->type, comp_state->instance_number, StorageLocation::GRIPPER);

  // Calculate storage position
  geometry_msgs::msg::Pose storage_pose = calculateComponentStoragePose(comp_state->type,
                                                                       comp_state->instance_number);
  geometry_msgs::msg::Pose storage_approach = storage_pose;
  storage_approach.position.z += approach_height_;

  if (!moveToPosition(storage_approach, "Move to storage location")) {
    return false;
  }

  // Place down - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(storage_approach, storage_pose, "Place component in storage", 0.5)) {
    return false;
  }

  // Detach
  if (!detachObject(comp_state->object_id)) {
    RCLCPP_WARN(this->get_logger(), "Could not detach component");
  }

  // Retreat - MIT PTP-FALLBACK
  if (!moveCartesianWithPTPFallback(storage_pose, storage_approach, "Retreat from storage", 0.5)) {
    return false;
  }

  // Update final state
  updateComponentState(comp_state->type, comp_state->instance_number,
                      StorageLocation::COMPONENT_STORAGE);
  comp_state->current_pose = storage_pose;

  // Update the component state in the map as well
  component_states_[{comp_state->type, comp_state->instance_number}].current_pose = storage_pose;

  RCLCPP_INFO(this->get_logger(), "Component %s instance %d returned to storage at (%.3f, %.3f, %.3f)",
              component_name.c_str(), comp_state->instance_number,
              storage_pose.position.x, storage_pose.position.y, storage_pose.position.z);

  // Remove from rail state
  auto it = std::remove_if(rail_state->components.begin(), rail_state->components.end(),
                          [&](const ComponentState& comp) {
                            return comp.type == comp_state->type &&
                                   comp.instance_number == comp_state->instance_number;
                          });
  rail_state->components.erase(it, rail_state->components.end());

  // Recalculate fill level
  recalculateRailFillLevel(workspace_rail_id_);

  RCLCPP_INFO(this->get_logger(), "Successfully removed %s (instance %d) from rail",
              component_name.c_str(), comp_state->instance_number);

  return true;
}

geometry_msgs::msg::Quaternion RobotHLInterface::parseQuaternion(const std::string& str) {
  geometry_msgs::msg::Quaternion q;
  std::vector<double> values;
  std::stringstream ss(str);
  std::string item;

  while (std::getline(ss, item, ',')) {
    values.push_back(std::stod(item));
  }

  if (values.size() >= 4) {
    q.x = values[0];
    q.y = values[1];
    q.z = values[2];
    q.w = values[3];
  } else {
    q.w = 1.0;
  }

  return q;
}

geometry_msgs::msg::Point RobotHLInterface::parsePosition(const std::string& str) {
  geometry_msgs::msg::Point p;
  std::vector<double> values;
  std::stringstream ss(str);
  std::string item;

  while (std::getline(ss, item, ',')) {
    values.push_back(std::stod(item));
  }

  if (values.size() >= 3) {
    p.x = values[0];
    p.y = values[1];
    p.z = values[2];
  }

  return p;
}

std::string RobotHLInterface::getAttributeValue(tinyxml2::XMLElement* element,
                                               const std::string& attrPath) {
  std::vector<std::string> path;
  std::stringstream ss(attrPath);
  std::string item;

  while (std::getline(ss, item, '/')) {
    path.push_back(item);
  }

  tinyxml2::XMLElement* current = element;

  for (size_t i = 0; i < path.size(); ++i) {
    if (i == path.size() - 1) {
      for (tinyxml2::XMLElement* attr = current->FirstChildElement("Attribute");
           attr != nullptr;
           attr = attr->NextSiblingElement("Attribute")) {
        const char* name = attr->Attribute("Name");
        if (name && std::string(name) == path[i]) {
          tinyxml2::XMLElement* value = attr->FirstChildElement("Value");
          if (value && value->GetText()) {
            return std::string(value->GetText());
          }
        }
      }
    } else {
      bool found = false;
      for (tinyxml2::XMLElement* attr = current->FirstChildElement("Attribute");
           attr != nullptr;
           attr = attr->NextSiblingElement("Attribute")) {
        const char* name = attr->Attribute("Name");
        if (name && std::string(name) == path[i]) {
          current = attr;
          found = true;
          break;
        }
      }
      if (!found) return "";
    }
  }

  return "";
}

// Position calculation functions
geometry_msgs::msg::Pose RobotHLInterface::calculateComponentStoragePose(ObjectType type, int instance) {
  geometry_msgs::msg::Pose pose = objects_[type].base_pick_pose;
  pose.position.x += instance * object_x_spacing_;

  RCLCPP_DEBUG(this->get_logger(), "Storage pose for %s instance %d: (%.3f, %.3f, %.3f)",
              objects_[type].name.c_str(), instance,
              pose.position.x, pose.position.y, pose.position.z);

  return pose;
}

geometry_msgs::msg::Pose RobotHLInterface::calculateRailStoragePose(int rail_index) {
  geometry_msgs::msg::Pose pose;

  // Calculate start position of rail in storage
  pose.position.x = rail_storage_start_.x();
  pose.position.y = rail_storage_start_.y() + (rail_index * rail_storage_spacing_);
  pose.position.z = rail_storage_start_.z();
  pose.orientation = rail_orientation_;

  RCLCPP_INFO(this->get_logger(), "Rail storage pose for rail %d: (%.3f, %.3f, %.3f)",
              rail_index + 1, pose.position.x, pose.position.y, pose.position.z);

  return pose;
}

geometry_msgs::msg::Pose RobotHLInterface::calculatePoseOnRail(int rail_id, double position) {
  geometry_msgs::msg::Pose pose;

  auto rail_state = getRailState(rail_id);
  if (!rail_state) {
    RCLCPP_ERROR(this->get_logger(), "Rail %d not found", rail_id);
    return pose;
  }

  if (rail_state->location == StorageLocation::WORKSPACE) {
    // Check if rail positions are initialized
    if (rail_start_.norm() == 0 || rail_end_.norm() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Rail positions not initialized!");
      // Fallback to standard position
      pose.position.x = position;
      pose.position.y = 0.0;
      pose.position.z = 0.15;
    } else {
      // Calculate position along rail
      Eigen::Vector3d direction = rail_end_ - rail_start_;
      direction.normalize();
      Eigen::Vector3d place_position = rail_start_ + direction * position;

      pose.position.x = place_position.x();
      pose.position.y = place_position.y();
      pose.position.z = place_position.z();

      RCLCPP_DEBUG(this->get_logger(), "Calculated pose on rail %d: pos(%.3f, %.3f, %.3f) for position %.3f",
                  rail_id, pose.position.x, pose.position.y, pose.position.z, position);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Rail not on workspace");
  }

  // Use rail orientation for placement
  pose.orientation = rail_orientation_;

  RCLCPP_DEBUG(this->get_logger(), "Using rail orientation for place: (%.3f, %.3f, %.3f, %.3f)",
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w);

  return pose;
}

double RobotHLInterface::calculateNextPositionOnRail(int rail_id, double component_width) {
  auto rail_state = getRailState(rail_id);
  if (!rail_state) return 0.0;

  if (rail_state->components.empty()) {
    return component_width / 2.0;
  }

  return rail_state->fill_level + component_spacing_ + component_width / 2.0;
}

// State query functions
RobotHLInterface::ComponentState* RobotHLInterface::getComponentState(ObjectType type, int instance) {
  auto key = std::make_pair(type, instance);
  auto it = component_states_.find(key);
  if (it != component_states_.end()) {
    return &it->second;
  }
  return nullptr;
}

RobotHLInterface::RailState* RobotHLInterface::getRailState(int rail_id) {
  auto it = rail_states_.find(rail_id);
  if (it != rail_states_.end()) {
    return &it->second;
  }
  return nullptr;
}

RobotHLInterface::CabinetPosition* RobotHLInterface::getCabinetPosition(int position) {
  auto it = cabinet_positions_.find(position);
  if (it != cabinet_positions_.end()) {
    return &it->second;
  }
  return nullptr;
}

int RobotHLInterface::getNextAvailableComponentInstance(ObjectType type) {
  RCLCPP_DEBUG(this->get_logger(), "Looking for available instance of type %d", static_cast<int>(type));

  for (int i = 0; i < objects_[type].count; i++) {
    auto state = getComponentState(type, i);
    if (state && state->location == StorageLocation::COMPONENT_STORAGE) {
      RCLCPP_INFO(this->get_logger(), "Found available instance %d at storage", i);
      return i;
    }
  }

  RCLCPP_WARN(this->get_logger(), "No available instance found for type %d", static_cast<int>(type));
  return -1;
}

int RobotHLInterface::getNextAvailableRailFromStorage() {
  for (const auto& [rail_id, state] : rail_states_) {
    if (state.location == StorageLocation::RAIL_STORAGE) {
      return rail_id;
    }
  }
  return -1;
}

int RobotHLInterface::getNextAvailableCabinetPosition() {
  for (const auto& [pos, cabinet_pos] : cabinet_positions_) {
    if (!cabinet_pos.occupied) {
      return pos;
    }
  }
  return -1;
}

bool RobotHLInterface::isRailFull(int rail_id, double component_width) {
  auto rail_state = getRailState(rail_id);
  if (!rail_state) return true;

  double next_position = calculateNextPositionOnRail(rail_id, component_width);
  return (next_position + component_width / 2.0) > rail_length_;
}

bool RobotHLInterface::isPositionFreeOnRail(int rail_id, double position, double required_width) {
  auto rail_state = getRailState(rail_id);
  if (!rail_state) return false;

  double half_width = required_width / 2.0;
  double start_pos = position - half_width;
  double end_pos = position + half_width;

  for (const auto& comp : rail_state->components) {
    double comp_half_width = objects_[comp.type].width / 2.0;
    double comp_start = comp.position_on_rail - comp_half_width;
    double comp_end = comp.position_on_rail + comp_half_width;

    if (!(end_pos + component_spacing_ <= comp_start ||
          start_pos >= comp_end + component_spacing_)) {
      return false;
    }
  }

  return start_pos >= 0.0 && end_pos <= rail_length_;
}

// State update functions
void RobotHLInterface::updateComponentState(ObjectType type, int instance,
                                           StorageLocation new_location,
                                           int rail_id, double rail_position) {
  auto key = std::make_pair(type, instance);
  if (component_states_.find(key) != component_states_.end()) {
    StorageLocation old_location = component_states_[key].location;
    component_states_[key].location = new_location;
    component_states_[key].rail_index = rail_id;
    component_states_[key].position_on_rail = rail_position;

    RCLCPP_INFO(this->get_logger(), "Updated %s instance %d: location %d->%d, rail %d, pos %.3f",
                objects_[type].name.c_str(), instance,
                static_cast<int>(old_location), static_cast<int>(new_location),
                rail_id, rail_position);
  }
}

void RobotHLInterface::updateRailState(int rail_id, StorageLocation new_location,
                                      int cabinet_position) {
  if (rail_states_.find(rail_id) != rail_states_.end()) {
    rail_states_[rail_id].location = new_location;
    rail_states_[rail_id].cabinet_position = cabinet_position;
  }
}

void RobotHLInterface::recalculateRailFillLevel(int rail_id) {
  auto rail_state = getRailState(rail_id);
  if (!rail_state) return;

  rail_state->fill_level = 0.0;
  for (const auto& comp : rail_state->components) {
    double comp_end = comp.position_on_rail + objects_[comp.type].width / 2.0;
    rail_state->fill_level = std::max(rail_state->fill_level, comp_end);
  }
}

// Movement and planning functions
bool RobotHLInterface::moveToPosition(const geometry_msgs::msg::Pose& target_pose,
                                     const std::string& description) {
  move_group_->setPoseTarget(target_pose);
  return planAndExecute(description);
}

bool RobotHLInterface::moveCartesian(const geometry_msgs::msg::Pose& start_pose,
                                    const geometry_msgs::msg::Pose& end_pose,
                                    const std::string& description,
                                    double velocity_factor,
                                    int max_attempts)
{
  RCLCPP_INFO(this->get_logger(), "=== Cartesian: %s ===", description.c_str());

  // Aktuelle Parameter sichern
  PlanningParams original_params = current_params_;

  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    // Adaptive Parameter für kartesische Bewegung
    if (attempt <= static_cast<int>(adaptive_levels_.size())) {
      const AdaptiveLevel& level = adaptive_levels_[attempt - 1];

      // Velocity-Factor mit adaptivem Level kombinieren
      double effective_velocity = velocity_factor * level.velocity_scaling;

      move_group_->setMaxVelocityScalingFactor(effective_velocity);
      move_group_->setMaxAccelerationScalingFactor(effective_velocity);

      RCLCPP_INFO(this->get_logger(), "Attempt %d/%d - (%.0f%%)",
                  attempt, max_attempts,
                  level.velocity_scaling * 100);
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(end_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 5.0;
    const double eef_step = 0.005;

    move_group_->setStartStateToCurrentState();

    double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

    if (fraction >= 0.95) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      auto result = move_group_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✓ Success");
        applyPlanningParams(original_params);
        return true;
      }
    }

    if (attempt < max_attempts) {
      rclcpp::sleep_for(std::chrono::milliseconds(200 + (attempt * 100)));
    }
  }

  applyPlanningParams(original_params);
  RCLCPP_ERROR(this->get_logger(), "✗ Failed");
  return false;
}

bool RobotHLInterface::attachObject(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Attaching %s", object_id.c_str());

  std::map<std::string, moveit_msgs::msg::CollisionObject> objects =
    planning_scene_interface_->getObjects({object_id});

  if (objects.find(object_id) == objects.end()) {
    RCLCPP_ERROR(this->get_logger(), "Object %s not found in planning scene", object_id.c_str());
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = eef_link_;
  attached_object.object = objects[object_id];
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  attached_object.touch_links.push_back(eef_link_);
  attached_object.touch_links.push_back("gripper_base");
  attached_object.touch_links.push_back("left_finger");
  attached_object.touch_links.push_back("right_finger");

  planning_scene_interface_->applyAttachedCollisionObject(attached_object);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  return true;
}

bool RobotHLInterface::detachObject(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Detaching %s", object_id.c_str());

  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.link_name = eef_link_;
  detach_object.object.id = object_id;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  planning_scene_interface_->applyAttachedCollisionObject(detach_object);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  return true;
}

bool RobotHLInterface::attachRailWithComponents(int rail_id) {
  std::string rail_obj_id = "hutschiene_" + std::to_string(rail_id);
  if (!attachObject(rail_obj_id)) {
    RCLCPP_WARN(this->get_logger(), "Could not attach rail");
  }

  auto rail_state = getRailState(rail_id);
  if (rail_state) {
    for (const auto& comp : rail_state->components) {
      if (!attachObject(comp.object_id)) {
        RCLCPP_WARN(this->get_logger(), "Could not attach component %s",
                    comp.object_id.c_str());
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  return true;
}

bool RobotHLInterface::detachRailWithComponents(int rail_id) {
  auto rail_state = getRailState(rail_id);
  if (rail_state) {
    for (const auto& comp : rail_state->components) {
      if (!detachObject(comp.object_id)) {
        RCLCPP_WARN(this->get_logger(), "Could not detach component %s",
                    comp.object_id.c_str());
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string rail_obj_id = "hutschiene_" + std::to_string(rail_id);
  if (!detachObject(rail_obj_id)) {
    RCLCPP_WARN(this->get_logger(), "Could not detach rail");
  }

  return true;
}

bool RobotHLInterface::checkAndMoveToHomeIfNeeded() {
  if (total_placed_count_ > 0 && total_placed_count_ % HOME_POSITION_INTERVAL == 0) {
    RCLCPP_INFO(this->get_logger(), "Placed %d components - Moving to home position",
                total_placed_count_);

    if (!move_to_home()) {
      RCLCPP_WARN(this->get_logger(), "Failed to move to home position");
      return false;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  }
  return true;
}

bool RobotHLInterface::parseAMLFile(const std::string& aml_file_path) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(aml_file_path.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load AML file: %s", aml_file_path.c_str());
    return false;
  }

  tinyxml2::XMLElement* root = doc.RootElement();
  if (!root) return false;

  // Parse Roboter-Parameter
  for (tinyxml2::XMLElement* hierarchy = root->FirstChildElement("InstanceHierarchy");
       hierarchy != nullptr;
       hierarchy = hierarchy->NextSiblingElement("InstanceHierarchy")) {

    const char* name = hierarchy->Attribute("Name");

    // Parse Roboter-Parameter
    if (name && std::string(name) == "Roboter-Parameter") {
      for (tinyxml2::XMLElement* elem = hierarchy->FirstChildElement("InternalElement");
           elem != nullptr;
           elem = elem->NextSiblingElement("InternalElement")) {
        const char* elem_name = elem->Attribute("Name");

        // Parse Greif-Parameter
        if (elem_name && std::string(elem_name) == "Greif-Parameter") {
          std::string approach_height = getAttributeValue(elem, "Approach-Höhe");
          std::string obj_spacing_x = getAttributeValue(elem, "Objekt-X-Spacing");
          std::string obj_spacing_rail = getAttributeValue(elem, "Objekt-Spacing-auf-Schiene");
          std::string pick_orient = getAttributeValue(elem, "Standard-Pick-Orientierung");
          std::string place_orient = getAttributeValue(elem, "Standard-Place-Orientierung");

          if (!approach_height.empty()) {
            approach_height_ = std::stod(approach_height) / 1000.0; // mm to m
          }
          if (!obj_spacing_x.empty()) {
            object_x_spacing_ = std::stod(obj_spacing_x) / 1000.0; // mm to m
          }
          if (!obj_spacing_rail.empty()) {
            component_spacing_ = std::stod(obj_spacing_rail) / 1000.0; // mm to m
          }
          if (!pick_orient.empty()) {
            standard_pick_orientation_ = parseQuaternion(pick_orient);
          }
          if (!place_orient.empty()) {
            standard_place_orientation_ = parseQuaternion(place_orient);
          }
        }

        // Parse Schaltschrank-Parameter
        if (elem_name && std::string(elem_name) == "Schaltschrank-Parameter") {
          std::string base_x = getAttributeValue(elem, "Basis-Position-X");
          std::string base_y = getAttributeValue(elem, "Basis-Position-Y");
          std::string base_z = getAttributeValue(elem, "Basis-Position-Z");
          std::string rail_spacing = getAttributeValue(elem, "Schienen-Abstand");

          // Parse global rail storage spacing
          std::string storage_spacing = getAttributeValue(elem, "Hutschienen-Lager-Abstand");

          if (!base_x.empty()) cabinet_base_.x() = std::stod(base_x);
          if (!base_y.empty()) cabinet_base_.y() = std::stod(base_y);
          if (!base_z.empty()) cabinet_base_.z() = std::stod(base_z);
          if (!rail_spacing.empty()) cabinet_rail_spacing_ = std::stod(rail_spacing);

          // Set parsed value
          if (!storage_spacing.empty()) {
            rail_storage_spacing_ = std::stod(storage_spacing);
            RCLCPP_INFO(this->get_logger(), "Parsed rail storage spacing: %.3f m", rail_storage_spacing_);
          }
        }
      }
    }

    // Parse Geometrie-Bibliothek
    if (name && std::string(name) == "Geometrie-Bibliothek") {
      for (tinyxml2::XMLElement* bauteile = hierarchy->FirstChildElement("InternalElement");
           bauteile != nullptr;
           bauteile = bauteile->NextSiblingElement("InternalElement")) {

        const char* bauteile_name = bauteile->Attribute("Name");
        if (bauteile_name && std::string(bauteile_name) == "Bauteile") {
          // Parse all components
          for (tinyxml2::XMLElement* bauteil = bauteile->FirstChildElement("InternalElement");
               bauteil != nullptr;
               bauteil = bauteil->NextSiblingElement("InternalElement")) {

            const char* bauteil_name = bauteil->Attribute("Name");
            if (!bauteil_name) continue;

            std::string name_str(bauteil_name);

            // Parse electronic terminals
            if (name_str == "CLIPFIX_35" || name_str == "PT_6_TWIN_BU" ||
                name_str == "UT_16_PE" || name_str == "VAL_MS_230" ||
                name_str == "VAL_MS_T1_T2" || name_str == "PTU_2_5_TWIN_BU" ||
                name_str == "QTCU_2_5" || name_str == "STU_35_4X10_YE" ||
                name_str == "UT_1_5_VT" || name_str == "UT_1_5_OG") {

              ObjectInfo obj_info;
              obj_info.name = name_str;

              // Parse width
              std::string breite = getAttributeValue(bauteil, "Geometrie/Breite");
              if (!breite.empty()) {
                obj_info.width = std::stod(breite) / 1000.0; // mm to m
              }

              // Parse pick position
              std::string pick_pos = getAttributeValue(bauteil, "Roboter-Daten/Pick-Position");
              if (!pick_pos.empty()) {
                obj_info.base_pick_pose.position = parsePosition(pick_pos);
              }

              // Use global pick orientation
              obj_info.base_pick_pose.orientation = standard_pick_orientation_;

              // Parse count
              std::string anzahl = getAttributeValue(bauteil, "Roboter-Daten/Anzahl");
              if (!anzahl.empty()) {
                obj_info.count = std::stoi(anzahl);
              }

              // base_id
              obj_info.base_id = name_str;
              std::transform(obj_info.base_id.begin(), obj_info.base_id.end(),
                           obj_info.base_id.begin(), ::tolower);
              obj_info.base_id += "_";

              // Map to ObjectType
              ObjectType type = getObjectTypeFromName(name_str);
              objects_[type] = obj_info;

              RCLCPP_INFO(this->get_logger(), "Loaded object from AML: %s (width: %.3fm, count: %d)",
                         obj_info.name.c_str(), obj_info.width, obj_info.count);
            }

            // Parse rail data
            if (name_str == "Hutschiene") {
              std::string start_pos = getAttributeValue(bauteil, "Roboter-Daten/Start-Position");
              std::string end_pos = getAttributeValue(bauteil, "Roboter-Daten/End-Position");
              std::string orient = getAttributeValue(bauteil, "Roboter-Daten/Greif-Orientierung");
              std::string y_spacing = getAttributeValue(bauteil, "Roboter-Daten/Y-Spacing");
              std::string anzahl = getAttributeValue(bauteil, "Roboter-Daten/Anzahl");
              std::string length = getAttributeValue(bauteil, "Geometrie/Länge");
              std::string spawn_base = getAttributeValue(bauteil, "Spawn-Daten/Basis-Position");

              // Parse rail length first
              if (!length.empty()) {
                rail_length_ = std::stod(length) / 1000.0; // mm to m
              }

              // Parse start position
              if (!start_pos.empty()) {
                auto pos = parsePosition(start_pos);
                rail_start_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
                RCLCPP_INFO(this->get_logger(), "Rail start position: %.3f, %.3f, %.3f",
                            rail_start_.x(), rail_start_.y(), rail_start_.z());
              }

              // Parse end position
              if (!end_pos.empty()) {
                auto pos = parsePosition(end_pos);
                rail_end_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
                RCLCPP_INFO(this->get_logger(), "Rail end position: %.3f, %.3f, %.3f",
                            rail_end_.x(), rail_end_.y(), rail_end_.z());
              }

              // Fallback: Use spawn position if start/end not available
              if ((rail_start_.norm() == 0 || rail_end_.norm() == 0) && !spawn_base.empty()) {
                auto pos = parsePosition(spawn_base);
                rail_storage_start_ = Eigen::Vector3d(pos.x, pos.y, pos.z);

                // Set rail_start_ to spawn position if not set
                if (rail_start_.norm() == 0) {
                  rail_start_ = rail_storage_start_;
                  RCLCPP_INFO(this->get_logger(), "Using spawn position as rail start: %.3f, %.3f, %.3f",
                              rail_start_.x(), rail_start_.y(), rail_start_.z());
                }

                // Calculate rail_end_ based on length if not set
                if (rail_end_.norm() == 0 && rail_length_ > 0) {
                  rail_end_ = rail_start_ + Eigen::Vector3d(rail_length_, 0, 0);
                  RCLCPP_INFO(this->get_logger(), "Calculated rail end based on length: %.3f, %.3f, %.3f",
                              rail_end_.x(), rail_end_.y(), rail_end_.z());
                }
              }

              // Set storage start position
              if (!spawn_base.empty()) {
                auto pos = parsePosition(spawn_base);
                rail_storage_start_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
              } else if (rail_start_.norm() > 0) {
                // Fallback: use rail_start_ as storage start
                rail_storage_start_ = rail_start_;
              }

              // Parse orientation
              if (!orient.empty()) {
                rail_orientation_ = parseQuaternion(orient);

                // Normalize the quaternion
                Eigen::Quaterniond rail_quat(rail_orientation_.w, rail_orientation_.x,
                                            rail_orientation_.y, rail_orientation_.z);
                rail_quat.normalize();
                rail_orientation_.x = rail_quat.x();
                rail_orientation_.y = rail_quat.y();
                rail_orientation_.z = rail_quat.z();
                rail_orientation_.w = rail_quat.w();

                RCLCPP_INFO(this->get_logger(), "Rail orientation: %.4f, %.4f, %.4f, %.4f",
                            rail_orientation_.x, rail_orientation_.y,
                            rail_orientation_.z, rail_orientation_.w);
              }

              // Parse other parameters
              // NOTE: Y-Spacing not used anymore, using global value

              if (!anzahl.empty()) {
                max_rails_ = std::stoi(anzahl);
              }

              // Calculate rail center
              if (rail_start_.norm() > 0 && rail_end_.norm() > 0) {
                rail_center_ = (rail_start_ + rail_end_) / 2.0;
                RCLCPP_INFO(this->get_logger(), "Rail center: %.3f, %.3f, %.3f",
                            rail_center_.x(), rail_center_.y(), rail_center_.z());
              }

              // Ensure rail_storage_start_ is initialized
              if (rail_storage_start_.norm() == 0 && rail_start_.norm() > 0) {
                rail_storage_start_ = rail_start_;
                RCLCPP_INFO(this->get_logger(), "Set rail_storage_start_ from rail_start_: %.3f, %.3f, %.3f",
                            rail_storage_start_.x(), rail_storage_start_.y(), rail_storage_start_.z());
              }

              RCLCPP_INFO(this->get_logger(), "Loaded Hutschiene data from AML");
              RCLCPP_INFO(this->get_logger(), "  Max rails: %d", max_rails_);
              RCLCPP_INFO(this->get_logger(), "  Rail length: %.3fm", rail_length_);
              RCLCPP_INFO(this->get_logger(), "  Storage spacing: %.3fm", rail_storage_spacing_);
            }
          }
        }
      }
    }
  }

  return true;
}

bool RobotHLInterface::validateAMLConfig() {
  if (objects_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No objects loaded from AML");
    return false;
  }

  // Check if all expected ObjectTypes are present
  std::vector<ObjectType> expected_types = {
    ObjectType::CLIPFIX_35,
    ObjectType::PT_6_TWIN_BU,
    ObjectType::UT_16_PE,
    ObjectType::VAL_MS_230,
    ObjectType::VAL_MS_T1_T2,
    ObjectType::PTU_2_5_TWIN_BU,
    ObjectType::QTCU_2_5,
    ObjectType::STU_35_4X10_YE,
    ObjectType::UT_1_5_VT,
    ObjectType::UT_1_5_OG
  };

  for (const auto& type : expected_types) {
    if (objects_.find(type) == objects_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Missing object type in AML configuration");
      return false;
    }
  }

  // Validate robot parameters
  if (approach_height_ <= 0 || component_spacing_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid robot parameters");
    return false;
  }

  // Validate rail configuration
  if (max_rails_ <= 0 || rail_length_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid rail configuration");
    return false;
  }

  // Validate cabinet configuration
  if (cabinet_rail_spacing_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid cabinet configuration");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "AML configuration validated successfully");
  return true;
}

bool RobotHLInterface::saveStateToAML(const std::string& state_file_path) {
  RCLCPP_INFO(this->get_logger(), "Saving state to %s", state_file_path.c_str());

  try {
    tinyxml2::XMLDocument doc;

    // Create root elements
    tinyxml2::XMLDeclaration* decl = doc.NewDeclaration();
    doc.InsertFirstChild(decl);

    tinyxml2::XMLElement* root = doc.NewElement("CAEXFile");
    root->SetAttribute("SchemaVersion", "3.0");
    root->SetAttribute("FileName", "SchaltschrankZustand.aml");
    root->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    root->SetAttribute("xmlns", "http://www.dke.de/CAEX");
    doc.InsertEndChild(root);

    // Add metadata
    tinyxml2::XMLElement* version = doc.NewElement("SuperiorStandardVersion");
    version->SetText("AutomationML 2.1");
    root->InsertEndChild(version);

    // Add instance hierarchy
    tinyxml2::XMLElement* hierarchy = doc.NewElement("InstanceHierarchy");
    hierarchy->SetAttribute("Name", "Schaltschrank-Zustand");
    hierarchy->SetAttribute("ID", "cabinet-state");
    root->InsertEndChild(hierarchy);

    // Add state info
    tinyxml2::XMLElement* stateInfo = doc.NewElement("InternalElement");
    stateInfo->SetAttribute("Name", "Zustandsinfo");
    stateInfo->SetAttribute("ID", "state-info");
    hierarchy->InsertEndChild(stateInfo);

    // Helper lambda to add attributes
    auto addAttribute = [&doc](tinyxml2::XMLElement* parent, const char* name,
                               const char* type, const std::string& value) {
      tinyxml2::XMLElement* attr = doc.NewElement("Attribute");
      attr->SetAttribute("Name", name);
      attr->SetAttribute("AttributeDataType", type);
      tinyxml2::XMLElement* val = doc.NewElement("Value");
      val->SetText(value.c_str());
      attr->InsertEndChild(val);
      parent->InsertEndChild(attr);
    };

    addAttribute(stateInfo, "Arbeitsplatz-Schiene", "xs:int",
                std::to_string(workspace_rail_id_));

    // Add rails
    tinyxml2::XMLElement* rails = doc.NewElement("InternalElement");
    rails->SetAttribute("Name", "Hutschienen");
    rails->SetAttribute("ID", "rails-state");
    hierarchy->InsertEndChild(rails);

    // Add each rail
    for (const auto& [rail_id, rail_state] : rail_states_) {
      tinyxml2::XMLElement* rail = doc.NewElement("InternalElement");
      rail->SetAttribute("Name", ("Schiene-" + std::to_string(rail_id)).c_str());
      rail->SetAttribute("ID", ("rail-" + std::to_string(rail_id)).c_str());
      rails->InsertEndChild(rail);

      // Rail attributes
      addAttribute(rail, "Location", "xs:string",
                  std::to_string(static_cast<int>(rail_state.location)));
      addAttribute(rail, "Cabinet-Position", "xs:int",
                  std::to_string(rail_state.cabinet_position));
      addAttribute(rail, "Füllstand", "xs:double",
                  std::to_string(rail_state.fill_level));

      // Add components
      tinyxml2::XMLElement* components = doc.NewElement("InternalElement");
      components->SetAttribute("Name", "Komponenten");
      components->SetAttribute("ID", ("rail-" + std::to_string(rail_id) + "-components").c_str());
      rail->InsertEndChild(components);

      int pos_idx = 1;
      for (const auto& comp : rail_state.components) {
        tinyxml2::XMLElement* comp_elem = doc.NewElement("InternalElement");
        comp_elem->SetAttribute("Name", ("Position-" + std::to_string(pos_idx)).c_str());
        comp_elem->SetAttribute("ID", ("rail-" + std::to_string(rail_id) + "-pos-" +
                                     std::to_string(pos_idx)).c_str());
        components->InsertEndChild(comp_elem);

        addAttribute(comp_elem, "Komponenten-Typ", "xs:string", objects_[comp.type].name);
        addAttribute(comp_elem, "Original-ID", "xs:string", comp.object_id);
        addAttribute(comp_elem, "Instance-Number", "xs:int",
                    std::to_string(comp.instance_number));
        addAttribute(comp_elem, "Position-auf-Schiene", "xs:double",
                    std::to_string(comp.position_on_rail));
        addAttribute(comp_elem, "Breite", "xs:double",
                    std::to_string(objects_[comp.type].width));

        pos_idx++;
      }
    }

    // Save to file
    if (doc.SaveFile(state_file_path.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save state file");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully saved state with %zu rails",
                rail_states_.size());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving state: %s", e.what());
    return false;
  }
}

bool RobotHLInterface::loadStateFromAML(const std::string& state_file_path) {
  RCLCPP_INFO(this->get_logger(), "Loading state from %s", state_file_path.c_str());

  try {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(state_file_path.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "State file not found or invalid - starting with fresh state");
      return true; // Not an error - just no previous state
    }

    // Clear current state (except object definitions)
    component_states_.clear();
    rail_states_.clear();
    workspace_rail_id_ = -1;

    // Reset cabinet positions
    for (auto& [pos, cab_pos] : cabinet_positions_) {
      cab_pos.occupied = false;
      cab_pos.rail_id = -1;
    }

    // Parse state
    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) return false;

    // Navigate to state hierarchy
    tinyxml2::XMLElement* hierarchy = root->FirstChildElement("InstanceHierarchy");
    if (!hierarchy) return false;

    // Get workspace rail
    tinyxml2::XMLElement* state_info = nullptr;
    for (tinyxml2::XMLElement* elem = hierarchy->FirstChildElement("InternalElement");
         elem != nullptr;
         elem = elem->NextSiblingElement("InternalElement")) {
      const char* name = elem->Attribute("Name");
      if (name && std::string(name) == "Zustandsinfo") {
        state_info = elem;
        break;
      }
    }

    if (state_info) {
      std::string workspace_rail_str = getAttributeValue(state_info, "Arbeitsplatz-Schiene");
      if (!workspace_rail_str.empty()) {
        workspace_rail_id_ = std::stoi(workspace_rail_str);
      }
    }

    // Navigate to rails
    tinyxml2::XMLElement* rails_elem = nullptr;
    for (tinyxml2::XMLElement* elem = hierarchy->FirstChildElement("InternalElement");
         elem != nullptr;
         elem = elem->NextSiblingElement("InternalElement")) {
      const char* name = elem->Attribute("Name");
      if (name && std::string(name) == "Hutschienen") {
        rails_elem = elem;
        break;
      }
    }

    if (!rails_elem) {
      RCLCPP_WARN(this->get_logger(), "No rails found in state file");
      return true; // Not an error - empty state
    }

    // Parse each rail
    for (tinyxml2::XMLElement* rail_elem = rails_elem->FirstChildElement("InternalElement");
         rail_elem != nullptr;
         rail_elem = rail_elem->NextSiblingElement("InternalElement")) {

      const char* rail_name = rail_elem->Attribute("Name");
      if (!rail_name) continue;

      // Extract rail number
      std::string name_str(rail_name);
      if (name_str.find("Schiene-") != 0) continue;

      int rail_id = std::stoi(name_str.substr(8));

      RailState rail_state;
      rail_state.rail_id = rail_id;

      // Parse attributes
      std::string location_str = getAttributeValue(rail_elem, "Location");
      std::string cabinet_pos_str = getAttributeValue(rail_elem, "Cabinet-Position");
      std::string fill_level_str = getAttributeValue(rail_elem, "Füllstand");

      if (!location_str.empty()) {
        rail_state.location = static_cast<StorageLocation>(std::stoi(location_str));
      }
      if (!cabinet_pos_str.empty()) {
        rail_state.cabinet_position = std::stoi(cabinet_pos_str);
      }
      if (!fill_level_str.empty()) {
        rail_state.fill_level = std::stod(fill_level_str);
      }

      // Update cabinet position if in cabinet
      if (rail_state.location == StorageLocation::CABINET && rail_state.cabinet_position > 0) {
        if (cabinet_positions_.find(rail_state.cabinet_position) != cabinet_positions_.end()) {
          cabinet_positions_[rail_state.cabinet_position].occupied = true;
          cabinet_positions_[rail_state.cabinet_position].rail_id = rail_id;
        }
        rail_state.current_pose = calculateCabinetPose(rail_state.cabinet_position);
      } else if (rail_state.location == StorageLocation::WORKSPACE) {
        // Workspace pose
        rail_state.current_pose.position.x = rail_center_.x();
        rail_state.current_pose.position.y = rail_center_.y();
        rail_state.current_pose.position.z = rail_center_.z();
        rail_state.current_pose.orientation = rail_orientation_;
      } else {
        // Storage pose
        rail_state.current_pose = calculateRailStoragePose(rail_id - 1);
      }

      // IMPORTANT: Add rail to map BEFORE parsing components!
      rail_states_[rail_id] = rail_state;

      // Parse components
      tinyxml2::XMLElement* components_elem = nullptr;
      for (tinyxml2::XMLElement* elem = rail_elem->FirstChildElement("InternalElement");
           elem != nullptr;
           elem = elem->NextSiblingElement("InternalElement")) {
        const char* name = elem->Attribute("Name");
        if (name && std::string(name) == "Komponenten") {
          components_elem = elem;
          break;
        }
      }

      if (components_elem) {
        // Get reference to the rail state in the map (not the local copy)
        RailState& rail_state_ref = rail_states_[rail_id];

        for (tinyxml2::XMLElement* comp_elem = components_elem->FirstChildElement("InternalElement");
             comp_elem != nullptr;
             comp_elem = comp_elem->NextSiblingElement("InternalElement")) {

          ComponentState comp;

          // Parse component attributes
          std::string comp_type_str = getAttributeValue(comp_elem, "Komponenten-Typ");
          std::string original_id = getAttributeValue(comp_elem, "Original-ID");
          std::string instance_str = getAttributeValue(comp_elem, "Instance-Number");
          std::string position_str = getAttributeValue(comp_elem, "Position-auf-Schiene");

          // Map name to ObjectType
          comp.type = getObjectTypeFromName(comp_type_str);
          comp.object_id = original_id;
          comp.instance_number = std::stoi(instance_str);
          comp.position_on_rail = std::stod(position_str);
          comp.rail_index = rail_id;

          if (rail_state_ref.location == StorageLocation::CABINET) {
            comp.location = StorageLocation::CABINET;
          } else if (rail_state_ref.location == StorageLocation::WORKSPACE) {
            comp.location = StorageLocation::WORKSPACE;
          }

          // Now calculatePoseOnRail should work because rail is already in map
          comp.current_pose = calculatePoseOnRail(rail_id, comp.position_on_rail);

          rail_state_ref.components.push_back(comp);

          // Update component state
          component_states_[{comp.type, comp.instance_number}] = comp;
        }
      }
    }

    // Initialize remaining components as in storage
    for (const auto& [type, info] : objects_) {
      for (int i = 0; i < info.count; i++) {
        auto key = std::make_pair(type, i);
        if (component_states_.find(key) == component_states_.end()) {
          ComponentState state;
          state.type = type;
          state.object_id = info.base_id + std::to_string(i + 1);
          state.instance_number = i;
          state.location = StorageLocation::COMPONENT_STORAGE;
          state.rail_index = -1;
          state.position_on_rail = 0.0;
          state.current_pose = calculateComponentStoragePose(type, i);

          component_states_[key] = state;
        }
      }
    }

    // Initialize remaining rails as in storage
    for (int i = 1; i <= max_rails_; i++) {
      if (rail_states_.find(i) == rail_states_.end()) {
        RailState state;
        state.rail_id = i;
        state.location = StorageLocation::RAIL_STORAGE;
        state.cabinet_position = -1;
        state.fill_level = 0.0;
        state.current_pose = calculateRailStoragePose(i - 1);

        rail_states_[i] = state;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded state");
    RCLCPP_INFO(this->get_logger(), "Workspace rail: %d, %zu rails in system",
                workspace_rail_id_, rail_states_.size());

    // Log cabinet state
    int rails_in_cabinet = 0;
    for (const auto& [pos, cab_pos] : cabinet_positions_) {
      if (cab_pos.occupied) {
        rails_in_cabinet++;
        RCLCPP_INFO(this->get_logger(), "Cabinet position %d: Rail %d",
                   pos, cab_pos.rail_id);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Rails in cabinet: %d", rails_in_cabinet);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading state: %s", e.what());
    return false;
  }
}

// Initialisierung der adaptiven Level
void RobotHLInterface::initializeAdaptiveLevels()
{
  adaptive_levels_ = {
    // Versuche 1-2: Standard (100%)
    {1.00, 0.001, 0.001, 10.0, "PRM"},
    {1.00, 0.001, 0.001, 10.0, "PRM"},

    // Versuche 3-4: 75% + lockerer
    {0.75, 0.003, 0.003, 15.0, "PRM"},
    {0.75, 0.003, 0.003, 15.0, "PRM"},

    // Versuch 5: 50% + noch lockerer
    {0.50, 0.005, 0.005, 20.0, "PRM"},

    // Versuch 6: 25% + sehr locker
    {0.25, 0.010, 0.010, 30.0, "PRM"},

    // Versuche 7-8: Wie 6, aber RRTConnect
    {0.25, 0.010, 0.010, 30.0, "RRTConnect"},
    {0.25, 0.010, 0.010, 30.0, "RRTConnect"},

    // Versuche 9-10: Wie 6, aber BiTRRT
    {0.25, 0.010, 0.010, 30.0, "BiTRRT"},
    {0.25, 0.010, 0.010, 30.0, "BiTRRT"}
  };
}

// Planning helper implementations
void RobotHLInterface::applyPlanningParams(const PlanningParams& params)
{
  move_group_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(params.acceleration_scaling);
  move_group_->setPlanningTime(params.planning_time);
  move_group_->setGoalPositionTolerance(params.goal_position_tolerance);
  move_group_->setGoalOrientationTolerance(params.goal_orientation_tolerance);
  move_group_->setGoalJointTolerance(params.goal_joint_tolerance);

  if (!params.planner_id.empty()) {
    move_group_->setPlannerId(params.planner_id);
  }

  // Update current parameters
  current_params_ = params;
}

void RobotHLInterface::restoreDefaultParams()
{
  applyPlanningParams(default_params_);
}

bool RobotHLInterface::planAndExecute(const std::string& description, int max_attempts)
{
  RCLCPP_INFO(this->get_logger(), "=== Planning: %s ===", description.c_str());

  // Aktuelle Parameter sichern
  PlanningParams original_params = current_params_;

  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    // Adaptive Parameter basierend auf Versuchsnummer auswählen
    PlanningParams attempt_params = default_params_;

    if (attempt <= static_cast<int>(adaptive_levels_.size())) {
      const AdaptiveLevel& level = adaptive_levels_[attempt - 1];

      attempt_params.velocity_scaling = level.velocity_scaling;
      attempt_params.acceleration_scaling = level.velocity_scaling;
      attempt_params.goal_position_tolerance = level.position_tolerance;
      attempt_params.goal_orientation_tolerance = level.orientation_tolerance;
      attempt_params.goal_joint_tolerance = level.position_tolerance;
      attempt_params.planning_time = level.planning_time;
      attempt_params.planner_id = level.planner_id;

      // Vereinfachtes Log - nur Prozent und Planer
      RCLCPP_INFO(this->get_logger(), "Attempt %d/%d - (%.0f%% - %s)",
                  attempt, max_attempts,
                  level.velocity_scaling * 100,
                  level.planner_id.c_str());
    } else {
      // Fallback für mehr als 10 Versuche
      RCLCPP_WARN(this->get_logger(), "Attempt %d/%d - Using last adaptive level",
                  attempt, max_attempts);
    }

    // Parameter anwenden
    applyPlanningParams(attempt_params);

    // Planung versuchen
    move_group_->setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool planning_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planning_success) {
      // Ausführung versuchen
      bool exec_success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (exec_success) {
        RCLCPP_INFO(this->get_logger(), "✓ Success");

        // Original-Parameter wiederherstellen
        applyPlanningParams(original_params);
        return true;
      }
    }

    // Adaptive Pause zwischen Versuchen
    if (attempt < max_attempts) {
      rclcpp::sleep_for(std::chrono::milliseconds(200 + (attempt * 100)));
    }
  }

  // Original-Parameter wiederherstellen
  applyPlanningParams(original_params);

  RCLCPP_ERROR(this->get_logger(), "✗ Failed after %d attempts", max_attempts);
  return false;
}