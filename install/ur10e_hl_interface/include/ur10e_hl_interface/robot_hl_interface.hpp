#ifndef ROBOT_HL_INTERFACE_HPP
#define ROBOT_HL_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <tinyxml2.h>

class RobotHLInterface : public rclcpp::Node
{
public:
  enum class ObjectType {
    CLIPFIX_35,
    PT_6_TWIN_BU,
    UT_16_PE,
    VAL_MS_230,
    VAL_MS_T1_T2,
    PTU_2_5_TWIN_BU,
    QTCU_2_5,
    STU_35_4X10_YE,
    UT_1_5_VT,
    UT_1_5_OG
  };

  enum class StorageLocation {
    COMPONENT_STORAGE,    // Original component storage
    RAIL_STORAGE,        // Rail storage area
    CABINET,            // In cabinet
    WORKSPACE,          // On workspace
    GRIPPER            // Attached to gripper
  };

  struct ObjectInfo {
    std::string name;
    std::string base_id;
    double width;  // in meters
    geometry_msgs::msg::Pose base_pick_pose;
    int count;  // Number of this object type
  };

  struct ComponentState {
    ObjectType type;
    std::string object_id;
    int instance_number;
    StorageLocation location;
    int rail_index;  // -1 if not on rail
    double position_on_rail;  // Position in meters from start
    geometry_msgs::msg::Pose current_pose;
  };

  struct RailState {
    int rail_id;
    StorageLocation location;
    int cabinet_position;  // -1 if not in cabinet
    std::vector<ComponentState> components;
    double fill_level;  // Current fill position
    geometry_msgs::msg::Pose current_pose;
  };

  struct CabinetPosition {
    std::string name;  // e.g., "Schaltschrank-Position1"
    geometry_msgs::msg::Pose pose;
    bool occupied;
    int rail_id;  // -1 if empty
  };

  // Planning parameters structure
  struct PlanningParams {
    double velocity_scaling;
    double acceleration_scaling;
    double goal_position_tolerance;
    double goal_orientation_tolerance;
    double goal_joint_tolerance;
    double planning_time;
    std::string planner_id;

    PlanningParams() :
      velocity_scaling(0.8),
      acceleration_scaling(0.8),
      goal_position_tolerance(0.001),
      goal_orientation_tolerance(0.001),
      goal_joint_tolerance(0.001),
      planning_time(10.0),
      planner_id("") {}
  };

  RobotHLInterface();

  // === CORE FUNCTIONS ===

  // Move to home position
  bool move_to_home();

  // Pick component and place on current rail
  bool pick(const std::string& component_name);

  // Place rail in cabinet (position = -1 for next available)
  bool place_rail(int position = -1);

  // Pick rail from storage or cabinet (position = -1 for next from storage)
  bool pick_rail(int position = -1);

  // Remove component from rail and return to storage
  bool detach_from_rail(const std::string& component_name);

  // State management
  bool loadStateFromAML(const std::string& state_file_path);
  bool saveStateToAML(const std::string& state_file_path);

  // Initialization
  void initialize();

  // Public query functions for state information
  double getRailLength() const { return rail_length_; }
  double getComponentSpacing() const { return component_spacing_; }
  double getRemainingSpaceOnCurrentRail() const;
  int getCurrentWorkspaceRailId() const { return workspace_rail_id_; }

private:
  // Struktur für adaptive Parameter-Stufen
  struct AdaptiveLevel {
    double velocity_scaling;
    double position_tolerance;    // in Metern
    double orientation_tolerance; // in Radiant
    double planning_time;
    std::string planner_id;
  };

  bool moveCartesianWithPTPFallback(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& end_pose,
    const std::string& description,
    double velocity_factor = 1.0);

  bool moveCartesianWithFallback(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& end_pose,
    const std::string& description,
    double velocity_factor = 1.0);

  // Adaptive Parameter-Konfiguration
  std::vector<AdaptiveLevel> adaptive_levels_;

  // Initialisierung der adaptiven Level
  void initializeAdaptiveLevels();

  // AML parsing
  bool parseAMLFile(const std::string& aml_file_path);
  bool validateAMLConfig();
  geometry_msgs::msg::Quaternion parseQuaternion(const std::string& str);
  geometry_msgs::msg::Point parsePosition(const std::string& str);
  std::string getAttributeValue(tinyxml2::XMLElement* element, const std::string& attrPath);

  // Position calculations
  geometry_msgs::msg::Pose calculateComponentStoragePose(ObjectType type, int instance);
  geometry_msgs::msg::Pose calculateRailStoragePose(int rail_index);
  geometry_msgs::msg::Pose calculateCabinetPose(int position);
  geometry_msgs::msg::Pose calculatePoseOnRail(int rail_id, double position);
  geometry_msgs::msg::Pose calculateRailGripPose(const geometry_msgs::msg::Pose& rail_start_pose,
                                                 StorageLocation location);
  double calculateNextPositionOnRail(int rail_id, double component_width);

  // State queries
  ComponentState* getComponentState(ObjectType type, int instance);
  RailState* getRailState(int rail_id);
  CabinetPosition* getCabinetPosition(int position);
  int getNextAvailableComponentInstance(ObjectType type);
  int getNextAvailableRailFromStorage();
  int getNextAvailableCabinetPosition();
  ComponentState* getFarthestComponentOnRail(int rail_id, const std::string& component_name);
  bool isRailFull(int rail_id, double component_width);
  bool isPositionFreeOnRail(int rail_id, double position, double required_width);

  // State updates
  void updateComponentState(ObjectType type, int instance, StorageLocation new_location,
                          int rail_id = -1, double rail_position = 0.0);
  void updateRailState(int rail_id, StorageLocation new_location, int cabinet_position = -1);
  void recalculateRailFillLevel(int rail_id);

  // Movement primitives
  bool moveToPosition(const geometry_msgs::msg::Pose& target_pose, const std::string& description);
  bool moveCartesian(const geometry_msgs::msg::Pose& start_pose,
                     const geometry_msgs::msg::Pose& end_pose,
                     const std::string& description,
                     double velocity_factor = 1.0,
                     int max_attempts = 10);
  bool planAndExecute(const std::string& description, int max_attempts = 15);

  // Planning helpers
  void applyPlanningParams(const PlanningParams& params);
  void restoreDefaultParams();

  // Attach/Detach operations
  bool attachObject(const std::string& object_id);
  bool detachObject(const std::string& object_id);
  bool attachRailWithComponents(int rail_id);
  bool detachRailWithComponents(int rail_id);

  // Home position management
  bool checkAndMoveToHomeIfNeeded();

  // Object type mapping
  ObjectType getObjectTypeFromName(const std::string& name);

  // Member variables
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::string eef_link_;

  // Object tracking
  std::map<ObjectType, ObjectInfo> objects_;
  std::map<std::pair<ObjectType, int>, ComponentState> component_states_;  // (type, instance) -> state

  // Rail tracking
  std::map<int, RailState> rail_states_;  // rail_id -> state
  int workspace_rail_id_ = -1;  // Current rail on workspace

  // Cabinet tracking
  std::map<int, CabinetPosition> cabinet_positions_;  // position -> info

  // Storage positions from AML
  Eigen::Vector3d rail_storage_start_;
  double rail_storage_spacing_ = 0.1;  // 10cm between rails
  geometry_msgs::msg::Quaternion rail_orientation_;
  int max_rails_ = 5;
  double rail_length_ = 0.683;  // 683mm

  // Rail position data
  Eigen::Vector3d rail_start_;     // Start position of rail
  Eigen::Vector3d rail_end_;       // End position of rail
  Eigen::Vector3d rail_center_;    // Center of rail

  // Cabinet base position from AML
  Eigen::Vector3d cabinet_base_;
  double cabinet_rail_spacing_ = 0.15;  // 15cm between rails in cabinet

  // Component spacing
  double component_spacing_ = 0.01;  // 10mm between components
  double approach_height_ = 0.1;  // 100mm
  // place_height_offset_ removed - components placed directly on rail

  // Object spacing in storage
  double object_x_spacing_ = 0.15;  // 150mm between same type objects

  // Orientations
  geometry_msgs::msg::Quaternion standard_pick_orientation_;
  geometry_msgs::msg::Quaternion standard_place_orientation_;

  // File paths
  std::string aml_file_path_;
  std::string state_file_path_;

  // Planning parameters
  PlanningParams default_params_;
  PlanningParams current_params_;

  // Tracking
  int total_placed_count_ = 0;
  static constexpr int HOME_POSITION_INTERVAL = 5;
};

#endif // ROBOT_HL_INTERFACE_HPP