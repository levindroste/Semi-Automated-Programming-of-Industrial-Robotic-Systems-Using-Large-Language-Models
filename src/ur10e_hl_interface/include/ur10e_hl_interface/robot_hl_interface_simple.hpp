#ifndef ROBOT_HL_INTERFACE_SIMPLE_HPP
#define ROBOT_HL_INTERFACE_SIMPLE_HPP

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <tinyxml2.h>

/**
 * @brief Simplified high-level robot interface for pick-and-place operations
 *
 * This class provides a simple interface for the ABB IRB 120 robot to perform
 * pick-and-place operations. Objects are tracked by their grid position (A1-E5)
 * rather than raw XYZ coordinates.
 */
class RobotHLInterfaceSimple : public rclcpp::Node
{
public:
  /**
   * @brief Object information loaded from AML
   */
  struct ObjectInfo {
    std::string name;           ///< Object name (e.g., "cube_red")
    std::string id;             ///< Collision object ID in planning scene
    std::string location;       ///< Current grid position (e.g., "A1" or "A1:1" for stacked)
    double width;               ///< Object width (X dimension)
    double height;              ///< Object height (Z dimension)
    double depth;               ///< Object depth (Y dimension)
    std::string color;          ///< Object color for visualization
    int stack_level = 0;        ///< Stack level (0 = floor, 1 = on first cube, etc.)
  };

  /**
   * @brief Motion configuration parameters
   */
  struct Config {
    double approach_height = 0.07;     ///< Approach height above objects (7cm)
    double velocity_scaling = 0.5;     ///< Movement speed scaling (0-1)
    double acceleration_scaling = 0.5; ///< Acceleration scaling (0-1)
    double planning_time = 5.0;        ///< Max planning time in seconds (reduced for faster simulation)
    std::string planner_id = "RRTConnect"; ///< MoveIt planner ID
    geometry_msgs::msg::Quaternion standard_orientation; ///< Default gripper orientation
  };

  /**
   * @brief Constructor - creates the node
   */
  RobotHLInterfaceSimple();

  /**
   * @brief Initialize MoveIt interfaces and load AML configuration
   * @return true on success, false on failure
   */
  bool initialize();

  // === MAIN INTERFACE ===

  /**
   * @brief Pick an object and place it at a goal position
   *
   * Sequence:
   * 1. Move to approach position above object (7cm)
   * 2. Move down linearly to object
   * 3. Attach object to gripper
   * 4. Move up linearly to approach position
   * 5. Move to approach position above goal
   * 6. Move down linearly to goal (height adjusted by level)
   * 7. Detach object from gripper
   * 8. Move up linearly to approach position
   * 9. Update object location in AML
   *
   * @param object_name Name of the object to pick (e.g., "cube_red")
   * @param goal_name Target grid position (e.g., "B3") or "X" for eject
   * @param level Stack level (0=floor, 1=on first cube, 2=on second, etc.)
   *              Each level adds 3cm (cube height) to the Z position
   * @return true on success, false on failure
   */
  bool PickAndPlace(const std::string& object_name, const std::string& goal_name, int level = 0);

  /**
   * @brief Add a colored cube collision object at a grid position
   * @param grid_position Grid position name (e.g., "A1", "B3")
   * @param color Color name (default: "Schwarz")
   *              Valid: "Schwarz", "Rot", "Gruen", "Gelb", "Blau", "Grau"
   * @param level Stack level (default 0)
   *              level=0: Place on table
   *              level=1: Stack on first cube (+3cm)
   *              level=2: Stack on second cube (+6cm)
   * @return Generated object name if successful, empty string if failed
   */
  std::string AddCube(const std::string& grid_position,
                      const std::string& color = "Schwarz",
                      int level = 0);

  /**
   * @brief Clear all cubes from the scene (no robot movement)
   *
   * Removes ALL cubes from the planning scene and AML file instantly.
   * Safe to call even if there are no cubes. Use this before AddCube()
   * when setting up a new cell configuration.
   *
   * Workflow for cell editing:
   * 1. ClearAllCubes()  - Remove everything
   * 2. AddCube(...)     - Add desired cubes
   */
  void ClearAllCubes();

  /**
   * @brief Move robot to home position (all joints at 0)
   * @return true on success, false on failure
   */
  bool moveToHome();

  // === QUERY FUNCTIONS ===

  /**
   * @brief Get list of available object names
   * @return Vector of object names
   */
  std::vector<std::string> getAvailableObjects() const;

  /**
   * @brief Get list of available grid positions (A1-E5)
   * @return Vector of grid position names
   */
  std::vector<std::string> getAvailableGoals() const;

  /**
   * @brief Get current location of an object
   * @param object_name Name of the object
   * @return Grid position name (e.g., "A1") or empty string if not found
   */
  std::string getObjectLocation(const std::string& object_name) const;

  /**
   * @brief Get the pose for a grid position
   * @param grid_name Grid position name (e.g., "A1")
   * @return Pose at that grid position
   */
  geometry_msgs::msg::Pose getGridPose(const std::string& grid_name) const;

  /**
   * @brief Check if a grid position is occupied
   * @param grid_name Grid position name (e.g., "A1")
   * @return true if occupied, false otherwise
   */
  bool isGridPositionOccupied(const std::string& grid_name) const;

private:
  // === AML PARSING ===

  /**
   * @brief Parse the AML configuration file
   * @param aml_file_path Path to the AML file
   * @return true on success, false on failure
   */
  bool parseAMLFile(const std::string& aml_file_path);

  /**
   * @brief Parse robot parameters from AML
   * @param root Root XML element
   * @return true on success
   */
  bool parseRobotParameters(tinyxml2::XMLElement* root);

  /**
   * @brief Parse grid configuration from AML (Grid-Config hierarchy)
   * @param root Root XML element
   * @return true on success
   */
  bool parseGridConfig(tinyxml2::XMLElement* root);

  /**
   * @brief Calculate spawn position for a grid name (e.g., "A1", "C3")
   * @param grid_name Grid position name
   * @return Spawn position (x, y, z)
   */
  geometry_msgs::msg::Point calculateSpawnPosition(const std::string& grid_name) const;

  /**
   * @brief Calculate grip position for a grid name
   * @param grid_name Grid position name
   * @return Grip position (spawn + grip_offset)
   */
  geometry_msgs::msg::Point calculateGripPosition(const std::string& grid_name) const;

  /**
   * @brief Check if a grid name is valid (A1-E5 or special positions like X)
   * @param grid_name Grid position name to validate
   * @return true if valid
   */
  bool isValidGridName(const std::string& grid_name) const;

  /**
   * @brief Check if gripper has clearance to access a grid position
   * Due to large gripper design, position X(N-1) must be free to access XN
   * @param grid_position Grid position to check access for
   * @return true if gripper can access the position (clearance OK)
   */
  bool isGripperClearanceOk(const std::string& grid_position) const;

  /**
   * @brief Find the cube blocking access to a grid position
   * @param grid_position Grid position being blocked
   * @return Name of blocking cube, or empty string if none
   */
  std::string findBlockingCube(const std::string& grid_position) const;

  /**
   * @brief Find a free position in row 1 (always accessible for temp storage)
   * @return Free row 1 position (A1-E1), or empty string if all occupied
   */
  std::string findFreeRow1Position() const;

  /**
   * @brief Parse objects from AML
   * @param root Root XML element
   * @return true on success
   */
  bool parseObjects(tinyxml2::XMLElement* root);

  /**
   * @brief Update object location in the AML file
   * @param object_name Object to update
   * @param new_location New grid position
   * @return true on success
   */
  bool updateObjectLocationInAML(const std::string& object_name, const std::string& new_location);

  /**
   * @brief Add a new object to the AML file
   * @param object_name Object name/ID
   * @param location Grid position (e.g., "A3")
   * @param color Color name (e.g., "Rot")
   * @param width Object width
   * @param depth Object depth
   * @param height Object height
   * @return true on success
   */
  bool addObjectToAML(const std::string& object_name,
                      const std::string& location,
                      const std::string& color,
                      double width, double depth, double height);

  /**
   * @brief Remove an object from the AML file
   * @param object_name Object name to remove
   * @return true on success
   */
  bool removeObjectFromAML(const std::string& object_name);

  // === AML HELPERS ===

  /**
   * @brief Parse a position string "x,y,z" into a Point
   * @param str Position string
   * @return Point with x, y, z values
   */
  geometry_msgs::msg::Point parsePosition(const std::string& str);

  /**
   * @brief Parse a quaternion string "x,y,z,w" into a Quaternion
   * @param str Quaternion string
   * @return Quaternion with x, y, z, w values
   */
  geometry_msgs::msg::Quaternion parseQuaternion(const std::string& str);

  /**
   * @brief Parse a dimensions string "w,d,h" into width, depth, height
   * @param str Dimensions string
   * @param width Output width
   * @param depth Output depth
   * @param height Output height
   */
  void parseDimensions(const std::string& str, double& width, double& depth, double& height);

  /**
   * @brief Get attribute value from XML element
   * @param element Parent element
   * @param attr_name Attribute name to find
   * @return Attribute value or empty string
   */
  std::string getAttributeValue(tinyxml2::XMLElement* element, const std::string& attr_name);

  /**
   * @brief Find an InstanceHierarchy by name
   * @param root Root element
   * @param name Hierarchy name
   * @return Pointer to hierarchy element or nullptr
   */
  tinyxml2::XMLElement* findInstanceHierarchy(tinyxml2::XMLElement* root, const std::string& name);

  // === MOVEMENT PRIMITIVES ===

  /**
   * @brief Move to a pose using point-to-point motion
   * @param target Target pose
   * @param description Description for logging
   * @return true on success
   */
  bool moveToPosition(const geometry_msgs::msg::Pose& target, const std::string& description);

  /**
   * @brief Move to a pose using point-to-point motion, returning final joint positions
   * @param target Target pose
   * @param description Description for logging
   * @param final_joints Output: final joint positions from the planned trajectory
   * @return true on success
   */
  bool moveToPosition(const geometry_msgs::msg::Pose& target, const std::string& description,
                      std::vector<double>& final_joints);

  /**
   * @brief Move linearly between two poses
   * @param start Start pose
   * @param end End pose
   * @param description Description for logging
   * @return true on success
   */
  bool moveCartesian(const geometry_msgs::msg::Pose& start,
                     const geometry_msgs::msg::Pose& end,
                     const std::string& description);

  /**
   * @brief Move linearly between two poses, returning final joint positions
   * @param start Start pose
   * @param end End pose
   * @param description Description for logging
   * @param final_joints Output: final joint positions from the planned trajectory
   * @return true on success
   */
  bool moveCartesian(const geometry_msgs::msg::Pose& start,
                     const geometry_msgs::msg::Pose& end,
                     const std::string& description,
                     std::vector<double>& final_joints);

  /**
   * @brief Plan and execute the current target
   * @param description Description for logging
   * @return true on success
   */
  bool planAndExecute(const std::string& description);

  /**
   * @brief Plan and execute the current target, returning final joint positions
   * @param description Description for logging
   * @param final_joints Output: final joint positions from the planned trajectory
   * @return true on success
   */
  bool planAndExecute(const std::string& description, std::vector<double>& final_joints);

  /**
   * @brief Calculate approach pose (7cm above target)
   * @param target Target pose
   * @return Approach pose
   */
  geometry_msgs::msg::Pose calculateApproachPose(const geometry_msgs::msg::Pose& target);

  // === ATTACH/DETACH ===

  /**
   * @brief Attach object to gripper in planning scene
   * @param object_id Collision object ID
   * @return true on success
   */
  bool attachObject(const std::string& object_id);

  /**
   * @brief Detach object from gripper in planning scene
   * @param object_id Collision object ID
   * @return true on success
   */
  bool detachObject(const std::string& object_id);

  // === COLLISION OBJECTS ===

  /**
   * @brief Add all objects to the planning scene as collision objects
   * @return true on success
   */
  bool addCollisionObjects();

  /**
   * @brief Update collision object position after move
   * @param object_name Object name
   * @param new_pose New pose
   * @return true on success
   */
  bool updateCollisionObjectPose(const std::string& object_name,
                                  const geometry_msgs::msg::Pose& new_pose);

  /**
   * @brief Remove object from planning scene (for eject feature)
   * @param object_id Collision object ID to remove
   * @param object_name Object name to remove from objects_ map
   * @return true on success
   */
  bool removeCollisionObject(const std::string& object_id, const std::string& object_name);

  /**
   * @brief Allow collisions between all cubes in the planning scene
   * This is needed for stacking cubes on top of each other
   */
  void allowCubeCollisions();

  /**
   * @brief Temporarily allow/disallow collision between gripper and target cube
   *
   * This is needed for pick operations because the gripper must enter the
   * cube's collision volume to grasp it. Call with allow=true before approaching,
   * and allow=false after the pick is complete (or rely on attachObject's touch_links).
   *
   * @param cube_id The collision object ID of the cube to pick
   * @param allow true to allow collision, false to disallow
   */
  void allowGripperCubeCollision(const std::string& cube_id, bool allow);

  /**
   * @brief Build Allowed Collision Matrix for current objects
   * @param acm The ACM message to populate
   */
  void buildACM(moveit_msgs::msg::AllowedCollisionMatrix& acm);

  /**
   * @brief Load STL mesh file manually (without vertex deduplication)
   *
   * Matches the Python add_collision_objects.py parsing approach.
   * Creates a new vertex for each triangle corner, ensuring consistent
   * appearance with cubes loaded at simulation startup.
   *
   * @param filepath Path to the STL file
   * @param scale Scale factor (0.001 for mm to m)
   * @return Mesh message
   */
  shape_msgs::msg::Mesh loadSTLMesh(const std::string& filepath, double scale);

  // === MEMBER VARIABLES ===

  /// MoveIt move group interface
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  /// MoveIt planning scene interface
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  /// Service client for applying planning scene (ACM updates)
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;

  /// Publisher to /planning_scene topic for scene updates
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  /// End effector link name
  std::string eef_link_;

  /// Planning frame
  std::string planning_frame_;

  /// Grid positions: "A1" -> Pose (spawn positions, calculated dynamically)
  std::map<std::string, geometry_msgs::msg::Pose> grid_positions_;

  /// Grid configuration for dynamic position calculation
  geometry_msgs::msg::Point a1_spawn_position_;  ///< A1 spawn position (base)
  double grid_spacing_ = 0.0475;                  ///< Grid spacing in meters (47.5mm)
  geometry_msgs::msg::Point grip_offset_;         ///< Offset from spawn to grip position

  /// Object information by name
  std::map<std::string, ObjectInfo> objects_;

  /// Motion configuration
  Config config_;

  /// Path to AML file
  std::string aml_file_path_;

  /// Touch links for gripper attachment
  std::vector<std::string> gripper_touch_links_;

  /// Counter for generating unique cube names
  int next_cube_id_ = 0;

  /// Track positions claimed during recursive auto-move operations
  /// This prevents multiple blocking cubes from being moved to the same position
  std::set<std::string> pending_auto_move_destinations_;

  /// Directory containing mesh files (Wuerfel.stl, etc.)
  std::string mesh_directory_;

  // === REAL ROBOT COMMUNICATION ===

  /// Real robot mode flag
  bool real_robot_mode_ = false;

  /// Robot IP address for socket connection
  std::string robot_ip_ = "192.168.125.1";

  /// Robot socket port
  int robot_port_ = 5000;

  /// Socket file descriptor (-1 if not connected)
  int socket_fd_ = -1;

  /// Last J6 angle sent to robot (for normalization to avoid large rotations)
  double last_j6_deg_ = 0.0;

  /**
   * @brief Connect to the real robot via socket
   * @return true on success
   */
  bool connectToRealRobot();

  /**
   * @brief Disconnect from the real robot
   */
  void disconnectFromRealRobot();

  /**
   * @brief Send a command to the robot and receive response
   * @param cmd Command string (e.g., "PING", "MOVEJ 0 0 0 0 0 0")
   * @param response Response string (output)
   * @param timeout_sec Timeout in seconds (default 30)
   * @return true if command sent and response received successfully
   */
  bool sendSocketCommand(const std::string& cmd, std::string& response, double timeout_sec = 30.0);

  /**
   * @brief Send joint move command to real robot
   * @param joints_rad Joint positions in radians (will be converted to degrees)
   * @return true on success
   */
  bool sendMoveJ(const std::vector<double>& joints_rad);

  /**
   * @brief Send linear cartesian move command to real robot
   * @param pose Target pose (position in meters, orientation as quaternion)
   * @return true on success
   */
  bool sendMoveL(const geometry_msgs::msg::Pose& pose);

  /**
   * @brief Get current cartesian position from real robot
   * @param x_mm, y_mm, z_mm Position in mm
   * @param q1, q2, q3, q4 Quaternion (ABB format: q1=w, q2=x, q3=y, q4=z)
   * @return true on success
   */
  bool getCartesianPosition(double& x_mm, double& y_mm, double& z_mm,
                            double& q1, double& q2, double& q3, double& q4);

  /**
   * @brief Move linearly to target Z, keeping current X, Y, and orientation
   * @param target_z_meters Target Z height in meters
   * @return true on success
   */
  bool sendMoveLZ(double target_z_meters);

  /**
   * @brief Move linearly with explicit position and orientation (all in mm, ABB quaternion)
   * @param x_mm, y_mm, z_mm Position in mm
   * @param q1, q2, q3, q4 Orientation quaternion (ABB format)
   * @return true on success
   */
  bool sendMoveLWithOrientation(double x_mm, double y_mm, double z_mm,
                                double q1, double q2, double q3, double q4);

  /**
   * @brief Move linearly by delta Z only (robot gets current pos, changes Z)
   * @param delta_z_mm Delta Z in mm (positive=up, negative=down)
   * @return true on success
   */
  bool sendMoveZ(double delta_z_mm);

  /**
   * @brief Send grip (close gripper) command to real robot
   * @return true on success
   */
  bool sendGrip();

  /**
   * @brief Send release (open gripper) command to real robot
   * @return true on success
   */
  bool sendRelease();
};

#endif // ROBOT_HL_INTERFACE_SIMPLE_HPP
