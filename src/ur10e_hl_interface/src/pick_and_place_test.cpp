#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotHLInterfaceSimple>();

  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "    ClearAllCubes + AddCube Test");
  RCLCPP_INFO(node->get_logger(), "========================================");

  // Initialize MoveIt and load AML configuration
  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface");
    rclcpp::shutdown();
    return 1;
  }

  // Display available objects before clearing
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "Objects before ClearAllCubes:");
  for (const auto& obj : node->getAvailableObjects()) {
    std::string location = node->getObjectLocation(obj);
    RCLCPP_INFO(node->get_logger(), "  - %s at %s", obj.c_str(), location.c_str());
  }

  // Clear all cubes
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Calling ClearAllCubes()...");
  RCLCPP_INFO(node->get_logger(), "========================================");
  node->ClearAllCubes();

  // Display objects after clearing (should be empty)
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "Objects after ClearAllCubes:");
  auto objects_after_clear = node->getAvailableObjects();
  if (objects_after_clear.empty()) {
    RCLCPP_INFO(node->get_logger(), "  (none - scene is empty)");
  } else {
    for (const auto& obj : objects_after_clear) {
      std::string location = node->getObjectLocation(obj);
      RCLCPP_INFO(node->get_logger(), "  - %s at %s", obj.c_str(), location.c_str());
    }
  }

  // Add new cubes in a traffic light pattern at C3
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Adding traffic light at C3...");
  RCLCPP_INFO(node->get_logger(), "========================================");

  std::string cube_red = node->AddCube("C3", "Rot", 0);
  RCLCPP_INFO(node->get_logger(), "Added: %s (red) at C3 level 0", cube_red.c_str());

  std::string cube_yellow = node->AddCube("C3", "Gelb", 1);
  RCLCPP_INFO(node->get_logger(), "Added: %s (yellow) at C3 level 1", cube_yellow.c_str());

  std::string cube_green = node->AddCube("C3", "Gruen", 2);
  RCLCPP_INFO(node->get_logger(), "Added: %s (green) at C3 level 2", cube_green.c_str());

  // Display final positions
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Final object positions:");
  RCLCPP_INFO(node->get_logger(), "========================================");
  for (const auto& obj : node->getAvailableObjects()) {
    std::string location = node->getObjectLocation(obj);
    RCLCPP_INFO(node->get_logger(), "  - %s at %s", obj.c_str(), location.c_str());
  }

  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "    Test Complete");
  RCLCPP_INFO(node->get_logger(), "========================================");

  rclcpp::shutdown();
  return 0;
}
