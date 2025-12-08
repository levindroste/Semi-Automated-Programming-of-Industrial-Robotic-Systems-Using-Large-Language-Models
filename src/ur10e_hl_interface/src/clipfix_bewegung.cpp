#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("irb120_test_movement");

    RCLCPP_INFO(node->get_logger(), "=== IRB 120 Test Movement ===");
    RCLCPP_INFO(node->get_logger(), "Initializing MoveIt interface...");

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "irb120_arm");

    // Set velocity and acceleration scaling for smooth movements
    move_group->setMaxVelocityScalingFactor(0.5);
    move_group->setMaxAccelerationScalingFactor(0.5);
    move_group->setPlanningTime(10.0);

    RCLCPP_INFO(node->get_logger(), "Planning group: %s", move_group->getName().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector: %s", move_group->getEndEffectorLink().c_str());

    // Test positions (same as demonstrated via CLI)
    std::vector<std::pair<std::string, std::vector<double>>> test_positions = {
        {"Position 1: Rotate base right, bend arm", {0.8, -0.5, 0.5, 0.0, 0.3, 0.0}},
        {"Position 2: Rotate base left, different config", {-0.8, -0.3, 0.8, 0.5, -0.3, 0.5}},
        {"Position 3: Reach upward", {0.0, -1.2, 1.0, 0.0, 0.5, 0.0}},
        {"Home: All joints zero", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}
    };

    RCLCPP_INFO(node->get_logger(), "Starting test movements...\n");

    for (size_t i = 0; i < test_positions.size(); ++i) {
        const auto& [description, joint_values] = test_positions[i];

        RCLCPP_INFO(node->get_logger(), "[%zu/%zu] %s", i + 1, test_positions.size(), description.c_str());
        RCLCPP_INFO(node->get_logger(), "  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    joint_values[0], joint_values[1], joint_values[2],
                    joint_values[3], joint_values[4], joint_values[5]);

        move_group->setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(node->get_logger(), "  Planning succeeded, executing...");
            auto exec_result = move_group->execute(plan);
            if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "  Execution succeeded!\n");
            } else {
                RCLCPP_ERROR(node->get_logger(), "  Execution failed!\n");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "  Planning failed!\n");
        }

        // Brief pause between movements
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(node->get_logger(), "=== Test Complete! ===");
    rclcpp::shutdown();
    return 0;
}
