#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"
#include <vector>
#include <string>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<RobotHLInterfaceSimple>();

    RCLCPP_INFO(robot->get_logger(), "=== FULL GRID COVERAGE TEST ===");
    RCLCPP_INFO(robot->get_logger(), "Moving one cube to every grid square (A1-E5)");

    // Initialize MoveIt and load AML configuration
    if (!robot->initialize()) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei der Initialisierung der Roboterschnittstelle");
        rclcpp::shutdown();
        return 1;
    }

    // Clear and create single cube at A1
    robot->ClearAllCubes();
    std::string cube = robot->AddCube("A1", "Rot", 0);
    RCLCPP_INFO(robot->get_logger(), "Created: %s (Red at A1)", cube.c_str());

    // Define all grid squares in order (row by row)
    std::vector<std::string> grid_squares = {
        "A1", "B1", "C1", "D1", "E1",
        "A2", "B2", "C2", "D2", "E2",
        "A3", "B3", "C3", "D3", "E3",
        "A4", "B4", "C4", "D4", "E4",
        "A5", "B5", "C5", "D5", "E5"
    };

    int total = grid_squares.size();
    int current = 1;

    // Move cube to each square (starting from second since cube is at A1)
    for (size_t i = 1; i < grid_squares.size(); i++) {
        const std::string& target = grid_squares[i];

        RCLCPP_INFO(robot->get_logger(), "=== Move %d/%d: %s -> %s ===",
                    current, total - 1, grid_squares[i-1].c_str(), target.c_str());

        if (!robot->PickAndPlace(cube, target, 0)) {
            RCLCPP_ERROR(robot->get_logger(), "Failed to move cube to %s!", target.c_str());
            rclcpp::shutdown();
            return 1;
        }

        current++;
    }

    RCLCPP_INFO(robot->get_logger(), "=== FULL GRID COVERAGE COMPLETE ===");
    RCLCPP_INFO(robot->get_logger(), "Cube visited all 25 grid squares!");
    RCLCPP_INFO(robot->get_logger(), "Final position: E5");

    // Return to home
    robot->moveToHome();

    RCLCPP_INFO(robot->get_logger(), "Test erfolgreich abgeschlossen!");
    rclcpp::shutdown();
    return 0;
}
