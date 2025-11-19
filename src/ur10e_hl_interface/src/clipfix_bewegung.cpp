#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<RobotHLInterface>();
    robot->initialize();

    std::string state_file = std::string(std::getenv("HOME")) + 
        "/ur10_ws/src/ur10e_hl_interface/config/SchaltschrankZustand.aml";

    if (!robot->move_to_home()) {
        RCLCPP_ERROR(robot->get_logger(), "Failed to move to home");
        return 1;
    }

    // Rail 1 - already at workspace
    robot->pick("CLIPFIX_35");
    robot->pick("CLIPFIX_35");
    robot->pick("CLIPFIX_35");
    robot->pick("CLIPFIX_35");
    robot->pick("CLIPFIX_35");
    robot->pick("CLIPFIX_35");

    robot->place_rail(-1);  // Store rail in next available cabinet position

    robot->saveStateToAML(state_file);
    robot->move_to_home();
    rclcpp::shutdown();
    return 0;
}