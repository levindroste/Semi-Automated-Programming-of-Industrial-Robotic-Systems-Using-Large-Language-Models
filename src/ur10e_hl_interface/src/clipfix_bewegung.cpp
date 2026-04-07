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
    robot->pick("VAL_MS_230");      // Rote Klemme
    robot->pick("STU_35_4X10_YE");  // Gelbe Klemme  
    robot->pick("UT_1_5_OG");       // Orange Klemme

    robot->place_rail(-1);  // Store rail 1 in cabinet

    robot->saveStateToAML(state_file);
    robot->move_to_home();
    rclcpp::shutdown();
    return 0;
}