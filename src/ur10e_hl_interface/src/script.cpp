#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<RobotHLInterfaceSimple>();
    RCLCPP_INFO(robot->get_logger(), "Starte Klemmen-Bestückung...");

    if (!robot->initialize()) {
        RCLCPP_ERROR(robot->get_logger(), "Initialisierung fehlgeschlagen");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(robot->get_logger(), "Bestücke Karton1 mit PaketA und 3x Klemme4...");
    
    if (!robot->PickAndPlace("Klemme2", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme2 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme2", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme2 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme3", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme3 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme5", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme5 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme5", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme5 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme4", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme4 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme4", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme4 -> Karton1");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme4", "Karton1", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme4 -> Karton1");
        return 1;
    }

    RCLCPP_INFO(robot->get_logger(), "Bestücke Karton2 mit PaketA und PaketB (ohne Klemme5)...");
    
    if (!robot->PickAndPlace("Klemme2", "Karton2", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme2 -> Karton2");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme2", "Karton2", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme2 -> Karton2");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme3", "Karton2", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme3 -> Karton2");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme3", "Karton2", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme3 -> Karton2");
        return 1;
    }
    if (!robot->PickAndPlace("Klemme3", "Karton2", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei Klemme3 -> Karton2");
        return 1;
    }

    robot->moveToHome();
    RCLCPP_INFO(robot->get_logger(), "Klemmen-Bestückung erfolgreich abgeschlossen!");
    rclcpp::shutdown();
    return 0;
}