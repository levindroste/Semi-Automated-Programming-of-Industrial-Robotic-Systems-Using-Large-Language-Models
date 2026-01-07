#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<RobotHLInterfaceSimple>();

    RCLCPP_INFO(robot->get_logger(), "Starte Aufgabenausführung...");

    // Initialize MoveIt and load AML configuration
    if (!robot->initialize()) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler bei der Initialisierung der Roboterschnittstelle");
        rclcpp::shutdown();
        return 1;
    }

    // === PHASE 1: ZELLKONFIGURATION (Instant-Operationen, keine Roboterbewegung) ===

    RCLCPP_INFO(robot->get_logger(), "Phase 1: Zellkonfiguration wird vorbereitet...");

    // Entferne alle existierenden Würfel
    robot->ClearAllCubes();
    RCLCPP_INFO(robot->get_logger(), "Alle Würfel wurden entfernt");

    // Erstelle roten Würfel bei C5
    std::string cube_rot = robot->AddCube("C5", "Rot", 0);
    RCLCPP_INFO(robot->get_logger(), "Roter Würfel erstellt: %s bei C5", cube_rot.c_str());

    // Erstelle gelben Würfel bei B5
    std::string cube_gelb = robot->AddCube("B5", "Gelb", 0);
    RCLCPP_INFO(robot->get_logger(), "Gelber Würfel erstellt: %s bei B5", cube_gelb.c_str());

    // Erstelle grünen Würfel bei A5
    std::string cube_gruen = robot->AddCube("A5", "Gruen", 0);
    RCLCPP_INFO(robot->get_logger(), "Grüner Würfel erstellt: %s bei A5", cube_gruen.c_str());

    RCLCPP_INFO(robot->get_logger(), "Phase 1 abgeschlossen: Drei Würfel in Reihe 5 platziert");

    // === PHASE 2: ROBOTER-AUFGABE (Physische Bewegungen) ===

    RCLCPP_INFO(robot->get_logger(), "Phase 2: Baue Ampel bei A2...");

    // Platziere roten Würfel als Basis bei A2 (Level 0)
    RCLCPP_INFO(robot->get_logger(), "Platziere roten Würfel (Basis) bei A2...");
    if (!robot->PickAndPlace(cube_rot, "A2", 0)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler beim Platzieren des roten Würfels bei A2!");
        return 1;
    }
    RCLCPP_INFO(robot->get_logger(), "Roter Würfel erfolgreich bei A2 platziert");

    // Stapel gelben Würfel auf roten Würfel bei A2 (Level 1)
    RCLCPP_INFO(robot->get_logger(), "Stapel gelben Würfel auf roten Würfel...");
    if (!robot->PickAndPlace(cube_gelb, "A2", 1)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler beim Stapeln des gelben Würfels auf Level 1!");
        return 1;
    }
    RCLCPP_INFO(robot->get_logger(), "Gelber Würfel erfolgreich auf Level 1 gestapelt");

    // Stapel grünen Würfel auf gelben Würfel bei A2 (Level 2)
    RCLCPP_INFO(robot->get_logger(), "Stapel grünen Würfel auf gelben Würfel...");
    if (!robot->PickAndPlace(cube_gruen, "A2", 2)) {
        RCLCPP_ERROR(robot->get_logger(), "Fehler beim Stapeln des grünen Würfels auf Level 2!");
        return 1;
    }
    RCLCPP_INFO(robot->get_logger(), "Grüner Würfel erfolgreich auf Level 2 gestapelt");

    RCLCPP_INFO(robot->get_logger(), "Ampel erfolgreich bei A2 gebaut (Rot-Gelb-Grün)");

    // Fahre zur Home-Position zurück
    RCLCPP_INFO(robot->get_logger(), "Fahre zur Home-Position...");
    robot->moveToHome();

    RCLCPP_INFO(robot->get_logger(), "Aufgabe erfolgreich abgeschlossen!");
    rclcpp::shutdown();
    return 0;
}