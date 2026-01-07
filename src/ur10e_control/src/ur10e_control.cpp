#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <vector>
#include <string>

// Struktur für eine Roboterpose
struct RobotPose {
    double x, y, z;           // Position
    double qx, qy, qz, qw;    // Orientierung als Quaternion
    std::string name;         // Name der Pose
};

// Speichert eine Pose in eine Datei
void savePoseToFile(const std::vector<RobotPose>& poses, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& pose : poses) {
            file << pose.name << ","
                 << pose.x << "," << pose.y << "," << pose.z << ","
                 << pose.qx << "," << pose.qy << "," << pose.qz << "," << pose.qw << std::endl;
        }
        file.close();
    }
}

// Lädt Posen aus einer Datei
std::vector<RobotPose> loadPosesFromFile(const std::string& filename) {
    std::vector<RobotPose> poses;
    std::ifstream file(filename);
    std::string line;
    
    if (file.is_open()) {
        while (getline(file, line)) {
            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            
            while (getline(ss, token, ',')) {
                tokens.push_back(token);
            }
            
            if (tokens.size() == 8) {
                RobotPose pose;
                pose.name = tokens[0];
                pose.x = std::stod(tokens[1]);
                pose.y = std::stod(tokens[2]);
                pose.z = std::stod(tokens[3]);
                pose.qx = std::stod(tokens[4]);
                pose.qy = std::stod(tokens[5]);
                pose.qz = std::stod(tokens[6]);
                pose.qw = std::stod(tokens[7]);
                poses.push_back(pose);
            }
        }
        file.close();
    }
    
    return poses;
}

int main(int argc, char * argv[])
{
  // ROS initialisieren und Node erstellen
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur10e_control",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .parameter_overrides({{"use_sim_time", true}})
  );

  // Logger erstellen
  auto const logger = rclcpp::get_logger("ur10e_control");
  RCLCPP_INFO(logger, "Node initialisiert");

  // Executor für den Node erstellen und in separatem Thread starten
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Warten damit ROS Zeit hat, sich zu initialisieren
  RCLCPP_INFO(logger, "Warte auf ROS Initialisierung...");
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Verwende die bekannte, korrekte Gruppenbezeichnung
  const std::string gruppe = "irb120_arm";
  
  try {
    RCLCPP_INFO(logger, "Verbinde mit Robotergruppe: %s", gruppe.c_str());
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, gruppe);
    RCLCPP_INFO(logger, "Erfolgreich mit Gruppe %s verbunden", gruppe.c_str());
    
    // Setze Parameter für sichere Bewegungen
    move_group->setMaxVelocityScalingFactor(0.1);
    move_group->setMaxAccelerationScalingFactor(0.1);
    move_group->setPlanningTime(15.0);
    
    // Zeige Informationen zum Roboter
    RCLCPP_INFO(logger, "Planungsframe: %s", move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "Endeffektorlink: %s", move_group->getEndEffectorLink().c_str());

    // Dateiname für das Speichern der Posen
    std::string poseFilename = "ur10e_poses.csv";
    std::vector<RobotPose> savedPoses;
    
    // Mode-Auswahl (Benutzeranpassung möglich)
    enum Mode { JOINT_MODE, CARTESIAN_MODE };
    Mode currentMode = JOINT_MODE;

    // Nach Benutzerparameter prüfen (für spätere Erweiterungen)
    if (argc > 1) {
      std::string arg = argv[1];
      if (arg == "cartesian") {
        currentMode = CARTESIAN_MODE;
        RCLCPP_INFO(logger, "Starte im kartesischen Modus - Bewege zu gespeicherten Posen");
      } else {
        RCLCPP_INFO(logger, "Starte im Gelenkmodus - Fahre Gelenkpositionen an und speichere Posen");
      }
    }

    if (currentMode == JOINT_MODE) {
      RCLCPP_INFO(logger, "MODUS: Gelenk-Bewegungen und Posenspeicherung");
      
      // Liste der anzufahrenden Gelenkpositionen
      std::vector<std::vector<double>> jointPositions = {
        {0.0, -1.57, 0.0, -1.57, 0.0, 0.0},         // Position 1: Grundstellung
        {0.7, -1.57, 0.0, -1.57, 0.0, 0.0},         // Position 2: Drehung um Basis
        {0.7, -1.0, 0.5, -1.57, 0.0, 0.0},          // Position 3: Arm angehoben
        {0.0, -1.0, 0.5, -1.57, 0.0, 0.0},          // Position 4: Zurück zur Mitte, aber angehoben
        {-0.7, -1.2, 0.2, -1.57, 0.7, 0.0}          // Position 5: Andere Seite mit Drehung
      };
      
      std::vector<std::string> positionNames = {
        "Grundstellung",
        "Seitlich_rechts",
        "Angehoben_rechts",
        "Angehoben_mitte",
        "Seitlich_links_gedreht"
      };

      // Jede Position nacheinander anfahren
      for (size_t i = 0; i < jointPositions.size(); ++i) {
        std::string posName = positionNames[i];
        RCLCPP_INFO(logger, "Fahre zu Gelenkposition %zu: %s", i+1, posName.c_str());
        
        move_group->setJointValueTarget(jointPositions[i]);
        
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        bool success = static_cast<bool>(move_group->plan(joint_plan));
        
        if (success) {
          RCLCPP_INFO(logger, "Führe Plan aus");
          move_group->execute(joint_plan);
          RCLCPP_INFO(logger, "Position erreicht");
          
          // Aktuelle kartesische Pose abfragen
          geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
          
          // Pose in Struktur speichern
          RobotPose pose;
          pose.name = posName;
          pose.x = current_pose.pose.position.x;
          pose.y = current_pose.pose.position.y;
          pose.z = current_pose.pose.position.z;
          pose.qx = current_pose.pose.orientation.x;
          pose.qy = current_pose.pose.orientation.y;
          pose.qz = current_pose.pose.orientation.z;
          pose.qw = current_pose.pose.orientation.w;
          
          // Pose speichern
          savedPoses.push_back(pose);
          
          // Pose ausgeben
          RCLCPP_INFO(logger, "Pose %zu (%s): Position x=%.3f, y=%.3f, z=%.3f",
                     i+1, posName.c_str(), pose.x, pose.y, pose.z);
          RCLCPP_INFO(logger, "           Orientierung qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                     pose.qx, pose.qy, pose.qz, pose.qw);
          
          // Kurze Pause
          rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
          RCLCPP_ERROR(logger, "Planung zu Position %zu fehlgeschlagen!", i+1);
        }
      }
      
      // Posen in Datei speichern
      savePoseToFile(savedPoses, poseFilename);
      RCLCPP_INFO(logger, "Posen wurden in Datei '%s' gespeichert", poseFilename.c_str());
      
      // Zurück zur Ausgangsposition
      RCLCPP_INFO(logger, "Kehre zur Ausgangsposition zurück");
      move_group->setJointValueTarget(jointPositions[0]);
      
      moveit::planning_interface::MoveGroupInterface::Plan return_plan;
      bool return_success = static_cast<bool>(move_group->plan(return_plan));
      
      if (return_success) {
        move_group->execute(return_plan);
        RCLCPP_INFO(logger, "Ausgangsposition erreicht");
      }
      
    } else {
      // CARTESIAN_MODE
      RCLCPP_INFO(logger, "MODUS: Kartesische Bewegungen basierend auf gespeicherten Posen");
      
      // Posen aus Datei laden
      std::vector<RobotPose> loadedPoses = loadPosesFromFile(poseFilename);
      
      if (loadedPoses.empty()) {
        RCLCPP_ERROR(logger, "Keine Posen geladen! Datei '%s' existiert nicht oder ist leer.", poseFilename.c_str());
      } else {
        RCLCPP_INFO(logger, "%zu Posen aus Datei geladen", loadedPoses.size());
        
        // Jede Pose nacheinander anfahren
        for (size_t i = 0; i < loadedPoses.size(); ++i) {
          const auto& pose = loadedPoses[i];
          RCLCPP_INFO(logger, "Fahre zu kartesischer Pose %zu: %s", i+1, pose.name.c_str());
          
          // Pose setzen
          geometry_msgs::msg::Pose target_pose;
          target_pose.position.x = pose.x;
          target_pose.position.y = pose.y;
          target_pose.position.z = pose.z;
          target_pose.orientation.x = pose.qx;
          target_pose.orientation.y = pose.qy;
          target_pose.orientation.z = pose.qz;
          target_pose.orientation.w = pose.qw;
          
          move_group->setPoseTarget(target_pose);
          
          // Plan erstellen und ausführen
          moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
          bool success = static_cast<bool>(move_group->plan(pose_plan));
          
          if (success) {
            RCLCPP_INFO(logger, "Führe Plan aus");
            move_group->execute(pose_plan);
            RCLCPP_INFO(logger, "Pose erreicht");
            
            // Kurze Pause
            rclcpp::sleep_for(std::chrono::seconds(2));
          } else {
            RCLCPP_ERROR(logger, "Planung zu Pose %zu fehlgeschlagen!", i+1);
          }
        }
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Fehler beim Verbinden oder Planen: %s", e.what());
  }

  // Cleanup
  RCLCPP_INFO(logger, "Programm beendet");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
