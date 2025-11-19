// Node zum Hinzufügen von Objekten aus AML-Konfiguration zur MoveIt Planning Scene

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tinyxml2.h>
#include <algorithm>
#include <map>
#include <sstream>

// Datenstruktur für AML-Objektdaten
struct AMLObjectData {
  std::string name;                    // Objektname
  std::string mesh_file;               // Pfad zur Mesh-Datei
  geometry_msgs::msg::Pose spawn_pose; // Spawn-Position und Orientierung
  std::string color = "Grau";          // Farbe des Objekts
  std::string base_id;                 // Basis-ID für Instanzen
  int count = 1;                       // Anzahl der Instanzen
  double custom_scale = 1.0;           // Benutzerdefinierter Skalierungsfaktor
  bool is_static = false;              // Statisches Objekt (nicht greifbar)
};

// Konfiguration aus AML-Datei
struct AMLConfig {
  std::map<std::string, AMLObjectData> objects;  // Alle Objekte
  std::string mesh_directory;                     // Verzeichnis mit Mesh-Dateien
  double scale_factor = 0.001;                    // Globaler Skalierungsfaktor (mm zu m)
  double object_x_spacing = 0.2;                  // Abstand zwischen Objekten (X-Achse)
  double rail_y_spacing = 0.18;                   // Abstand zwischen Hutschienen (Y-Achse)
};

// Klasse zum Hinzufügen von Objekten zur Planning Scene
class AddObjectsNode : public rclcpp::Node {
public:
  AddObjectsNode() : Node("add_objects_node") {
    // Parameter deklarieren
    declare_parameter("mesh_directory", "");
    declare_parameter("aml_file", "");

    // Konfiguration laden
    if (!loadConfiguration()) {
      throw std::runtime_error("Fehler beim Laden der Konfiguration");
    }

    // Objekte zur Planning Scene hinzufügen
    addObjectsToPlanningScene();
  }

private:
  AMLConfig config_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  // Lädt die Konfiguration aus der AML-Datei
  bool loadConfiguration() {
    config_.mesh_directory = get_parameter("mesh_directory").as_string();
    std::string aml_file = get_parameter("aml_file").as_string();

    if (aml_file.empty()) {
      RCLCPP_ERROR(get_logger(), "Keine AML-Datei angegeben");
      return false;
    }

    return parseAMLFile(aml_file) && validateConfig();
  }

  // Parst die AML-Datei und extrahiert Objektdaten
  bool parseAMLFile(const std::string& filename) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Kann AML-Datei nicht laden: %s", filename.c_str());
      return false;
    }

    auto* root = doc.RootElement();
    if (!root) return false;

    // Suche nach relevanten Hierarchien
    for (auto* hierarchy = root->FirstChildElement("InstanceHierarchy");
         hierarchy; hierarchy = hierarchy->NextSiblingElement("InstanceHierarchy")) {

      const char* name = hierarchy->Attribute("Name");
      if (!name) continue;

      if (strcmp(name, "Roboter-Parameter") == 0) {
        parseRobotParameters(hierarchy);
      } else if (strcmp(name, "Geometrie-Bibliothek") == 0) {
        parseGeometryLibrary(hierarchy);
      }
    }

    return true;
  }

  // Parst Roboter-Parameter aus der AML-Hierarchie
  void parseRobotParameters(tinyxml2::XMLElement* hierarchy) {
    for (auto* elem = hierarchy->FirstChildElement("InternalElement");
         elem; elem = elem->NextSiblingElement("InternalElement")) {

      const char* name = elem->Attribute("Name");
      if (!name) continue;

      if (strcmp(name, "Greif-Parameter") == 0) {
        if (auto spacing = getAttributeValue(elem, "Objekt-X-Spacing"); !spacing.empty()) {
          config_.object_x_spacing = std::stod(spacing) / 1000.0; // mm zu m
        }
      } else if (strcmp(name, "Schaltschrank-Parameter") == 0) {
        if (auto spacing = getAttributeValue(elem, "Hutschienen-Lager-Abstand"); !spacing.empty()) {
          config_.rail_y_spacing = std::stod(spacing); // bereits in m
        }
      }
    }
  }

  // Parst die Geometrie-Bibliothek
  void parseGeometryLibrary(tinyxml2::XMLElement* hierarchy) {
    auto* bauteile = findElement(hierarchy, "InternalElement", "Name", "Bauteile");
    if (!bauteile) return;

    for (auto* bauteil = bauteile->FirstChildElement("InternalElement");
         bauteil; bauteil = bauteil->NextSiblingElement("InternalElement")) {

      const char* name = bauteil->Attribute("Name");
      if (!name) continue;

      AMLObjectData obj;
      obj.name = name;

      // Basis-Attribute parsen
      obj.color = getAttributeValue(bauteil, "Basics/Farbe");
      obj.mesh_file = getAttributeValue(bauteil, "Spawn-Daten/Mesh-Datei");

      // Position und Orientierung
      auto pos_str = getAttributeValue(bauteil, "Spawn-Daten/Basis-Position");
      if (pos_str.empty()) {
        pos_str = getAttributeValue(bauteil, "Spawn-Daten/Position");
      }
      obj.spawn_pose.position = parseVector3(pos_str);

      auto orient_str = getAttributeValue(bauteil, "Spawn-Daten/Spawn-Orientierung");
      if (orient_str.empty()) {
        orient_str = getAttributeValue(bauteil, "Spawn-Daten/Orientierung");
      }
      obj.spawn_pose.orientation = parseQuaternion(orient_str);

      // Skalierung
      if (auto scale = getAttributeValue(bauteil, "Spawn-Daten/Skalierungs-Faktor"); !scale.empty()) {
        obj.custom_scale = std::stod(scale);
      }

      // Anzahl (für bewegliche Objekte)
      if (auto count = getAttributeValue(bauteil, "Roboter-Daten/Anzahl"); !count.empty()) {
        obj.count = std::stoi(count);
      }

      // Statische Objekte markieren
      static const std::set<std::string> static_objects = {
        "Schaltschrank", "Hutschienen-Halter", "Grundplatte",
        "Montageblock", "LPS-Logo"
      };
      obj.is_static = static_objects.count(obj.name) > 0;

      // Basis-ID generieren
      obj.base_id = generateBaseId(obj.name);

      config_.objects[obj.name] = obj;
    }
  }

  // Validiert die geladene Konfiguration
  bool validateConfig() {
    if (config_.objects.empty()) {
      RCLCPP_ERROR(get_logger(), "Keine Objekte in AML-Konfiguration gefunden");
      return false;
    }

    // Prüfe ob Mesh-Dateien für nicht-primitive Objekte vorhanden sind
    for (const auto& [name, obj] : config_.objects) {
      if (!isPrimitiveObject(name) && obj.mesh_file.empty()) {
        RCLCPP_ERROR(get_logger(), "Objekt %s hat keine Mesh-Datei", name.c_str());
        return false;
      }
    }

    return true;
  }

  // Fügt alle Objekte zur Planning Scene hinzu
  void addObjectsToPlanningScene() {
    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
    rclcpp::sleep_for(std::chrono::seconds(1));

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::vector<moveit_msgs::msg::ObjectColor> colors;

    for (const auto& [name, data] : config_.objects) {
      addObjectInstances(data, objects, colors);
    }

    if (!objects.empty()) {
      // Publiziere mit Farbinformationen
      moveit_msgs::msg::PlanningScene scene_msg;
      scene_msg.is_diff = true;
      scene_msg.world.collision_objects = objects;
      scene_msg.object_colors = colors;
      scene_pub_->publish(scene_msg);

      // Auch über Interface für Kompatibilität
      planning_scene_.applyCollisionObjects(objects);
    }
  }

  // Fügt Instanzen eines Objekts hinzu
  void addObjectInstances(const AMLObjectData& data,
                         std::vector<moveit_msgs::msg::CollisionObject>& objects,
                         std::vector<moveit_msgs::msg::ObjectColor>& colors) {

    // Spezialbehandlung für primitive Objekte
    if (isPrimitiveObject(data.name)) {
      auto obj = createPrimitiveObject(data);
      if (!obj.id.empty()) {
        objects.push_back(obj);
        colors.push_back(createObjectColor(obj.id, data.color));
      }
      return;
    }

    // Spezialbehandlung für Schaltschrank (alternative Dateinamen)
    if (data.name == "Schaltschrank") {
      auto obj = createMeshObject(data.base_id, data, data.spawn_pose);
      if (!obj.id.empty()) {
        objects.push_back(obj);
        colors.push_back(createObjectColor(obj.id, data.color));
      } else {
        // Versuche alternativen Dateinamen
        AMLObjectData alt_data = data;
        alt_data.mesh_file = "Schaltschrank_ohne_Tur.stl";
        obj = createMeshObject(data.base_id, alt_data, data.spawn_pose);
        if (!obj.id.empty()) {
          objects.push_back(obj);
          colors.push_back(createObjectColor(obj.id, data.color));
        }
      }
      return;
    }

    // Mesh-basierte Objekte mit mehreren Instanzen
    for (int i = 0; i < data.count; ++i) {
      auto pose = calculateInstancePose(data, i);
      std::string id = data.base_id + std::to_string(i + 1);

      auto obj = createMeshObject(id, data, pose);
      if (!obj.id.empty()) {
        objects.push_back(obj);
        colors.push_back(createObjectColor(id, data.color));
      }
    }
  }

  // Berechnet die Pose für eine Objektinstanz
  geometry_msgs::msg::Pose calculateInstancePose(const AMLObjectData& data, int instance) {
    auto pose = data.spawn_pose;

    if (data.count > 1) {
      double spacing = (data.name == "Hutschiene") ? config_.rail_y_spacing : config_.object_x_spacing;

      if (data.name == "Hutschiene") {
        pose.position.y += instance * spacing;
      } else {
        pose.position.x += instance * spacing;
      }
    }

    return pose;
  }

  // Erstellt ein Mesh-basiertes Kollisionsobjekt
  moveit_msgs::msg::CollisionObject createMeshObject(const std::string& id,
                                                     const AMLObjectData& data,
                                                     const geometry_msgs::msg::Pose& pose) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = id;

    std::string full_path = config_.mesh_directory + "/" + data.mesh_file;
    std::string mesh_path = "file://" + full_path;

    try {
      auto* mesh = shapes::createMeshFromResource(mesh_path);
      if (!mesh) {
        RCLCPP_ERROR(get_logger(), "Kann Mesh nicht laden: %s", full_path.c_str());
        obj.id.clear();
        return obj;
      }

      // Skalierung anwenden
      double scale = config_.scale_factor * data.custom_scale;

      for (unsigned int i = 0; i < mesh->vertex_count * 3; ++i) {
        mesh->vertices[i] *= scale;
      }

      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(mesh, mesh_msg);
      obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
      obj.mesh_poses.push_back(pose);
      obj.operation = obj.ADD;

      delete mesh;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Fehler beim Laden von %s: %s", data.mesh_file.c_str(), e.what());
      obj.id.clear();
    }

    return obj;
  }

  // Erstellt ein primitives Kollisionsobjekt (Box)
  moveit_msgs::msg::CollisionObject createPrimitiveObject(const AMLObjectData& data) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = data.base_id;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);

    // Dimensionen basierend auf Objekttyp
    if (data.name == "Grundplatte") {
      box.dimensions[0] = 4.0;
      box.dimensions[1] = 4.0;
      box.dimensions[2] = 0.02;
    } else if (data.name == "Montageblock") {
      box.dimensions[0] = 0.4;
      box.dimensions[1] = 0.4;
      box.dimensions[2] = 0.4;
    } else {
      obj.id.clear();
      return obj;
    }

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(data.spawn_pose);
    obj.operation = obj.ADD;

    return obj;
  }

  // Erstellt Farbinformation für ein Objekt
  moveit_msgs::msg::ObjectColor createObjectColor(const std::string& id, const std::string& color_name) {
    moveit_msgs::msg::ObjectColor color;
    color.id = id;
    color.color = parseColor(color_name);
    return color;
  }

  // Konvertiert Farbnamen zu RGBA
  std_msgs::msg::ColorRGBA parseColor(const std::string& name) {
    static const std::map<std::string, std::array<float, 3>> color_map = {
      {"Grau", {0.5, 0.5, 0.5}},
      {"Blau", {0.0, 0.0, 1.0}},
      {"Grün-Gelb", {0.5, 1.0, 0.0}},
      {"Rot", {1.0, 0.0, 0.0}},
      {"Gelb", {1.0, 1.0, 0.0}},
      {"Violett", {0.5, 0.0, 0.5}},
      {"Orange", {1.0, 0.5, 0.0}},
      {"Metall", {0.7, 0.7, 0.7}},
      {"Weiß", {1.0, 1.0, 1.0}},
      {"Weiss", {1.0, 1.0, 1.0}},
      {"Schwarz", {0.2, 0.2, 0.2}}
    };

    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;

    if (auto it = color_map.find(name); it != color_map.end()) {
      color.r = it->second[0];
      color.g = it->second[1];
      color.b = it->second[2];
    } else {
      color.r = color.g = color.b = 0.5; // Standard: Grau
    }

    return color;
  }

  // Parst einen 3D-Vektor aus einem String
  geometry_msgs::msg::Point parseVector3(const std::string& str) {
    geometry_msgs::msg::Point point;
    std::istringstream ss(str);
    char comma;
    ss >> point.x >> comma >> point.y >> comma >> point.z;
    return point;
  }

  // Parst ein Quaternion aus einem String
  geometry_msgs::msg::Quaternion parseQuaternion(const std::string& str) {
    geometry_msgs::msg::Quaternion q;
    if (str.empty()) {
      q.w = 1.0; // Standard: keine Rotation
      return q;
    }

    std::istringstream ss(str);
    char comma;
    ss >> q.x >> comma >> q.y >> comma >> q.z >> comma >> q.w;
    return q;
  }

  // Generiert eine Basis-ID aus einem Namen
  std::string generateBaseId(const std::string& name) {
    std::string id = name;
    std::transform(id.begin(), id.end(), id.begin(), ::tolower);
    std::replace(id.begin(), id.end(), '-', '_');
    std::replace(id.begin(), id.end(), ',', '_');
    std::replace(id.begin(), id.end(), '.', '_');

    // Spezialfälle (wichtig für Kompatibilität!)
    if (name == "Schaltschrank") return "schaltschrank_ohne_tuer";
    if (name == "LPS-Logo") return "lps_logo";

    return id + "_";
  }

  // Prüft ob ein Objekt als Primitiv erstellt werden soll
  bool isPrimitiveObject(const std::string& name) {
    return name == "Grundplatte" || name == "Montageblock";
  }

  // Holt einen Attributwert aus einem XML-Element
  std::string getAttributeValue(tinyxml2::XMLElement* element, const std::string& path) {
    std::vector<std::string> parts;
    std::istringstream ss(path);
    std::string part;

    while (std::getline(ss, part, '/')) {
      parts.push_back(part);
    }

    auto* current = element;
    for (size_t i = 0; i < parts.size(); ++i) {
      if (i == parts.size() - 1) {
        // Letztes Element: Suche Attribut
        if (auto* attr = findAttribute(current, parts[i])) {
          if (auto* value = attr->FirstChildElement("Value")) {
            return value->GetText() ? value->GetText() : "";
          }
        }
      } else {
        // Navigiere zum nächsten Container
        current = findAttribute(current, parts[i]);
        if (!current) return "";
      }
    }

    return "";
  }

  // Findet ein Attribut-Element mit gegebenem Namen
  tinyxml2::XMLElement* findAttribute(tinyxml2::XMLElement* parent, const std::string& name) {
    for (auto* attr = parent->FirstChildElement("Attribute");
         attr; attr = attr->NextSiblingElement("Attribute")) {
      if (const char* attr_name = attr->Attribute("Name")) {
        if (name == attr_name) return attr;
      }
    }
    return nullptr;
  }

  // Findet ein Element mit bestimmtem Attributwert
  tinyxml2::XMLElement* findElement(tinyxml2::XMLElement* parent,
                                   const char* elem_name,
                                   const char* attr_name,
                                   const char* attr_value) {
    for (auto* elem = parent->FirstChildElement(elem_name);
         elem; elem = elem->NextSiblingElement(elem_name)) {
      if (const char* value = elem->Attribute(attr_name)) {
        if (strcmp(value, attr_value) == 0) return elem;
      }
    }
    return nullptr;
  }
};

// Hauptfunktion
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<AddObjectsNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Fehler: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}