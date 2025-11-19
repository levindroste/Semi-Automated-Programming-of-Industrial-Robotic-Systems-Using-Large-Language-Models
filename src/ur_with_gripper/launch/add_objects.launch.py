from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    ur10e_hl_interface_dir = get_package_share_directory('ur10e_hl_interface')
    
    # Path to the mesh files
    mesh_dir = os.path.join(ur10e_hl_interface_dir, 'meshes')
    
    # Path to the AML configuration file
    aml_file = os.path.join(ur10e_hl_interface_dir, 'config', 'AML-Datei-V04.aml')
    
    # Planning scene interface node
    scene_node = Node(
        package='ur10e_hl_interface',
        executable='add_objects_node',
        name='add_objects_node',
        output='screen',
        parameters=[{
            'mesh_directory': mesh_dir,
            'aml_file': aml_file  # NEU: AML-Datei Parameter hinzugefügt
        }]
    )
    
    return LaunchDescription([scene_node])

