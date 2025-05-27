import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch import LaunchDescription
from pathlib import Path


def generate_launch_description():
    # Declare the launch description
    ld = LaunchDescription()
    
    
    rviz_config_dir = os.path.join(FindPackageShare('ros_visuals').find('ros_visuals'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'talos.rviz')


    
    talos_description_pkg_share = FindPackageShare('talos_description').find('talos_description')
    urdf_file_path = os.path.join(talos_description_pkg_share, 'robots', 'talos_reduced.urdf')
    
    try:
        with open(urdf_file_path, 'r', encoding='utf-8') as infp: # Added encoding='utf-8'
            robot_desc = infp.read()
    except FileNotFoundError:
        
        print(f"ERROR: The URDF file was not found at {urdf_file_path}")
        print("Please check the path and ensure the 'talos_description' package is built and sourced.")
        robot_desc = "" # robot_state_publisher will fail with empty string, but FileNotFoundError is the key
    except Exception as e:
        print(f"ERROR: An unexpected error occurred while reading the URDF file: {e}")
        robot_desc = ""


    # Add the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file], # Use your rviz config file
    )
    
    
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
            "robot_description": robot_desc,
            "use_sim_time": False, 
        }
    ],
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node) 
    

    # Return the launch description
    return ld