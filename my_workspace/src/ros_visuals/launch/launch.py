from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch description
    ld = LaunchDescription()
    
    rviz_path = "/home/devel/.rviz2/cube_config.rviz"

    # Add the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )
    
    ros_visuals_node = Node(
        package='ros_visuals',
        executable='ros_visuals_node',
        name='ros_visuals_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    
    
    ld.add_action(rviz_node)
    ld.add_action(ros_visuals_node)

    # Return the launch description
    return ld