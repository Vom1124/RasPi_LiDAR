from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    lidar_node = Node(       
        package="velodyne_lidar",
        executable="lidar_read",
        output='screen'
    )
    
    ld.add_action(lidar_node)
    
    return ld

