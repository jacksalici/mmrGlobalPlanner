import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('global_planner'),
        'config',
        'params.yaml'
        )
        
    return LaunchDescription([
        Node(
            package='global_planner',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters = [config]
            )
        ] 
    )