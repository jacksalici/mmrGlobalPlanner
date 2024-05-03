import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('global_planner'),
        'config',
        'params.yaml'
        )
        
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='global_planner',
            executable='global_planner_node',
            name='global_planner_node',
            parameters = [config]),
  ])