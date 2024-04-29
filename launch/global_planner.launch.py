import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='global_planner',
            executable='global_planner_node',
            name='global_planner_node'),
  ])