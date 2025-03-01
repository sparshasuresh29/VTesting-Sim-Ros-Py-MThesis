from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    """Generate launch description for CARLA ScenarioRunner with ROS2 integration."""
    
    # Declare launch arguments
    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='25.0',
        description='Distance threshold for overtaking (meters)'
    )
    
    min_overtaking_time_arg = DeclareLaunchArgument(
        'min_overtaking_time',
        default_value='15.0',
        description='Minimum time to spend in overtaking lane (seconds)'
    )
    
    scenario_file_arg = DeclareLaunchArgument(
        'scenario_file',
        default_value='Overtaking.xosc',
        description='OpenSCENARIO file to execute'
    )
    
    # Decision maker node
    decision_node = Node(
        package='carla_ros2_bridge',
        executable='overtaking_decision_maker',
        name='overtaking_decision_maker',
        output='screen',
        parameters=[{
            'distance_threshold': LaunchConfiguration('distance_threshold'),
            'safe_return_distance': 35.0,
            'min_overtaking_speed': 35.0,
            'min_overtaking_time': LaunchConfiguration('min_overtaking_time')
        }]
    )
    
    # Note: The ScenarioRunner launch is commented out because it should be run 
    # separately with the ROS2 bridge agent as an external agent
    
    # scenario_runner = ExecuteProcess(
    #     cmd=[
    #         'python3', '/path/to/scenario_runner/scenario_runner.py',
    #         '--openscenario', LaunchConfiguration('scenario_file'),
    #         '--agent', os.path.expanduser('~/ros2_ws/src/carla_ros2_bridge/scripts/ros2_bridge_agent.py')
    #     ],
    #     output='screen'
    # )
    
    return LaunchDescription([
        distance_threshold_arg,
        min_overtaking_time_arg,
        scenario_file_arg,
        decision_node,
        # scenario_runner  # Uncomment if you want to launch ScenarioRunner from here
    ])
