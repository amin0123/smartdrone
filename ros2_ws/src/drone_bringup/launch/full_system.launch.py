"""The complete ROS portion. Start the pinned simulator/PX4 separately."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from drone_bringup.launch_helpers import arguments, node


def generate_launch_description():
    root = os.environ.get('DRONE_ROOT', os.getcwd())
    return LaunchDescription(arguments() + [
        DeclareLaunchArgument('start_agent', default_value='true'),
        ExecuteProcess(cmd=['bash', os.path.join(root, 'scripts/run_agent.sh')], output='screen',
                       condition=IfCondition(LaunchConfiguration('start_agent'))),
        node('drone_px4_interface', 'gateway'), node('drone_safety', 'battery'),
        node('drone_safety', 'safety'), node('drone_perception', 'perception'),
        node('drone_inspection', 'recorder'), node('drone_docking', 'marker_detector'),
        node('drone_mission_manager', 'mission_manager'),
    ])
