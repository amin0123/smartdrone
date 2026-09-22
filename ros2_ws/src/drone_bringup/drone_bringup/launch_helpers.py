from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def arguments():
    config = Path(get_package_share_directory('drone_bringup')) / 'config'
    return [DeclareLaunchArgument('params', default_value=str(config/'mission.yaml')),
            DeclareLaunchArgument('world', default_value=str(config/'world.yaml')),
            DeclareLaunchArgument('log_level', default_value='info')]


def node(package, executable):
    return Node(package=package, executable=executable, output='screen',
                parameters=[LaunchConfiguration('params'), {'environment_file': LaunchConfiguration('world')}],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')])
