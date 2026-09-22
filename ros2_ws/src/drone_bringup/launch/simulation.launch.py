import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = os.environ.get('DRONE_ROOT', os.getcwd())
    experimental = LaunchConfiguration('experimental_isaac6')
    return LaunchDescription([
        DeclareLaunchArgument('experimental_isaac6', default_value='false'),
        ExecuteProcess(cmd=['bash', os.path.join(root, 'scripts/run_sim.sh'), '--cameras'],
                       output='screen', condition=UnlessCondition(experimental)),
        ExecuteProcess(cmd=['bash', os.path.join(root, 'scripts/run_sim6.sh'), '--cameras'],
                       output='screen', condition=IfCondition(experimental)),
    ])
