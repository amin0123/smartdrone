from launch import LaunchDescription
from drone_bringup.launch_helpers import arguments, node


def generate_launch_description():
    return LaunchDescription(arguments() + [node('drone_mission_manager', 'mission_manager'),
        node('drone_safety', 'battery'), node('drone_safety', 'safety')])
