from launch import LaunchDescription
from drone_bringup.launch_helpers import arguments, node


def generate_launch_description():
    return LaunchDescription(arguments() + [node('drone_perception', 'perception'),
        node('drone_inspection', 'recorder'), node('drone_docking', 'marker_detector')])
