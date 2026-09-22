from launch import LaunchDescription
from drone_bringup.launch_helpers import arguments, node


def generate_launch_description():
    # Start scripts/run_agent.sh once before this standalone bridge launch.
    return LaunchDescription(arguments() + [node('drone_px4_interface', 'gateway')])
