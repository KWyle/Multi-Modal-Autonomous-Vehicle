import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_car_description = get_package_share_directory('car_description')

    # Include your original spawn_car launch file.
    spawn_car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_car_description, 'launch', 'spawn_car.launch.py')
        )
    )

    # Start the drive_car node in a new terminal window to capture keyboard input
    drive_car_node = Node(
        package='car_description',
        executable='drive_car', # This is the name defined in setup.py
        name='drive_car_node',
        output='screen',
        prefix='xterm -e'  # This is the crucial change to open a new terminal
    )

    # Return the complete launch description
    return LaunchDescription([
        spawn_car_launch,
        drive_car_node
    ])

