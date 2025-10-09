import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    This function creates the ROS 2 launch description, which defines the actions
    to be executed when the launch file is run.
    """

    # --- 1. Locate Necessary Packages and Files ---

    # Find the directory where the 'car_description' package is installed.
    # This makes the launch file portable and not dependent on absolute paths.
    pkg_car_description = get_package_share_directory('car_description')
    
    # Find the directory for 'gazebo_ros', which contains standard Gazebo launch files.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Construct the full path to your car's SDF model file.
    model_path = os.path.join(pkg_car_description, 'models', 'car_chassis.sdf')


    # --- 2. Define the Actions to be Executed ---

    # Action to launch the Gazebo simulator.
    # We are including (re-using) a launch file provided by the gazebo_ros package.
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )

    # Action to spawn your car model into the Gazebo world.
    # This runs the 'spawn_entity.py' script provided by gazebo_ros.
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'car_chassis',    # The name of the entity in Gazebo
            '-file', model_path,        # The full path to the model file
            '-x', '0',                  # The initial X coordinate
            '-y', '0',                  # The initial Y coordinate
            '-z', '0.5'                 # The initial Z coordinate (slightly above ground)
        ],
        output='screen' # Show the output of this command in the terminal
    )

    
    # --- 3. Assemble and Return the Launch Description ---

    # The LaunchDescription is a container for all the actions to be performed.
    # The actions will be executed in the order they are listed.
    return LaunchDescription([
        start_gazebo_cmd,
        spawn_entity_cmd,
    ])

