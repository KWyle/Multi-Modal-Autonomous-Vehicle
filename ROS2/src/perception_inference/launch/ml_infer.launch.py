from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('perception_inference')
    default_model = os.path.join(pkg_share, 'models', 'yolo12n.pt')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/oak/color/image_raw'),
        DeclareLaunchArgument('model_path', default_value=default_model), 
        DeclareLaunchArgument('conf_thresh', default_value='0.25'), 
        DeclareLaunchArgument('publish_annotated', default_value='true'),

        Node(
            package='perception_inference',
            executable='infer_node',
            name='inference_node',
            output='screen',
            emulate_tty=True, 
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'), 
                'conf_thresh': LaunchConfiguration('conf_thresh'), 
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }]
        )
    ])