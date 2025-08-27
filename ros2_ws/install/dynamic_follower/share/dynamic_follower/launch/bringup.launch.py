from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('dynamic_follower')

    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file to load'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_yaml,
                'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
                'bt_xml_file': os.path.join(bringup_dir, 'trees', 'follow_dynamic_bt.xml')
            }.items()
        )
    ])
