import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get directories
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_rover = get_package_share_directory('rover')
    
    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Points to the map we saved earlier
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_rover, 'maps', 'my_maze_map.yaml'))
    # Points to the params file we created
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_rover, 'config', 'nav2_params.yaml'))

    return LaunchDescription([
        # Declare arguments for flexibility
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_rover, 'maps', 'my_maze_map.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_rover, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        # Include the standard Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
                'params_file': params_file,
                'autostart': 'true' # Automatically transition nodes to Active
            }.items()
        )
    ])