import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'base_frame': 'chassis'},
                {'odom_frame': 'base_footprint'}, # FIXED: Changed from 'odom' to match your URDF
                {'map_frame': 'map'},
                {'scan_topic': '/scan'},
                {'mode': 'mapping'},
                {'map_update_interval': 3.0},
                {'max_laser_range': 10.0},
                {'minimum_time_interval': 0.5},
                {'transform_timeout': 0.2},
                {'tf_buffer_duration': 30.0},
            ],
        ),

        # Lifecycle Manager (Required to start SLAM Toolbox)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['slam_toolbox']}
            ]
        )
    ])