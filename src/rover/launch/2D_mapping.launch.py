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
                {'odom_frame': 'base_footprint'}, # Matches your rover.urdf
                {'map_frame': 'map'},
                {'scan_topic': '/scan'},
                {'mode': 'mapping'},
                
                # --- STABILITY TUNING (Fixes Misalignment) ---
                {'map_update_interval': 1.0},     # Update map once per second (Stable)
                {'resolution': 0.05},
                {'max_laser_range': 20.0},
                
                # --- MOTION THRESHOLDS ---
                # Only update map when robot moves significantly
                {'minimum_time_interval': 0.5},   # Wait 0.5s between processing
                {'minimum_travel_distance': 0.3}, # Move 30cm
                {'minimum_travel_heading': 0.25}, # Rotate ~15 degrees
                
                # --- ALGORITHMIC FIXES ---
                {'use_scan_matching': True},      # Helps correct odometry drift
                {'do_loop_closing': True},
                {'loop_match_minimum_chain_size': 10}, # Require better matches for loops
                
                # --- TF BUFFER (Prevents Lag) ---
                {'transform_timeout': 0.5},      
                {'tf_buffer_duration': 60.0},    
                {'stack_size_to_use': 40000000}, 
                {'enable_interactive_mode': True},
            ],
        ),

        # Lifecycle Manager
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