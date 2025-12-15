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
                {'odom_frame': 'base_footprint'}, 
                {'map_frame': 'map'},
                {'scan_topic': '/scan'},
                {'mode': 'mapping'},
                
                # --- QUALITY & SPEED TUNING ---
                {'map_update_interval': 0.5},   # Faster visual updates
                {'resolution': 0.05},          
                {'max_laser_range': 20.0},     
                
                # --- STABILITY TUNING (Fix for Triangle Gaps) ---
                # We update frequently on TURNS (heading) to clear the "triangle" gaps,
                # but keep linear distance moderate to avoid drift.
                {'minimum_time_interval': 0.2},   # Allow faster processing
                {'minimum_travel_distance': 0.3}, # 0.3m linear move
                {'minimum_travel_heading': 0.15}, # 0.15 rad (~8 deg) to catch turns
                {'use_scan_matching': True},
                {'do_loop_closing': True},
                
                # --- CRITICAL FIXES FOR TF LAG ---
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