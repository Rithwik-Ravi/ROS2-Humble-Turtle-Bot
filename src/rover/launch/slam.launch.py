import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Parameters for SLAM
    slam_params = {
        'frame_id': 'chassis',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_imu': True,
        'approx_sync': True,
        'use_sim_time': True, 
        'RGBD/ProximityBySpace': 'false',
        'Reg/Strategy': '1', 
        'Reg/Force3DoF': 'true',
        'Grid/RangeMax': '5.0',
    }

    # Remappings for all our topics
    remap_topics = [
        ('imu/data', '/imu/data'),
        ('odom', '/odom'),
        ('rgb/image', '/camera/realsense_d435/image_raw'),
        ('rgb/camera_info', '/camera/realsense_d435/camera_info'),
        ('depth/image', '/camera/realsense_d435/depth/image_raw'),
        ('depth/camera_info', '/camera/realsense_d435/depth/camera_info')
    ]

    return LaunchDescription([

        # Start the rtabmap SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[slam_params],
            remappings=remap_topics,
            arguments=['-d']
        ),

        # Start the rtabmap visualization GUI
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',  # <--- THIS IS THE FIX (with underscore)
            name='rtabmapviz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=remap_topics
        )
    ])