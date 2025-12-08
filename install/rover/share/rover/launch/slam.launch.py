from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the path to the rtabmap_launch package
    rtabmap_launch_dir = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
            ),
            # All parameters and topics are passed here
            launch_arguments={
                # --- Basic RTAB-Map parameters ---
                'frame_id': 'base_footprint',
                'approx_sync': 'true',
                'use_sim_time': 'true',
                'subscribe_depth': 'true',
                'subscribe_rgb': 'true',
                'subscribe_imu': 'true',
                'rtabmap_args': '--delete_db_on_start',
                
                # --- Topic Remappings ---
                'imu_topic': '/imu/data',
                'odom_topic': '/odom',
                'rgb_topic': '/camera/realsense_d435/image_raw',
                'camera_info_topic': '/camera/realsense_d435/camera_info',
                'depth_topic': '/camera/realsense_d435/depth/image_raw',
                
                # --- SLAM parameters ---
                'Reg/Strategy': '1',
                'Reg/Force3DoF': 'true',
                'Grid/RangeMax': '5.0',
                'RGBD/ProximityBySpace': 'false',

                # --- Visualization ---
                'rtabmap_viz': 'true',
                'rviz': 'false'
            }.items()
        )
    ])