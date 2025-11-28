import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the path to the rtabmap_launch package
    rtabmap_launch_dir = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
            ),
            # All parameters and topics are passed here
            launch_arguments={
                # --- Basic RTAB-Map parameters ---
                'frame_id': 'base_footprint',
                'approx_sync': 'true',
                'use_sim_time': use_sim_time,
                'subscribe_depth': 'true',
                'subscribe_rgb': 'true',
                'subscribe_imu': 'true',  # Re-enabled IMU
                'rtabmap_args': '--delete_db_on_start',
                
                # --- Topic Remappings ---
                'imu_topic': '/imu/data',
                'odom_topic': '/odom',
                'rgb_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'depth_topic': '/camera/depth/image_raw',
                
                # --- Synchronization & QoS ---
                'qos': '2',           # Best Effort (Critical for Gazebo)
                'queue_size': '30',   # Standard queue size
                
                # --- SLAM parameters ---
                'Reg/Strategy': '1',      # 1=ICP (good for Lidar/Depth)
                'Reg/Force3DoF': 'true',  # Force 2D movement
                'Grid/RangeMax': '5.0',
                'RGBD/ProximityBySpace': 'false',

                # --- Visualization ---
                'rtabmap_viz': 'true',
                'rviz': 'false'
            }.items()
        )
    ])