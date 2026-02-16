from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: The Serial Bridge (Hardware Connection)
        Node(
            package='rover',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen'
        ),

        # Node 2: The Madgwick Filter (The "Brain")
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu'
            }]
        )
    ])
