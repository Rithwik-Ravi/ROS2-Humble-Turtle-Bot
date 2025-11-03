import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    urdf_path = os.path.join(package_dir, 'rover.urdf')

    robot_description_config = xacro.process_file(urdf_path)
    urdf_content = robot_description_config.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
                'use_sim_time': True
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
# Gazebo launch configuration
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "dolly"])
    ])
# This launch file starts the robot state publisher, joint state publisher GUI, and RViz for visualization.
# Make sure the URDF file path is correct and the necessary packages are installed.
# To run this launch file, use the command:
# ros2 launch rover 1_rviz.launch.py