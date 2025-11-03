import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    urdf_path = os.path.join(package_dir, 'rover.urdf')

    with open(urdf_path, 'r') as infp:
        urdf_content = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
# This launch file starts the robot state publisher, joint state publisher GUI, and RViz for visualization.
# Make sure the URDF file path is correct and the necessary packages are installed.
# To run this launch file, use the command:
# ros2 launch rover 1_rviz.launch.py